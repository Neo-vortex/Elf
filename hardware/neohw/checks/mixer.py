"""Mixer and baseband check (spec §5). Transient runs with the LO as a behavioural quadrature
stimulus, one multi-tone run per test frequency (band centres; the ÷4 path below 3.2 MHz and at
1.9 MHz), a Monte Carlo of sampling-capacitor mismatch and LO skew for image rejection, and a
two-tone run on the mixer core for IIP3. Writes mixer.json (spec §6)."""
from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from neohw import availability, charfile, export
from neohw.blocks import bpf, mixer
from neohw.checks import BlockResult, CheckResult
from neohw.ngspice import run_decks
from neohw.parts import CS5361_FS_VPP
from neohw.spice import QuadratureLO, Source, Supply, Tran, node_name, to_deck
from neohw.touchstone import read_s2p

DIRECT_MIN_HZ = 3.2e6          # below this the LO comes from the 74LVC74 ÷4 (parent §5)
TONE_DBM = -40.0
# baseband offsets on a 3 kHz grid (Hann window, 1 kHz bins: products stay >= 3 bins apart)
OFFSETS_HZ = (3e3, 12e3, 21e3, 30e3, 39e3, 48e3, 57e3, -6e3, -15e3, -24e3, -33e3, -42e3, -51e3, -60e3)
FLAT_SPAN_HZ = 40e3
HARMONICS = {2: 9e3, 3: 18e3, 5: 27e3}     # LO harmonic -> offset of its test tone
SETTLE_S, WINDOW_S, OUT_STEP_S = 30e-6, 1e-3, 0.25e-6
STEPS_PER_LO_PERIOD = 280
OPTIONS = ".options reltol=1e-4 method=gear"    # trapezoidal rings on the switched caps
FLATNESS_DB = 0.5
HARMONIC_TOL_DB = 1.0
HARMONIC_MIN_LO_HZ = 1.7e6     # below ~1 MHz the sampling RC is no longer << T_LO (bands 1–2 reported only)
TWO_F_MAX_DBC = -40.0          # mixer alone: reported; gated as system rejection with the filter
SYSTEM_REJ_MIN_DB = 60.0       # filter attenuation at k·LO + mixer k-th harmonic response, bands with a 3f target
IMAGE_MIN_DB = 30.0
IIP3_MIN_DBM = 25.0
MC_RUNS = 40
MC_POINTS = {"direct": 14.2e6, "div4": 1.9e6}
LO_SKEW_S = {"direct": 0.2e-9, "div4": 0.5e-9}   # uniform ±: Si5351 output skew / 74LVC74 flop mismatch
# uniform ± error of each select's high time: Si5351 rise/fall asymmetry (its 45–55 % duty spec is
# at 160 MHz, where the edges are a large part of the period) / 74LVC74 propagation mismatch
LO_DUTY_S = {"direct": 0.5e-9, "div4": 0.3e-9}
SAMPLE_CAP_TOL = 0.01                             # the four caps are selected to ±1 % (BOM note)
ADC_GAIN_MISMATCH_DB = 0.1                        # CS5361 interchannel gain mismatch (datasheet), uniform ±
BASEBAND_VALUES = ("2.2kΩ", "240Ω", "100Ω", "2.7nF")   # in-amp Rf, Rg, anti-alias R and C (their own tolerances)
LM4562_VOS_V = (0.1e-3, 0.7e-3)                   # input offset voltage typical, maximum (datasheet)
OPAMP_SWING_V = 1.0           # LM4562 at 5 V: ±1 V per output around mid-supply (datasheet ~1.4 V from rails at ±15 V)
ADC_FS_PEAK_V = CS5361_FS_VPP * 5.0 / 2


@dataclass(frozen=True)
class TestPoint:
    f_lo: float
    path: str


def test_points() -> list[TestPoint]:
    pts = [TestPoint(math.sqrt(lo * hi), "direct" if math.sqrt(lo * hi) >= DIRECT_MIN_HZ else "div4")
           for lo, hi in bpf.BANDS]
    return pts + [TestPoint(1.9e6, "div4")]


def _amp_in(dbm: float) -> float:
    """Voltage amplitude a matched 50 Ω load sees for `dbm` available power."""
    return math.sqrt(2 * 50 * 1e-3 * 10 ** (dbm / 10))


def _deck(blk, f_lo: float, tones, skew: float = 0.0, variation=None, tap: str = "adc", duty=(0.0, 0.0)):
    if tap == "adc":
        nodes = [*blk.ports["ain_i"], *blk.ports["ain_q"]]
    else:   # mixer core: I = B0 − B3, Q = B1 − B2
        b = blk.ports["b"]
        nodes = [b[0], b[3], b[1], b[2]]
    names = [node_name(n) for n in nodes]
    deck, raws = to_deck(blk.circuit, f"mixer {f_lo:g}",
                         [Source(blk.ports["rf_in"], "rf", tones=tuple(tones)), Supply(blk.ports["vcc"], 5.0),
                          QuadratureLO(blk.ports["s0"], blk.ports["s1"], f_lo, skew_s=skew,
                                       duty_err_s0=duty[0], duty_err_s1=duty[1])],
                         [Tran(OUT_STEP_S, SETTLE_S + WINDOW_S, SETTLE_S, 1 / f_lo / STEPS_PER_LO_PERIOD)],
                         extra_lines=[OPTIONS, _initial_bias(blk), ".save " + " ".join(f"v({x})" for x in names)],
                         variation=variation)
    return deck, raws, names


def _initial_bias(blk) -> str:
    """Start the RF node and the four sampling caps at the 2.5 V bias: a DC solve with the LO
    frozen leaves three caps floating, and they would then drag the bias down for ~1 ms."""
    nodes = [blk.ports["a"], *blk.ports["b"]]
    return ".ic " + " ".join(f"v({node_name(n)})=2.5" for n in nodes)


def _waveforms(res, raws, names) -> list[np.ndarray]:
    d = res[raws[0]][0].data
    t = d["time"].real
    n = int(round(WINDOW_S / OUT_STEP_S))
    m = t >= SETTLE_S - OUT_STEP_S / 2
    return [d[f"v({x})"].real[m][:n] for x in names]


def _spectrum(res, raws, names, i_gain: float = 1.0):
    """`i_gain` scales the I channel (ADC interchannel gain mismatch)."""
    v = _waveforms(res, raws, names)
    n = len(v[0])
    i, q = (v[0] - v[1]) * i_gain, v[2] - v[3]
    z = i - 1j * q                     # upper sideband at positive baseband frequency
    spec = np.fft.fft(z * np.hanning(n)) / (n / 2)
    freqs = np.fft.fftfreq(n, OUT_STEP_S)
    return freqs, spec, i, q


def _bin(freqs, spec, f):
    return spec[int(np.argmin(np.abs(freqs - f)))]


def multitone_tones(f_lo: float):
    tones = [(f_lo + o, TONE_DBM) for o in OFFSETS_HZ]
    tones += [(h * f_lo + o, TONE_DBM) for h, o in HARMONICS.items()]
    return tones


def _h_at(h: dict, f: float) -> complex:
    """Fundamental response at baseband frequency f, linearly interpolated (complex) between the
    measured offsets on the same side."""
    side = sorted(o for o in h if (o > 0) == (f > 0))
    return complex(np.interp(f, side, [h[o].real for o in side]) + 1j * np.interp(f, side, [h[o].imag for o in side]))


def analyse_multitone(f_lo: float, freqs, spec, i, q) -> dict:
    a = _amp_in(TONE_DBM)
    h = {}
    for o in OFFSETS_HZ:
        h[o] = _bin(freqs, spec, o) / a * np.exp(-2j * np.pi * o * SETTLE_S)   # phase referred to t = 0
    ref = np.mean([abs(h[o]) for o in OFFSETS_HZ if abs(o) <= FLAT_SPAN_HZ])
    flat = [20 * math.log10(abs(h[o]) / ref) for o in OFFSETS_HZ if abs(o) <= FLAT_SPAN_HZ]
    image = [20 * math.log10(abs(_bin(freqs, spec, o)) / abs(_bin(freqs, spec, -o))) for o in OFFSETS_HZ]
    harm = {}
    for k, o in HARMONICS.items():
        up, lo = _bin(freqs, spec, o), _bin(freqs, spec, -o)
        side = "upper" if abs(up) >= abs(lo) else "lower"
        f_bb = o if side == "upper" else -o
        c = (up if side == "upper" else lo) / a * np.exp(-2j * np.pi * f_bb * SETTLE_S)
        rel = c / _h_at(h, f_bb)      # complex gain relative to the fundamental at the same baseband frequency
        harm[k] = {"db": 20 * math.log10(max(abs(rel), 1e-30)), "phase_deg": math.degrees(np.angle(rel)), "sideband": side}
    return {"h": h, "ref_gain": ref, "flatness_db": max(abs(x) for x in flat), "image_min_db": min(image),
            "harmonics": harm, "dc_offset_v": (float(i.mean()), float(q.mean()))}


def image_rejection_db(freqs, spec, offset: float = 12e3) -> float:
    return 20 * math.log10(abs(_bin(freqs, spec, offset)) / abs(_bin(freqs, spec, -offset)))


def iip3_dbm(blk=None, f_lo: float = 14.2e6, levels=(-15.0, -10.0)) -> dict:
    blk = blk or mixer.build()
    f1, f2 = 12e3, 21e3                 # products at 3 kHz and 30 kHz
    decks = [_deck(blk, f_lo, [(f_lo + f1, p), (f_lo + f2, p)], tap="core") for p in levels]
    results = run_decks([d for d, _, _ in decks], decks[0][1])
    out = []
    for p, (d, raws, names), res in zip(levels, decks, results):
        freqs, spec, _, _ = _spectrum(res, raws, names)
        fund = (abs(_bin(freqs, spec, f1)) + abs(_bin(freqs, spec, f2))) / 2
        imd = max(abs(_bin(freqs, spec, 2 * f1 - f2)), abs(_bin(freqs, spec, 2 * f2 - f1)))
        dbc = 20 * math.log10(fund / imd)
        out.append({"tone_dbm": p, "imd3_dbc": dbc, "iip3_dbm": p + dbc / 2})
    slope = (out[0]["imd3_dbc"] - out[-1]["imd3_dbc"]) / (levels[-1] - levels[0])   # 2 dB/dB when third-order
    return {"iip3_dbm": out[-1]["iip3_dbm"], "imd3_slope_db_per_db": slope, "runs": out}


def mc_variation(blk, rng, path: str):
    """One Monte Carlo draw: part multipliers (sampling caps ±1 %, baseband R and C at their own
    tolerances), the two select duty errors and the S1 skew."""
    caps = sorted(p.ref for p in blk.circuit.parts if p.value == mixer_cap_value())
    if len(caps) != 4:
        raise RuntimeError(f"expected 4 sampling capacitors, found {len(caps)}")
    var = {ref: 1 + rng.uniform(-SAMPLE_CAP_TOL, SAMPLE_CAP_TOL) for ref in caps}
    for p in sorted(blk.circuit.parts, key=lambda p: p.ref):
        if p.value in BASEBAND_VALUES:
            var[p.ref] = 1 + rng.uniform(-p.tol, p.tol)
    duty = tuple(rng.uniform(-LO_DUTY_S[path], LO_DUTY_S[path]) for _ in range(2))
    skew = rng.uniform(-LO_SKEW_S[path], LO_SKEW_S[path])
    return var, duty, skew


def mixer_cap_value() -> str:
    from neohw.units import format_si, parse_si
    return format_si(parse_si(mixer.C_SAMPLE), "F")


def iq_imbalance(i: np.ndarray, q: np.ndarray, f: float, step: float) -> tuple[float, float]:
    """Gain error (dB, I over Q) and phase error (degrees from quadrature) for an upper-sideband
    tone at baseband frequency f (whole periods in the record)."""
    t = np.arange(len(i)) * step
    ph = np.exp(-2j * np.pi * f * t)
    pi, pq = np.mean(i * ph), np.mean(q * ph)
    gain = 20 * math.log10(abs(pi) / abs(pq))
    phase = math.degrees(np.angle(pq / pi)) - 90.0
    return gain, (phase + 180) % 360 - 180


def image_monte_carlo(blk, path: str, runs: int) -> list[dict]:
    f_lo, f_bb = MC_POINTS[path], 12e3
    decks, adc = [], []
    for r in range(runs):
        rng = np.random.default_rng([7, int(f_lo), r])
        var, duty, skew = mc_variation(blk, rng, path)
        adc.append(10 ** (rng.uniform(-ADC_GAIN_MISMATCH_DB, ADC_GAIN_MISMATCH_DB) / 20))
        decks.append(_deck(blk, f_lo, [(f_lo + f_bb, TONE_DBM)], skew=skew, variation=var, duty=duty))
    results = run_decks([d for d, _, _ in decks], decks[0][1])
    out = []
    for (_, raws, names), res, g in zip(decks, results, adc):
        freqs, spec, i, q = _spectrum(res, raws, names, i_gain=g)
        gain_db, phase_deg = iq_imbalance(i, q, f_bb, OUT_STEP_S)
        out.append({"image_db": image_rejection_db(freqs, spec, f_bb), "gain_err_db": gain_db, "phase_err_deg": phase_deg})
    return out


def system_rejection_db(char_dir: Path, band: int, f_lo: float, harmonic: int, mixer_db: float) -> float:
    """Rejection of the k-th LO harmonic response: filter attenuation at k·f_lo (from the
    committed bpf_<band>.s2p) minus the mixer's k-th harmonic response (dB re fundamental)."""
    path = Path(char_dir) / f"bpf_{band}.s2p"
    if not path.exists():
        raise FileNotFoundError(f"{path} is missing: the bpf_bank check must run before the mixer check")
    f, s = read_s2p(path)
    s21_db = np.interp(harmonic * f_lo, f, 20 * np.log10(np.abs(s["21"])))
    s21_lo = np.interp(f_lo, f, 20 * np.log10(np.abs(s["21"])))
    return float(-(s21_db - s21_lo) - mixer_db)


def system_rejection_band_db(char_dir: Path, band: int, lo: float, hi: float, harmonic: int,
                             mixer_db_at, points: int = 9) -> float:
    """Worst system rejection of the k-th harmonic response over the band (LO from lo to hi,
    geometric steps): the filter falls off fastest near the lower edge."""
    los = np.geomspace(lo, hi, points)
    return min(system_rejection_db(char_dir, band, f, harmonic, mixer_db_at(f)) for f in los)


def dc_offset_bound_v() -> tuple[float, float]:
    """Differential DC offset at the ADC from the two op-amps' input offsets (datasheet):
    typical (uncorrelated) and worst case (opposite signs)."""
    gain = 1 + 2 * float(mixer.R_F.rstrip("k")) * 1e3 / float(mixer.R_G)
    typ, worst = LM4562_VOS_V
    return math.sqrt(2) * typ * gain, 2 * worst * gain


def full_scale_dbm(gain: float) -> float:
    """Input power that drives the ADC (or, first, the op-amp swing) to full scale."""
    v_fs = min(ADC_FS_PEAK_V, 2 * OPAMP_SWING_V)
    a = v_fs / gain
    return 10 * math.log10(a ** 2 / (2 * 50) / 1e-3)


def _image_summary(runs: list[dict], p5: float) -> dict:
    """Median and 5th-percentile image rejection with the I/Q imbalance of the runs at those ranks."""
    order = sorted(runs, key=lambda x: x["image_db"])
    med, worst = order[len(order) // 2], order[max(0, int(round(0.05 * (len(order) - 1))))]
    return {"median": med["image_db"], "p5": p5, "runs": len(runs),
            "median_gain_err_db": med["gain_err_db"], "median_phase_err_deg": med["phase_err_deg"],
            "p5_gain_err_db": worst["gain_err_db"], "p5_phase_err_deg": worst["phase_err_deg"]}


def run(out_dir: Path, char_dir: Path, mc_runs: int = MC_RUNS, points=None) -> BlockResult:
    out_dir, char_dir = Path(out_dir), Path(char_dir)
    blk = mixer.build()
    pts = list(points or test_points())
    decks = [_deck(blk, p.f_lo, multitone_tones(p.f_lo)) for p in pts]
    results = run_decks([d for d, _, _ in decks], decks[0][1])
    per_point = []
    for p, (_, raws, names), res in zip(pts, decks, results):
        per_point.append((p, analyse_multitone(p.f_lo, *_spectrum(res, raws, names))))
    mc = {path: image_monte_carlo(blk, path, mc_runs) for path in MC_POINTS}
    ip3 = iip3_dbm(blk)
    flat = max(r["flatness_db"] for _, r in per_point)
    gated = [r for p, r in per_point if p.f_lo >= HARMONIC_MIN_LO_HZ]
    dev3 = max(abs(r["harmonics"][3]["db"] - 20 * math.log10(1 / 3)) for r in gated)
    dev5 = max(abs(r["harmonics"][5]["db"] - 20 * math.log10(1 / 5)) for r in gated)
    two_f = max(r["harmonics"][2]["db"] for _, r in per_point)
    band_pts = sorted(per_point[:len(bpf.BANDS)], key=lambda pr: pr[0].f_lo)
    def mixer_db_at(k):
        xs = [math.log(p.f_lo) for p, _ in band_pts]
        ys = [r["harmonics"][k]["db"] for _, r in band_pts]
        return lambda f: float(np.interp(math.log(f), xs, ys))
    system = {}
    for band in range(1, len(bpf.BANDS) + 1):
        lo, hi = bpf.BANDS[band - 1]
        if 3 * lo > hi:      # bands with a 3f rejection target (spec §4)
            system[band] = {k: system_rejection_band_db(char_dir, band, lo, hi, k, mixer_db_at(k)) for k in (2, 3)}
    sys2 = min(v[2] for v in system.values())
    sys3 = min(v[3] for v in system.values())
    p5 = {path: float(np.percentile([x["image_db"] for x in v], 5)) for path, v in mc.items()}
    nominal = min(per_point, key=lambda pr: abs(pr[0].f_lo - MC_POINTS["direct"]))[1]
    gain = nominal["ref_gain"]
    checks = [
        CheckResult("flatness_max_dev_db", flat, f"≤ {FLATNESS_DB:g}", flat <= FLATNESS_DB),
        CheckResult("third_harmonic_dev_db", dev3, f"≤ {HARMONIC_TOL_DB:g}", dev3 <= HARMONIC_TOL_DB),
        CheckResult("fifth_harmonic_dev_db", dev5, f"≤ {HARMONIC_TOL_DB:g}", dev5 <= HARMONIC_TOL_DB),
        CheckResult("second_harmonic_max_dbc", two_f, f"report (spec {TWO_F_MAX_DBC:g})", True, "dBc"),
        CheckResult("system_2f_rejection_min_db", sys2, f"≥ {SYSTEM_REJ_MIN_DB:g}", sys2 >= SYSTEM_REJ_MIN_DB),
        CheckResult("system_3f_rejection_min_db", sys3, f"≥ {SYSTEM_REJ_MIN_DB:g}", sys3 >= SYSTEM_REJ_MIN_DB),
        CheckResult("image_rejection_p5_direct_db", p5["direct"], f"≥ {IMAGE_MIN_DB:g}", p5["direct"] >= IMAGE_MIN_DB),
        CheckResult("image_rejection_p5_div4_db", p5["div4"], f"≥ {IMAGE_MIN_DB:g}", p5["div4"] >= IMAGE_MIN_DB),
        CheckResult("iip3_dbm", ip3["iip3_dbm"], f"≥ {IIP3_MIN_DBM:g}", ip3["iip3_dbm"] >= IIP3_MIN_DBM, "dBm"),
        CheckResult("imd3_slope_db_per_db", ip3["imd3_slope_db_per_db"], "report (2 = third order)", True, ""),
        CheckResult("conversion_gain_db", 20 * math.log10(gain), "report", True),
        CheckResult("full_scale_input_dbm", full_scale_dbm(gain), "report", True, "dBm"),
    ]
    result = BlockResult("mixer", checks, plots={
        f"{p.f_lo / 1e6:.2f} MHz ({p.path})": (np.array(sorted(r["h"])), np.array(
            [20 * math.log10(abs(r["h"][o])) for o in sorted(r["h"])])) for p, r in per_point})
    data = availability.load()
    h = nominal["h"]
    harmonics = {"1": {"db": 0.0, "phase_deg": 0.0, "sideband": "upper"}}
    harmonics.update({str(k): v for k, v in nominal["harmonics"].items()})
    result.artefacts = {
        "kicad_netlist": export.kicad_netlist(blk.circuit, out_dir / "mixer.net"),
        "bom": export.write_bom(blk.circuit, out_dir / "mixer_bom.csv", data),
        "svg": export.svg_diagram(blk.circuit, out_dir / "mixer.svg"),
        "characterisation": charfile.write(char_dir / "mixer.json", "mixer", "pass" if result.passed else "fail", {
            "conv_gain_db": 20 * math.log10(gain),
            "conv_gain_definition": "per-channel differential amplitude at the ADC pins / matched-source amplitude at rf_in",
            "baseband_response": {"offset_hz": sorted(h), "re": [float(h[o].real / gain) for o in sorted(h)],
                                  "im": [float(h[o].imag / gain) for o in sorted(h)]},
            "harmonic_responses": harmonics,
            "iip3_dbm": ip3["iip3_dbm"],
            "image_rejection_db": {path: _image_summary(v, p5[path]) for path, v in mc.items()},
            "dc_offset_v": dict(zip(("typical", "worst"), dc_offset_bound_v()),
                                source="LM4562 input offset 0.1 mV typ / 0.7 mV max x in-amp gain (the model has none)"),
            "adc_full_scale_dbm": full_scale_dbm(gain),
            "full_scale_limit": "op-amp swing" if 2 * OPAMP_SWING_V < ADC_FS_PEAK_V else "ADC",
            "test_points": [{"f_lo_hz": p.f_lo, "path": p.path, "flatness_db": r["flatness_db"],
                             "image_min_db": r["image_min_db"], "conv_gain_db": 20 * math.log10(r["ref_gain"]),
                             "harmonics": r["harmonics"]} for p, r in per_point],
            "system_rejection_db": {str(b): {str(k): v for k, v in rej.items()} for b, rej in system.items()},
            "checks": [c.__dict__ for c in checks]}),
    }
    return result
