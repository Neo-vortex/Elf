"""Receiver model built from the committed hardware characterisation (phase 2 spec §6–§7).

Signals are complex baseband envelopes in volts: a tone of available power P from a 50 Ω
source has |x| = sqrt(2·R0·P), the amplitude a matched load would see. The front end
(attenuator, filter, preamp) multiplies that envelope by its transducer gain, the mixer turns
it into the differential ADC voltages z = I − jQ with the measured conversion gain, and the
ADC clips and quantises. Band and receiver noise use the same reference: a noise density N
(W/Hz) gives a complex envelope density 2·R0·N per Hz.
"""
from __future__ import annotations

import json
import math
from dataclasses import dataclass
from pathlib import Path

import numpy as np

SCHEMA_VERSION = 1
K_B = 1.380649e-23
T0 = 290.0
R0 = 50.0
DEFAULT_DIR = Path(__file__).resolve().parents[2] / "hardware" / "characterized"


class HardwareModelError(RuntimeError):
    """The characterisation files are missing, from another schema, or failed their checks."""


# fields the twin reads beyond the plan 2.4 contract (spec §6, plan 2.5)
REQUIRED = {"mixer": ("nf_db", "direct_min_hz", "adc_fs_peak_v", "clip_peak_v", "adc_noise", "bypass_correction", "iip3_dbm",
                       "adc_full_scale_dbm"),
            "lo_phase_noise": ("reference_hz", "offset_hz", "dbc_per_hz", "source"),
            "preamp": ("freq_hz", "gain_db", "nf_db", "bypass_loss_db")}


def _read(path: Path, block: str) -> dict:
    if not path.exists():
        raise HardwareModelError(f"{path} is missing: run `make hw` to generate the characterisation files")
    doc = json.loads(path.read_text())
    if doc.get("schema_version") != SCHEMA_VERSION:
        raise HardwareModelError(f"{path}: schema_version {doc.get('schema_version')} is not supported "
                                 f"(expected {SCHEMA_VERSION})")
    if doc.get("block") != block:
        raise HardwareModelError(f"{path}: block is {doc.get('block')!r}, expected {block!r}")
    if doc.get("status") != "pass":
        failed = [c["name"] for c in doc.get("checks", []) if not c.get("passed")]
        raise HardwareModelError(f"{path}: status is {doc.get('status')!r} (failed checks: {failed}); "
                                 "the twin only runs on hardware that passes its checks")
    missing = sorted(k for k in REQUIRED.get(block, ()) if k not in doc)
    if missing:
        raise HardwareModelError(f"{path} lacks {', '.join(missing)}: it predates this twin; run `make hw`")
    return doc


def _read_s21(path: Path) -> tuple[np.ndarray, np.ndarray]:
    """Frequencies and complex S21 from a 2-port Touchstone file (RI or MA/DB, Hz/kHz/MHz/GHz)."""
    if not path.exists():
        raise HardwareModelError(f"{path} is missing: run `make hw`")
    scale, fmt, f, s21 = 1.0, "MA", [], []
    units = {"HZ": 1.0, "KHZ": 1e3, "MHZ": 1e6, "GHZ": 1e9}
    for line in path.read_text().splitlines():
        line = line.split("!")[0].strip()
        if not line:
            continue
        if line.startswith("#"):
            tok = line[1:].upper().split()
            scale = next((units[t] for t in tok if t in units), 1.0)
            fmt = next((t for t in tok if t in ("RI", "MA", "DB")), "MA")
            continue
        v = [float(x) for x in line.split()]
        a, b = v[3], v[4]
        if fmt == "RI":
            s = complex(a, b)
        else:
            mag = 10 ** (a / 20) if fmt == "DB" else a
            s = mag * np.exp(1j * math.radians(b))
        f.append(v[0] * scale)
        s21.append(s)
    return np.array(f), np.array(s21)


def _interp_complex(x, xp, fp) -> np.ndarray:
    """Interpolate magnitude (dB) and unwrapped phase separately; clamps outside xp."""
    mag = np.interp(x, xp, 20 * np.log10(np.maximum(np.abs(fp), 1e-30)))
    ph = np.interp(x, xp, np.unwrap(np.angle(fp)))
    return 10 ** (mag / 20) * np.exp(1j * ph)


@dataclass(frozen=True)
class Imbalance:
    gain_db: float      # I over Q
    phase_deg: float    # departure of Q from quadrature (mixer.json convention)


@dataclass
class Hardware:
    bank: dict
    s21: dict
    preamp: dict
    attenuator: dict
    mixer: dict

    lo_phase_noise: dict

    @classmethod
    def load(cls, char_dir: Path = DEFAULT_DIR) -> "Hardware":
        d = Path(char_dir)
        bank = _read(d / "bpf_bank.json", "bpf_bank")
        s21 = {f["index"]: _read_s21(d / f"bpf_{f['index']}.s2p") for f in bank["filters"]}
        return cls(bank, s21, _read(d / "preamp.json", "preamp"), _read(d / "attenuator.json", "attenuator"),
                   _read(d / "mixer.json", "mixer"), _read(d / "lo_phase_noise.json", "lo_phase_noise"))

    # --- front end -------------------------------------------------------------------------
    def filter_index(self, f_lo: float) -> int:
        """bpf_bank.json selection rule: filter n with lo_n <= f < hi_n; the top edge -> last."""
        filters = sorted(self.bank["filters"], key=lambda x: x["index"])
        for flt in filters:
            if flt["lo_hz"] <= f_lo < flt["hi_hz"]:
                return flt["index"]
        if f_lo == filters[-1]["hi_hz"]:
            return filters[-1]["index"]
        raise HardwareModelError(f"LO {f_lo / 1e6:.6f} MHz is outside the filter bank "
                                 f"({filters[0]['lo_hz'] / 1e6:g}–{filters[-1]['hi_hz'] / 1e6:g} MHz)")

    def s2p_range(self, band: int) -> tuple[float, float]:
        f = self.s21[band][0]
        return float(f[0]), float(f[-1])

    def front_gain(self, f_rf, f_lo: float, preamp: bool, attenuator_db: int) -> np.ndarray:
        """Complex transducer gain from the antenna to the mixer input at RF frequencies f_rf,
        with the filter selected for f_lo."""
        g_in, g_pre = self.front_parts(f_rf, f_lo, preamp, attenuator_db)
        return g_in * g_pre

    def front_parts(self, f_rf, f_lo: float, preamp: bool, attenuator_db: int):
        """(attenuator × filter, complex; preamp or bypass amplitude gain): the front end split at
        the preamp input, where its IMD is referred."""
        f_rf = np.asarray(f_rf, dtype=float)
        a = self.attenuator
        att_db = np.interp(f_rf, a["freq_hz"], a["pad_db"] if attenuator_db else a["bypass_db"])
        fs, s21 = self.s21[self.filter_index(f_lo)]
        flt = _interp_complex(f_rf, fs, s21)
        p = self.preamp
        pre_db = np.interp(f_rf, p["freq_hz"], p["gain_db"]) if preamp else -p["bypass_loss_db"] + 0 * f_rf
        return 10 ** (att_db / 20) * flt, 10 ** (pre_db / 20)

    def phase_noise_dbc_hz(self, offset_hz, f_lo: float):
        """LO single-sideband phase noise L(offset) at f_lo: the measured curve (lo_phase_noise.json,
        at reference_hz) interpolated over log offset, held flat outside its range, scaled by
        20·log10(f_lo / reference_hz)."""
        c = self.lo_phase_noise
        o = np.log10(np.maximum(np.abs(np.asarray(offset_hz, dtype=float)), 1e-3))
        db = np.interp(o, np.log10(c["offset_hz"]), c["dbc_per_hz"])
        return db + 20 * math.log10(f_lo / c["reference_hz"])

    # --- mixer -----------------------------------------------------------------------------
    def conv_gain(self, f_lo: float) -> float:
        """Conversion gain (linear, ADC volts per matched-source volt) at f_lo, interpolated
        over log f between the test points."""
        pts = sorted(self.mixer["test_points"], key=lambda p: p["f_lo_hz"])
        g_db = np.interp(math.log(f_lo), [math.log(p["f_lo_hz"]) for p in pts], [p["conv_gain_db"] for p in pts])
        return float(10 ** (g_db / 20))

    def baseband_response(self, f_bb) -> np.ndarray:
        """H(f): complex baseband response normalised to the conversion gain (clamped beyond ±60 kHz)."""
        r = self.mixer["baseband_response"]
        o = np.array(r["offset_hz"])
        h = np.array(r["re"]) + 1j * np.array(r["im"])
        order = np.argsort(o)
        o, h = o[order], h[order]
        return np.interp(f_bb, o, h.real) + 1j * np.interp(f_bb, o, h.imag)

    def bypass_correction(self, f_lo: float, f_bb) -> np.ndarray:
        """Preamp bypassed: the filter → mixer cascade's response over the product of the
        filter's S21 and the mixer's 50 Ω-source response, for the selected filter (measured at
        its band centre, applied across the band)."""
        c = self.mixer["bypass_correction"][str(self.filter_index(f_lo))]
        o = np.array(c["offset_hz"])
        return np.interp(f_bb, o, c["re"]) + 1j * np.interp(f_bb, o, c["im"])

    def harmonics(self, f_lo: float) -> dict[int, tuple[complex, str]]:
        """LO harmonic k -> (complex response relative to the fundamental, sideband) at f_lo:
        magnitude (dB) and phase interpolated over log f between the mixer's test points (the
        response changes across the bank: band 1's 3rd harmonic is −5 dB, not the nominal −9.5);
        the sideband is that of the nearest test point."""
        pts = sorted(self.mixer["test_points"], key=lambda p: p["f_lo_hz"])
        x = [math.log(p["f_lo_hz"]) for p in pts]
        out = {}
        for k in self.mixer["harmonic_responses"]:
            if int(k) == 1:
                continue
            db = np.interp(math.log(f_lo), x, [p["harmonics"][k]["db"] for p in pts])
            ph = np.interp(math.log(f_lo), x, np.unwrap([math.radians(p["harmonics"][k]["phase_deg"]) for p in pts]))
            near = min(pts, key=lambda p: abs(math.log(p["f_lo_hz"]) - math.log(f_lo)))
            out[int(k)] = (10 ** (db / 20) * np.exp(1j * ph), near["harmonics"][k]["sideband"])
        return out

    def imbalance(self, f_lo: float) -> Imbalance:
        """The 5th-percentile Monte Carlo run's I/Q imbalance for the LO path in use."""
        ir = self.mixer["image_rejection_db"]["direct" if f_lo >= self.mixer["direct_min_hz"] else "div4"]
        return Imbalance(ir["p5_gain_err_db"], ir["p5_phase_err_deg"])

    # --- noise -----------------------------------------------------------------------------
    def noise_budget(self, f_lo: float, preamp: bool, attenuator_db: int) -> dict:
        """Cascaded noise factor at f_lo (Friis): attenuator and filter as passive losses at T0,
        preamp (or its bypass loss), mixer (mixer.json nf_db). Plus the preamp's output noise
        at the LO harmonics, folded into the window by the mixer's harmonic responses, referred
        to the antenna (W/Hz)."""
        stages = []
        a = self.attenuator
        att_db = float(np.interp(f_lo, a["freq_hz"], a["pad_db"] if attenuator_db else a["bypass_db"]))
        stages.append(("attenuator" if attenuator_db else "attenuator bypass", 10 ** (-att_db / 10), 10 ** (att_db / 10)))
        fs, s21 = self.s21[self.filter_index(f_lo)]
        g_flt = float(abs(_interp_complex(f_lo, fs, s21)) ** 2)
        stages.append((f"filter {self.filter_index(f_lo)}", 1 / g_flt, g_flt))
        p = self.preamp
        if preamp:
            g_pre = 10 ** (float(np.interp(f_lo, p["freq_hz"], p["gain_db"])) / 10)
            stages.append(("preamp", 10 ** (float(np.interp(f_lo, p["freq_hz"], p["nf_db"])) / 10), g_pre))
        else:
            loss = 10 ** (p["bypass_loss_db"] / 10)
            stages.append(("preamp bypass", loss, 1 / loss))
            # the interface loss reduces signal and source noise alike, not the mixer's own noise
            g_if = 10 ** (self.mixer["bypass_correction"][str(self.filter_index(f_lo))]["mean_db"] / 10)
            stages.append(("filter → mixer interface", 1.0, g_if))
        # nf_db is referred at the nominal conversion gain (mixer.json conv_gain_db); the mixer's
        # added noise is fixed at the ADC, so its input-referred excess scales with 1/G(f_lo)²
        g_ratio = (10 ** (self.mixer["conv_gain_db"] / 20) / self.conv_gain(f_lo)) ** 2
        stages.append(("mixer", 1 + (10 ** (self.mixer["nf_db"] / 10) - 1) * g_ratio, 1.0))
        f_total, g = 1.0, 1.0
        rows = []
        for name, f, gain in stages:
            contrib = (f - 1) / g
            f_total += contrib
            rows.append({"stage": name, "nf_db": 10 * math.log10(f), "gain_db": 10 * math.log10(gain),
                         "added_factor": contrib})
            g *= gain
        # the CS5361's own noise (added at the ADC by receive(), not part of the antenna-referred
        # receiver noise): per channel N, so like the mixer's F − 1 = 2·N / (4·R·G²·k·T0)
        adc = 2 * self.adc_noise_v2_per_hz / (4 * R0 * self.conv_gain(f_lo) ** 2 * K_B * T0) / g
        folded = 0.0
        if preamp:
            front = float(abs(self.front_gain(f_lo, f_lo, True, attenuator_db)) ** 2)
            for k, (c, _) in self.harmonics(f_lo).items():
                fk = k * f_lo
                if fk > p["freq_hz"][-1]:
                    continue    # beyond the characterised preamp range: not modelled (reported)
                g_k = 10 ** (float(np.interp(fk, p["freq_hz"], p["gain_db"])) / 10)
                f_k = 10 ** (float(np.interp(fk, p["freq_hz"], p["nf_db"])) / 10)
                folded += K_B * T0 * g_k * f_k * abs(c) ** 2 / front
        rows.append({"stage": "ADC (CS5361)", "nf_db": float("nan"), "gain_db": 0.0, "added_factor": adc})
        f_all = f_total + folded / (K_B * T0) + adc
        return {"stages": rows, "f_cascade": f_total, "nf_db": 10 * math.log10(f_total),
                "harmonic_noise_w_per_hz": folded,
                "receiver_noise_w_per_hz": K_B * T0 * (f_total - 1) + folded,
                "adc_added_factor": adc, "nf_total_db": 10 * math.log10(f_all)}

    # --- ADC -------------------------------------------------------------------------------
    @property
    def adc_fs_peak_v(self) -> float:
        return self.mixer["adc_fs_peak_v"]

    @property
    def clip_peak_v(self) -> float:
        return self.mixer["clip_peak_v"]

    @property
    def adc_noise_v2_per_hz(self) -> float:
        """One-sided noise density per channel at the ADC input (V²/Hz), from its dynamic range."""
        n = self.mixer["adc_noise"]
        fs_rms = self.adc_fs_peak_v / math.sqrt(2)
        return fs_rms ** 2 * 10 ** (-n["dynamic_range_db"] / 10) / n["bandwidth_hz"]


def apply_imbalance(z: np.ndarray, imb: Imbalance) -> np.ndarray:
    """z = I − jQ with Q's phasor rotated by phase_deg and scaled by 1/gain relative to I, the
    convention neohw's iq_imbalance measures: q = −Im(z·e^{jφ}) / g."""
    g = 10 ** (imb.gain_db / 20)
    i = z.real
    q = -(z * np.exp(1j * math.radians(imb.phase_deg))).imag / g
    return i - 1j * q


def image_rejection_db(imb: Imbalance) -> float:
    """Image rejection of apply_imbalance for a tone (|wanted| / |image| in dB)."""
    g = 10 ** (imb.gain_db / 20)
    e = np.exp(1j * math.radians(imb.phase_deg))
    # z_out = i − j·q = (z + conj z)/2 + (z·e − conj(z·e))/(2g)
    mu = 0.5 + e / (2 * g)
    nu = 0.5 - np.conj(e) / (2 * g)
    return math.inf if abs(nu) < 1e-15 else 20 * math.log10(abs(mu) / abs(nu))


@dataclass(frozen=True)
class HardwareStats:
    filter_index: int
    noise: dict
    conv_gain_db: float
    image_rejection_db: float
    folded: tuple[str, ...]          # station ids received through an LO harmonic
    clipped_samples: int
    peak_dbfs: float
    imd3: tuple = ()                 # (id_i, id_j, product Hz, antenna-referred dBm) for 2·f_i − f_j in the window
    overload: tuple[str, ...] = ()   # stations above the full-scale level at the mixer input
    reciprocal_dbm_per_hz: float = float("-inf")   # mean reciprocal-mixing density in the window, antenna-referred
    imbalance: Imbalance | None = None


def _amp(dbm) -> float:
    return math.sqrt(2 * R0 * 1e-3 * 10 ** (dbm / 10))


def _imd3(scenario, hw: "Hardware", usable, lo: float, n: int, fs: int, window_hz: float, envelope):
    """Third-order products 2·f_i − f_j of every ordered station pair that land in the window,
    from the mixer IIP3 (at its input) and, with the preamp on, the preamp IIP3 (OIP3 − gain, at
    its input). Each product is c·u_i²·conj(u_j) at f_im; the two stages' products add with the
    phases of the front-end gains (in phase for a flat front end: the worst case). Returned
    antenna-referred, ready to add to the in-window envelope."""
    rx = scenario.receiver
    a_mix = _amp(hw.mixer["iip3_dbm"])
    t = np.arange(n) / fs
    x = np.zeros(n, dtype=complex)
    found = []
    for si in usable:
        for sj in usable:
            if si is sj:
                continue
            f_im = 2 * si.rf_hz - sj.rf_hz
            if abs(f_im - lo) + (2 * si.bandwidth_hz + sj.bandwidth_hz) / 2 > window_hz:
                continue
            ai, aj = _amp(si.level_dbm), _amp(sj.level_dbm)
            gi_in, gi_pre = hw.front_parts(si.rf_hz, lo, rx.preamp, rx.attenuator_db)
            gj_in, gj_pre = hw.front_parts(sj.rf_hz, lo, rx.preamp, rx.attenuator_db)
            fi, fj = gi_in * gi_pre, gj_in * gj_pre
            c = fi ** 2 * np.conj(fj) * ai ** 2 * aj / a_mix ** 2
            if rx.preamp:
                g_im = float(np.interp(f_im, hw.preamp["freq_hz"], hw.preamp["gain_db"]))
                a_pre = _amp(hw.preamp["oip3_dbm"] - g_im)
                c = c + gi_in ** 2 * np.conj(gj_in) * ai ** 2 * aj / a_pre ** 2 * 10 ** (g_im / 20)
            c_ant = complex(c / hw.front_gain(f_im, lo, rx.preamp, rx.attenuator_db))
            u = envelope(si) ** 2 * np.conj(envelope(sj))
            x += c_ant * u * np.exp(2j * np.pi * (f_im - lo) * t)
            found.append((si.id, sj.id, f_im, 10 * math.log10(abs(c_ant) ** 2 / (2 * R0) / 1e-3)))
    return x, tuple(found)


def _complex_noise(n: int, density: float, fs: float, rng: np.random.Generator) -> np.ndarray:
    """Complex white noise whose envelope density is `density` (V²/Hz) at every frequency."""
    return (rng.normal(size=n) + 1j * rng.normal(size=n)) * math.sqrt(density * fs / 2)


def receive(scenario, rng: np.random.Generator, hw: Hardware, render, fs: int, window_hz: float):
    """Compose the scenario through the hardware model. Returns (digital I/Q in [-1, 1),
    exclusions as (station id, reason), HardwareStats)."""
    rx = scenario.receiver
    lo = scenario.lo_hz
    n = int(round(scenario.duration_s * fs))
    band = hw.filter_index(lo)
    f_bins = np.fft.fftfreq(n, 1 / fs)
    budget = hw.noise_budget(lo, rx.preamp, rx.attenuator_db)
    n_ant = 10 ** (scenario.noise_dbm_per_hz / 10) * 1e-3
    # antenna-referred envelope: band noise + receiver noise, then the front end per bin
    x = _complex_noise(n, 2 * R0 * (n_ant + budget["receiver_noise_w_per_hz"]), fs, rng)
    excluded: list[tuple[str, str]] = []
    folded = []
    # stations the front-end data covers (a harmonic station beyond the filter data is excluded below)
    f_lo_s2p, f_hi_s2p = hw.s2p_range(band)
    usable = [st for st in scenario.stations if f_lo_s2p <= st.rf_hz <= f_hi_s2p]
    envelopes: dict[str, np.ndarray] = {}

    def envelope(st):   # unit-power envelope about the station's own frequency
        if st.id not in envelopes:
            envelopes[st.id] = render(st, st.rf_hz, n)
        return envelopes[st.id]

    # reciprocal mixing: each station's power at the mixer × L(offset), offset = f_s − LO − f_bb
    rm = np.zeros(n)
    overload = []
    for st in usable:
        p_mix = 1e-3 * 10 ** (st.level_dbm / 10) * float(abs(hw.front_gain(st.rf_hz, lo, rx.preamp, rx.attenuator_db)) ** 2)
        rm += p_mix * 10 ** (hw.phase_noise_dbc_hz(st.rf_hz - lo - f_bins, lo) / 10)
        if 10 * math.log10(p_mix / 1e-3) > hw.mixer["adc_full_scale_dbm"]:
            overload.append(st.id)
    rm_ant = rm / np.abs(hw.front_gain(lo + f_bins, lo, rx.preamp, rx.attenuator_db)) ** 2
    w = rng.normal(size=n) + 1j * rng.normal(size=n)
    x += np.fft.ifft(np.fft.fft(w) * np.sqrt(2 * R0 * rm_ant * fs / 2))
    x_imd, imd3 = _imd3(scenario, hw, usable, lo, n, fs, window_hz, envelope)
    x += x_imd
    spec_h: dict[tuple[int, str], np.ndarray] = {}
    harmonics = hw.harmonics(lo)
    for st in scenario.stations:
        amp = math.sqrt(2 * R0 * 1e-3 * 10 ** (st.level_dbm / 10))
        if st.reaches(lo, window_hz):
            x += render(st, lo, n) * amp
            continue
        for k, (_, side) in harmonics.items():
            if st.reaches(k * lo, window_hz):
                if not f_lo_s2p <= st.rf_hz <= f_hi_s2p:
                    excluded.append((st.id, f"{st.rf_hz / 1e6:.6f} MHz near {k}·LO is beyond the filter "
                                            f"data ({f_hi_s2p / 1e6:g} MHz)"))
                    break
                s = render(st, k * lo, n) * amp
                spec_h[(k, side)] = spec_h.get((k, side), 0) + np.fft.fft(s if side == "upper" else np.conj(s))
                folded.append(st.id)
                break
        else:
            excluded.append((st.id, f"{st.rf_hz / 1e6:.6f} MHz is outside the receiver window "
                                    f"{lo / 1e6:.6f} MHz ±{window_hz / 1e3:.1f} kHz and its LO harmonics: "
                                    "it contributes only reciprocal mixing and IMD3"))
    y = np.fft.fft(x) * hw.front_gain(lo + f_bins, lo, rx.preamp, rx.attenuator_db)
    if not rx.preamp:
        y *= hw.bypass_correction(lo, f_bins)
    for (k, side), spec in spec_h.items():
        c, _ = harmonics[k]
        f_rf = k * lo + (f_bins if side == "upper" else -f_bins)
        y += spec * hw.front_gain(f_rf, lo, rx.preamp, rx.attenuator_db) * c
    g = hw.conv_gain(lo)
    z = np.fft.ifft(y * g * hw.baseband_response(f_bins))
    imb = hw.imbalance(lo)
    z = apply_imbalance(z, imb)
    # DC offset (op-amp input offsets, typical, random sign), the op-amp swing, then the ADC:
    # its own noise, full scale and 24-bit quantisation
    dc = hw.mixer["dc_offset_v"]["typical"] * rng.choice([-1.0, 1.0], size=2)
    i_v, q_v = z.real + dc[0], -z.imag + dc[1]
    clip = hw.clip_peak_v
    clipped = int(np.count_nonzero((np.abs(i_v) > clip) | (np.abs(q_v) > clip)))
    i = np.clip(i_v, -clip, clip)
    q = np.clip(q_v, -clip, clip)
    sigma = math.sqrt(hw.adc_noise_v2_per_hz * fs / 2)
    fsv = hw.adc_fs_peak_v
    lsb = 2.0 ** -23
    di = np.clip(np.round((i + rng.normal(scale=sigma, size=n)) / fsv / lsb) * lsb, -1.0, 1.0 - lsb)
    dq = np.clip(np.round((q + rng.normal(scale=sigma, size=n)) / fsv / lsb) * lsb, -1.0, 1.0 - lsb)
    peak = float(max(np.max(np.abs(di)), np.max(np.abs(dq)), 1e-30))
    band_rm = np.abs(f_bins) <= window_hz
    rm_db = 10 * math.log10(max(float(np.mean(rm_ant[band_rm])), 1e-300) / 1e-3)
    stats = HardwareStats(band, budget, 20 * math.log10(g), image_rejection_db(imb), tuple(folded), clipped,
                          20 * math.log10(peak), imd3, tuple(overload), rm_db, imb)
    return di - 1j * dq, excluded, stats
