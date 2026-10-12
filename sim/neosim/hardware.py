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
DIRECT_MIN_HZ = 3.2e6        # below this the LO comes from the ÷4 counter (mixer.json "div4")
DEFAULT_DIR = Path(__file__).resolve().parents[2] / "hardware" / "characterized"


class HardwareModelError(RuntimeError):
    """The characterisation files are missing, from another schema, or failed their checks."""


# fields the twin reads beyond the plan 2.4 contract (spec §6, plan 2.5)
REQUIRED = {"mixer": ("nf_db", "adc_fs_peak_v", "clip_peak_v", "adc_noise", "bypass_correction"),
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

    @classmethod
    def load(cls, char_dir: Path = DEFAULT_DIR) -> "Hardware":
        d = Path(char_dir)
        bank = _read(d / "bpf_bank.json", "bpf_bank")
        s21 = {f["index"]: _read_s21(d / f"bpf_{f['index']}.s2p") for f in bank["filters"]}
        return cls(bank, s21, _read(d / "preamp.json", "preamp"), _read(d / "attenuator.json", "attenuator"),
                   _read(d / "mixer.json", "mixer"))

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
        f_rf = np.asarray(f_rf, dtype=float)
        a = self.attenuator
        att_db = np.interp(f_rf, a["freq_hz"], a["pad_db"] if attenuator_db else a["bypass_db"])
        fs, s21 = self.s21[self.filter_index(f_lo)]
        flt = _interp_complex(f_rf, fs, s21)
        p = self.preamp
        pre_db = np.interp(f_rf, p["freq_hz"], p["gain_db"]) if preamp else -p["bypass_loss_db"]
        return 10 ** ((att_db + pre_db) / 20) * flt

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

    def harmonics(self) -> dict[int, tuple[complex, str]]:
        """LO harmonic k -> (complex response relative to the fundamental, sideband)."""
        out = {}
        for k, v in self.mixer["harmonic_responses"].items():
            if int(k) == 1:
                continue
            out[int(k)] = (10 ** (v["db"] / 20) * np.exp(1j * math.radians(v["phase_deg"])), v["sideband"])
        return out

    def imbalance(self, f_lo: float) -> Imbalance:
        """The 5th-percentile Monte Carlo run's I/Q imbalance for the LO path in use."""
        ir = self.mixer["image_rejection_db"]["direct" if f_lo >= DIRECT_MIN_HZ else "div4"]
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
        stages.append(("mixer", 10 ** (self.mixer["nf_db"] / 10), 1.0))
        f_total, g = 1.0, 1.0
        rows = []
        for name, f, gain in stages:
            contrib = (f - 1) / g
            f_total += contrib
            rows.append({"stage": name, "nf_db": 10 * math.log10(f), "gain_db": 10 * math.log10(gain),
                         "added_factor": contrib})
            g *= gain
        folded = 0.0
        if preamp:
            front = float(abs(self.front_gain(f_lo, f_lo, True, attenuator_db)) ** 2)
            for k, (c, _) in self.harmonics().items():
                fk = k * f_lo
                if fk > p["freq_hz"][-1]:
                    continue    # beyond the characterised preamp range: not modelled (reported)
                g_k = 10 ** (float(np.interp(fk, p["freq_hz"], p["gain_db"])) / 10)
                f_k = 10 ** (float(np.interp(fk, p["freq_hz"], p["nf_db"])) / 10)
                folded += K_B * T0 * g_k * f_k * abs(c) ** 2 / front
        return {"stages": rows, "f_cascade": f_total, "nf_db": 10 * math.log10(f_total),
                "harmonic_noise_w_per_hz": folded,
                "receiver_noise_w_per_hz": K_B * T0 * (f_total - 1) + folded}

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
    spec_h: dict[tuple[int, str], np.ndarray] = {}
    for st in scenario.stations:
        amp = math.sqrt(2 * R0 * 1e-3 * 10 ** (st.level_dbm / 10))
        if abs(st.rf_hz - lo) + st.bandwidth_hz / 2 <= window_hz:
            x += render(st, lo, n) * amp
            continue
        for k, (_, side) in hw.harmonics().items():
            if abs(st.rf_hz - k * lo) + st.bandwidth_hz / 2 <= window_hz:
                f_lo_s2p, f_hi_s2p = hw.s2p_range(band)
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
                                    f"{lo / 1e6:.6f} MHz ±{window_hz / 1e3:.1f} kHz and its LO harmonics "
                                    "(out-of-window effects: plan 2.6)"))
    y = np.fft.fft(x) * hw.front_gain(lo + f_bins, lo, rx.preamp, rx.attenuator_db)
    if not rx.preamp:
        y *= hw.bypass_correction(lo, f_bins)
    harmonics = hw.harmonics()
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
    stats = HardwareStats(band, budget, 20 * math.log10(g), image_rejection_db(imb), tuple(folded), clipped,
                          20 * math.log10(peak))
    return di - 1j * dq, excluded, stats
