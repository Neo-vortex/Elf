"""Recorded antenna (spec 2026-10-13-recorded-antenna-design.md): real I/Q recordings placed on the
virtual board's antenna, alongside or instead of imaginary stations.

A recording is a SigMF file (cf32_le or ci16_le; rate and centre frequency from its metadata) or a
stereo WAV of I and Q (16-bit, 24-bit or float), whose centre frequency comes from the antenna file.
Its level at the antenna is set by a reference, never guessed: `full_scale_dbm` (the antenna power
of a full-scale tone in the file) or `reference: {hz, dbm}` (a carrier in the file and its measured
antenna level, e.g. a KiwiSDR S-meter reading).

Each recording becomes a station of mode "recording": resampled to the board's 96 kHz, shifted by
its offset from the firmware's LO, normalised to unit power and scaled to its antenna level, so the
hardware model treats it exactly like an imaginary station (front-end gain, harmonics, mixing,
noise, ADC). A recording shorter than the render loops with a 20 ms raised-cosine cross-fade.
"""
from __future__ import annotations

import json
import math
from dataclasses import dataclass, field
from fractions import Fraction
from pathlib import Path

import numpy as np
import yaml
from scipy.io import wavfile
from scipy.signal import resample_poly

from neosim import hardware, sources
from neosim.scenario import FS_HZ, WINDOW_HZ, Receiver, Scenario, Station, _station

CROSSFADE_S = 0.020
CARRIER_BW_HZ = 40.0          # the window a reference carrier's power is measured in


class AntennaError(ValueError):
    """The antenna file or a recording is invalid."""


@dataclass
class Recording:
    name: str
    rate: float
    center_hz: float
    iq96: np.ndarray = field(repr=False)      # resampled to FS_HZ, unit mean power
    level_dbm: float = 0.0                    # mean antenna power of the whole recording

    @property
    def span_hz(self) -> float:
        return min(self.rate, FS_HZ)

    def baseband(self, offset_hz: float, n: int) -> np.ndarray:
        """n samples at FS_HZ, unit power, centred `offset_hz` from the reference frequency; loops with
        a cross-fade when the recording is shorter than n."""
        x = self.iq96
        if len(x) >= n:
            y = x[:n].copy()
        else:
            k = min(int(CROSSFADE_S * FS_HZ), len(x) // 4)
            ramp = 0.5 - 0.5 * np.cos(np.pi * np.arange(k) / k)
            body = x[k:].copy()                       # each repeat: the faded seam, then the rest
            seam = x[len(x) - k:] * (1 - ramp) + x[:k] * ramp
            unit = np.concatenate([seam, body[: len(body) - k]])
            y = np.concatenate([x[: len(x) - k]] + [unit] * (n // len(unit) + 1))[:n]
        if offset_hz:
            y = y * np.exp(2j * np.pi * offset_hz * np.arange(n) / FS_HZ)
        return y


def _read_sigmf(meta_path: Path) -> tuple[np.ndarray, float, float]:
    meta = json.loads(meta_path.read_text())
    g = meta["global"]
    dtype = {"cf32_le": "<c8", "ci16_le": "<i2"}.get(g["core:datatype"])
    if dtype is None:
        raise AntennaError(f"{meta_path.name}: datatype {g['core:datatype']} is not supported (cf32_le, ci16_le)")
    raw = np.fromfile(meta_path.with_suffix(".sigmf-data"), dtype=dtype)
    iq = raw.astype(np.complex128) if dtype == "<c8" else (raw[0::2] + 1j * raw[1::2]) / 32768.0
    return iq, float(g["core:sample_rate"]), float(meta["captures"][0]["core:frequency"])


def _read_wav(path: Path) -> tuple[np.ndarray, float]:
    rate, d = wavfile.read(path)
    if d.ndim != 2 or d.shape[1] != 2:
        raise AntennaError(f"{path.name}: an I/Q WAV has two channels (I left, Q right)")
    scale = {np.dtype("int16"): 32768.0, np.dtype("int32"): 2.0 ** 31}.get(d.dtype, 1.0)
    return (d[:, 0].astype(np.float64) + 1j * d[:, 1]) / scale, float(rate)


def _to_96k(iq: np.ndarray, rate: float) -> np.ndarray:
    if rate == FS_HZ:
        return iq.astype(np.complex128)
    r = Fraction(FS_HZ / rate).limit_denominator(1000)      # e.g. KiwiSDR's 11 999 Hz → 8/1 (0.008 %)
    return resample_poly(iq, r.numerator, r.denominator)


def _carrier_power(iq: np.ndarray, rate: float, offset_hz: float) -> float:
    t = np.arange(len(iq)) / rate
    z = iq * np.exp(-2j * np.pi * offset_hz * t)
    k = max(1, int(rate / CARRIER_BW_HZ))
    return float(np.mean(np.abs(np.convolve(z, np.ones(k) / k, mode="valid")) ** 2))


def load_recording(spec: dict, base: Path) -> Recording:
    path = Path(spec["file"])
    path = path if path.is_absolute() else Path(base) / path
    if path.suffix == ".sigmf-meta":
        iq, rate, center = _read_sigmf(path)
        center = float(spec.get("center_hz", center))
    elif path.suffix.lower() == ".wav":
        iq, rate = _read_wav(path)
        if "center_hz" not in spec:
            raise AntennaError(f"{path.name}: a WAV recording needs center_hz in the antenna file")
        center = float(spec["center_hz"])
    else:
        raise AntennaError(f"{path.name}: unsupported recording (SigMF .sigmf-meta or I/Q .wav)")
    if "full_scale_dbm" in spec:
        full_scale_dbm = float(spec["full_scale_dbm"])
    elif "reference" in spec:
        ref = spec["reference"]
        p = _carrier_power(iq, rate, float(ref["hz"]) - center)
        if p <= 0:
            raise AntennaError(f"{path.name}: no carrier at the reference frequency {ref['hz']} Hz")
        full_scale_dbm = float(ref["dbm"]) - 10 * math.log10(p)
    else:
        raise AntennaError(f"{path.name}: no level reference (full_scale_dbm or reference: {{hz, dbm}}); "
                           "levels decide overload, so they are never guessed")
    full_scale_dbm += float(spec.get("gain_db", 0.0))
    power = float(np.mean(np.abs(iq) ** 2))
    if power <= 0:
        raise AntennaError(f"{path.name}: the recording is silent")
    iq96 = _to_96k(iq, rate)
    iq96 /= math.sqrt(np.mean(np.abs(iq96) ** 2))
    return Recording(path.name, rate, center, iq96, full_scale_dbm + 10 * math.log10(power))


@dataclass
class Antenna:
    """An antenna file: recordings plus optional imaginary stations, band noise, render length."""
    name: str
    duration_s: float
    seed: int
    noise_dbm_per_hz: float
    recordings: list[Recording]
    stations: tuple[Station, ...]

    @classmethod
    def load(cls, path: Path) -> "Antenna":
        path = Path(path)
        d = yaml.safe_load(path.read_text())
        if not d.get("recordings"):
            raise AntennaError(f"{path.name}: an antenna file lists recordings")
        noise = float(d.get("noise", {}).get("dbm_per_hz", -174.0))
        recs = [load_recording(r, path.parent) for r in d["recordings"]]
        st = [Station(f"rec{k}:{r.name}", "recording", r.center_hz, None, level_dbm=r.level_dbm, span_hz=r.span_hz,
                      source=r) for k, r in enumerate(recs)]
        st += [_station(s, noise) for s in d.get("stations", [])]
        return cls(d.get("name", path.stem), float(d.get("duration_s", 20.0)), int(d.get("seed", 1)), noise, recs, tuple(st))

    def scenario(self, lo_hz: float, preamp: bool, attenuator_db: int) -> Scenario:
        return Scenario(self.name, lo_hz, self.duration_s, self.seed, 0.0, self.stations, (),
                        Receiver("hardware", preamp, attenuator_db), self.noise_dbm_per_hz)

    def render(self, lo_hz: float, hw: hardware.Hardware, preamp: bool, attenuator_db: int) -> np.ndarray:
        """Digital I/Q (di − j·dq, full scale 1) for the LO, through the hardware model."""
        rng = np.random.default_rng(self.seed)
        iq, _excluded, _stats = hardware.receive(self.scenario(lo_hz, preamp, attenuator_db), rng, hw,
                                                 sources.render_station, FS_HZ, WINDOW_HZ)
        return iq
