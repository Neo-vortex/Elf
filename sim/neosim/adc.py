"""Ideal ADC: gain, quantisation and saturation; no other impairments."""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class AdcStats:
    clipped_samples: int
    peak_dbfs: float
    scale: float


def ideal_adc(iq: np.ndarray, noise_dbfs: float, noise_power: float, bits: int = 24) -> tuple[np.ndarray, AdcStats]:
    scale = float(np.sqrt(10 ** (noise_dbfs / 10) / noise_power))
    x = iq * scale
    lsb = 2.0 ** -(bits - 1)
    hi = 1.0 - lsb
    clipped = (x.real > hi) | (x.real < -1.0) | (x.imag > hi) | (x.imag < -1.0)
    re = np.clip(np.round(x.real / lsb) * lsb, -1.0, hi)
    im = np.clip(np.round(x.imag / lsb) * lsb, -1.0, hi)
    y = re + 1j * im
    peak = float(max(np.max(np.abs(re)), np.max(np.abs(im)), 1e-30))
    return y, AdcStats(int(np.count_nonzero(clipped)), 20 * np.log10(peak), scale)
