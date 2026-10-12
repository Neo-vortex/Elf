"""RF figures from simulation vectors: noise figure, two-tone IMD, stability, shunt impedance."""
from __future__ import annotations

import math

import numpy as np

K_B = 1.380649e-23
T_SIM = 300.15   # ngspice default temperature (27 °C)


T0 = 290.0       # IEEE reference temperature for noise figure


def noise_figure_db(inoise_v_per_rthz: np.ndarray, z0: float = 50.0, t_kelvin: float = T_SIM) -> np.ndarray:
    """IEEE noise figure from ngspice's input-referred noise density (referred to the source
    voltage). The simulated source resistor is at `t_kelvin` (the circuit temperature); its own
    noise is removed and the excess is referred to a source at 290 K."""
    e2 = np.asarray(inoise_v_per_rthz) ** 2
    excess = e2 - 4 * K_B * t_kelvin * z0
    return 10 * np.log10(1 + excess / (4 * K_B * T0 * z0))


def tone_powers_dbm(v: np.ndarray, step_s: float, freqs_hz, r: float = 50.0) -> list[float]:
    """Power in r of each frequency in `freqs_hz` from a uniformly sampled voltage record.
    The record must hold a whole number of periods of every frequency (rectangular window)."""
    n = len(v)
    spec = np.fft.rfft(np.asarray(v)) / n * 2
    bins = np.fft.rfftfreq(n, step_s)
    out = []
    for f in freqs_hz:
        k = int(np.argmin(np.abs(bins - f)))
        if abs(bins[k] - f) > 1e-6 * f:
            raise ValueError(f"{f} Hz is not on the FFT grid (record must hold whole periods)")
        out.append(10 * math.log10(abs(spec[k]) ** 2 / 2 / r / 1e-3 + 1e-30))
    return out


def two_tone(v: np.ndarray, step_s: float, f1: float, f2: float, r: float = 50.0) -> dict:
    """Fundamental and worst third-order product powers, and the output intercept."""
    p1, p2, l3, u3 = tone_powers_dbm(v, step_s, (f1, f2, 2 * f1 - f2, 2 * f2 - f1), r)
    pout, imd3 = (p1 + p2) / 2, max(l3, u3)
    return {"pout_dbm": pout, "imd3_dbm": imd3, "oip3_dbm": pout + (pout - imd3) / 2}


def stability(s11, s21, s12, s22) -> tuple[np.ndarray, np.ndarray]:
    """Rollett K and |Δ|; unconditionally stable where K > 1 and |Δ| < 1."""
    delta = s11 * s22 - s12 * s21
    k = (1 - np.abs(s11) ** 2 - np.abs(s22) ** 2 + np.abs(delta) ** 2) / (2 * np.abs(s12 * s21))
    return k, np.abs(delta)


def shunt_impedance_at_port1(s11, s21, s12, s22, z0: float = 50.0) -> np.ndarray:
    """|Z| of the shunt branch at port 1 of a two-port seen as a pi network (Y11 + Y12)."""
    d = (1 + s11) * (1 + s22) - s12 * s21
    y11 = ((1 - s11) * (1 + s22) + s12 * s21) / d / z0
    y12 = -2 * s12 / d / z0
    return np.abs(1 / (y11 + y12))
