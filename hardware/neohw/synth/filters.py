"""Chebyshev ladder synthesis (Matthaei, Young & Jones, 1964, §4.05)."""
from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class Element:
    """One ladder position, from the source side. kind: C_shunt, L_series, C_series or L_shunt."""
    kind: str
    value: float


def chebyshev_g(order: int, ripple_db: float) -> list[float]:
    """Lowpass prototype values g1..gn for equal 1 Ω terminations (odd orders only)."""
    if order < 1 or order % 2 == 0:
        raise ValueError(f"order must be odd and positive, got {order}")
    if ripple_db <= 0:
        raise ValueError(f"ripple must be positive, got {ripple_db}")
    beta = math.log(1 / math.tanh(ripple_db / 17.37))
    gamma = math.sinh(beta / (2 * order))
    a = [math.sin((2 * k - 1) * math.pi / (2 * order)) for k in range(1, order + 1)]
    b = [gamma ** 2 + math.sin(k * math.pi / order) ** 2 for k in range(1, order + 1)]
    g = [2 * a[0] / gamma]
    for k in range(1, order):
        g.append(4 * a[k - 1] * a[k] / (b[k - 1] * g[-1]))
    return g


def lowpass(fc_hz: float, order: int, ripple_db: float, r0: float = 50.0) -> list[Element]:
    """Shunt-C-first lowpass: (order + 1) / 2 capacitors, (order - 1) / 2 inductors."""
    w = 2 * math.pi * fc_hz
    return [Element("C_shunt", g / (r0 * w)) if k % 2 == 0 else Element("L_series", g * r0 / w)
            for k, g in enumerate(chebyshev_g(order, ripple_db))]


def highpass(fc_hz: float, order: int, ripple_db: float, r0: float = 50.0) -> list[Element]:
    """Series-C-first highpass (lowpass-to-highpass transform of `lowpass`)."""
    w = 2 * math.pi * fc_hz
    return [Element("C_series", 1 / (g * r0 * w)) if k % 2 == 0 else Element("L_shunt", r0 / (g * w))
            for k, g in enumerate(chebyshev_g(order, ripple_db))]
