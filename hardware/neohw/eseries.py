"""E-series preferred values and one- or two-part combinations."""
from __future__ import annotations

import math

E12 = (1.0, 1.2, 1.5, 1.8, 2.2, 2.7, 3.3, 3.9, 4.7, 5.6, 6.8, 8.2)
E24 = (1.0, 1.1, 1.2, 1.3, 1.5, 1.6, 1.8, 2.0, 2.2, 2.4, 2.7, 3.0,
       3.3, 3.6, 3.9, 4.3, 4.7, 5.1, 5.6, 6.2, 6.8, 7.5, 8.2, 9.1)


def values(series: tuple[float, ...], lo: float = 1e-13, hi: float = 1e-2) -> list[float]:
    """Every value of `series` from `lo` to `hi`, built from decimal text so 4.7e-06 is exact."""
    out = []
    for d in range(math.floor(math.log10(lo)), math.ceil(math.log10(hi)) + 1):
        out += [float(f"{s}e{d}") for s in series]
    return sorted(v for v in out if lo <= v <= hi)


def snap(value: float, series: tuple[float, ...]) -> float:
    """Nearest preferred value on a log scale."""
    if value <= 0:
        raise ValueError(f"value must be positive, got {value!r}")
    d = math.floor(math.log10(value))
    candidates = [float(f"{s}e{dd}") for dd in (d - 1, d, d + 1) for s in series]
    return min(candidates, key=lambda c: abs(math.log(c / value)))


def combo(value: float, series: tuple[float, ...], tolerance: float = 0.01,
          minimum: float = 0.0) -> tuple[float, ...]:
    """One or two preferred values whose sum is closest to `value`.

    Inductors in series and capacitors in parallel both add, so one rule serves both.
    A single part is used when it lands within `tolerance` (fractional). No part is
    smaller than `minimum` (the smallest value that can be bought).
    """
    single = max(snap(value, series), snap(minimum, series) if minimum else 0.0)
    best_err, best = abs(math.log(single / value)), (single,)
    if best_err <= tolerance:
        return best
    for a in values(series, max(value / 1000, minimum or 1e-13), value):
        if a >= value:
            break
        b = snap(value - a, series)
        if b < minimum:
            continue
        err = abs(math.log((a + b) / value))
        if err < best_err - 1e-12:
            best_err, best = err, tuple(sorted((a, b), reverse=True))
    return best
