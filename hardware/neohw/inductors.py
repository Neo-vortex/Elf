"""Filter inductors from guaranteed datasheet data (hardware/neohw/data/inductors.json).

Rev A spec §2, user decision 2026-10-19: every filter inductor, in both populations, is a real
series with published, guaranteed Q (China: TDK NL453232 1812 from 1 µH, TDK NLV32 1210 below;
Iran: Bourns 78F axial), and both are modelled the same way, anchored only on guaranteed numbers:

- at and above the datasheet test frequency f_t the series resistance grows with f, so Q stays
  at its guaranteed minimum (a real part's Q rises above f_t; holding it is conservative);
- below f_t the resistance falls linearly from R(f_t) = 2π·f_t·L / Q_min towards the DC
  resistance maximum, never below it;
- the parallel capacitance gives the minimum self-resonance.

A filter evaluates R at one frequency per filter (the band centre, or the upper edge for the
worst case: `blocks.bpf.add_filter`), so the SPICE model is L + constant R + parallel C.
"""
from __future__ import annotations

import json
import math
from dataclasses import dataclass
from functools import lru_cache
from pathlib import Path

DATA = Path(__file__).resolve().parent / "data" / "inductors.json"
CHINA_SMALL, CHINA_LARGE, IRAN = "tdk_nlv32", "tdk_nl453232", "bourns_78f"
CHINA_LARGE_FROM_H = 1e-6          # NL453232 starts at 1 µH; below it NLV32 (CM453232 sub-µH is ±20 %)
CURRENT_SPEC = "I≥20 mA"           # filters carry microamps (user, 2026-10-19)


@dataclass(frozen=True)
class Row:
    series: str
    mpn: str
    henries: float
    q_min: float
    f_test_hz: float
    srf_min_hz: float
    dcr_max: float


@lru_cache(maxsize=None)
def _series() -> dict:
    return json.loads(DATA.read_text(encoding="utf-8"))["series"]


def series_info(series: str) -> dict:
    return _series()[series]


def row(series: str, henries: float) -> Row:
    for r in _series()[series]["rows"]:
        if math.isclose(r["uh"] * 1e-6, henries, rel_tol=1e-6):
            return Row(series, r["mpn"], r["uh"] * 1e-6, r["q_min"], r["f_test_mhz"] * 1e6,
                       r["srf_min_mhz"] * 1e6, r["dcr_max_ohm"])
    raise ValueError(f"{henries * 1e6:g} µH is not in {series} ({DATA.name})")


def china_series(henries: float) -> str:
    return CHINA_LARGE if henries >= CHINA_LARGE_FROM_H * (1 - 1e-9) else CHINA_SMALL


def r_series(r: Row, f_hz: float) -> float:
    """Series loss resistance at `f_hz` (see the module docstring)."""
    r_t = 2 * math.pi * r.f_test_hz * r.henries / r.q_min
    if f_hz >= r.f_test_hz:
        return r_t * f_hz / r.f_test_hz
    return max(r.dcr_max, r.dcr_max + (r_t - r.dcr_max) * f_hz / r.f_test_hz)


def q_at(r: Row, f_hz: float) -> float:
    return 2 * math.pi * f_hz * r.henries / r_series(r, f_hz)


def c_par(r: Row) -> float:
    return 1 / ((2 * math.pi * r.srf_min_hz) ** 2 * r.henries)
