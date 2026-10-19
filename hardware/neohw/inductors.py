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

import numpy as np

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
    tol: float | None = None      # grade in percent (row `tol_pct`, else the series')


@lru_cache(maxsize=None)
def _series() -> dict:
    return json.loads(DATA.read_text(encoding="utf-8"))["series"]


def series_info(series: str) -> dict:
    return _series()[series]


def row(series: str, henries: float) -> Row:
    for r in _series()[series]["rows"]:
        if math.isclose(r["uh"] * 1e-6, henries, rel_tol=1e-6):
            return Row(series, r["mpn"], r["uh"] * 1e-6, r["q_min"], r["f_test_mhz"] * 1e6,
                       r["srf_min_mhz"] * 1e6, r["dcr_max_ohm"],
                       r.get("tol_pct", _series()[series].get("tol_pct")))
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


# Robust model (spec 2026-10-19 robust filter bank): one worst-case physical bound, not a fit.
ROBUST_DERATE = 0.75            # 25 % below the guaranteed Q
LEAD_NH_PER_MM = 1.0
LEAD_EXTRA_MM = 1.0             # per side: body end to the board, and through it
FILTER_LARGE, FILTER_SMALL = "tdk_bc", "bourns_78f"
IRAN_SELECT_TOL = 0.03          # Iran: parts selected on an LCR meter


def filter_series(henries: float) -> str:
    """BC axial from 1 µH, Bourns 78F below."""
    return FILTER_LARGE if henries >= 1e-6 * (1 - 1e-9) else FILTER_SMALL


def robust_r(r: Row, f_hz):
    """Worst-case series R: R(f_t) = 2π·f_t·L / (0.75·Q_min) up to f_t, then growing with f
    so Q is held at 0.75·Q_min above f_t. Accepts a scalar or an array."""
    r_t = 2 * math.pi * r.f_test_hz * r.henries / (ROBUST_DERATE * r.q_min)
    f = np.asarray(f_hz, dtype=float)
    out = np.where(f < r.f_test_hz, r_t, r_t * f / r.f_test_hz)
    return float(out) if out.ndim == 0 else out


def lead_l(series: str) -> float:
    """Lead inductance in henries: 1 nH/mm over the free lead (pitch minus body) plus the extra."""
    info = series_info(series)
    return (info["pitch_mm"] - info["body_mm"][1] + 2 * LEAD_EXTRA_MM) * LEAD_NH_PER_MM * 1e-9


def available(series: str) -> list[float]:
    """Inductances (henries) the series has datasheet rows for, ascending."""
    return sorted(r["uh"] * 1e-6 for r in _series()[series]["rows"])


def nearest_row(series: str, henries: float) -> Row:
    """The datasheet row closest to `henries` on a log scale."""
    return row(series, min(available(series), key=lambda h: abs(math.log(h / henries))))


def _row_z(r: Row, henries, f_hz, cpar: bool = True):
    """Robust impedance of the choke of row `r` at an actual inductance `henries` (a scalar, or
    an array that broadcasts against f_hz): the row's Q_min, f_t and SRF; R scales with the
    actual inductance; lead L added in series. cpar=False leaves out the parallel capacitance
    (a part whose self-resonance is far above its guaranteed minimum)."""
    scaled = Row(r.series, r.mpn, henries, r.q_min, r.f_test_hz, r.srf_min_hz, r.dcr_max, r.tol)
    w = 2 * math.pi * np.asarray(f_hz, dtype=float)
    z = robust_r(scaled, f_hz) + 1j * w * (henries + lead_l(r.series))
    return z / (1 + 1j * w * c_par(r) * z) if cpar else z


def robust_z(henries: float, f_hz, scale=1.0, cpar: bool = True):
    """Impedance of one filter choke of `henries` (any value in its series' range), from the
    nearest row (`_row_z`), at henries × scale (scale may be an array of draws)."""
    return _row_z(nearest_row(filter_series(henries), henries), henries * np.asarray(scale), f_hz, cpar)


def part_z(bought: float, henries: float, f_hz, scale=1.0, cpar: bool = True):
    """Impedance of the bought choke `bought` (a value of `available(filter_series(bought))`)
    at its actual inductance henries × scale (selected, or drawn within tolerance)."""
    return _row_z(row(filter_series(bought), bought), henries * np.asarray(scale), f_hz, cpar)


def tolerance(r: Row, population: str) -> float:
    """Inductance tolerance as a fraction: the row's grade for China, 3 % (selected) for Iran."""
    if population == "iran":
        return IRAN_SELECT_TOL
    pct = r.tol if r.tol is not None else series_info(r.series)["tol_pct"]
    return pct / 100
