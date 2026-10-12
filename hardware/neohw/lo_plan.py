"""Si5351C frequency planner for the LO (phase 3 spec §3.1): the reference the firmware's C
planner is tested against.

Limits (Linux drivers/clk/clk-si5351.h, from Silicon Labs AN619): PLL VCO 600–900 MHz;
feedback multiplier a + b/c with 15 ≤ a + b/c ≤ 90 and c ≤ 1 048 575; MultiSynth divider
6–1800 (integer) or 8–1800 (fractional); output R divider 1–128 (powers of 2). The phase
register CLKx_PHOFF is 7 bits in units of ¼ VCO period (AN619 p. 10), so a 90° offset needs an
even integer divider N ≤ 127 programmed as PHOFF = N: direct quadrature works from
600 MHz / 126 = 4.76 MHz up, and the board switches to the ÷4 path (CLK2 = 4·LO into a 74LVC74
Johnson counter) below DIRECT_MIN_HZ.

Outputs (Si5351C-B-GM1, 4 outputs): CLK0/CLK1 = direct I/Q on PLLA; CLK2 = 4·LO on PLLB;
CLK3 = calibration tone on PLLB.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from fractions import Fraction

XTAL_HZ = 25_000_000
VCO_MIN_HZ, VCO_MAX_HZ = 600_000_000, 900_000_000
PLL_MULT_MIN, PLL_MULT_MAX = 15, 90
DENOM_MAX = 1_048_575
MS_INT_MIN, MS_FRAC_MIN, MS_MAX = 6, 8, 1800
R_DIVS = (1, 2, 4, 8, 16, 32, 64, 128)
PHOFF_MAX = 127
DIRECT_MIN_HZ = 4.8e6
LO_MIN_HZ, LO_MAX_HZ = 0.1e6, 54e6


@dataclass(frozen=True)
class Output:
    pll: str
    ms: Fraction           # MultiSynth divider
    r_div: int
    phoff: int
    actual_hz: Fraction


@dataclass(frozen=True)
class Plan:
    f_lo_hz: float
    path: str                                   # "direct" or "div4"
    vco_hz: dict = field(default_factory=dict)   # PLL name -> exact VCO frequency
    pll_mult: dict = field(default_factory=dict)  # PLL name -> feedback multiplier (a + b/c)
    outputs: dict = field(default_factory=dict)  # "CLK0".. -> Output


def _pll(vco: Fraction) -> tuple[Fraction, Fraction]:
    """Feedback multiplier for a VCO frequency, limited to the 20-bit denominator; returns
    (multiplier, VCO actually produced)."""
    mult = (vco / XTAL_HZ).limit_denominator(DENOM_MAX)
    if not PLL_MULT_MIN <= mult <= PLL_MULT_MAX:
        raise ValueError(f"PLL multiplier {float(mult):.3f} outside {PLL_MULT_MIN}–{PLL_MULT_MAX}")
    return mult, mult * XTAL_HZ


def _integer_plan(f: Fraction, n_max: int = MS_MAX) -> tuple[int, int, Fraction]:
    """Largest even integer divider N (with an R divider when f is too low) such that the VCO
    N·R·f lies in range; returns (N, R, VCO)."""
    for r in R_DIVS:
        top = min(n_max, int(VCO_MAX_HZ // (f * r)))
        n = top - (top % 2)
        if n >= MS_INT_MIN and n * r * f >= VCO_MIN_HZ:
            return n, r, n * r * f
    raise ValueError(f"no integer MultiSynth plan for {float(f):.1f} Hz")


def _fractional_ms(vco: Fraction, f: Fraction) -> tuple[Fraction, int]:
    """MultiSynth divider (fractional) and R divider for f from a fixed VCO."""
    for r in R_DIVS:
        ms = (vco / (f * r)).limit_denominator(DENOM_MAX)
        if MS_FRAC_MIN <= ms <= MS_MAX:
            return ms, r
    raise ValueError(f"no MultiSynth divider for {float(f):.1f} Hz from VCO {float(vco):.0f} Hz")


def plan(f_lo_hz: float, cal_offset_hz: float | None = None) -> Plan:
    """Si5351C settings for an LO frequency, and optionally the calibration tone at
    f_lo + cal_offset_hz on CLK3."""
    if not LO_MIN_HZ <= f_lo_hz <= LO_MAX_HZ:
        raise ValueError(f"LO {f_lo_hz / 1e6:.6f} MHz is outside the receiver's 0.1–54 MHz range")
    f = Fraction(f_lo_hz).limit_denominator(1000)
    p = Plan(f_lo_hz, "direct" if f_lo_hz >= DIRECT_MIN_HZ else "div4")
    if p.path == "direct":
        n, r, vco = _integer_plan(f, n_max=PHOFF_MAX)
        if r != 1:
            raise ValueError(f"direct quadrature at {f_lo_hz} Hz would need an R divider")
        mult, vco_a = _pll(vco)
        p.vco_hz["A"], p.pll_mult["A"] = vco_a, mult
        for clk, phoff in (("CLK0", 0), ("CLK1", n)):
            p.outputs[clk] = Output("A", Fraction(n), 1, phoff, vco_a / n)
        vco_b = None
    else:
        n, r, vco = _integer_plan(4 * f)
        mult, vco_b = _pll(vco)
        p.vco_hz["B"], p.pll_mult["B"] = vco_b, mult
        p.outputs["CLK2"] = Output("B", Fraction(n), r, 0, vco_b / (n * r))
    if cal_offset_hz is not None:
        f_cal = f + Fraction(cal_offset_hz).limit_denominator(1000)
        if vco_b is None:     # direct path: PLLB is free for the tone
            n, r, vco = _integer_plan(f_cal)
            mult, vco_b = _pll(vco)
            p.vco_hz["B"], p.pll_mult["B"] = vco_b, mult
            p.outputs["CLK3"] = Output("B", Fraction(n), r, 0, vco_b / (n * r))
        else:
            ms, r = _fractional_ms(vco_b, f_cal)
            p.outputs["CLK3"] = Output("B", ms, r, 0, vco_b / (ms * r))
    return p


def lo_error_hz(p: Plan) -> float:
    """Frequency error of the quadrature LO the mixer sees (CLK0, or CLK2 / 4)."""
    got = p.outputs["CLK0"].actual_hz if p.path == "direct" else p.outputs["CLK2"].actual_hz / 4
    return float(abs(got - Fraction(p.f_lo_hz).limit_denominator(1000)))


def coverage(start_hz: float, stop_hz: float, step_hz: float, cal_offset_hz: float = 10e3) -> dict:
    """Plan every LO on a grid and collect the worst error and any failure."""
    failures, worst, n = [], 0.0, 0
    k_stop = int(round((stop_hz - start_hz) / step_hz))
    for k in range(k_stop + 1):
        f = start_hz + k * step_hz
        n += 1
        try:
            p = plan(f, cal_offset_hz)
        except ValueError as e:
            failures.append((f, str(e)))
            continue
        err = max(lo_error_hz(p),
                  float(abs(p.outputs["CLK3"].actual_hz - Fraction(f + cal_offset_hz).limit_denominator(1000))))
        worst = max(worst, err)
    return {"points": n, "failures": failures, "max_error_hz": worst}
