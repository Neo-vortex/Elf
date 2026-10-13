"""Si5351C register decoder for the virtual board (phase 4 spec §3): written from AN619
independently of the firmware's encoder, so a wrong register shows up as a wrong frequency.

a + b/c = (P1 + 512 + P2 / P3) / 128 for both the PLL feedback and the MultiSynth dividers;
f_out = f_xtal · PLL / (MS · R)."""
from __future__ import annotations

from dataclasses import dataclass
from fractions import Fraction

XTAL_HZ = 25_000_000
PLL_BASE = {"A": 26, "B": 34}
MS_BASE = 42
CLK_CTRL, OE, PHOFF = 16, 3, 165


def _p123(r: bytes, base: int) -> tuple[int, int, int]:
    p3 = (r[base] << 8) | r[base + 1] | ((r[base + 5] >> 4) << 16)
    p1 = ((r[base + 2] & 0x03) << 16) | (r[base + 3] << 8) | r[base + 4]
    p2 = ((r[base + 5] & 0x0F) << 16) | (r[base + 6] << 8) | r[base + 7]
    return p1, p2, p3


def ratio(r: bytes, base: int) -> Fraction:
    p1, p2, p3 = _p123(r, base)
    return (p1 + 512 + Fraction(p2, p3)) / 128


@dataclass(frozen=True)
class Clock:
    enabled: bool
    pll: str
    hz: Fraction
    ms: Fraction
    phoff: int


def clocks(r: bytes) -> dict[int, Clock]:
    out = {}
    for k in range(4):
        ctrl = r[CLK_CTRL + k]
        on = not (ctrl & 0x80) and not (r[OE] >> k) & 1 and (ctrl & 0x0C) == 0x0C
        pll = "B" if ctrl & 0x20 else "A"
        base = MS_BASE + 8 * k
        if not on:
            out[k] = Clock(False, pll, Fraction(0), Fraction(0), 0)
            continue
        ms = ratio(r, base)
        rdiv = 1 << ((r[base + 2] >> 4) & 0x07)
        hz = XTAL_HZ * ratio(r, PLL_BASE[pll]) / (ms * rdiv)
        out[k] = Clock(True, pll, hz, ms, r[PHOFF + k] & 0x7F)
    return out


@dataclass(frozen=True)
class LoState:
    lo_hz: Fraction | None        # None: no valid quadrature LO
    path: str
    cal_hz: Fraction | None
    reason: str


def lo_state(r: bytes, lo_path_high: bool) -> LoState:
    """What the mixer sees: direct path = CLK0 and CLK1 a quarter period apart; ÷4 path =
    CLK2 / 4 through the Johnson counter. The mux select must match."""
    c = clocks(r)
    cal = c[3].hz if c[3].enabled else None
    if lo_path_high:
        if not c[2].enabled:
            return LoState(None, "div4", cal, "÷4 path selected but CLK2 is off")
        return LoState(c[2].hz / 4, "div4", cal, "")
    a, b = c[0], c[1]
    if not (a.enabled and b.enabled):
        return LoState(None, "direct", cal, "direct path selected but CLK0/CLK1 are off")
    if a.pll != b.pll or a.ms != b.ms or a.ms.denominator != 1:
        return LoState(None, "direct", cal, "CLK0/CLK1 not the same integer divider on one PLL")
    if (b.phoff - a.phoff) != a.ms:
        return LoState(None, "direct", cal, f"phase offset {b.phoff - a.phoff} is not a quarter period ({a.ms})")
    return LoState(a.hz, "direct", cal, "")
