"""One band-pass filter of the bank: Chebyshev HPF(5) + LPF(7/9), switched by a G6K-2F (spec §4)."""
from __future__ import annotations

import math
from dataclasses import dataclass

from skidl import Circuit, Net

from neohw import eseries, parts
from neohw.blocks import Block
from neohw.synth.filters import Element, highpass, lowpass

# Parent spec §4 table; filter n covers BANDS[n - 1] (Hz).
BANDS = ((0.1e6, 0.5e6), (0.5e6, 1.7e6), (1.7e6, 2.5e6), (2.5e6, 4.5e6), (4.5e6, 7.5e6),
         (7.5e6, 12e6), (12e6, 16e6), (16e6, 22e6), (22e6, 30e6), (30e6, 54e6))
LPF_ORDER = (9, 9, 9, 9, 9, 9, 7, 7, 7, 9)  # 7 where rejection at 3x has > 5 dB to spare
HPF_ORDER = 5
RIPPLE_DB = 0.02
EDGE_MARGIN = 1.03   # HPF cutoff 3 % below the band's lower edge
LPF_MARGIN = (1.03,) * 9 + (1.05,)   # LPF cutoff above the upper edge; filter 10 trades
                                     # 2 dB of its spare rejection for in-band loss
Q_ASSUMED = 30.0
L_MIN = 0.1e-6       # smallest axial colour-ring inductor commonly sold
C_MIN = 1e-12        # smallest E24 C0G 0805


@dataclass(frozen=True)
class Position:
    """One ladder position realised with 1–2 preferred-value parts."""
    kind: str
    ideal: float
    parts: tuple[float, ...]


@dataclass(frozen=True)
class FilterDesign:
    index: int
    lo_hz: float
    hi_hz: float
    positions: tuple[Position, ...]   # HPF positions then LPF positions, source to load

    @property
    def centre_hz(self) -> float:
        return math.sqrt(self.lo_hz * self.hi_hz)


def _realise(e: Element) -> Position:
    if e.kind.startswith("L"):
        return Position(e.kind, e.value, eseries.combo(e.value, eseries.E12, minimum=L_MIN))
    return Position(e.kind, e.value, eseries.combo(e.value, eseries.E24, minimum=C_MIN))


def design(index: int) -> FilterDesign:
    if not 1 <= index <= len(BANDS):
        raise ValueError(f"filter index must be 1..{len(BANDS)}, got {index}")
    lo, hi = BANDS[index - 1]
    elements = (highpass(lo / EDGE_MARGIN, HPF_ORDER, RIPPLE_DB)
                + lowpass(hi * LPF_MARGIN[index - 1], LPF_ORDER[index - 1], RIPPLE_DB))
    return FilterDesign(index, lo, hi, tuple(_realise(e) for e in elements))


def _inductors(pos: Position, a: Net, b: Net, prefix: str, d: FilterDesign) -> None:
    """Series chain of 1–2 inductors from a to b, selected so their sum is pos.ideal."""
    values = pos.parts
    nodes = [a] + [Net(f"{prefix}_s{k}") for k in range(1, len(values))] + [b]
    trim = pos.ideal / sum(values)
    for k, v in enumerate(values):
        ind = parts.inductor(v, Q_ASSUMED, d.centre_hz, trim=trim)
        ind[1] += nodes[k]
        ind[2] += nodes[k + 1]


def _capacitors(values, a: Net, b: Net) -> None:
    """1–2 C0G capacitors in parallel between a and b."""
    for v in values:
        c = parts.capacitor(v, note="C0G/NP0 ±2 %")
        c[1] += a
        c[2] += b


def add_filter(d: FilterDesign, f_in: Net, f_out: Net, gnd: Net) -> None:
    """Add the filter's parts to the current circuit between f_in and f_out.

    The last series position ends on f_out itself (nets are never merged: SKiDL keeps
    merged nets as separate objects, which would split the SPICE node).
    """
    series_at = [k for k, pos in enumerate(d.positions) if pos.kind in ("C_series", "L_series")]
    node = f_in
    for k, pos in enumerate(d.positions):
        prefix = f"f{d.index}_p{k + 1}"
        if k in series_at:
            nxt = f_out if k == series_at[-1] else Net(f"f{d.index}_n{k + 1}")
            if pos.kind == "C_series":
                _capacitors(pos.parts, node, nxt)
            else:
                _inductors(pos, node, nxt, prefix, d)
            node = nxt
        elif pos.kind == "C_shunt":
            _capacitors(pos.parts, node, gnd)
        else:
            _inductors(pos, node, gnd, prefix, d)


def add_switched(index: int, state: str, bus_in: Net, bus_out: Net, gnd: Net) -> None:
    """Filter plus its relay. Energised: COM-NO joins the filter ends to the buses.
    Released: COM-NC grounds the filter ends and the buses see only open contacts."""
    f_in, f_out = Net(f"f{index}_in"), Net(f"f{index}_out")
    k = parts.relay_dpdt(state)
    k["COM1"] += f_in
    k["NO1"] += bus_in
    k["NC1"] += gnd
    k["COM2"] += f_out
    k["NO2"] += bus_out
    k["NC2"] += gnd
    add_filter(design(index), f_in, f_out, gnd)


def build(index: int, state: str = "energised") -> Block:
    ckt = Circuit()
    with ckt:
        rf_in, rf_out, gnd = Net("rf_in"), Net("rf_out"), Net("GND")
        add_switched(index, state, rf_in, rf_out, gnd)
    return Block(f"bpf{index}", ckt, {"rf_in": rf_in, "rf_out": rf_out})
