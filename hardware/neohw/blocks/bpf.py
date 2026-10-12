"""One band-pass filter of the bank: Chebyshev HPF(5) + LPF(7/9), switched by two G6K-2F (spec §4)."""
from __future__ import annotations

import math
from dataclasses import dataclass

from skidl import Circuit, Net

from neohw import eseries, parts
from neohw.blocks import Block
from neohw.synth.filters import Element, highpass, lowpass

# Parent spec §4 table; filter n covers BANDS[n - 1] (Hz). 30–54 MHz is split at 40 MHz (final
# review of plan 2.4): as one filter, the mixer's 2f response at a 30 MHz LO (60 MHz) sat just
# above its low-pass cutoff and was rejected by only ~45 dB.
BANDS = ((0.1e6, 0.5e6), (0.5e6, 1.7e6), (1.7e6, 2.5e6), (2.5e6, 4.5e6), (4.5e6, 7.5e6),
         (7.5e6, 12e6), (12e6, 16e6), (16e6, 22e6), (22e6, 30e6), (30e6, 40e6), (40e6, 54e6))
LPF_ORDER = (9, 9, 9, 9, 9, 9, 7, 7, 7, 7, 7)  # 7 where rejection at 3x has > 5 dB to spare
HPF_ORDER = 5
# Chebyshev ripple per filter: 0.02 dB; lower on 9–11, where ten open relay contacts on each bus
# (~6 pF) eat into the return loss and the filters have rejection to spare
RIPPLE_DB = (0.02,) * 8 + (0.01, 0.005, 0.01)
EDGE_MARGIN = 1.03   # HPF cutoff 3 % below the band's lower edge
LPF_MARGIN = (1.03,) * 11            # LPF cutoff 3 % above the upper edge
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
    ripple = RIPPLE_DB[index - 1]
    elements = (highpass(lo / EDGE_MARGIN, HPF_ORDER, ripple)
                + lowpass(hi * LPF_MARGIN[index - 1], LPF_ORDER[index - 1], ripple))
    return FilterDesign(index, lo, hi, tuple(_realise(e) for e in elements))


Q_REFS = ("centre", "upper")
INDUCTOR_NOTE = ("select each group so its sum is within ±3 % of target (bpf_bank.json build sheet); "
                 f"Q={Q_ASSUMED:g} at band centre assumed")
CAPACITOR_NOTE = "C0G/NP0; select to ±2 % with LCR meter"


def _inductors(pos: Position, a: Net, b: Net, prefix: str, d: FilterDesign, q_freq: float) -> None:
    """Series chain of 1–2 inductors from a to b, selected so their sum is pos.ideal."""
    values = pos.parts
    nodes = [a] + [Net(f"{prefix}_s{k}") for k in range(1, len(values))] + [b]
    trim = pos.ideal / sum(values)
    for k, v in enumerate(values):
        ind = parts.inductor(v, Q_ASSUMED, q_freq, trim=trim, note=INDUCTOR_NOTE)
        ind.select_group, ind.select_target = prefix, pos.ideal
        ind[1] += nodes[k]
        ind[2] += nodes[k + 1]


def _capacitors(values, a: Net, b: Net) -> None:
    """1–2 C0G capacitors in parallel between a and b."""
    for v in values:
        c = parts.capacitor(v, note=CAPACITOR_NOTE)
        c[1] += a
        c[2] += b


def add_filter(d: FilterDesign, f_in: Net, f_out: Net, gnd: Net, q_ref: str = "centre") -> None:
    """Add the filter's parts to the current circuit between f_in and f_out.

    `q_ref` is where the assumed Q holds: the band centre (nominal) or the upper edge
    (worst case: lower Q across the band). The last series position ends on f_out itself.
    """
    if q_ref not in Q_REFS:
        raise ValueError(f"q_ref must be one of {Q_REFS}, got {q_ref!r}")
    q_freq = d.centre_hz if q_ref == "centre" else d.hi_hz
    series_at = [k for k, pos in enumerate(d.positions) if pos.kind in ("C_series", "L_series")]
    node = f_in
    for k, pos in enumerate(d.positions):
        prefix = f"f{d.index}_p{k + 1}"
        if k in series_at:
            nxt = f_out if k == series_at[-1] else Net(f"f{d.index}_n{k + 1}")
            if pos.kind == "C_series":
                _capacitors(pos.parts, node, nxt)
            else:
                _inductors(pos, node, nxt, prefix, d, q_freq)
            node = nxt
        elif pos.kind == "C_shunt":
            _capacitors(pos.parts, node, gnd)
        else:
            _inductors(pos, node, gnd, prefix, d, q_freq)


def _switch(state: str, end: Net, bus: Net, gnd: Net) -> None:
    """One G6K-2F, pole 1 switching: energised joins the filter end to the bus, released
    grounds it and leaves the bus on one open contact. Pole 2 is grounded (paralleling it
    would double the open-contact load on the bus: 9 relays x 1 pF spoils RL on 6 m)."""
    k = parts.relay_dpdt(state)
    k["COM1"] += end
    k["NO1"] += bus
    k["NC1"] += gnd
    for pin in ("COM2", "NO2", "NC2"):
        k[pin] += gnd


def add_switched(index: int, state: str, bus_in: Net, bus_out: Net, gnd: Net, q_ref: str = "centre") -> None:
    """Filter with an input relay and a separate output relay. One relay carrying both buses
    would couple them through its inter-pole capacitance (x10 relays), which breaks the
    3f rejection and mute targets on the upper bands."""
    f_in, f_out = Net(f"f{index}_in"), Net(f"f{index}_out")
    _switch(state, f_in, bus_in, gnd)
    _switch(state, f_out, bus_out, gnd)
    add_filter(design(index), f_in, f_out, gnd, q_ref)


def build(index: int, state: str = "energised", q_ref: str = "centre") -> Block:
    ckt = Circuit()
    with ckt:
        rf_in, rf_out, gnd = Net("rf_in"), Net("rf_out"), Net("GND")
        add_switched(index, state, rf_in, rf_out, gnd, q_ref)
    return Block(f"bpf{index}", ckt, {"rf_in": rf_in, "rf_out": rf_out})
