"""One band-pass filter of the bank: Chebyshev HPF(5) + LPF(7/9), switched by two G6K-2F (spec §4).

Two populations (Rev A spec §2): China fits TDK NL453232 / NLV32 wirewound chips as bought and
±1 % C0G capacitors from E24; Iran fits Bourns 78F axial chokes and E12 C0G capacitors, both
selected with an LCR meter. Every inductor is modelled from its guaranteed datasheet Q, DCR and
SRF (neohw.inductors). Inductors share one footprint with SMD and THT pads; capacitor positions
have as many slots as the longer of the two realisations."""
from __future__ import annotations

import math
from dataclasses import dataclass

from skidl import Circuit, Net

from neohw import eseries, parts
from neohw.units import format_si
from neohw.population import Variant, set_variants
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
RIPPLE_DB = (0.02,) * 8 + (0.01, 0.005, 0.005)   # filter 11: 0.005 dB with the datasheet inductors (2026-10-19)
EDGE_MARGIN = 1.03   # HPF cutoff 3 % below the band's lower edge
LPF_MARGIN = (1.03,) * 9 + (1.05, 1.05)  # LPF cutoff 3 % above the upper edge; 5 % on filters 10–11 (datasheet inductors, 2026-10-19)
L_MIN = 0.1e-6       # smallest axial colour-ring inductor commonly sold (and the smallest NLV32)
C_MIN = 1e-12        # smallest E24 C0G 0805
CAP_TOL = {"china": 0.01, "iran": 0.02}   # China ±1 % as bought; Iran selected to ±2 %
CAP_FOOTPRINT = "Capacitor_SMD:C_0805_2012Metric"


@dataclass(frozen=True)
class Position:
    """One ladder position realised with 1–2 preferred-value parts: `parts` for China (E24
    capacitors), `iran_parts` for Iran (E12 capacitors); inductors are E12 in both."""
    kind: str
    ideal: float
    parts: tuple[float, ...]
    iran_parts: tuple[float, ...]


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
        parts_ = eseries.combo(e.value, eseries.E12, minimum=L_MIN)
        return Position(e.kind, e.value, parts_, parts_)
    return Position(e.kind, e.value, eseries.combo(e.value, eseries.E24, minimum=C_MIN),
                    eseries.combo(e.value, eseries.E12, minimum=C_MIN))


def design(index: int) -> FilterDesign:
    if not 1 <= index <= len(BANDS):
        raise ValueError(f"filter index must be 1..{len(BANDS)}, got {index}")
    lo, hi = BANDS[index - 1]
    ripple = RIPPLE_DB[index - 1]
    elements = (highpass(lo / EDGE_MARGIN, HPF_ORDER, ripple)
                + lowpass(hi * LPF_MARGIN[index - 1], LPF_ORDER[index - 1], ripple))
    return FilterDesign(index, lo, hi, tuple(_realise(e) for e in elements))


Q_REFS = ("centre", "upper")
INDUCTOR_NOTE = "select each group so its sum is within ±3 % of target (bpf_bank.json build sheet)"
CAPACITOR_NOTE = "C0G/NP0; select to ±2 % with LCR meter"
CAPACITOR_NOTE_CHINA = "C0G/NP0 ±1 %, fitted as bought"


def _inductors(pos: Position, a: Net, b: Net, prefix: str, d: FilterDesign, q_freq: float) -> None:
    """Series chain of 1–2 inductors from a to b. Iran selects them so their sum is pos.ideal
    (the trim); China fits the bought values."""
    values = pos.parts
    nodes = [a] + [Net(f"{prefix}_s{k}") for k in range(1, len(values))] + [b]
    trim = pos.ideal / sum(values)
    for k, v in enumerate(values):
        ind = parts.filter_inductor(v, q_freq, trim, note=INDUCTOR_NOTE)
        ind.select_group, ind.select_target = prefix, pos.ideal
        ind[1] += nodes[k]
        ind[2] += nodes[k + 1]


def _cap_variant(farads: float, pop: str, part) -> Variant:
    note = CAPACITOR_NOTE_CHINA if pop == "china" else CAPACITOR_NOTE
    return Variant(format_si(farads, "F"),
                   parts.capacitor_spec(farads, CAP_TOL[pop], parts.package_of(CAP_FOOTPRINT)), note,
                   spice_lines=parts._capacitor_lines(farads, 0.0, 0.0, part), tol=CAP_TOL[pop])


def _capacitors(pos: Position, a: Net, b: Net) -> None:
    """1–2 C0G capacitor slots in parallel between a and b: slot k carries the China part k of
    pos.parts and the Iran part k of pos.iran_parts (unfitted where a population has fewer)."""
    for k in range(max(len(pos.parts), len(pos.iran_parts))):
        china = pos.parts[k] if k < len(pos.parts) else None
        iran = pos.iran_parts[k] if k < len(pos.iran_parts) else None
        c = parts.capacitor(china if china is not None else iran, footprint=CAP_FOOTPRINT,
                            tol=CAP_TOL["iran"], note=CAPACITOR_NOTE)
        set_variants(c, china=None if china is None else _cap_variant(china, "china", c),
                     iran=None if iran is None else _cap_variant(iran, "iran", c))
        c[1] += a
        c[2] += b


def add_filter(d: FilterDesign, f_in: Net, f_out: Net, gnd: Net, q_ref: str = "centre") -> None:
    """Add the filter's parts to the current circuit between f_in and f_out.

    `q_ref` is where each inductor's datasheet loss is evaluated: the band centre (nominal) or
    the upper edge (worst case: the largest series R in the band). The last series position
    ends on f_out itself.
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
                _capacitors(pos, node, nxt)
            else:
                _inductors(pos, node, nxt, prefix, d, q_freq)
            node = nxt
        elif pos.kind == "C_shunt":
            _capacitors(pos, node, gnd)
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
