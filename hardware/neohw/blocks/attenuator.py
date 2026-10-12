"""20 dB π-pad switched by a G6K-2F DPDT relay (spec §4)."""
from __future__ import annotations

from skidl import Circuit, Net

from neohw import parts
from neohw.blocks import Block

SHUNT_OHMS = "61.9"   # E96, ideal 61.11 Ω for 20 dB at 50 Ω
SERIES_OHMS = "249"   # E96, ideal 247.5 Ω


def build(state: str, ckt: Circuit | None = None) -> Block:
    ckt = ckt or Circuit()   # a shared circuit when the board netlist builds every block
    with ckt:
        rf_in, rf_out, gnd = Net("rf_in"), Net("rf_out"), Net("GND")
        bypass, pad_a, pad_b = Net("bypass"), Net("pad_a"), Net("pad_b")
        k = parts.relay_dpdt(state)
        k["COM1"] += rf_in
        k["COM2"] += rf_out
        k["NC1"] += bypass
        k["NC2"] += bypass
        k["NO1"] += pad_a
        k["NO2"] += pad_b
        r_a = parts.resistor(SHUNT_OHMS, mpn="RC0805FR-0761R9L")
        r_a[1] += pad_a
        r_a[2] += gnd
        r_s = parts.resistor(SERIES_OHMS, mpn="RC0805FR-07249RL")
        r_s[1] += pad_a
        r_s[2] += pad_b
        r_b = parts.resistor(SHUNT_OHMS, mpn="RC0805FR-0761R9L")
        r_b[1] += pad_b
        r_b[2] += gnd
    return Block("attenuator", ckt, {"rf_in": rf_in, "rf_out": rf_out})
