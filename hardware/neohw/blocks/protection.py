"""Light limiter after the attenuator (parent §4.3): two BAV99 in anti-parallel, each a series
string of two 1N4148-class diodes, from the RF line to ground. Each diode sees half the line
voltage, so at normal levels the pair stays out of conduction (IMD3 target, spec §4), and it
clamps strong signals near 2 V peak. No bias network: a biased single-diode pair pumped its
bias nodes above 5 V under overload (final review of plan 2.3)."""
from __future__ import annotations

from skidl import Circuit, Net

from neohw import parts
from neohw.blocks import Block


def build() -> Block:
    ckt = Circuit()
    with ckt:
        rf, gnd = Net("rf"), Net("GND")
        up = parts.diode_pair_bav99()
        up["A"] += rf
        up["K"] += gnd
        down = parts.diode_pair_bav99()
        down["A"] += gnd
        down["K"] += rf
    return Block("protection", ckt, {"rf_in": rf, "rf_out": rf})
