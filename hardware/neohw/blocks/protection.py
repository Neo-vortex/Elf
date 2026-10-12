"""Light limiter after the attenuator (parent §4.3): BAV99 anti-parallel pair, AC-coupled and
held about 1 V from conduction by a resistor ladder on 5V_A, so it clamps near ±1.6 V
(~+14 dBm) and stays out of conduction at normal levels (IMD3 target, spec §4)."""
from __future__ import annotations

from skidl import Circuit, Net

from neohw import parts
from neohw.blocks import Block

# 5V_A - 1.5k - hi (3.5 V) - 1k - mid (2.5 V) - 1k - lo (1.5 V) - 1.5k - GND
LADDER = ("1.5k", "1k", "1k", "1.5k")


def build() -> Block:
    ckt = Circuit()
    with ckt:
        rf, gnd, vcc = Net("rf"), Net("GND"), Net("+5V_A")
        hi, mid, lo, y = Net("lim_hi"), Net("lim_mid"), Net("lim_lo"), Net("lim_y")
        cc = parts.capacitor("100n", note="X7R coupling")
        cc[1] += rf
        cc[2] += y
        rby = parts.resistor("10k")
        rby[1] += mid
        rby[2] += y
        d = parts.diode_pair_bav99()
        d["A"] += lo
        d["COM"] += y
        d["K"] += hi
        nodes = [vcc, hi, mid, lo, gnd]
        for k, value in enumerate(LADDER):
            r = parts.resistor(value)
            r[1] += nodes[k]
            r[2] += nodes[k + 1]
        for node in (hi, mid, lo):
            c = parts.capacitor("100n", note="X7R decoupling")
            c[1] += node
            c[2] += gnd
    return Block("protection", ckt, {"rf_in": rf, "rf_out": rf, "vcc": vcc})
