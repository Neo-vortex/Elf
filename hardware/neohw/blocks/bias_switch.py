"""Bias-T switch and current sense (phase 3 spec §3.3): 5V_BIAS → 1 Ω shunt → SY6280 (R_SET 17.4 kΩ:
I_LIM ≈ 0.39 A, the part's minimum) → bias feed.

A BC857B pair mirrors the shunt drop. The diode-connected reference sits on the shunt's load side
through 100 Ω and the output transistor on its supply side through 100 Ω, so the shunt drop raises
the output current by about I_load · R_shunt / (R_E + r_e). 680 Ω turns it into a voltage for an
ESP32-S3 ADC pin, on top of the reference bias (≈ 0.6 V; the firmware reads it as zero with the
feed switched off). The transfer is monotonic and slightly non-linear (r_e falls as the current
rises), so the check exports it as a table for the firmware. The 680 Ω keeps the reading below
3.1 V even at the SY6280's current limit. The firmware cuts the feed at about 150 mA; the SY6280
limit is the hardware backstop."""
from __future__ import annotations

from skidl import Circuit, Net

from neohw import parts
from neohw.blocks import Block

SHUNT, RE, RL, RBIAS, RSET = "1", "100", "680", "4.7k", 17.4e3


def _r(value, a, b):
    r = parts.resistor(value)
    r[1] += a
    r[2] += b


def build(ckt: Circuit | None = None) -> Block:
    ckt = ckt or Circuit()
    with ckt:
        gnd, bias5, en, feed, sense = Net("GND"), Net("+5V_BIAS"), Net("bias_en"), Net("bias_feed"), Net("bias_isense")
        lo, raw, b, e1, e2, iset = (Net(f"bias_{x}") for x in ("shunt_lo", "isense_raw", "mirror_b", "mirror_e1",
                                                                "mirror_e2", "iset"))
        _r(SHUNT, bias5, lo)
        sw = parts.sy6280(RSET)
        sw["IN"] += lo
        sw["OUT"] += feed
        sw["GND"] += gnd
        sw["EN"] += en
        sw["ISET"] += iset
        _r("17.4k", iset, gnd)
        _r("100k", en, gnd)
        c = parts.capacitor("1u", note="X7R switch input")
        c[1] += lo
        c[2] += gnd
        q1, q2 = parts.pnp_bc857b(), parts.pnp_bc857b()
        _r(RE, lo, e1)          # reference (diode-connected): load side of the shunt
        _r(RE, bias5, e2)       # output: supply side
        q1["E"] += e1
        q1["B"] += b
        q1["C"] += b
        _r(RBIAS, b, gnd)
        q2["E"] += e2
        q2["B"] += b
        q2["C"] += raw
        _r(RL, raw, gnd)
        _r("10k", raw, sense)
        cf = parts.capacitor("100n", note="X7R ADC filter")
        cf[1] += sense
        cf[2] += gnd
    return Block("bias_switch", ckt, {"bias5": bias5, "en": en, "feed": feed, "sense": sense, "raw": raw})
