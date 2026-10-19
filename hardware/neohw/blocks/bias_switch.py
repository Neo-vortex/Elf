"""Bias-T switch and current sense (phase 3 spec §3.3): 5V_BIAS → 1 Ω shunt → SY6280 (R_SET 17.4 kΩ:
I_LIM ≈ 0.39 A, the part's minimum) → bias feed.

Two current-sense circuits share the shunt and the ADC filter; one is fitted per population.

China: an INA180A1 (gain 20 V/V) reads the 1 Ω shunt drop, powered from +3V3_D. Its output is 20 mV
per mA on top of a zero within a few mV (input offset at most ±150 µV): ≈ 3.0 V at the firmware's
150 mA cut-off, and it saturates about 20 mV below its 3.3 V supply (≈ 3.28 V), inside the STM32F411
ADC range. The firmware reads a zero near 0 V.

Iran: a BC857B pair mirrors the shunt drop. The diode-connected reference sits on the shunt's load
side through 100 Ω and the output transistor on its supply side through 100 Ω, so the shunt drop
raises the output current by about I_load · R_shunt / (R_E + r_e). 680 Ω turns it into a voltage
on top of the reference bias (≈ 0.63 V; the firmware reads it as zero with the feed switched off).
The transfer is monotonic and slightly non-linear (r_e falls as the current rises). The 680 Ω keeps
the reading below 3.1 V even at the SY6280's current limit.

A 1 MΩ pull-up from the sense node to +3V3_D is fitted in both: with neither circuit fitted the node
reads ≈ 3.3 V at zero (a sense fault in the firmware); the INA180's low output impedance overrides it
(3.3 µA) and the mirror's reading moves slightly (the tables are simulated with it).

The firmware tells the circuits apart by the zero reading with the feed off (≈ 0 V against ≈ 0.63 V)
and uses that circuit's table. The check exports both tables. The firmware cuts the feed at about
150 mA; the SY6280 limit is the hardware backstop."""
from __future__ import annotations

from skidl import Circuit, Net

from neohw import parts, population
from neohw.blocks import Block

SHUNT, RE, RL, RBIAS, RSET = "1", "100", "680", "4.7k", 17.4e3


def _r(value, a, b, pop: str | None = None):
    r = parts.resistor(value)
    r[1] += a
    r[2] += b
    if pop:
        population.only(r, pop)


def build(ckt: Circuit | None = None) -> Block:
    ckt = ckt or Circuit()
    with ckt:
        gnd, bias5, en, feed, sense, v3d = (Net("GND"), Net("+5V_BIAS"), Net("bias_en"), Net("bias_feed"),
                                            Net("bias_isense"), Net("+3V3_D"))
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
        # Iran population: BC857B current mirror
        q1, q2 = population.only(parts.pnp_bc857b(), "iran"), population.only(parts.pnp_bc857b(), "iran")
        _r(RE, lo, e1, "iran")          # reference (diode-connected): load side of the shunt
        _r(RE, bias5, e2, "iran")       # output: supply side
        q1["E"] += e1
        q1["B"] += b
        q1["C"] += b
        _r(RBIAS, b, gnd, "iran")
        q2["E"] += e2
        q2["B"] += b
        q2["C"] += raw
        _r(RL, raw, gnd, "iran")
        # China population: INA180A1 across the shunt, supplied from +3V3_D
        u = population.only(parts.ina180a1(), "china")
        u["IN+"] += bias5
        u["IN-"] += lo
        u["OUT"] += raw
        u["GND"] += gnd
        u["VS"] += v3d
        cv = population.only(parts.capacitor("100n", dielectric="X7R", voltage="16 V", note="X7R INA180 supply"), "china")
        cv[1] += v3d
        cv[2] += gnd
        _r("1M", raw, v3d)      # both populations: with no sense circuit fitted the node reads ≈ 3.3 V, not a believable zero
        _r("10k", raw, sense)
        cf = parts.capacitor("100n", note="X7R ADC filter")
        cf[1] += sense
        cf[2] += gnd
    return Block("bias_switch", ckt, {"bias5": bias5, "en": en, "feed": feed, "sense": sense, "raw": raw,
                                        "v3d": v3d})
