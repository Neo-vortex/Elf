"""Antenna input: GDT + 1 MΩ bleed at the connector, then the 5 V bias-T (spec §4).

The bias feed (switched by an AO3401 + PTC in phase 3) is decoupled to ground here. The
choke chain runs smallest-first from the RF line: 4.7 µH, 47 µH (damped by 2.2 kΩ: without
it the 47 µH and the capacitive 1 mH form a series resonance near 38 MHz) and 1 mH.
"""
from __future__ import annotations

from skidl import Circuit, Net

from neohw import parts
from neohw.blocks import Block

RADIAL_FOOTPRINT = "Inductor_THT:L_Radial_D10.0mm_P5.00mm_Fastron_07HCP"   # verify in layout
# (inductance, parallel C for the SRF, Q, frequency the Q is quoted at, DC resistance, footprint,
# BOM note). 4.7 µH and 47 µH: axial colour-ring chokes (SRF ~50 / ~15 MHz). 1 mH: a radial
# drum choke (~10 x 12 mm, SRF ~1.5 MHz) — an axial 1 mH has tens of ohms DCR and is rated
# well under the 50–100 mA an active antenna draws.
CHOKES = (
    (4.7e-6, 2.2e-12, 40.0, 8e6, 0.2, parts.INDUCTOR_FOOTPRINT, "choke, axial, rated >= 150 mA, DCR <= 0.3 Ω"),
    (47e-6, 2.4e-12, 40.0, 2.5e6, 1.0, parts.INDUCTOR_FOOTPRINT, "choke, axial, rated >= 150 mA, DCR <= 1.5 Ω"),
    (1e-3, 11e-12, 40.0, 0.3e6, 2.5, RADIAL_FOOTPRINT, "choke, radial drum, rated >= 150 mA, DCR <= 3 Ω"),
)
DAMPING_OHMS = {1: "2.2k"}   # index into CHOKES -> parallel resistor


def build() -> Block:
    ckt = Circuit()
    with ckt:
        rf_in, rf_out, gnd, feed = Net("ant"), Net("rf_out"), Net("GND"), Net("bias_feed")
        g = parts.gdt()
        g[1] += rf_in
        g[2] += gnd
        bleed = parts.resistor("1M")
        bleed[1] += rf_in
        bleed[2] += gnd
        block = parts.capacitor("1u", note="X7R DC block")
        block[1] += rf_in
        block[2] += rf_out
        node = rf_in
        for k, (henries, c_par, q, q_freq, dcr, footprint, note) in enumerate(CHOKES):
            nxt = feed if k == len(CHOKES) - 1 else Net(f"choke{k + 1}")
            ch = parts.inductor(henries, q, q_freq, c_par=c_par, dcr=dcr, footprint=footprint, note=note)
            ch[1] += node
            ch[2] += nxt
            if k in DAMPING_OHMS:
                rd = parts.resistor(DAMPING_OHMS[k])
                rd[1] += node
                rd[2] += nxt
            node = nxt
        for value, note in (("100n", "X7R decoupling"), ("10u", "X5R decoupling")):
            c = parts.capacitor(value, note=note)
            c[1] += feed
            c[2] += gnd
    return Block("bias_tee", ckt, {"rf_in": rf_in, "rf_out": rf_out, "bias_feed": feed})
