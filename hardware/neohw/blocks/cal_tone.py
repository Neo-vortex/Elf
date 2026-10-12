"""Calibration tone pad (phase 3 spec §3.1): Si5351 CLK3 → 100 Ω → node P (51 Ω to ground) →
22 kΩ → the RF line ahead of the filter bank. About −11.9 dB into P (with the Si5351's 50 Ω output) and
−58.9 dB from P into the 25 Ω the RF line presents (50 Ω each way): ≈ −70.8 dB from CLK3's open-circuit voltage.
The 22 kΩ loads the RF line by ≈ 0.01 dB; with CLK3 disabled (driven low) nothing is injected.
The 0805 body capacitance (≈ 0.1 pF, modelled) raises the coupling at the top of the HF range."""
from __future__ import annotations

from skidl import Circuit, Net

from neohw import parts
from neohw.blocks import Block

R_IN, R_SHUNT, R_SERIES = "100", "51", "22k"
C_PAR_0805 = 0.1e-12


def build(ckt: Circuit | None = None) -> Block:
    ckt = ckt or Circuit()   # a shared circuit when the board netlist builds every block
    with ckt:
        clk, p, rf, gnd = Net("cal_clk"), Net("cal_p"), Net("rf_line"), Net("GND")
        a = parts.resistor(R_IN)
        a[1] += clk
        a[2] += p
        b = parts.resistor(R_SHUNT)
        b[1] += p
        b[2] += gnd
        c = parts.resistor(R_SERIES, c_par=C_PAR_0805)
        c[1] += p
        c[2] += rf
    return Block("cal_tone", ckt, {"clk": clk, "rf": rf})
