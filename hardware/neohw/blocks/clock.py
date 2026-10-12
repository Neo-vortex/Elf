"""LO generation (phase 3 spec §3.1): Si5351C-B-GM1 with a 25 MHz crystal on 3V3_RF.

- Direct path (LO ≥ 4.8 MHz, neohw.lo_plan): CLK0/CLK1 in quadrature from PLLA.
- ÷4 path (LO < 4.8 MHz): CLK2 = 4·LO clocks a 74LVC74 Johnson counter (FF1.D = /Q2,
  FF2.D = Q1), whose Q1/Q2 are in quadrature at LO.
- A 74LVC157 selects the pair for the FST3253 selects (S = 0: direct, S = 1: ÷4, from an ESP32
  GPIO); 33 Ω series resistors damp the select lines.
- CLK3 is the calibration tone (cal_tone block).
The I2C pull-ups live in the controller block (shared bus).
"""
from __future__ import annotations

from skidl import Circuit, Net

from neohw import parts
from neohw.blocks import Block

# select-line skew per stage, uniform ± (seconds): output-to-output skew tsk(o) ≤ 1.0 ns for both
# the 74LVC74A and the 74LVC157A (Nexperia, VCC 3.0–3.6 V) -> ±0.5 ns each. The Si5351 datasheet
# specifies no output-to-output skew: ±0.2 ns is assumed (phase 2 spec §5).
SI5351_SKEW_S = 0.2e-9
LVC74_TSK_S = 1.0e-9
LVC157_TSK_S = 1.0e-9
SKEW_STAGES_S = {"direct": (SI5351_SKEW_S, LVC157_TSK_S / 2), "div4": (LVC74_TSK_S / 2, LVC157_TSK_S / 2)}


def _r(value, a, b):
    r = parts.resistor(value)
    r[1] += a
    r[2] += b


def _c(value, a, b, note="X7R decoupling"):
    c = parts.capacitor(value, note=note)
    c[1] += a
    c[2] += b


def build() -> Block:
    ckt = Circuit()
    with ckt:
        vcc, gnd = Net("+3V3_RF"), Net("GND")
        sda, scl, lo_path = Net("i2c_sda"), Net("i2c_scl"), Net("lo_path")
        s0, s1, cal = Net("lo_s0"), Net("lo_s1"), Net("cal_clk")
        clk0, clk1, clk2 = Net("si_clk0"), Net("si_clk1"), Net("si_clk2")
        xa, xb, clkin = Net("si_xa"), Net("si_xb"), Net("si_clkin")
        q1, q2, nq2 = Net("div_q1"), Net("div_q2"), Net("div_nq2")
        y1, y2 = Net("mux_y1"), Net("mux_y2")
        si = parts.si5351c()
        for pin in ("VDD", "VDDOA", "VDDOB", "VDDOC", "VDDOD"):
            si[pin] += vcc
            _c("100n", vcc, gnd)
        si["GND"] += gnd
        si["PAD"] += gnd
        si["OEB"] += gnd                  # outputs enabled; the firmware gates each in its registers
        si["SDA"] += sda
        si["SCL"] += scl
        si["XA"] += xa
        si["XB"] += xb
        si["CLKIN"] += clkin
        _r("10k", clkin, gnd)             # CLKIN unused (no internal pull): held low; TCXO option at layout
        si["CLK0"] += clk0
        si["CLK1"] += clk1
        si["CLK2"] += clk2
        si["CLK3"] += cal
        x = parts.crystal_25mhz()
        x["1"] += xa
        x["2"] += xb
        x["GND1"] += gnd
        x["GND2"] += gnd
        _c("10u", vcc, gnd, "X5R bulk")
        ff = parts.lvc74(nc=("/1Q",))
        ff["VCC"] += vcc
        ff["GND"] += gnd
        for pin in ("/1RD", "/1SD", "/2RD", "/2SD"):
            ff[pin] += vcc
        ff["1CP"] += clk2
        ff["2CP"] += clk2
        ff["1D"] += nq2
        ff["1Q"] += q1
        ff["2D"] += q1
        ff["2Q"] += q2
        ff["/2Q"] += nq2
        _c("100n", vcc, gnd)
        mux = parts.lvc157(nc=("3Y", "4Y"))
        mux["VCC"] += vcc
        mux["GND"] += gnd
        mux["/E"] += gnd
        mux["S"] += lo_path
        mux["1I0"] += clk0
        mux["2I0"] += clk1
        mux["1I1"] += q1
        mux["2I1"] += q2
        for pin in ("3I0", "3I1", "4I0", "4I1"):
            mux[pin] += gnd
        mux["1Y"] += y1
        mux["2Y"] += y2
        _r("33", y1, s0)
        _r("33", y2, s1)
        _c("100n", vcc, gnd)
        _r("10k", lo_path, gnd)           # direct path until the firmware drives the select
    return Block("clock", ckt, {"vcc": vcc, "sda": sda, "scl": scl, "lo_path": lo_path, "s0": s0, "s1": s1,
                                "cal": cal})
