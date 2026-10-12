"""Tayloe mixer and baseband (spec §5): FST3253 four-phase switch, four sampling capacitors,
one LM4562 per channel as the front half of an instrumentation amplifier, anti-alias RC and
the CS5361 input.

The FST3253 decodes (S1, S0) = 00, 01, 11, 10 into B0, B1, B3, B2, so the four LO phases
0/90/180/270° land on B0, B1, B3, B2: I = B0 − B3 and Q = B1 − B2. The RF node is AC-coupled
and biased at 2.5 V (VQ of the CS5361; the in-amp passes it through as the output
common mode). FST3253 and op-amps run from 5V_A (the FST3253 needs 4.0–5.5 V).
"""
from __future__ import annotations

from skidl import Circuit, Net

from neohw import parts
from neohw.blocks import Block

C_SAMPLE = "5.6n"     # 4·(50 + 4) Ω · 5.6 nF -> corner ~130 kHz: ±0.5 dB over ±40 kHz
R_F, R_G = "2.2k", "240"   # in-amp front half: differential gain 1 + 2·Rf/Rg = 19.3 (25.7 dB)
R_AA, C_AA = "100", "2.7n"


def _r(value, a: Net, b: Net) -> None:
    r = parts.resistor(value)
    r[1] += a
    r[2] += b


def _c(value, a: Net, b: Net, note: str = "X7R") -> None:
    c = parts.capacitor(value, note=note)
    c[1] += a
    c[2] += b


def _channel(plus: Net, minus: Net, ain_p: Net, ain_n: Net, name: str, vcc: Net, gnd: Net) -> None:
    u = parts.lm4562_dual()
    n1, n2, o1, o2 = (Net(f"{name}_{x}") for x in ("n1", "n2", "o1", "o2"))
    u["+INA"] += plus
    u["-INA"] += n1
    u["OUTA"] += o1
    u["+INB"] += minus
    u["-INB"] += n2
    u["OUTB"] += o2
    u["V+"] += vcc
    u["V-"] += gnd
    _r(R_F, o1, n1)
    _r(R_F, o2, n2)
    _r(R_G, n1, n2)
    _r(R_AA, o1, ain_p)
    _r(R_AA, o2, ain_n)
    _c(C_AA, ain_p, ain_n, "C0G")


def build() -> Block:
    ckt = Circuit()
    with ckt:
        rf_in, gnd, vcc, s0, s1 = Net("rf_in"), Net("GND"), Net("+5V_A"), Net("lo_s0"), Net("lo_s1")
        a, vb = Net("mix_a"), Net("mix_vb")
        b = [Net(f"mix_b{k}") for k in range(4)]
        ail_p, ail_n, air_p, air_n, vq = (Net(x) for x in ("ain_i_p", "ain_i_n", "ain_q_p", "ain_q_n", "adc_vq"))
        _c("100n", rf_in, a, "X7R coupling")
        _r("10k", vcc, vb)
        _r("10k", vb, gnd)
        _c("10u", vb, gnd, "X5R decoupling")
        _r("10k", vb, a)
        sw = parts.fst3253()
        sw["/1OE"] += gnd
        sw["/2OE"] += vcc
        sw["VCC"] += vcc
        sw["GND"] += gnd
        sw["S0"] += s0
        sw["S1"] += s1
        sw["1A"] += a
        for k in range(4):
            sw[f"1B{k}"] += b[k]
            _c(C_SAMPLE, b[k], gnd, "C0G; select the four to ±1 % of each other with the LCR meter")
        _channel(b[0], b[3], ail_p, ail_n, "bb_i", vcc, gnd)
        _channel(b[1], b[2], air_p, air_n, "bb_q", vcc, gnd)
        adc = parts.cs5361()
        adc["AINL+"] += ail_p
        adc["AINL-"] += ail_n
        adc["AINR+"] += air_p
        adc["AINR-"] += air_n
        adc["VQ"] += vq
        adc["GND"] += gnd
        _c("1u", vq, gnd, "X7R VQ decoupling")
        _c("100n", vcc, gnd, "X7R decoupling")
        _c("10u", vcc, gnd, "X5R decoupling")
    return Block("mixer", ckt, {"rf_in": rf_in, "vcc": vcc, "s0": s0, "s1": s1, "a": a, "b": tuple(b),
                                "ain_i": (ail_p, ail_n), "ain_q": (air_p, air_n)})
