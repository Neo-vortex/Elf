"""BFR92A resistive shunt-feedback preamp with a G6K-2F bypass relay (spec §4).

Gain ≈ Rf / Re1 for Rf·Re1 ≈ 50²; split emitter (5.6 Ω at RF, + 12 Ω bypassed for DC);
collector fed by a 10 µH choke damped by 1 kΩ; 10 Ω base stopper; base divider 4.7k / 2.7k.
State "amp": relay energised (COM-NO through the amplifier); "bypass": released.
"""
from __future__ import annotations

from skidl import Circuit, Net

from neohw import parts
from neohw.blocks import Block

VALUES = {"rf": "360", "re1": "5.6", "re2": "12", "r1": "4.7k", "r2": "2.7k", "rbs": "10",
          "rcl": "1k", "lc": 10e-6}
STATES = ("amp", "bypass")


def _r(value, a: Net, b: Net) -> None:
    r = parts.resistor(value)
    r[1] += a
    r[2] += b


def _c(value, a: Net, b: Net, note: str = "X7R") -> None:
    c = parts.capacitor(value, note=note)
    c[1] += a
    c[2] += b


def build(state: str = "amp") -> Block:
    if state not in STATES:
        raise ValueError(f"preamp state must be one of {STATES}, got {state!r}")
    v = VALUES
    ckt = Circuit()
    with ckt:
        rf_in, rf_out, gnd, vcc = Net("rf_in"), Net("rf_out"), Net("GND"), Net("+5V_A")
        amp_in, amp_out, bypass = Net("pre_in"), Net("pre_out"), Net("pre_bypass")
        b_in, b, c, e, e2, fb = (Net(f"pre_{n}") for n in ("bin", "b", "c", "e", "e2", "fb"))
        k = parts.relay_dpdt("energised" if state == "amp" else "released")
        k["COM1"] += rf_in
        k["NO1"] += amp_in
        k["NC1"] += bypass
        k["COM2"] += rf_out
        k["NO2"] += amp_out
        k["NC2"] += bypass
        _c("100n", amp_in, b_in)
        _r(v["rbs"], b_in, b)
        _r(v["r1"], vcc, b_in)
        _r(v["r2"], b_in, gnd)
        _r(v["rf"], c, fb)
        _c("100n", fb, b_in)
        q = parts.npn_bfr92a()
        q["C"] += c
        q["B"] += b
        q["E"] += e
        _r(v["re1"], e, e2)
        _r(v["re2"], e2, gnd)
        _c("1u", e2, gnd)
        lc = parts.inductor(v["lc"], 30.0, 14e6, note="collector choke, rated >= 50 mA")
        lc[1] += vcc
        lc[2] += c
        _r(v["rcl"], vcc, c)
        _c("100n", c, amp_out)
        _c("100n", vcc, gnd, "X7R decoupling")
        _c("10u", vcc, gnd, "X5R decoupling")
    return Block("preamp", ckt, {"rf_in": rf_in, "rf_out": rf_out, "vcc": vcc})
