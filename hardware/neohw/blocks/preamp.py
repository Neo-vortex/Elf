"""BFR92A resistive shunt-feedback preamp with active bias and a G6K-2F bypass relay (spec §4).

RF: gain ≈ Rf / Re1 for Rf·Re1 ≈ 50²; 10 Ω base stopper; collector fed by a 47 µH choke (10 µH loaded the output below 2 MHz)
damped by 1 kΩ. DC: a BC857B senses the supply current across Rs (47 Ω, ~1 V) and feeds the
base through 2.2 kΩ from an RF-decoupled node, so the collector current (~20 mA) follows
neither the BFR92A's hFE nor its temperature much and stays below its 25 mA maximum.
State "amp": relay energised (COM-NO through the amplifier); "bypass": released.
"""
from __future__ import annotations

from skidl import Circuit, Net

from neohw import parts
from neohw.blocks import Block

VALUES = {"rf": "360", "re1": "5.6", "rbs": "10", "rs": "47", "r3": "1k", "r4": "2.2k",
          "rbb": "2.2k", "rpd": "4.7k", "rcl": "1k", "lc": 47e-6}
STATES = ("amp", "bypass")


def _r(value, a: Net, b: Net) -> None:
    r = parts.resistor(value)
    r[1] += a
    r[2] += b


def _c(value, a: Net, b: Net, note: str = "X7R") -> None:
    c = parts.capacitor(value, note=note)
    c[1] += a
    c[2] += b


def build(state: str = "amp", bf: float | None = None, ckt: Circuit | None = None) -> Block:
    """`bf` overrides the BFR92A model's current gain (hFE corner runs)."""
    if state not in STATES:
        raise ValueError(f"preamp state must be one of {STATES}, got {state!r}")
    v = VALUES
    ckt = ckt or Circuit()   # a shared circuit when the board netlist builds every block
    with ckt:
        rf_in, rf_out, gnd, vcc = Net("rf_in"), Net("rf_out"), Net("GND"), Net("+5V_A")
        amp_in, amp_out, bypass = Net("pre_in"), Net("pre_out"), Net("pre_bypass")
        b_in, b, c, e, fb = (Net(f"pre_{n}") for n in ("bin", "b", "c", "e", "fb"))
        vf, vref, qb = Net("pre_vf"), Net("pre_vref"), Net("pre_qb")
        k = parts.relay_dpdt("energised" if state == "amp" else "released")
        k["COM1"] += rf_in
        k["NO1"] += amp_in
        k["NC1"] += bypass
        k["COM2"] += rf_out
        k["NO2"] += amp_out
        k["NC2"] += bypass
        # RF stage
        _c("100n", amp_in, b_in)
        _r(v["rbs"], b_in, b)
        _r(v["rf"], c, fb)
        _c("100n", fb, b_in)
        q = parts.npn_bfr92a(bf=bf)
        q["C"] += c
        q["B"] += b
        q["E"] += e
        _r(v["re1"], e, gnd)
        lc = parts.inductor(v["lc"], 40.0, 2.5e6, c_par=2.4e-12, dcr=1.0,
                            note="collector choke, axial, rated >= 50 mA, DCR <= 1.5 Ω")
        lc[1] += vf
        lc[2] += c
        _r(v["rcl"], vf, c)
        _c("100n", c, amp_out)
        # active bias
        _r(v["rs"], vcc, vf)
        _c("100n", vf, gnd, "X7R decoupling")
        _r(v["r3"], vcc, vref)
        _r(v["r4"], vref, gnd)
        _c("100n", vref, gnd, "X7R decoupling")
        qp = parts.pnp_bc857b()
        qp["E"] += vf
        qp["B"] += vref
        qp["C"] += qb
        _c("100n", qb, gnd, "X7R decoupling")
        _r(v["rbb"], qb, b_in)
        _r(v["rpd"], b_in, gnd)
        _c("100n", vcc, gnd, "X7R decoupling")
        _c("10u", vcc, gnd, "X5R decoupling")
    return Block("preamp", ckt, {"rf_in": rf_in, "rf_out": rf_out, "vcc": vcc, "sense_hi": vcc, "sense_lo": vf})
