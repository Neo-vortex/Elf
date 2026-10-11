"""Part constructors: SKiDL parts that also know their SPICE model."""
from __future__ import annotations

from skidl import SKIDL, TEMPLATE, Part, Pin

from neohw.units import format_si, parse_si


def _template(name: str, prefix: str, pins: list[str], footprint: str) -> Part:
    return Part(tool=SKIDL, name=name, ref_prefix=prefix, dest=TEMPLATE, footprint=footprint,
                pins=[Pin(num=p, name=p) for p in pins])


def _positive(value) -> float:
    v = parse_si(value)
    if v <= 0:
        raise ValueError(f"value must be positive, got {value!r}")
    return v


def _num(v: float) -> str:
    return f"{v:.6g}"


def resistor(value, footprint: str = "Resistor_SMD:R_0805_2012Metric", mpn: str | None = None,
             torob: str | None = None) -> Part:
    ohms = _positive(value)
    p = _template("R", "R", ["1", "2"], footprint)(value=format_si(ohms, "Ω"))
    p.mpn, p.torob = mpn, torob
    p.spice_lines = lambda n: [f"R{p.ref} {n['1']} {n['2']} {_num(ohms)}"]
    return p


def capacitor(value, footprint: str = "Capacitor_SMD:C_0805_2012Metric", mpn: str | None = None,
              torob: str | None = None, esr: float = 0.0) -> Part:
    farads = _positive(value)
    p = _template("C", "C", ["1", "2"], footprint)(value=format_si(farads, "F"))
    p.mpn, p.torob = mpn, torob

    def lines(n):
        if esr > 0:
            mid = f"{p.ref}_esr"
            return [f"R{p.ref}_esr {n['1']} {mid} {_num(esr)}", f"C{p.ref} {mid} {n['2']} {_num(farads)}"]
        return [f"C{p.ref} {n['1']} {n['2']} {_num(farads)}"]

    p.spice_lines = lines
    return p


RELAY_PINS = ["COM1", "NC1", "NO1", "COM2", "NC2", "NO2", "COIL+", "COIL-"]


def relay_dpdt(state: str, contact_r: float = 0.05, open_c: float = 0.5e-12,
               footprint: str = "Relay_SMD:Relay_DPDT_Omron_G6K-2F", mpn: str = "G6K-2F-5VDC",
               torob: str = "G6K-2F") -> Part:
    if state not in ("released", "energised"):
        raise ValueError(f"relay state must be 'released' or 'energised', got {state!r}")
    p = _template("G6K-2F", "K", RELAY_PINS, footprint)(value="G6K-2F")
    p.mpn, p.torob, p.state = mpn, torob, state

    def lines(n):
        out = []
        for pole in ("1", "2"):
            com = n[f"COM{pole}"]
            closed, open_ = ("NC", "NO") if state == "released" else ("NO", "NC")
            out.append(f"R{p.ref}_{closed}{pole} {com} {n[closed + pole]} {_num(contact_r)}")
            out.append(f"C{p.ref}_{open_}{pole} {com} {n[open_ + pole]} {_num(open_c)}")
        return out

    p.spice_lines = lines
    return p
