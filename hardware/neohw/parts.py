"""Part constructors: SKiDL parts that also know their SPICE model."""
from __future__ import annotations

from skidl import SKIDL, TEMPLATE, Part, Pin

from neohw.units import format_si, parse_si


def _tagged(part: Part) -> Part:
    """A stable tag (the ref) instead of SKiDL's random one: quiet, reproducible netlists."""
    part.tag = part.ref
    return part


def _template(name: str, prefix: str, pins: list[str] | dict[str, str], footprint: str) -> Part:
    """`pins` is a list of names (pad number = name) or a name -> footprint pad number map."""
    numbered = pins if isinstance(pins, dict) else {p: p for p in pins}
    return Part(tool=SKIDL, name=name, ref_prefix=prefix, dest=TEMPLATE, footprint=footprint,
                pins=[Pin(num=num, name=name) for name, num in numbered.items()])


def _positive(value) -> float:
    v = parse_si(value)
    if v <= 0:
        raise ValueError(f"value must be positive, got {value!r}")
    return v


def _num(v: float) -> str:
    return f"{v:.6g}"


def resistor(value, footprint: str = "Resistor_SMD:R_0805_2012Metric", mpn: str | None = None,
             torob: str | None = None, tol: float = 0.01) -> Part:
    ohms = _positive(value)
    p = _tagged(_template("R", "R", ["1", "2"], footprint)(value=format_si(ohms, "Ω")))
    p.mpn, p.torob, p.tol = mpn, torob, tol
    p.spice_lines = lambda n, k=1.0: [f"R{p.ref} {n['1']} {n['2']} {_num(ohms * k)}"]
    return p


def capacitor(value, footprint: str = "Capacitor_SMD:C_0805_2012Metric", mpn: str | None = None,
              torob: str | None = None, esr: float = 0.0, tol: float = 0.02, note: str = "") -> Part:
    farads = _positive(value)
    p = _tagged(_template("C", "C", ["1", "2"], footprint)(value=format_si(farads, "F")))
    p.mpn, p.torob, p.tol, p.bom_note = mpn, torob, tol, note

    def lines(n, k=1.0):
        if esr > 0:
            mid = f"{p.ref}_esr"
            return [f"R{p.ref}_esr {n['1']} {mid} {_num(esr)}", f"C{p.ref} {mid} {n['2']} {_num(farads * k)}"]
        return [f"C{p.ref} {n['1']} {n['2']} {_num(farads * k)}"]

    p.spice_lines = lines
    return p


# G6K terminal arrangement (pin name -> footprint pad), as drawn in Omron's
# G6K-2F-RF-V datasheet (top view). The -RF-V variant adds ground pads between
# these terminals; its footprint differs from the plain G6K-2F.
RELAY_PINS = {"COIL+": "1", "NC1": "2", "COM1": "3", "NO1": "4",
              "NO2": "5", "COM2": "6", "NC2": "7", "COIL-": "8"}
G6K_INSULATION_OHMS = 1e9  # Omron G6K datasheet: insulation resistance 1,000 MΩ min (500 VDC)


def relay_dpdt(state: str, contact_r: float = 0.05, open_c: float = 0.5e-12,
               footprint: str = "Relay_SMD:Relay_DPDT_Omron_G6K-2F", mpn: str = "G6K-2F-5VDC",
               torob: str = "G6K-2F") -> Part:
    if state not in ("released", "energised"):
        raise ValueError(f"relay state must be 'released' or 'energised', got {state!r}")
    p = _tagged(_template("G6K-2F", "K", RELAY_PINS, footprint)(value="G6K-2F"))
    p.mpn, p.torob, p.state, p.tol = mpn, torob, state, 0.0

    def lines(n, k=1.0):
        out = []
        for pole in ("1", "2"):
            com = n[f"COM{pole}"]
            closed, open_ = ("NC", "NO") if state == "released" else ("NO", "NC")
            out.append(f"R{p.ref}_{closed}{pole} {com} {n[closed + pole]} {_num(contact_r)}")
            out.append(f"C{p.ref}_{open_}{pole} {com} {n[open_ + pole]} {_num(open_c)}")
            # the insulation resistance also gives every contact node a DC path
            out.append(f"R{p.ref}_{open_}{pole}_ins {com} {n[open_ + pole]} {_num(G6K_INSULATION_OHMS)}")
        return out

    p.spice_lines = lines
    return p
