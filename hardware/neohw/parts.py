"""Part constructors: SKiDL parts that also know their SPICE model."""
from __future__ import annotations

import math
import re
from dataclasses import dataclass
from pathlib import Path
from typing import Callable

import numpy as np

from skidl import SKIDL, TEMPLATE, Part, Pin

from neohw import footprints, inductors
from neohw.availability import inductor_hit
from neohw.population import Variant, set_variants
from neohw.units import format_si, parse_si


# What identifies a part in a BOM: "mpn" (active parts, relays, crystals, oscillators, connectors,
# protection: a manufacturer part number) or "spec" (resistors, capacitors, inductors, LEDs and
# buttons: a full specification). Strict by default; `ident.violations` enforces it.
IDENT_MPN, IDENT_SPEC = "mpn", "spec"


def _tagged(part: Part, ident_kind: str = IDENT_MPN) -> Part:
    """A stable tag (the ref) instead of SKiDL's random one: quiet, reproducible netlists."""
    part.tag = part.ref
    part.ident_kind = ident_kind
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


def package_of(footprint: str) -> str:
    """The footprint's short name: `0805` from `Capacitor_SMD:C_0805_2012Metric`; other
    footprints keep their library name (`CP_Elec_8x10`)."""
    name = footprint.split(":")[-1]
    m = re.fullmatch(r"[RCL]_(\d{4})_\d+Metric", name)
    return m.group(1) if m else name


def _spaced(value: float, unit: str) -> str:
    """`47 pF`, `61.9 Ω`, `400 mA` (capacitors from 1 mF up stay in µF)."""
    if unit == "F" and value >= 1e-3:
        return f"{value * 1e6:g} µF"
    m = re.fullmatch(r"([0-9.]+)(\D*)", format_si(value, unit))
    return f"{m.group(1)} {m.group(2).replace('u', 'µ')}"


def _stated(note: str, fallback: str, *keywords: str) -> str:
    """The keyword (case-insensitive) that appears first in a part's note, else `fallback`."""
    low = note.lower()
    found = [(low.index(k.lower()), k) for k in keywords if k.lower() in low]
    return min(found)[1] if found else fallback


def resistor(value, footprint: str = "Resistor_SMD:R_0805_2012Metric", mpn: str | None = None,
             torob: str | None = None, tol: float = 0.01, c_par: float = 0.0) -> Part:
    """`c_par` adds the body's parallel capacitance to the SPICE model only (high-value
    resistors at RF)."""
    ohms = _positive(value)
    p = _tagged(_template("R", "R", ["1", "2"], footprint)(value=format_si(ohms, "Ω")), IDENT_SPEC)
    p.mpn, p.torob, p.tol = mpn, torob, tol
    p.package = package_of(footprint)
    p.spec = f"{_spaced(ohms, 'Ω')} ±{tol * 100:g} % {p.package}"

    def lines(n, k=1.0):
        out = [f"R{p.ref} {n['1']} {n['2']} {_num(ohms * k)}"]
        if c_par > 0:
            out.append(f"C{p.ref}_par {n['1']} {n['2']} {_num(c_par)}")
        return out

    p.spice_lines = lines
    return p


def default_voltage(farads: float, dielectric: str) -> str:
    """A buyable rating when none is given: large ceramics are not sold at 50 V (22 µF X5R 1206
    16 V, 47 µF 0805 10 V). Callers pass `voltage` for the rail when they know it."""
    if dielectric in ("C0G", "X7R", "X5R") and farads >= 47e-6:
        return "10 V"
    if dielectric in ("X7R", "X5R") and farads >= 4.7e-6:
        return "16 V"
    return "50 V"


CAP_CLASS_TOL = {"X7R": 0.10, "X5R": 0.10, "tantalum": 0.20, "aluminium electrolytic": 0.20}


def capacitor(value, footprint: str = "Capacitor_SMD:C_0805_2012Metric", mpn: str | None = None,
              torob: str | None = None, esr: float = 0.0, tol: float = 0.02, note: str = "",
              esl: float = 0.0, dielectric: str | None = None, voltage: str | None = None) -> Part:
    """`esr` and `esl` are SPICE-only parasitics (power filtering at MHz needs the ESL).

    `dielectric` and `voltage` go into the BOM specification. Left out, the dielectric is the one
    the note names (X7R, X5R, tantalum, aluminium electrolytic; else C0G) and the voltage is the
    "<n> V" in the note (else 50 V)."""
    farads = _positive(value)
    p = _tagged(_template("C", "C", ["1", "2"], footprint)(value=format_si(farads, "F")), IDENT_SPEC)
    p.mpn, p.torob, p.tol, p.bom_note = mpn, torob, tol, note
    dielectric = dielectric or _stated(note, "C0G", "X7R", "X5R", "tantalum", "aluminium electrolytic")
    if voltage is None:
        m = re.search(r"(\d+(?:\.\d+)?) V\b", note)
        voltage = f"{m.group(1)} V" if m else default_voltage(farads, dielectric)
    p.package = package_of(footprint)
    p.spec = capacitor_spec(farads, tol, p.package, dielectric, voltage)
    p.spice_lines = _capacitor_lines(farads, esr, esl, p)
    return p


def capacitor_spec(farads: float, tol: float, package: str, dielectric: str = "C0G", voltage: str | None = None) -> str:
    """The BOM specification of a capacitor (spec §2b)."""
    voltage = voltage or default_voltage(farads, dielectric)
    spec_tol = max(tol, CAP_CLASS_TOL.get(dielectric, 0.0))   # X7R/X5R are not sold tighter than ±10 %
    return f"{_spaced(farads, 'F')} ±{spec_tol * 100:g} % {dielectric} {package} {voltage}"


def _capacitor_lines(farads: float, esr: float, esl: float, part):
    """SPICE lines of a capacitor of `farads` on `part` (read for its ref when the deck is
    written): also the model of a population variant of another value on the same footprint."""
    def lines(n, k=1.0):
        ref = part.ref
        if esr <= 0 and esl <= 0:
            return [f"C{ref} {n['1']} {n['2']} {_num(farads * k)}"]
        out, node = [], n["1"]
        if esr > 0:      # internal node names as before ESL was added: existing netlists are unchanged
            out.append(f"R{ref}_esr {node} {ref}_esr {_num(esr)}")
            node = f"{ref}_esr"
        if esl > 0:
            out.append(f"L{ref}_esl {node} {ref}_esl {_num(esl)}")
            node = f"{ref}_esl"
        return out + [f"C{ref} {node} {n['2']} {_num(farads * k)}"]
    return lines


# G6K terminal arrangement (pin name -> footprint pad), as drawn in Omron's
# G6K-2F-RF-V datasheet (top view). The -RF-V variant adds ground pads between
# these terminals; its footprint differs from the plain G6K-2F.
RELAY_PINS = {"COIL+": "1", "NC1": "2", "COM1": "3", "NO1": "4",
              "NO2": "5", "COM2": "6", "NC2": "7", "COIL-": "8"}
G6K_INSULATION_OHMS = 1e9  # Omron G6K datasheet: insulation resistance 1,000 MΩ min (500 VDC)


# Not published for the plain G6K-2F. Omron gives >= 30 dB isolation between poles at 1 GHz
# for the G6K-2F-RF (about 0.05 pF in a 50-ohm system), which has grounded pins between the
# contacts; the plain part is assumed twice as bad. Rev 0 measures it.
G6K_INTER_POLE_C = 0.1e-12


def relay_dpdt(state: str, contact_r: float = 0.05, open_c: float = 0.5e-12,
               inter_pole_c: float = G6K_INTER_POLE_C,
               footprint: str = "Relay_SMD:Relay_DPDT_Omron_G6K-2F", mpn: str = "G6K-2F-5VDC",
               torob: str = "G6K-2F") -> Part:
    if state not in ("released", "energised"):
        raise ValueError(f"relay state must be 'released' or 'energised', got {state!r}")
    p = _tagged(_template("G6K-2F", "K", RELAY_PINS, footprint)(value="G6K-2F"))
    p.mpn, p.torob, p.state, p.tol = mpn, torob, state, 0.0
    p.nc_pins = ("COIL+", "COIL-")   # coil drive is phase 3; no SPICE element
    p.bom_note = f"inter-pole C {inter_pole_c * 1e12:g} pF assumed"

    def lines(n, k=1.0):
        out = []
        for pole in ("1", "2"):
            com = n[f"COM{pole}"]
            closed, open_ = ("NC", "NO") if state == "released" else ("NO", "NC")
            out.append(f"R{p.ref}_{closed}{pole} {com} {n[closed + pole]} {_num(contact_r)}")
            out.append(f"C{p.ref}_{open_}{pole} {com} {n[open_ + pole]} {_num(open_c)}")
            # the insulation resistance also gives every contact node a DC path
            out.append(f"R{p.ref}_{open_}{pole}_ins {com} {n[open_ + pole]} {_num(G6K_INSULATION_OHMS)}")
        for contact in ("COM", "NO", "NC"):   # stray C between the poles' matching contacts
            out.append(f"C{p.ref}_x{contact} {n[contact + '1']} {n[contact + '2']} {_num(inter_pole_c)}")
        return [line for line in out if line.split()[1] != line.split()[2]]  # drop node-to-itself

    p.spice_lines = lines
    return p


# Axial colour-ring (moulded) inductors, measured and selected with an LCR meter.
INDUCTOR_FOOTPRINT = "Inductor_THT:L_Axial_L6.6mm_D2.7mm_P10.16mm_Horizontal_Vishay_IM-2"
INDUCTOR_C_PAR = 2.5e-12  # SRF: 1 µH ≈ 100 MHz, 10 µH ≈ 32 MHz, 100 µH ≈ 10 MHz (IM-2 class minimums)


def inductor(value, q: float, q_freq_hz: float, tol: float = 0.03, trim: float = 1.0,
             mpn: str | None = None, torob: str | None = None, note: str | None = None,
             c_par: float = INDUCTOR_C_PAR, dcr: float | None = None,
             footprint: str = INDUCTOR_FOOTPRINT, construction: str | None = None,
             current: str | None = None) -> Part:
    """L + series R (giving `q` at `q_freq_hz`) with a parallel C for the self-resonance.

    `value` is the part to buy. `trim` scales it to what the builder selects with an LCR
    meter (a pair is picked so its sum hits the design target); `tol` is the selection
    tolerance. Q is assumed; both are stated in the BOM note.

    With `dcr` (chokes that carry DC), the series R is the DC resistance and the Q loss
    becomes a parallel resistor Q·2πf·L instead.

    The BOM specification states the minimum Q at `q_freq_hz`, the minimum self-resonance the
    model assumes (from `c_par`) and a rated current. `construction` and `current` default to
    what the footprint and the note say (moulded axial, 100 mA when the note gives none).
    """
    henries = _positive(value)
    if q <= 0 or q_freq_hz <= 0:
        raise ValueError(f"q and q_freq_hz must be positive, got {q!r}, {q_freq_hz!r}")
    if not 0.9 <= trim <= 1.1:
        raise ValueError(f"trim {trim:.3f} is outside the ±10 % a selected part can reach")
    henries_built = henries * trim
    r_s = 2 * math.pi * q_freq_hz * henries_built / q
    p = _tagged(_template("L", "L", ["1", "2"], footprint)(value=format_si(henries, "H")), IDENT_SPEC)
    p.mpn, p.torob, p.tol, p.dcr = mpn, torob, tol, dcr
    p.torob_match = lambda name: inductor_hit(name, henries)
    p.bom_note = note or f"select to ±{tol * 100:g} % with LCR meter; Q={q:g} assumed"
    p.package = package_of(footprint)
    if construction is None:
        construction = ("axial moulded" if "Axial" in footprint else "wirewound" if "1210" in footprint
                        else "shielded power" if "12x12" in footprint else "radial drum" if "Radial" in footprint
                        else "wirewound")
    if current is None:
        m = re.search(r"(?:rated|Isat) ≥ ?([\d.]+) ?(m?A)", p.bom_note)
        current = f"I≥{m.group(1)} {m.group(2)}" if m else "I≥100 mA"
    srf = 1 / (2 * math.pi * math.sqrt(henries * c_par))
    p.spec = (f"{_spaced(henries, 'H')} ±{tol * 100:g} % {p.package} {construction}, "
              f"Q≥{q:g} at {_spaced(q_freq_hz, 'Hz')}, SRF≥{_spaced(math.floor(srf / 1e5) * 1e5, 'Hz')}, {current}")

    def lines(n, k=1.0):
        mid = f"{p.ref}_m"
        out = [f"L{p.ref} {n['1']} {mid} {_num(henries_built * k)}",
               f"R{p.ref} {mid} {n['2']} {_num((dcr if dcr is not None else r_s) * k)}",
               f"C{p.ref}_p {n['1']} {n['2']} {_num(c_par)}"]
        if dcr is not None:
            out.append(f"R{p.ref}_q {n['1']} {n['2']} {_num(q * 2 * math.pi * q_freq_hz * henries_built * k)}")
        return out

    p.spice_lines = lines
    return p


SMD_INDUCTOR_TOL = 0.05           # China: TDK NL453232 / NLV32 J grade, fitted as bought
FILTER_INDUCTOR_TOL_IRAN = 0.03   # Iran: Bourns 78F class, selected with an LCR meter


def _datasheet_lines(r, henries_built: float, f_eval_hz: float, part):
    """L + series R at `f_eval_hz` + parallel C for the minimum self-resonance, from a datasheet
    row (neohw.inductors). `part` is read for its ref when the deck is written."""
    r_s, c_p = inductors.r_series(r, f_eval_hz), inductors.c_par(r)

    def lines(n, k=1.0):
        mid = f"{part.ref}_m"
        return [f"L{part.ref} {n['1']} {mid} {_num(henries_built * k)}",
                f"R{part.ref} {mid} {n['2']} {_num(r_s * k)}",
                f"C{part.ref}_p {n['1']} {n['2']} {_num(c_p)}"]
    return lines


def _datasheet_ident(r, tol: float, how: str) -> str:
    info = inductors.series_info(r.series)
    return (f"{_spaced(r.henries, 'H')} ±{tol * 100:g} % {how}, Q≥{r.q_min:g} at {_spaced(r.f_test_hz, 'Hz')}, "
            f"SRF≥{_spaced(r.srf_min_hz, 'Hz')}, DCR≤{r.dcr_max:g} Ω, {inductors.CURRENT_SPEC} "
            f"({info['maker']} {r.mpn} or equivalent)")


def filter_inductor(value, f_eval_hz: float, trim_iran: float, note: str | None = None) -> Part:
    """A band-pass filter inductor on a combined SMD/axial footprint (neohw.footprints).

    China: a TDK NL453232 (1812, from 1 µH) or NLV32 (1210, below) wirewound chip, fitted as
    bought (±5 %). Iran: a Bourns 78F axial choke selected with an LCR meter (±3 %, scaled by
    `trim_iran` so a pair hits the design target). Both are modelled from their guaranteed
    datasheet numbers at `f_eval_hz` (neohw.inductors)."""
    henries = _positive(value)
    if not 0.9 <= trim_iran <= 1.1:
        raise ValueError(f"trim {trim_iran:.3f} is outside the ±10 % a selected part can reach")
    china_row = inductors.row(inductors.china_series(henries), henries)
    iran_row = inductors.row(inductors.IRAN, henries)
    china_pkg = inductors.series_info(china_row.series)["package"]
    footprint = f"NeoRadio:{footprints.combined_inductor_name(china_pkg)}"
    p = _tagged(_template("L", "L", ["1", "2"], footprint)(value=format_si(henries, "H")), IDENT_SPEC)
    p.mpn, p.torob, p.tol = None, None, FILTER_INDUCTOR_TOL_IRAN
    p.torob_match = lambda name: inductor_hit(name, henries)
    p.bom_note = note or f"select to ±{FILTER_INDUCTOR_TOL_IRAN * 100:g} % with LCR meter"
    p.package = package_of(footprint)
    p.spec = _datasheet_ident(iran_row, FILTER_INDUCTOR_TOL_IRAN, f"axial on {p.package}, selected with an LCR meter")
    p.spice_lines = _datasheet_lines(iran_row, henries * trim_iran, f_eval_hz, p)
    china = Variant(p.value, _datasheet_ident(china_row, SMD_INDUCTOR_TOL, f"{china_pkg} wirewound"),
                    f"{china_pkg} wirewound, fitted as bought",
                    spice_lines=_datasheet_lines(china_row, henries, f_eval_hz, p), tol=SMD_INDUCTOR_TOL)
    iran = Variant(p.value, p.spec, p.bom_note, spice_lines=p.spice_lines, tol=FILTER_INDUCTOR_TOL_IRAN,
                   torob=p.torob, torob_match=p.torob_match)
    return set_variants(p, china, iran)


MODELS_DIR = Path(__file__).resolve().parents[1] / "models"


def model_text(name: str) -> str:
    """Contents of a committed SPICE model file (hardware/models/)."""
    path = MODELS_DIR / name
    if not path.exists():
        raise FileNotFoundError(f"SPICE model {path} is missing (it is committed in hardware/models/)")
    return path.read_text().strip()


BFR92A_IC_MAX = 25e-3   # NXP BFR92A absolute maximum collector current (the BFR92P model is rated 45 mA)


def npn_bfr92a(mpn: str = "BFR92A", torob: str | None = "BFR92", bf: float | None = None) -> Part:
    """NXP BFR92A, SOT-23 (pin 1 base, 2 emitter, 3 collector); BFR92P model.

    `bf` overrides the model's forward current gain (94.7) for hFE corner runs; each value
    gets its own subcircuit name."""
    p = _tagged(_template("BFR92A", "Q", {"B": "1", "E": "2", "C": "3"},
                          "Package_TO_SOT_SMD:SOT-23")(value="BFR92A"))
    p.mpn, p.torob, p.tol = mpn, torob, 0.0
    model, name = model_text("bfr92p.lib"), "BFR92P"
    if bf is not None:
        name = f"BFR92P_bf{bf:g}"
        model = (model.replace("BF=94.733", f"BF={bf:g}").replace("BFR92P_chip", f"{name}_chip")
                 .replace(".subckt BFR92P ", f".subckt {name} ").replace(".ends BFR92P", f".ends {name}"))
    p.spice_models = (model,)
    p.spice_lines = lambda n, k=1.0: [f"X{p.ref} {n['C']} {n['B']} {n['E']} {name}"]
    return p


def pnp_bc857b(mpn: str = "BC857B", torob: str | None = "BC857B") -> Part:
    """Nexperia BC857B general-purpose PNP, SOT-23 (pin 1 base, 2 emitter, 3 collector)."""
    p = _tagged(_template("BC857B", "Q", {"B": "1", "E": "2", "C": "3"},
                          "Package_TO_SOT_SMD:SOT-23")(value="BC857B"))
    p.mpn, p.torob, p.tol = mpn, torob, 0.0
    p.spice_models = (model_text("bc857b.lib"),)
    p.spice_lines = lambda n, k=1.0: [f"X{p.ref} {n['C']} {n['B']} {n['E']} BC857B"]
    return p


def diode_pair_bav99(mpn: str = "BAV99", torob: str | None = "BAV99") -> Part:
    """BAV99, SOT-23: pin 1 anode of D1, pin 3 = D1 cathode + D2 anode, pin 2 cathode of D2."""
    p = _tagged(_template("BAV99", "D", {"A": "1", "K": "2", "COM": "3"},
                          "Package_TO_SOT_SMD:SOT-23")(value="BAV99"))
    p.mpn, p.torob, p.tol = mpn, torob, 0.0
    p.spice_models = (model_text("bav99.lib"),)
    p.nc_pins = ("COM",)   # as a two-diode series string the middle pin stays open
    p.spice_lines = lambda n, k=1.0: [f"D{p.ref}a {n['A']} {n['COM']} D1N4148",
                                      f"D{p.ref}b {n['COM']} {n['K']} D1N4148"]
    return p


GDT_MPN = "2038-09-SM-RPLF"   # Bourns 90 V two-electrode SMD tube


def gdt(capacitance: float = 1.5e-12, torob: str | None = "گازی برقگیر GDT") -> Part:
    """Gas discharge tube at the connector: an open circuit with its capacitance below the
    breakdown voltage (only small-signal behaviour is simulated). Footprint is project-local,
    created in the layout phase."""
    p = _tagged(_template("GDT", "GDT", ["1", "2"], "NeoRadio:GDT_SMD_2pin")(value="GDT 90V"))
    p.mpn, p.torob, p.tol = GDT_MPN, torob, 0.0
    p.bom_note = "90 V two-electrode SMD gas discharge tube, or equivalent"
    p.spice_lines = lambda n, k=1.0: [f"C{p.ref} {n['1']} {n['2']} {_num(capacitance)}",
                                      f"R{p.ref} {n['1']} {n['2']} {_num(G6K_INSULATION_OHMS)}"]
    return p



FST3253_PINS = {"/1OE": "1", "S1": "2", "1B3": "3", "1B2": "4", "1B1": "5", "1B0": "6", "1A": "7",
                "GND": "8", "2A": "9", "2B0": "10", "2B1": "11", "2B2": "12", "2B3": "13", "S0": "14",
                "/2OE": "15", "VCC": "16"}


def fst3253(mpn: str = "74FST3253", torob: str | None = "FST3253") -> Part:
    """onsemi/Fairchild FST3253 dual 4:1 bus switch, SOIC-16 (VCC 4.0–5.5 V). Section 2 may
    stay unconnected except /2OE."""
    p = _tagged(_template("FST3253", "U", FST3253_PINS, "Package_SO:SOIC-16_3.9x9.9mm_P1.27mm")(value="FST3253"))
    p.mpn, p.torob, p.tol = mpn, torob, 0.0
    p.spice_models = (model_text("fst3253.lib"),)
    p.nc_pins = ("2A", "2B0", "2B1", "2B2", "2B3")
    order = ["/1OE", "S1", "1B3", "1B2", "1B1", "1B0", "1A", "GND", "2A", "2B0", "2B1", "2B2", "2B3",
             "S0", "/2OE", "VCC"]
    p.spice_lines = lambda n, k=1.0: [f"X{p.ref} " + " ".join(n[x] for x in order) + " FST3253"]
    return p


LM4562_PINS = {"OUTA": "1", "-INA": "2", "+INA": "3", "V-": "4", "+INB": "5", "-INB": "6", "OUTB": "7", "V+": "8"}


def lm4562_dual(mpn: str = "LM4562MAX", torob: str | None = "LM4562") -> Part:
    """TI LM4562 dual op-amp, SOIC-8 (datasheet macromodel, models/lm4562.lib)."""
    p = _tagged(_template("LM4562", "U", LM4562_PINS, "Package_SO:SOIC-8_3.9x4.9mm_P1.27mm")(value="LM4562"))
    p.mpn, p.torob, p.tol = mpn, torob, 0.0
    p.spice_models = (model_text("lm4562.lib"),)
    p.spice_lines = lambda n, k=1.0: [
        f"X{p.ref}a {n['+INA']} {n['-INA']} {n['OUTA']} {n['V+']} {n['V-']} LM4562",
        f"X{p.ref}b {n['+INB']} {n['-INB']} {n['OUTB']} {n['V+']} {n['V-']} LM4562"]
    return p


CS5361_PINS = {"RST": "1", "M/S": "2", "LRCK": "3", "SCLK": "4", "MCLK": "5", "VD": "6", "GND": "7",
               "VL": "8", "SDOUT": "9", "MDIV": "10", "HPF": "11", "I2S/LJ": "12", "M0": "13", "M1": "14",
               "OVFL": "15", "AINL+": "16", "AINL-": "17", "GND2": "18", "VA": "19", "AINR-": "20",
               "AINR+": "21", "VQ": "22", "REFGND": "23", "FILT+": "24"}
CS5361_ZIN_DIFF = 7.5e3      # datasheet minimum differential input impedance
CS5361_FS_VPP = 1.13         # full-scale differential input, x VA (Vpp)
CS5361_DR_40K_DB = 108.0     # dynamic range, double-speed mode, 40 kHz bandwidth unweighted, typical (DS467)


def cs5361(mpn: str = "CS5361-KZZ", torob: str | None = "CS5361") -> Part:
    """Cirrus CS5361 ADC, TSSOP-24. Only the analog input is simulated: 7.5 kΩ differential per
    channel and VQ at 2.5 V; digital and supply pins are left to phase 3 (no SPICE elements)."""
    p = _tagged(_template("CS5361", "U", CS5361_PINS, "Package_SO:TSSOP-24_4.4x7.8mm_P0.65mm")(value="CS5361"))
    p.mpn, p.torob, p.tol = mpn, torob, 0.0
    p.nc_pins = tuple(x for x in CS5361_PINS if x not in ("AINL+", "AINL-", "AINR+", "AINR-", "VQ", "GND"))
    p.spice_lines = lambda n, k=1.0: [
        f"R{p.ref}_l {n['AINL+']} {n['AINL-']} {_num(CS5361_ZIN_DIFF)}",
        f"R{p.ref}_r {n['AINR+']} {n['AINR-']} {_num(CS5361_ZIN_DIFF)}",
        f"V{p.ref}_vq {p.ref}_vqi {n['GND']} dc 2.5",
        f"R{p.ref}_vq {p.ref}_vqi {n['VQ']} 25"]
    return p


def _ic(value: str, pins: dict[str, str], footprint: str, mpn: str | None, torob: str | None, nc=(),
        spec: str | None = None, ident_kind: str = IDENT_MPN) -> Part:
    """An IC used for the netlist and BOM only (board-level blocks are not simulated in SPICE).
    `spec` identifies a part no manufacturer number fits (an LED, a crystal) by its full specification."""
    p = _tagged(_template(value, "U", pins, footprint)(value=value), ident_kind)
    p.mpn, p.torob, p.tol = mpn, torob, 0.0
    if spec:
        p.spec = spec
    p.package = package_of(footprint)
    p.nc_pins = tuple(nc)
    return p


SI5351C_GM1_PINS = {"XA": "1", "XB": "2", "OEB": "3", "SCL": "4", "SDA": "5", "CLKIN": "6", "CLK1": "7",
                    "VDDOB": "8", "VDDOA": "9", "CLK0": "10", "VDDOD": "11", "CLK3": "12", "CLK2": "13",
                    "VDDOC": "14", "GND": "15", "VDD": "16", "PAD": "17"}


def si5351c() -> Part:
    """Silicon Labs Si5351C-B-GM1: 16-QFN 3×3 mm, 4 outputs, crystal and CLKIN (datasheet Rev. 1.3,
    §10.6). The 20-QFN 8-output Si5351C-B-GM is not stocked on Torob; four outputs suffice."""
    return _ic("Si5351C", SI5351C_GM1_PINS, "Package_DFN_QFN:QFN-16-1EP_3x3mm_P0.5mm_EP1.75x1.75mm",
               "Si5351C-B-GM1", "SI5351C")


_FA_DIGITS = str.maketrans("۰۱۲۳۴۵۶۷۸۹", "0123456789")


def crystal_25mhz_hit(name: str) -> bool:
    """A Torob listing is this crystal if it names 25 MHz and the 3225 package and is not an
    oscillator (Persian "اسیلاتور", "OSC", or an ECS-…MV oscillator part number)."""
    n = name.translate(_FA_DIGITS).upper().replace(" ", "")
    is_25mhz = "25MHZ" in n or "25مگاهرتز" in n or "25.000MHZ" in n
    oscillator = "اسیلاتور" in name or "OSC" in n or "MV-" in n
    return is_25mhz and "3225" in n and not oscillator


def crystal_25mhz() -> Part:
    """25 MHz fundamental crystal, 3225 4-pad (pads 2 and 4 are case ground). The Si5351 supplies
    the load capacitance internally (6/8/10 pF, register 183), so the crystal's CL must be one of
    those: buy CL ≤ 10 pF (generic 3225 crystals are often 12–20 pF, which pulls the frequency by
    tens of ppm)."""
    # pin names must not be digits: SKiDL resolves x["2"] as pad number 2 (a case-ground pad)
    p = _ic("25MHz", {"X1": "1", "GND1": "2", "X2": "3", "GND2": "4"}, "Crystal:Crystal_SMD_3225-4Pin_3.2x2.5mm",
            "NX3225GA-25.000M-STD-CRG-2", "کریستال 25MHZ 3225")
    p.torob_match = crystal_25mhz_hit
    p.bom_note = ("NDK NX3225GA, 25 MHz, CL 8 pF, ±20 ppm, ESR 50 Ω max, or equivalent "
                  "(CL ≤ 10 pF: Si5351 internal load 6/8/10 pF)")
    return p


LVC74_PINS = {"/1RD": "1", "1D": "2", "1CP": "3", "/1SD": "4", "1Q": "5", "/1Q": "6", "GND": "7", "/2Q": "8",
              "2Q": "9", "/2SD": "10", "2CP": "11", "2D": "12", "/2RD": "13", "VCC": "14"}


def lvc74(nc=()) -> Part:
    """Nexperia 74LVC74A dual D flip-flop, SO-14 (datasheet Rev. 13, Table 2)."""
    return _ic("74LVC74", LVC74_PINS, "Package_SO:SOIC-14_3.9x8.7mm_P1.27mm", "74LVC74AD", "74LVC74", nc)


LVC157_PINS = {"S": "1", "1I0": "2", "1I1": "3", "1Y": "4", "2I0": "5", "2I1": "6", "2Y": "7", "GND": "8",
               "3Y": "9", "3I1": "10", "3I0": "11", "4Y": "12", "4I1": "13", "4I0": "14", "/E": "15", "VCC": "16"}


def lvc157(nc=()) -> Part:
    """Nexperia 74LVC157A quad 2-input multiplexer, SO-16 (datasheet Rev. 13, Table 2)."""
    return _ic("74LVC157", LVC157_PINS, "Package_SO:SOIC-16_3.9x9.9mm_P1.27mm", "74LVC157AD", "74LVC157", nc)


# --- power (phase 3 spec §3.2) -----------------------------------------------------------

def barrel_jack() -> Part:
    """5.5/2.1 mm DC jack (DC-005 style): 1 tip (+), 2 sleeve (−), 3 switch (unused)."""
    p = _ic("DC jack 5.5/2.1", {"TIP": "1", "SLEEVE": "2", "SW": "3"}, "Connector_BarrelJack:BarrelJack_Horizontal",
            "PJ-002A", "جک آداپتور 5.5 2.1", nc=("SW",))
    p.bom_note = "CUI/Same Sky 5.5 × 2.1 mm horizontal DC jack (2.0 mm centre pin), or equivalent"
    p.spice_lines = lambda n, k=1.0: []          # the source is the stimulus
    return p


PTC_HOLD_A, PTC_TRIP_A, PTC_V_MAX, PTC_OHM = 0.75, 1.5, 33.0, 0.3


def ptc_resettable() -> Part:
    """Resettable PTC fuse (polyfuse), 1812: hold 0.75 A, trip 1.5 A, 33 V maximum (above the 20 V input),
    ≈ 0.3 Ω. It ignores millisecond plug-in pulses (it trips on heat, in seconds, at a sustained
    fault) and drops only ≈ 0.1 V at the board's 0.45 A maximum input, so the board starts at 9 V
    whatever the temperature. A passive: the manufacturer sources it (e.g. a 1812L075/33-class part)."""
    p = _tagged(_template("PTC", "F", ["1", "2"], "Fuse:Fuse_1812_4532Metric")(value="PTC 0.75A 33V"))
    p.mpn, p.torob, p.tol, p.bom_note = "1812L075/33DR", None, 0.3, "resettable fuse, hold 0.75 A, trip 1.5 A, ≥ 33 V, or equivalent"
    p.package = package_of("Fuse:Fuse_1812_4532Metric")
    p.spice_lines = lambda n, k=1.0: [f"R{p.ref} {n['1']} {n['2']} {PTC_OHM:g}"]
    return p


SS36_IFSM_A = 80.0             # non-repetitive peak forward surge (8.3 ms half sine), conservative end of the family


def ss36() -> Part:
    """3 A 60 V Schottky, SMC/SMB-class DO-214AB (K 1, A 2): the input's reverse-polarity diode and the
    buck's catch diode. Fit: 0.45 V at 0.3 A, 0.7 V at 3 A (datasheet typical)."""
    p = _ic("SS36", {"K": "1", "A": "2"}, "Diode_SMD:D_SMC", "SS36", "SS36")
    p.spice_models = (".model SS36_D d(is=5e-7 n=1.05 rs=0.06 cjo=150p bv=60)",)
    p.spice_lines = lambda n, k=1.0: [f"D{p.ref} {n['A']} {n['K']} SS36_D"]
    return p


SMBJ20A_CLAMP_V, SMBJ20A_IPP_A, SMBJ20A_RS = 32.4, 18.6, 0.465


def smbj20a() -> Part:
    """SMBJ20A 600 W TVS, SMB / DO-214AA (K 1, A 2), unidirectional. SMBJ series datasheet, SMBJ20A row
    (Littelfuse-family table; Born Semiconductor "SMBJ Series" rev. 2018, same figures as the
    Littelfuse DigiKey listing for the A-suffix parts): stand-off VR 20 V, breakdown VBR 22.2–24.5 V
    at 1 mA, clamp VC 32.4 V at IPP 18.6 A (10/1000 µs), leakage ≤ 1 µA at VR. The 20 V maximum
    input sits at the stand-off; the clamp is under the buck's 60 V rating and the PTC's 33 V.
    SPICE: breakdown at the typical 23.35 V (middle of the VBR range), series resistance chosen so the
    clamp at IPP is 32.4 V."""
    p = _ic("SMBJ20A", {"K": "1", "A": "2"}, "Diode_SMD:D_SMB", "SMBJ20A", "SMBJ20A")
    p.spice_models = (f".model SMBJ20_D d(is=1e-12 n=1.5 bv=23.35 ibv=1m rs={SMBJ20A_RS:g} cjo=700p)",)
    p.spice_lines = lambda n, k=1.0: [f"D{p.ref} {n['A']} {n['K']} SMBJ20_D"]
    return p


LM2576_VREF, LM2576_EFFICIENCY, LM2576_VSAT = 1.23, 0.77, 1.4


def lm2576hv() -> Part:
    """TI/ON LM2576HVS-ADJ 3 A buck, 52 kHz, 60 V input (63 V abs.), TO-263-5 (1 VIN, 2 OUTPUT, 3 GND
    and tab, 4 FEEDBACK, 5 /ON-OFF: low = on). Feedback 1.23 V. Torob: buy listings not marked
    "copy" (a clone's input rating cannot be trusted).
    DC model (averaged): the regulator pushes current into the switch node until FEEDBACK reaches
    1.23 V; the inductor carries it (its DC resistance included); VIN supplies that power over the
    efficiency η = 0.77 (datasheet curve, 5 V out, 12–30 V in, 0.5 A); the switch cannot raise the
    output above VIN − 1.4 V (saturation)."""
    p = _ic("LM2576HV-ADJ", {"VIN": "1", "OUT": "2", "GND": "3", "FB": "4", "ONOFF": "5"},
            "Package_TO_SOT_SMD:TO-263-5_TabPin3", "LM2576HVS-ADJ", "LM2576HV")
    def lines(n, k=1.0):
        reg = (f"max(0, min(3, 50 * ({LM2576_VREF:g} - v({n['FB']})))) "
               f"* 0.5 * (1 + tanh(10 * (v({n['VIN']}) - v({n['OUT']}) - {LM2576_VSAT:g})))")
        return [f"B{p.ref}_reg {n['GND']} {n['OUT']} I = {reg}",
                f"B{p.ref}_in {n['VIN']} {n['GND']} I = {reg} * max(v({n['OUT']}), 0) / "
                f"({LM2576_EFFICIENCY:g} * max(v({n['VIN']}), 1))",
                f"R{p.ref}_on {n['ONOFF']} {n['GND']} 1e9"]
    p.spice_lines = lines
    return p


@dataclass(frozen=True)
class LdoData:
    """What the power check reads from a linear regulator: the dropout (V, maximum at the datasheet
    load), the junction-to-ambient thermal resistance (°C/W) and the ripple rejection at f (dB)."""
    dropout_v: float
    theta_ja: float
    psrr_db: Callable


def _psrr_curve(points, floor_db: float = 15.0, slope_db_dec: float = 20.0):
    """Ripple rejection through datasheet `points` ((Hz, dB), ...), interpolated on log f. Held at
    the first point below it; above the last point it falls 20 dB/decade to `floor_db` (the
    datasheets stop at 1 MHz; the board's spurs reach 50 MHz)."""
    lf = np.log10([f for f, _ in points])
    db = np.array([d for _, d in points], dtype=float)

    def psrr_db(f):
        x = np.log10(np.maximum(f, 1.0))
        inside = np.interp(x, lf, db)
        beyond = db[-1] - slope_db_dec * (x - lf[-1])
        return np.clip(np.where(x > lf[-1], beyond, inside), floor_db, None)
    return psrr_db


def _ldo_part(p: Part, v_out: float, data: LdoData) -> Part:
    p.v_out, p.ldo = v_out, data
    p.dropout_v, p.theta_ja, p.psrr_db = data.dropout_v, data.theta_ja, data.psrr_db
    return p


# MIC5205: 75 dB at 100 Hz (datasheet DS20005785 Table 1-1), assumed -20 dB/decade above 1 kHz to a
# 15 dB floor (the datasheet's curves are not tabulated); θJA 220 °C/W on the recommended minimum
# footprint (Table 4-1; 170 °C/W on square copper); dropout 165 mV at 150 mA.
MIC5205_PSRR_DB = lambda f: np.clip(75 - 20 * np.log10(np.maximum(f, 1e3) / 1e3), 15, 75)
MIC5205_DATA = LdoData(0.165, 220.0, MIC5205_PSRR_DB)
# TPS7A20 (SBVS338H): dropout 145 mV max at 300 mA, 2.5–5.5 V out, DBV (Section 5.5); θJA 187.1 °C/W
# DBV (Section 5.4); PSRR at 300 mA, VIN = VOUT + 1 V (Section 5.5): 92 dB at 1 kHz, 60 dB at 100 kHz,
# 40 dB at 1 MHz (typical; Figures 5-61 to 5-64 show the same curve)
TPS7A20_DATA = LdoData(0.145, 187.1, _psrr_curve([(1e3, 92.0), (1e5, 60.0), (1e6, 40.0)]))
# TLV757P (SBVS322C): dropout 475 mV max at 1 A, 3.3–5.0 V out (Section 5.5, the load-independent bound);
# θJA DBV 231.1 °C/W on the JEDEC board (Section 5.4; 100.8 °C/W on TI's EVM); PSRR at 50 mA,
# VIN = VOUT + 1 V (Section 5.5): 52 dB at 1 kHz, 46 dB at 100 kHz, 52 dB at 1 MHz (Figures 5-1 to 5-3)
TLV757P_DATA = LdoData(0.475, 231.1, _psrr_curve([(1e3, 52.0), (1e5, 46.0), (1e6, 52.0)]))
# AMS1117 (Advanced Monolithic Systems DS1117): dropout 1.3 V max at 0.8 A; θJA SOT-223 90 °C/W
# (ϕJA, Absolute Maximum Ratings, "46 °C/W to > 90 °C/W depending on mounting"); ripple rejection 60 dB
# min at 120 Hz (75 typ, 22 µF tantalum, 1 A); assumed -20 dB/decade above that to a 15 dB floor
AMS1117_DATA = LdoData(1.3, 90.0, lambda f: np.clip(60 - 20 * np.log10(np.maximum(f, 120.0) / 120.0), 15, 60))


def ams1117_33() -> Part:
    """AMS1117-3.3, SOT-223 (GND 1, OUT 2 and tab, IN 3; Advanced Monolithic DS1117, "Pin Connections").
    From 5.5 V its dropout (1.3 V max) is not a limit; the SOT-223 tab carries its 0.33 W (≈ 90 °C/W)."""
    p = _ic("AMS1117-3.3", {"GND": "1", "OUT": "2", "IN": "3"}, "Package_TO_SOT_SMD:SOT-223-3_TabPin2",
            "AMS1117-3.3", "AMS1117-3.3")
    _ldo_part(p, 3.3, AMS1117_DATA)
    p.spice_lines = lambda n, k=1.0: _ldo_lines(p.ref, n, 3.3, AMS1117_DATA.dropout_v)
    return p


def _mic5205_torob(v_out: float) -> str:
    return "MIC5205-3.3" if v_out < 4 else "MIC5205YM5"


def mic5205(v_out: float) -> Part:
    """Microchip MIC5205 150 mA LDO, SOT-23-5 (IN 1, GND 2, EN 3, BYP 4, OUT 5). Dropout 165 mV
    at 150 mA (110 mV at 50 mA); ripple rejection 75 dB at 100 Hz (datasheet DS20005785; no
    curve in the text, so the ripple analysis assumes a roll-off); θJA 220 °C/W. DC model: OUT =
    min(v_out, V_in − dropout) behind 50 mΩ; input current = output current."""
    p = _ic(f"MIC5205-{v_out:.1f}", {"IN": "1", "GND": "2", "EN": "3", "BYP": "4", "OUT": "5"},
            "Package_TO_SOT_SMD:SOT-23-5", f"MIC5205-{v_out:.1f}YM5", _mic5205_torob(v_out))
    _ldo_part(p, v_out, MIC5205_DATA)
    p.spice_lines = lambda n, k=1.0: _ldo_lines(p.ref, n, v_out, MIC5205_DATA.dropout_v, "BYP")
    return p


def tps7a20(v_out: float) -> Part:
    """TI TPS7A20 300 mA ultra-low-noise LDO, SOT-23-5 DBV (IN 1, GND 2, EN 3, NC 4, OUT 5): the China
    fitting of the RF supplies. Pins from the datasheet SBVS338H (July 2024), Figure 4-4 "DBV Package,
    5-Pin SOT-23 (Top View)" and the Pin Functions table ("N/C, SOT-23 pin 4: No internal electrical
    connection"), fetched from ti.com. KiCad's Regulator_Linear library has only the DQN symbol, so it
    is not a source here. Pin 4 is the MIC5205's BYP: the board leaves it open in China.
    Input 1.6–6.0 V (6.5 V absolute), output capacitor 1–200 µF effective, ESR ≤ 100 mΩ.
    Ripple rejection, dropout and θJA: `TPS7A20_DATA`. Only 5.0 V (TPS7A2050PDBVR) and 3.3 V
    (TPS7A2033PDBVR) are built."""
    mpn = {5.0: "TPS7A2050PDBVR", 3.3: "TPS7A2033PDBVR"}[round(v_out, 1)]
    p = _ic(f"TPS7A20-{v_out:.1f}", {"IN": "1", "GND": "2", "EN": "3", "NC": "4", "OUT": "5"},
            "Package_TO_SOT_SMD:SOT-23-5", mpn, None, nc=("NC",))
    _ldo_part(p, v_out, TPS7A20_DATA)
    p.spice_lines = lambda n, k=1.0: _tps7a20_lines(p.ref, n, v_out)
    return p


def _tps7a20_lines(ref: str, n: dict, v_out: float) -> list[str]:
    """DC model: OUT = min(v_out, V_in − dropout) behind 50 mΩ; EN has a 500 kΩ pulldown."""
    return _ldo_lines(ref, n, v_out, TPS7A20_DATA.dropout_v, "NC", en_r=500e3)


def tlv75733() -> Part:
    """TI TLV75733P 1 A LDO, 3.3 V, SOT-23-5 DBV (IN 1, GND 2, EN 3, NC 4, OUT 5): the China 3V3_D
    regulator. Pins from the datasheet SBVS322C (March 2024), Figure 4-1 "DBV Package, 5-Pin SOT-23
    (Top View)" and Table 4-1 ("NC, DBV pin 4: No internal connection"), fetched from ti.com;
    KiCad's Regulator_Linear TLV75733PDBV symbol has the same footprint. Input 1.45–5.5 V
    recommended (6.0 V absolute), 1 µF minimum on IN and OUT. Dropout, θJA, PSRR: `TLV757P_DATA`."""
    p = _ic("TLV75733", {"IN": "1", "GND": "2", "EN": "3", "NC": "4", "OUT": "5"},
            "Package_TO_SOT_SMD:SOT-23-5", "TLV75733PDBVR", None, nc=("NC",))
    _ldo_part(p, 3.3, TLV757P_DATA)
    p.spice_lines = lambda n, k=1.0: _ldo_lines(p.ref, n, 3.3, TLV757P_DATA.dropout_v, "NC", en_r=1e7)
    return p


def rf_ldo(v_out: float) -> Part:
    """An RF-rail regulator on shared SOT-23-5 pads (IN 1, GND 2, EN 3, 4, OUT 5): TPS7A20 in China,
    MIC5205 in Iran. Pin 4 is NC on the TPS7A20 and BYP (470 pF to GND) on the MIC5205; the pin is
    named NC and the MIC5205 model reads its bypass node there. `p.ldo_by_population` holds each
    fitting's `LdoData`."""
    p = tps7a20(v_out)
    china = Variant(p.value, p.mpn, "TPS7A20 300 mA ultra-low-noise LDO; pin 4 not connected")
    iran = Variant(f"MIC5205-{v_out:.1f}", f"MIC5205-{v_out:.1f}YM5",
                   "MIC5205 150 mA low-noise LDO; pin 4 is the 470 pF noise bypass",
                   spice_lines=lambda n, k=1.0: _ldo_lines(p.ref, n, v_out, MIC5205_DATA.dropout_v, "NC"),
                   torob=_mic5205_torob(v_out))
    p.ldo_by_population = {"china": TPS7A20_DATA, "iran": MIC5205_DATA}
    return set_variants(p, china, iran)


def ldo_data(part: Part, population: str) -> LdoData | None:
    """The `LdoData` of the regulator fitted in `population` (None for a part that is not one)."""
    by_pop = getattr(part, "ldo_by_population", None)
    return by_pop[population] if by_pop else getattr(part, "ldo", None)


def _ldo_lines(ref: str, n: dict, v_out: float, dropout: float, byp: str | None = None, en_r: float = 1e7) -> list[str]:
    out = [f"B{ref}_reg {ref}_r {n['GND']} V = min({v_out:g}, v({n['IN']}) - {dropout:g})",
           f"V{ref}_sense {ref}_r {ref}_s dc 0",
           f"R{ref}_o {ref}_s {n['OUT']} 0.05",
           f"F{ref}_in {n['IN']} {n['GND']} V{ref}_sense 1"]
    if byp:
        out += [f"R{ref}_byp {n[byp]} {n['GND']} 1e9", f"R{ref}_en {n['EN']} {n['GND']} {en_r:g}"]
    return out


# --- controller, USB, ADC interface, relay drive (phase 3 spec §3.3) -----------------------

# ESP32-S3-WROOM-1 pads, from the KiCad 9 RF_Module symbol (IO19/IO20 are USB D−/D+;
# IO35–IO37 carry the octal PSRAM on the N16R8 and are not brought out for use)
ESP32S3_PADS = {"GND": "1", "3V3": "2", "EN": "3", "IO4": "4", "IO5": "5", "IO6": "6", "IO7": "7", "IO15": "8",
                "IO16": "9", "IO17": "10", "IO18": "11", "IO8": "12", "IO19": "13", "IO20": "14", "IO3": "15",
                "IO46": "16", "IO9": "17", "IO10": "18", "IO11": "19", "IO12": "20", "IO13": "21", "IO14": "22",
                "IO21": "23", "IO47": "24", "IO48": "25", "IO45": "26", "IO0": "27", "IO35": "28", "IO36": "29",
                "IO37": "30", "IO38": "31", "IO39": "32", "IO40": "33", "IO41": "34", "IO42": "35", "RXD0": "36",
                "TXD0": "37", "IO2": "38", "IO1": "39", "GND2": "40", "EPAD": "41"}
ESP32S3_PSRAM_PINS = ("IO35", "IO36", "IO37")
ESP32S3_STRAPPING_PINS = ("IO0", "IO3", "IO45", "IO46")


def esp32s3_wroom1(nc=()) -> Part:
    """Espressif ESP32-S3-WROOM-1-N16R8 module (16 MB flash, 8 MB octal PSRAM)."""
    return _ic("ESP32-S3-WROOM-1", ESP32S3_PADS, "RF_Module:ESP32-S3-WROOM-1", "ESP32-S3-WROOM-1-N16R8",
               "ESP32-S3-WROOM-1 N16R8", nc)


USB_C_PADS = {p: p for p in ("A1", "A4", "A5", "A6", "A7", "A8", "A9", "A12", "B1", "B4", "B5", "B6", "B7", "B8",
                             "B9", "B12", "S1")}


def usb_c_16p() -> Part:
    """USB Type-C receptacle, 16 pins (USB 2.0), HRO TYPE-C-31-M-12 footprint: A1/A12/B1/B12 GND,
    A4/A9/B4/B9 VBUS, A5 CC1, B5 CC2, A6/B6 D+, A7/B7 D−, A8/B8 SBU (unused), S1 shell."""
    return _ic("USB-C 16P", USB_C_PADS, "Connector_USB:USB_C_Receptacle_HRO_TYPE-C-31-M-12", "TYPE-C-31-M-12",
               "USB TYPE-C 16PIN", nc=("A8", "B8"))


def usblc6() -> Part:
    """ST USBLC6-2SC6 ESD array, SOT-23-6: 1 I/O1, 2 GND, 3 I/O2, 4 I/O2, 5 VBUS, 6 I/O1 (flow-through)."""
    return _ic("USBLC6-2SC6", {"IO1A": "1", "GND": "2", "IO2A": "3", "IO2B": "4", "VBUS": "5", "IO1B": "6"},
               "Package_TO_SOT_SMD:SOT-23-6", "USBLC6-2SC6", "USBLC6-2SC6")


def lm75a() -> Part:
    """NXP LM75A temperature sensor, SO-8: 1 SDA, 2 SCL, 3 OS, 4 GND, 5 A2, 6 A1, 7 A0, 8 VCC."""
    return _ic("LM75A", {"SDA": "1", "SCL": "2", "OS": "3", "GND": "4", "A2": "5", "A1": "6", "A0": "7", "VCC": "8"},
               "Package_SO:SOIC-8_3.9x4.9mm_P1.27mm", "LM75AD", "LM75AD", nc=("OS",))


HC595_PINS = {"QB": "1", "QC": "2", "QD": "3", "QE": "4", "QF": "5", "QG": "6", "QH": "7", "GND": "8", "QHS": "9",
              "/SRCLR": "10", "SRCLK": "11", "RCLK": "12", "/OE": "13", "SER": "14", "QA": "15", "VCC": "16"}


def hc595(nc=()) -> Part:
    """74HC595 8-bit shift register with output latch, SO-16 (TI SN74HC595 pinout)."""
    return _ic("74HC595", HC595_PINS, "Package_SO:SOIC-16_3.9x9.9mm_P1.27mm", "74HC595D", "74HC595 SMD", nc)


ULN2803_PINS = {**{f"I{k}": str(k) for k in range(1, 9)}, "GND": "9", "COM": "10",
                **{f"O{k}": str(19 - k) for k in range(1, 9)}}


def uln2803(nc=()) -> Part:
    """ULN2803A Darlington array, SO-18W: inputs 1–8, GND 9, COM 10 (flyback diodes' common, to
    the coil supply), outputs 18–11 (O1 = 18 … O8 = 11)."""
    return _ic("ULN2803", ULN2803_PINS, "Package_SO:SOIC-18W_7.5x11.6mm_P1.27mm", "ULN2803ADWR", "ULN2803", nc)


def sy6280(r_set: float) -> Part:
    """Silergy SY6280AAC current-limited load switch, SOT-23-5: 1 OUT, 2 GND, 3 ISET, 4 EN, 5 IN;
    I_LIM = 6800 / R_SET (0.4 A minimum), R_on 80 mΩ, EN high ≥ 2.4 V (VIN 4.2–5.5 V), no body
    diode (reverse blocking), thermal auto-retry (datasheet). Behavioural model (smooth, for Newton):
    IN→OUT current I_LIM·tanh(((V_IN − V_OUT)/80 mΩ) / I_LIM) gated by a logistic step at EN = 1.6 V;
    nothing flows backwards."""
    p = _ic("SY6280", {"OUT": "1", "GND": "2", "ISET": "3", "EN": "4", "IN": "5"},
            "Package_TO_SOT_SMD:SOT-23-5", "SY6280AAC", "SY6280")
    i_lim = 6800.0 / r_set
    p.spice_lines = lambda n, k=1.0: [
        f"B{p.ref}_sw {n['IN']} {n['OUT']} I = 1 / (1 + exp(-20 * (v({n['EN']}) - 1.6))) * "
        f"{i_lim:.4g} * tanh(max(0, v({n['IN']}) - v({n['OUT']})) / 0.08 / {i_lim:.4g})",
        f"R{p.ref}_en {n['EN']} {n['GND']} 1e9", f"R{p.ref}_off {n['OUT']} {n['GND']} 1e6"]
    return p


INA180_GAIN, INA180_OFFSET_V, INA180_ROUT, INA180_HEADROOM_V = 20.0, 0.15e-3, 10.0, 0.02


def ina180a1() -> Part:
    """Texas Instruments INA180A1IDBVR current-sense amplifier, SOT-23-5 (DBV): 1 OUT, 2 GND,
    3 IN+, 4 IN-, 5 VS. Gain 20 V/V, input offset at most ±150 µV (A1 grade), common mode up to
    26 V independent of VS. Pinout as in the Pin Functions table of the INA180 datasheet, as listed
    by KiCad's Amplifier_Current:INA180A1 symbol (ti.com was not reachable when this was written; see
    the Task 4 report). Behavioural model: V(OUT) = clamp(20 · (V(IN+) − V(IN−)) + 20 · 150 µV, 0,
    V(VS) − 20 mV) through 10 Ω; the offset has the sign that reads high, so the zero is never
    below the true zero."""
    p = _ic("INA180A1", {"OUT": "1", "GND": "2", "IN+": "3", "IN-": "4", "VS": "5"},
            "Package_TO_SOT_SMD:SOT-23-5", "INA180A1IDBVR", "INA180A1")

    def lines(n, k=1.0):
        mid = f"{p.ref}_int"
        return [f"B{p.ref} {mid} {n['GND']} V = min(max({INA180_GAIN:g} * (v({n['IN+']}) - v({n['IN-']})) + "
                f"{INA180_GAIN * INA180_OFFSET_V:.4g}, 0), v({n['VS']}) - v({n['GND']}) - {INA180_HEADROOM_V:g})",
                f"R{p.ref}_out {mid} {n['OUT']} {INA180_ROUT:g}"]

    p.spice_lines = lines
    return p


def oscillator_24m576() -> Part:
    """24.576 MHz 3.3 V CMOS crystal oscillator, 3.2 × 2.5 mm 4-pad (1 EN/tri-state, 2 GND, 3 OUT, 4 VDD):
    the CS5361's master clock (master mode, MDIV = 1 → 96 kHz double speed). Torob availability to
    confirm (pending, user away 2026-10-13)."""
    p = _ic("24.576MHz", {"EN": "1", "GND": "2", "OUT": "3", "VDD": "4"},
            "Oscillator:Oscillator_SMD_Abracon_ASE-4Pin_3.2x2.5mm", "ASE-24.576MHZ-LC-T", "اسیلاتور 24.576")
    p.bom_note = "24.576 MHz 3.3 V CMOS oscillator, 3.2 × 2.5 mm, or equivalent: match the footprint to the bought package"
    return p


LED_TOROB = {"green": "LED SMD 0805 سبز", "yellow": "LED SMD 0805 زرد", "red": "LED SMD 0805 قرمز"}


def led_0805(colour: str = "green") -> Part:
    p = _ic(f"LED {colour}", {"K": "1", "A": "2"}, "LED_SMD:LED_0805_2012Metric", None, LED_TOROB.get(colour),
            spec=f"LED {colour} 0805, 20 mA, Vf ≤ 2.4 V", ident_kind=IDENT_SPEC)
    return p


def button() -> Part:
    """6 × 3.5 mm two-pin SMD tactile switch (CK PTS636-class; on Torob as a 3 × 6 mm SMD switch)."""
    p = _ic("button", {"1": "1", "2": "2"}, "Button_Switch_SMD:SW_Tactile_SPST_NO_Straight_CK_PTS636Sx25SMTRLFS",
            "PTS636SL25SMTRLFS", "تک سوئیچ SMD")
    p.ident_kind = IDENT_SPEC
    p.bom_note = "6 × 3.5 mm SMD tactile switch, or equivalent"
    return p
