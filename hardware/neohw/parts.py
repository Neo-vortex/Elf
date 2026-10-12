"""Part constructors: SKiDL parts that also know their SPICE model."""
from __future__ import annotations

import math
from pathlib import Path

from skidl import SKIDL, TEMPLATE, Part, Pin

from neohw.availability import inductor_hit
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
             torob: str | None = None, tol: float = 0.01, c_par: float = 0.0) -> Part:
    """`c_par` adds the body's parallel capacitance to the SPICE model only (high-value
    resistors at RF)."""
    ohms = _positive(value)
    p = _tagged(_template("R", "R", ["1", "2"], footprint)(value=format_si(ohms, "Ω")))
    p.mpn, p.torob, p.tol = mpn, torob, tol

    def lines(n, k=1.0):
        out = [f"R{p.ref} {n['1']} {n['2']} {_num(ohms * k)}"]
        if c_par > 0:
            out.append(f"C{p.ref}_par {n['1']} {n['2']} {_num(c_par)}")
        return out

    p.spice_lines = lines
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
             footprint: str = INDUCTOR_FOOTPRINT) -> Part:
    """L + series R (giving `q` at `q_freq_hz`) with a parallel C for the self-resonance.

    `value` is the part to buy. `trim` scales it to what the builder selects with an LCR
    meter (a pair is picked so its sum hits the design target); `tol` is the selection
    tolerance. Q is assumed; both are stated in the BOM note.

    With `dcr` (chokes that carry DC), the series R is the DC resistance and the Q loss
    becomes a parallel resistor Q·2πf·L instead.
    """
    henries = _positive(value)
    if q <= 0 or q_freq_hz <= 0:
        raise ValueError(f"q and q_freq_hz must be positive, got {q!r}, {q_freq_hz!r}")
    if not 0.9 <= trim <= 1.1:
        raise ValueError(f"trim {trim:.3f} is outside the ±10 % a selected part can reach")
    henries_built = henries * trim
    r_s = 2 * math.pi * q_freq_hz * henries_built / q
    p = _tagged(_template("L", "L", ["1", "2"], footprint)(value=format_si(henries, "H")))
    p.mpn, p.torob, p.tol, p.dcr = mpn, torob, tol, dcr
    p.torob_match = lambda name: inductor_hit(name, henries)
    p.bom_note = note or f"select to ±{tol * 100:g} % with LCR meter; Q={q:g} assumed"

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


def pnp_bc857b(mpn: str = "BC857B", torob: str | None = None) -> Part:
    """Nexperia BC857B general-purpose PNP, SOT-23 (pin 1 base, 2 emitter, 3 collector)."""
    p = _tagged(_template("BC857B", "Q", {"B": "1", "E": "2", "C": "3"},
                          "Package_TO_SOT_SMD:SOT-23")(value="BC857B"))
    p.mpn, p.torob, p.tol = mpn, torob, 0.0
    p.spice_models = (model_text("bc857b.lib"),)
    p.spice_lines = lambda n, k=1.0: [f"X{p.ref} {n['C']} {n['B']} {n['E']} BC857B"]
    return p


def diode_pair_bav99(mpn: str = "BAV99", torob: str | None = None) -> Part:
    """BAV99, SOT-23: pin 1 anode of D1, pin 3 = D1 cathode + D2 anode, pin 2 cathode of D2."""
    p = _tagged(_template("BAV99", "D", {"A": "1", "K": "2", "COM": "3"},
                          "Package_TO_SOT_SMD:SOT-23")(value="BAV99"))
    p.mpn, p.torob, p.tol = mpn, torob, 0.0
    p.spice_models = (model_text("bav99.lib"),)
    p.nc_pins = ("COM",)   # as a two-diode series string the middle pin stays open
    p.spice_lines = lambda n, k=1.0: [f"D{p.ref}a {n['A']} {n['COM']} D1N4148",
                                      f"D{p.ref}b {n['COM']} {n['K']} D1N4148"]
    return p


def gdt(capacitance: float = 1.5e-12, torob: str | None = "گازی برقگیر GDT") -> Part:
    """Gas discharge tube at the connector: an open circuit with its capacitance below the
    breakdown voltage (only small-signal behaviour is simulated). Footprint is project-local,
    created in the layout phase."""
    p = _tagged(_template("GDT", "GDT", ["1", "2"], "NeoRadio:GDT_SMD_2pin")(value="GDT 90V"))
    p.mpn, p.torob, p.tol = None, torob, 0.0
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


def _ic(value: str, pins: dict[str, str], footprint: str, mpn: str, torob: str | None, nc=()) -> Part:
    """An IC used for the netlist and BOM only (board-level blocks are not simulated in SPICE)."""
    p = _tagged(_template(value, "U", pins, footprint)(value=value))
    p.mpn, p.torob, p.tol = mpn, torob, 0.0
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


def crystal_25mhz() -> Part:
    """25 MHz fundamental crystal, 3225 4-pad (pads 2 and 4 are case ground). The Si5351's internal
    load capacitance (6/8/10 pF, register 183) is set to the crystal's CL; no external caps."""
    return _ic("25MHz", {"1": "1", "GND1": "2", "2": "3", "GND2": "4"}, "Crystal:Crystal_SMD_3225-4Pin_3.2x2.5mm",
               "25MHz 3225 CL=8pF", "کریستال 25MHZ")


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
