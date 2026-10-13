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
              torob: str | None = None, esr: float = 0.0, tol: float = 0.02, note: str = "",
              esl: float = 0.0) -> Part:
    """`esr` and `esl` are SPICE-only parasitics (power filtering at MHz needs the ESL)."""
    farads = _positive(value)
    p = _tagged(_template("C", "C", ["1", "2"], footprint)(value=format_si(farads, "F")))
    p.mpn, p.torob, p.tol, p.bom_note = mpn, torob, tol, note

    def lines(n, k=1.0):
        if esr <= 0 and esl <= 0:
            return [f"C{p.ref} {n['1']} {n['2']} {_num(farads * k)}"]
        out, node = [], n["1"]
        if esr > 0:      # internal node names as before ESL was added: existing netlists are unchanged
            out.append(f"R{p.ref}_esr {node} {p.ref}_esr {_num(esr)}")
            node = f"{p.ref}_esr"
        if esl > 0:
            out.append(f"L{p.ref}_esl {node} {p.ref}_esl {_num(esl)}")
            node = f"{p.ref}_esl"
        return out + [f"C{p.ref} {node} {n['2']} {_num(farads * k)}"]

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
            "25MHz 3225 CL=8pF", "کریستال 25MHZ 3225")
    p.torob_match = crystal_25mhz_hit
    p.bom_note = "CL ≤ 10 pF (Si5351 internal load 6/8/10 pF); confirm CL with the seller"
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
            "DC-005", "جک آداپتور 5.5 2.1", nc=("SW",))
    p.spice_lines = lambda n, k=1.0: []          # the source is the stimulus
    return p


PTC_HOLD_A, PTC_TRIP_A, PTC_V_MAX, PTC_OHM = 0.75, 1.5, 33.0, 0.3


def ptc_resettable() -> Part:
    """Resettable PTC fuse (polyfuse), 1812: hold 0.75 A, trip 1.5 A, 33 V maximum (≥ the 30 V input),
    ≈ 0.3 Ω. It ignores millisecond plug-in pulses (it trips on heat, in seconds, at a sustained
    fault) and drops only ≈ 0.1 V at the board's 0.45 A maximum input, so the board starts at 9 V
    whatever the temperature. A passive: the manufacturer sources it (e.g. a 1812L075/33-class part)."""
    p = _tagged(_template("PTC", "F", ["1", "2"], "Fuse:Fuse_1812_4532Metric")(value="PTC 0.75A 33V"))
    p.mpn, p.torob, p.tol, p.bom_note = "1812 PTC hold 0.75 A, 33 V", None, 0.3, "resettable fuse, hold 0.75 A, trip 1.5 A, ≥ 33 V"
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


def smbj30a() -> Part:
    """SMBJ30A 600 W TVS, SMB (K 1, A 2): stand-off 30 V, breakdown 33.3 V min, clamp 48.4 V at
    12.4 A (8/20-class surge): the buck's 60 V rating sits above the clamp."""
    p = _ic("SMBJ30A", {"K": "1", "A": "2"}, "Diode_SMD:D_SMB", "SMBJ30A", "SMBJ30A")
    p.spice_models = (".model SMBJ30_D d(is=1e-12 n=1.5 bv=35 ibv=1m rs=1.1 cjo=600p)",)
    p.spice_lines = lambda n, k=1.0: [f"D{p.ref} {n['A']} {n['K']} SMBJ30_D"]
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


def ams1117_33() -> Part:
    """AMS1117-3.3, SOT-223 (GND 1, OUT 2 and tab, IN 3). From 5.5 V its dropout (1.3 V max) is
    not a limit; the SOT-223 tab carries its 0.33 W (≈ 90 °C/W)."""
    p = _ic("AMS1117-3.3", {"GND": "1", "OUT": "2", "IN": "3"}, "Package_TO_SOT_SMD:SOT-223-3_TabPin2",
            "AMS1117-3.3", "AMS1117-3.3")
    p.v_out, p.dropout_v = 3.3, 1.3
    p.spice_lines = lambda n, k=1.0: _ldo_lines(p.ref, n, 3.3, 1.3)
    return p


def mic5205(v_out: float) -> Part:
    """Microchip MIC5205 150 mA LDO, SOT-23-5 (IN 1, GND 2, EN 3, BYP 4, OUT 5). Dropout 165 mV
    at 150 mA (110 mV at 50 mA); ripple rejection 75 dB at 100 Hz (datasheet DS20005785; no
    curve in the text, so the ripple analysis assumes a roll-off). DC model: OUT = min(v_out,
    V_in − dropout) behind 50 mΩ; input current = output current."""
    p = _ic(f"MIC5205-{v_out:.1f}", {"IN": "1", "GND": "2", "EN": "3", "BYP": "4", "OUT": "5"},
            "Package_TO_SOT_SMD:SOT-23-5", f"MIC5205-{v_out:.1f}YM5", "MIC5205-3.3" if v_out < 4 else "MIC5205YM5")
    p.v_out, p.dropout_v = v_out, 0.165
    p.spice_lines = lambda n, k=1.0: _ldo_lines(p.ref, n, v_out, 0.165, "BYP")
    return p


def _ldo_lines(ref: str, n: dict, v_out: float, dropout: float, byp: str | None = None) -> list[str]:
    out = [f"B{ref}_reg {ref}_r {n['GND']} V = min({v_out:g}, v({n['IN']}) - {dropout:g})",
           f"V{ref}_sense {ref}_r {ref}_s dc 0",
           f"R{ref}_o {ref}_s {n['OUT']} 0.05",
           f"F{ref}_in {n['IN']} {n['GND']} V{ref}_sense 1"]
    if byp:
        out += [f"R{ref}_byp {n[byp]} {n['GND']} 1e9", f"R{ref}_en {n['EN']} {n['GND']} 1e7"]
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


def oscillator_24m576() -> Part:
    """24.576 MHz 3.3 V CMOS crystal oscillator, 3.2 × 2.5 mm 4-pad (1 EN/tri-state, 2 GND, 3 OUT, 4 VDD):
    the CS5361's master clock (master mode, MDIV = 1 → 96 kHz double speed). Torob availability to
    confirm (pending, user away 2026-10-13)."""
    p = _ic("24.576MHz", {"EN": "1", "GND": "2", "OUT": "3", "VDD": "4"},
            "Oscillator:Oscillator_SMD_Abracon_ASE-4Pin_3.2x2.5mm", "24.576MHz 3.3V CMOS XO", "اسیلاتور 24.576")
    p.bom_note = "on Torob (2026-10-13) as a 2 × 3 mm part: match the footprint to the bought package"
    return p


LED_TOROB = {"green": "LED SMD 0805 سبز", "yellow": "LED SMD 0805 زرد", "red": "LED SMD 0805 قرمز"}


def led_0805(colour: str = "green") -> Part:
    p = _ic(f"LED {colour}", {"K": "1", "A": "2"}, "LED_SMD:LED_0805_2012Metric", f"LED 0805 {colour}",
            LED_TOROB.get(colour))
    return p


def button() -> Part:
    """6 × 3.5 mm two-pin SMD tactile switch (CK PTS636-class; on Torob as a 3 × 6 mm SMD switch)."""
    return _ic("button", {"1": "1", "2": "2"}, "Button_Switch_SMD:SW_Tactile_SPST_NO_Straight_CK_PTS636Sx25SMTRLFS",
               "6x3.5 mm SMD tactile", "تک سوئیچ SMD")
