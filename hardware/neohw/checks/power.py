"""Power check (phase 3 spec §3.2, plan 3.4 amendment, plan 5.1): the 9–20 V jack as the only supply.
Rails at 9 and 20 V input and maximum load, a reversed plug, the TVS surge clamp against the
buck's and the PTC's rating, regulator junction temperatures, the buck's conducted ripple per amateur band,
the hot-plug transient (buck input overshoot, peak current, rails up and staying up) and the input current.
The regulators differ by population (TPS7A20 / TLV75733 in China, MIC5205 / AMS1117 in Iran), so every
rail, junction and spur result is computed for each; the China results head power.json and
`populations` holds both. Writes power.json."""
from __future__ import annotations

import json
import math
from pathlib import Path

import numpy as np

from neohw import availability, charfile, export, parts
from neohw import population as pops
from neohw.blocks import bias_tee, mixer, power, preamp
from neohw.checks import BlockResult, CheckResult
from neohw.checks.controller import ADC_MAX_V
from neohw.ngspice import run_decks
from neohw.spice import Ac, Load, Op, Tran, node_name, to_deck
from neohw.touchstone import read_s2p

# rail loads (A), from datasheet currents: 5V_A = CS5361 VA 50 mA + 2 × LM4562 20 mA + preamp 22 mA;
# 3V3_RF = Si5351C 35 mA + logic; 3V3_D continuous maximum = ESP32-S3 240 MHz dual core, radio off
# (≈ 107 mA) + CS5361 VL/VD 17 mA + 24.576 MHz oscillator 10 mA + 74HC595/ULN2803 inputs and LED
# 8 mA ≈ 0.15 A; 5V_MAIN = G6K-2F coils at ≈ 4.6 V (178 Ω): 26 mA each (2 filter relays + preamp
# typ, + attenuator max); BIAS = active antenna (0 typ, 100 mA max)
LOADS_TYP = {"v5a": 0.092, "v3rf": 0.040, "v3d": 0.130, "main": 0.078, "bias5": 0.0}
LOADS_MAX = {"v5a": 0.110, "v3rf": 0.050, "v3d": 0.150, "main": 0.104, "bias5": 0.100}
RAIL_MIN_V = {"v5a": 4.95, "v3rf": 3.25, "v3d": 3.25}     # every rail in regulation
MAIN_V = (5.3, 5.7)            # 5V_MAIN window: coils, LDO headroom, AMS1117 dissipation
BUCK_VIN_MAX_V = 60.0          # LM2576HV recommended maximum input (63 V absolute)
INPUT_V = (9.0, 20.0)          # the jack's range (user decision, 2026-10-18)
SURGE_A = parts.SMBJ20A_IPP_A  # SMBJ20A rated peak pulse current (its clamp voltage, 32.4 V, is specified there)
PLUG_IN_V = (8.55, 9.0, 12.0, 20.0)   # 8.55 V: a 9 V adapter at −5 %
LEAK_MAX_A = 1e-3
# thermal: θJA per regulator from its datasheet (neohw.parts.LdoData: TPS7A20 DBV 187.1, TLV75733 DBV
# 231.1 (JEDEC board), MIC5205 SOT-23-5 220, AMS1117 SOT-223 90 °C/W); LM2576 TO-263 on ≈ 1 in² copper
# ≈ 50 °C/W, all of the converter's loss charged to it: conservative; 50 °C ambient
AMBIENT_C, TJ_MAX_C = 50.0, 125.0
THETA_JA_LM2576 = 50.0
F_SW = 52e3                    # LM2576 (fixed frequency, 47–58 kHz)
SPUR_MARGIN_DB = 20.0          # conducted spurs at least this far below MDS
MDS_DBM = -140.5
BANDS = {"160m": 1.9e6, "80m": 3.65e6, "60m": 5.35e6, "40m": 7.1e6, "30m": 10.12e6, "20m": 14.1e6, "17m": 18.1e6,
         "15m": 21.2e6, "12m": 24.94e6, "10m": 28.5e6, "6m": 50.1e6}
# name: (jack volts, loads)
SCENARIOS = {
    "jack 8.55 V (9 V − 5 %), maximum load": (8.55, LOADS_MAX),
    "jack 9 V, maximum load": (9.0, LOADS_MAX),
    "jack 12 V, typical load": (12.0, LOADS_TYP),
    "jack 12 V, maximum load": (12.0, LOADS_MAX),
    "jack 20 V, maximum load": (20.0, LOADS_MAX),
    "jack 20 V, typical load": (20.0, LOADS_TYP),
    "jack reversed (−12 V)": (-12.0, {}),        # unpowered: the loads draw nothing
}


psrr_db = parts.MIC5205_PSRR_DB
"""MIC5205 ripple rejection: 75 dB at 100 Hz (datasheet); assumed −20 dB/decade above 1 kHz to a
15 dB floor (no curve in the datasheet text). The TPS7A20 and TLV75733 read theirs from their datasheet
tables (neohw.parts)."""


def regulators(blk, population: str = pops.DEFAULT) -> dict:
    """The linear regulators fitted in `population`: name -> (input port, output port, load key,
    LdoData). The name is the fitted variant's value (TPS7A20-5.0 or MIC5205-5.0 on the shared pads)."""
    key_of = {node_name(net): k for k, net in blk.ports.items()}
    out = {}
    for p in blk.circuit.parts:
        data = parts.ldo_data(p, population)
        if data is None or not pops.fitted(p, population):
            continue
        v = pops.variant(p, population)
        k_out = key_of[node_name(p["OUT"].net)]
        out[v.value if v is not None else p.value] = (key_of[node_name(p["IN"].net)], k_out, k_out, data)
    return out


def _ports(blk):
    return [Load(blk.ports["vbus"], "vbus_hz", 1e9), Load(blk.ports["ext_tip"], "tip_hz", 1e9)]


def dc_scenarios(blk, population: str = pops.DEFAULT) -> dict:
    P = {k: node_name(v) for k, v in blk.ports.items()}
    decks = []
    for name, (ve, loads) in SCENARIOS.items():
        lines = [f"VEXT {P['ext_tip']} 0 dc {ve}", f"VUSB {P['vbus']} 0 dc 5"]
        lines += [f"ILD_{k} {P[k]} 0 dc {i}" for k, i in loads.items()] + [".options gmin=1e-12 itl1=500"]
        decks.append(to_deck(blk.circuit, f"power {name} ({population})", _ports(blk), [Op()], extra_lines=lines,
                             population=population)[0])
    out = {}
    for name, r in zip(SCENARIOS, run_decks(decks, ["a0.raw"])):
        d = r["a0.raw"][0].data
        out[name] = {k: float(d[f"v({P[k]})"][0].real)
                     for k in ("ext_p", "buck_in", "main", "ldo_a_in", "ldo_rf_in", "v5a", "v3rf", "v3d", "bias5",
                               "sense_ext", "sense_main", "sense_vbus")}
        out[name]["i_ext"] = float(-d["i(vext)"][0].real)
        out[name]["i_usb"] = float(-d["i(vusb)"][0].real)
    return out


def surge_clamp(blk, population: str = pops.DEFAULT) -> float:
    """Voltage at the buck side of the TVS with the TVS's rated peak current forced into it."""
    P = {k: node_name(v) for k, v in blk.ports.items()}
    deck = to_deck(blk.circuit, "power surge clamp", _ports(blk), [Op()],
                   extra_lines=[f"REXT {P['ext_tip']} 0 1e6", f"ISURGE 0 {P['ext_p']} dc {SURGE_A}",
                                f"VUSB {P['vbus']} 0 dc 0", ".options gmin=1e-12 itl1=500"],
                   population=population)[0]
    d = run_decks([deck], ["a0.raw"])[0]["a0.raw"][0].data
    return float(d[f"v({P['ext_p']})"][0].real)


def junction_temps(dc: dict, blk, population: str = pops.DEFAULT) -> dict:
    """Linear regulators fitted in `population` at their own input over every maximum-load case, with
    their own θJA; the buck at its loss (output power × (1/η − 1)) charged entirely to the TO-263."""
    worst = [s for name, s in dc.items() if "maximum load" in name]
    tj = {reg: AMBIENT_C + data.theta_ja * max(s[i] - s[o] for s in worst) * LOADS_MAX[load]
          for reg, (i, o, load, data) in regulators(blk, population).items()}
    p_out = max(s["main"] * sum(LOADS_MAX.values()) for s in worst)
    tj["LM2576HV"] = AMBIENT_C + THETA_JA_LM2576 * p_out * (1 / parts.LM2576_EFFICIENCY - 1)
    return tj


def plug_in(blk, volts: float, population: str = pops.DEFAULT) -> dict:
    """The jack plugged into a live adapter: a step to `volts` through 1 µH / 0.2 Ω of cable, rails
    loaded as resistors at maximum load. Peak buck input (LC overshoot), peak input current
    (carried by the SS36: the PTC does not trip on a millisecond pulse), the time every rail is
    up and 5V_MAIN at the end (still in regulation, not a first crossing)."""
    P = {k: node_name(v) for k, v in blk.ports.items()}
    nominal = {"v5a": 5.0, "v3rf": 3.3, "v3d": 3.3, "main": 5.5, "bias5": 5.5}
    lines = [f"VSRC {P['ext_tip']}_src 0 pwl(0 0 0.1m 0 0.101m {volts})", f"RCAB {P['ext_tip']}_src {P['ext_tip']}_l 0.2",
             f"LCAB {P['ext_tip']}_l {P['ext_tip']} 1u", f"VUSB {P['vbus']} 0 dc 0"]
    lines += [f"RLD_{k} {P[k]} 0 {nominal[k] / i:g}" for k, i in LOADS_MAX.items()]
    lines += [".options method=gear"]        # the input diode blocking the LC's reverse swing is stiff
    deck = to_deck(blk.circuit, f"power plug-in {volts:g} V ({population})", _ports(blk), [Tran(2e-6, 20e-3, 0.0, 1e-6)],
                   extra_lines=lines, population=population)[0]
    d = run_decks([deck], ["a0.raw"])[0]["a0.raw"][0].data
    t, i_in = d["time"].real, -d["i(vsrc)"].real
    i_steady = float(np.mean(i_in[t > 18e-3]))
    up = []
    for k in ("v5a", "v3rf", "v3d"):
        v = d[f"v({P[k]})"].real
        ok = v >= RAIL_MIN_V[k]
        up.append(float(t[np.argmax(ok)]) if ok.any() else math.inf)
    return {"buck_in_peak_v": float(d[f"v({P['buck_in']})"].real.max()), "peak_a": float(i_in.max()),
            "steady_a": i_steady, "rails_up_s": max(up) - 0.1e-3, "main_end_v": float(d[f"v({P['main']})"].real[-1])}


def ripple_spurs(blk, population: str = pops.DEFAULT) -> dict:
    """Conducted buck ripple → RF path, per amateur band (the harmonic of F_SW nearest the band):
    the inductor's triangular current (worst at 20 V in) is driven into the simulated output
    network; 5V_A ripple reaches the RF path through the preamp's and the mixer bias network's
    supply rejection after its LC and LDO, and 5V_MAIN ripple through the bias-T feed. Spurs are
    referred to the antenna (equivalent available power)."""
    P = {k: node_name(v) for k, v in blk.ports.items()}
    b = next(p for p in blk.circuit.parts if p.value == "LM2576HV-ADJ")
    ov = {b.ref: lambda n: [f"R{b.ref}_o {n['OUT']} {n['GND']} 1e9"]}   # the source is the stimulus
    ports = _ports(blk)
    ac = Ac(20e3, 70e6, 200)
    # 5V_MAIN held at its DC operating point through 1 H (≥ 125 kΩ above 20 kHz), so the LDO
    # models work at their real headroom instead of in dropout
    extra = [f"VEXT {P['ext_tip']} 0 dc 12", f"VUSB {P['vbus']} 0 dc 0",
             f"VMDC {P['main']}_dc 0 dc 5.5", f"LMDC {P['main']}_dc {P['main']} 1"]
    decks = [to_deck(blk.circuit, f"ripple main ({population})", ports, [ac],
                     extra_lines=extra + [f"IRIP 0 {P['sw']} dc 0 ac 1"], overrides=ov, population=population)[0]]
    pre, bt, mx = preamp.build("amp"), bias_tee.build(), mixer.build()
    sw = next(p for p in mx.circuit.parts if p.name == "FST3253")
    decks += [to_deck(pre.circuit, "preamp supply rejection", [Load(pre.ports["rf_in"], "term", 50.0), Load(pre.ports["rf_out"])],
                      [ac], extra_lines=[f"VSUP {node_name(pre.ports['vcc'])} 0 dc 5 ac 1"])[0],
              to_deck(bt.circuit, "bias feed", [Load(bt.ports["rf_in"], "ant", 50.0), Load(bt.ports["rf_out"])],
                      [ac], extra_lines=[f"VFEED {node_name(bt.ports['bias_feed'])} 0 dc 5 ac 1"])[0],
              to_deck(mx.circuit, "mixer supply rejection",
                      [Load(mx.ports["rf_in"], "rf", 50.0), Load(mx.ports["s0"], "s0", 1e6), Load(mx.ports["s1"], "s1", 1e6)],
                      [ac], extra_lines=[f"VSUP {node_name(mx.ports['vcc'])} 0 dc 5 ac 1"],
                      overrides={sw.ref: lambda n: [f"R{sw.ref}_a {n['1A']} 0 1e9"]
                                 + [f"R{sw.ref}_b{k} {n[f'1B{k}']} 0 1e9" for k in range(4)]})[0]]
    res = run_decks(decks, ["a0.raw"])

    def tr(r, node):
        d = r["a0.raw"][0].data
        return d["frequency"].real, np.abs(d[f"v({node})"])
    z_a = tr(res[0], P["ldo_a_in"])
    z_bias = tr(res[0], P["bias5"])
    t_pre = tr(res[1], node_name(pre.ports["rf_out"]))
    t_bias = tr(res[2], node_name(bt.ports["rf_out"]))
    t_mix = tr(res[3], node_name(mx.ports["a"]))
    v_in, v_out = INPUT_V[1], 5.5
    psrr_5va = next(r[3] for r in regulators(blk, population).values() if r[1] == "v5a").psrr_db
    duty = v_out / v_in
    di = (v_in - v_out) * duty / (power.BUCK_L * F_SW)
    char = Path(__file__).resolve().parents[2] / "characterized"
    pj = json.loads((char / "preamp.json").read_text())
    bank = json.loads((char / "bpf_bank.json").read_text())

    def front(f):
        idx = next((x["index"] for x in bank["filters"] if x["lo_hz"] <= f < x["hi_hz"]), bank["filters"][-1]["index"])
        ff, s = read_s2p(char / f"bpf_{idx}.s2p")
        return float(np.interp(f, ff, np.abs(s["21"]))) * 10 ** (float(np.interp(f, pj["freq_hz"], pj["gain_db"])) / 20)

    def at(t, f):
        return float(np.interp(f, *t))

    def dbm(v):      # node voltage amplitude → equivalent antenna source available power (50 Ω both ways)
        return 10 * math.log10(max(v, 1e-30) ** 2 / 100 / 1e-3)

    out = {}
    for band, f in BANDS.items():
        n = max(1, round(f / F_SW))
        fn = n * F_SW
        c = di * abs(math.sin(math.pi * n * duty)) / (math.pi ** 2 * n ** 2 * duty * (1 - duty))
        v5a = c * at(z_a, fn) * 10 ** (-float(psrr_5va(fn)) / 20)
        paths = {"preamp": dbm(v5a * at(t_pre, fn) / front(fn)), "mixer": dbm(v5a * at(t_mix, fn) / front(fn)),
                 "bias_tee": dbm(c * at(z_bias, fn) * at(t_bias, fn))}
        out[band] = {"harmonic": n, "f_hz": fn, **paths,
                     "total_dbm": 10 * math.log10(sum(10 ** (p / 10) for p in paths.values()))}
    return out


def _tag(pop: str) -> str:
    """China keeps the original check names; Iran's carry `iran`."""
    return "" if pop == pops.DEFAULT else f"{pop}_"


def _population_checks(pop: str, dc: dict, tj: dict, hot: dict, spurs: dict) -> list[CheckResult]:
    tag = _tag(pop)
    powered = [s for name, s in dc.items() if "reversed" not in name]
    rail_min = {k: min(s[k] for s in powered) for k in RAIL_MIN_V}
    main_lo, main_hi = min(s["main"] for s in powered), max(s["main"] for s in powered)
    worst_spur = max(s["total_dbm"] for s in spurs.values())
    usb_draw = max(abs(s["i_usb"]) - 5.0 / sum(power.VBUS_DIVIDER) for s in powered)   # beyond the sense divider
    checks = [CheckResult(f"{tag}{k}_min_v", v, f"≥ {RAIL_MIN_V[k]:g}", v >= RAIL_MIN_V[k], "V") for k, v in rail_min.items()]
    checks += [
        CheckResult(f"{tag}main_min_v", main_lo, f"≥ {MAIN_V[0]:g}", main_lo >= MAIN_V[0], "V"),
        CheckResult(f"{tag}main_max_v", main_hi, f"≤ {MAIN_V[1]:g}", main_hi <= MAIN_V[1], "V"),
        CheckResult(f"{tag}input_current_12v_typ_ma", dc["jack 12 V, typical load"]["i_ext"] * 1e3, "report", True, "mA"),
        CheckResult(f"{tag}input_current_9v_max_ma", dc["jack 9 V, maximum load"]["i_ext"] * 1e3, "report (adapter rating)", True, "mA"),
        CheckResult(f"{tag}usb_vbus_draw_ma", usb_draw * 1e3, f"≤ {LEAK_MAX_A * 1e3:g} (USB is data only)", usb_draw <= LEAK_MAX_A, "mA"),
    ]
    for v, h in hot.items():
        t = f"{tag}plug_in_{v:g}v"
        checks += [
            CheckResult(f"{t}_buck_in_peak_v", h["buck_in_peak_v"], f"≤ {BUCK_VIN_MAX_V:g}", h["buck_in_peak_v"] <= BUCK_VIN_MAX_V, "V"),
            CheckResult(f"{t}_peak_a", h["peak_a"], f"≤ {parts.SS36_IFSM_A / 2:g} (SS36 surge / 2)",
                        h["peak_a"] <= parts.SS36_IFSM_A / 2, "A"),
            CheckResult(f"{t}_rails_up_ms", h["rails_up_s"] * 1e3, "≤ 20", h["rails_up_s"] <= 20e-3, "ms"),
            CheckResult(f"{t}_main_end_v", h["main_end_v"], f"{MAIN_V[0]:g}–{MAIN_V[1]:g}",
                        MAIN_V[0] <= h["main_end_v"] <= MAIN_V[1], "V"),
        ]
    checks += [CheckResult(f"{tag}tj_{k}", t, f"≤ {TJ_MAX_C:g}", t <= TJ_MAX_C, "°C") for k, t in tj.items()]
    checks.append(CheckResult(f"{tag}conducted_spur_worst_dbm", worst_spur,
                              f"≤ MDS − {SPUR_MARGIN_DB:g} = {MDS_DBM - SPUR_MARGIN_DB:g}", worst_spur <= MDS_DBM - SPUR_MARGIN_DB, "dBm"))
    return checks


def run(out_dir: Path, char_dir: Path) -> BlockResult:
    out_dir, char_dir = Path(out_dir), Path(char_dir)
    blk = power.build()
    dc = {pop: dc_scenarios(blk, pop) for pop in pops.POPULATIONS}
    tj = {pop: junction_temps(dc[pop], blk, pop) for pop in pops.POPULATIONS}
    spurs = {pop: ripple_spurs(blk, pop) for pop in pops.POPULATIONS}
    hot = {pop: {v: plug_in(blk, v, pop) for v in PLUG_IN_V} for pop in pops.POPULATIONS}
    clamp = surge_clamp(blk)
    china = pops.DEFAULT
    rev = dc[china]["jack reversed (−12 V)"]
    sense_ext = dc[china][f"jack {INPUT_V[1]:g} V, maximum load"]["sense_ext"]
    checks = [
        CheckResult("reversed_jack_current_ma", abs(rev["i_ext"]) * 1e3, f"≤ {LEAK_MAX_A * 1e3:g}", abs(rev["i_ext"]) <= LEAK_MAX_A, "mA"),
        CheckResult("reversed_jack_main_v", rev["main"], "≥ −0.3 (nothing reverse-biased)", rev["main"] >= -0.3, "V"),
        CheckResult("surge_clamp_v", clamp, f"≤ {BUCK_VIN_MAX_V:g} (LM2576HV input)", clamp <= BUCK_VIN_MAX_V, "V"),
        CheckResult("surge_clamp_under_ptc_v", clamp, f"≤ {parts.PTC_V_MAX:g} (PTC maximum voltage)", clamp <= parts.PTC_V_MAX, "V"),
        CheckResult("sense_ext_at_20v_v", sense_ext, f"≤ {ADC_MAX_V:g} (STM32F411 ADC)", sense_ext <= ADC_MAX_V, "V"),
    ]
    for pop in pops.POPULATIONS:
        checks += _population_checks(pop, dc[pop], tj[pop], hot[pop], spurs[pop])
    result = BlockResult("power", checks)
    data = availability.load()
    result.artefacts = {
        "kicad_netlist": export.kicad_netlist(blk.circuit, out_dir / "power.net"),
        "bom": export.write_bom(blk.circuit, out_dir / "power_bom.csv", data),
        "bom_iran": export.write_bom(blk.circuit, out_dir / "power_bom_iran.csv", data, "iran"),
        "svg": export.svg_diagram(blk.circuit, out_dir / "power.svg"),
        "characterisation": charfile.write(char_dir / "power.json", "power", "pass" if result.passed else "fail", {
            "input_v": list(INPUT_V), "loads_typ_a": LOADS_TYP, "loads_max_a": LOADS_MAX, "scenarios": dc[china],
            "sense_dividers": power.SENSE_DIVIDERS, "junction_c": tj[china], "surge_clamp_v": clamp,
            "plug_in": {f"{v:g}": h for v, h in hot[china].items()}, "spurs_dbm": spurs[china],
            "populations": {pop: {"scenarios": dc[pop], "junction_c": tj[pop],
                                  "plug_in": {f"{v:g}": h for v, h in hot[pop].items()}, "spurs_dbm": spurs[pop]}
                            for pop in pops.POPULATIONS},
            "psrr_assumption": "MIC5205 75 dB at 100 Hz (datasheet), assumed -20 dB/dec above 1 kHz to 15 dB; "
                               "TPS7A20 92/60/40 dB at 1 kHz/100 kHz/1 MHz (300 mA) and TLV75733 52/46/52 dB (50 mA) from their "
                               "datasheet tables, log-f interpolation, -20 dB/dec above 1 MHz to 15 dB",
            "spur_note": "conducted only (inductor ripple, triangle model); switch-node edges and radiated/layout coupling are checked in the layout phase",
            "checks": [c.__dict__ for c in checks]}),
    }
    return result
