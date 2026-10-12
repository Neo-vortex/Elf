"""Power check (phase 3 spec §3.2): input switch-over and back-feed, rail voltages at minimum input
and maximum load, regulator junction temperatures, conducted boost-ripple spurs per amateur band,
and the current budget. Writes power.json."""
from __future__ import annotations

import json
import math
from pathlib import Path

import numpy as np

from neohw import availability, charfile, export
from neohw.blocks import bias_tee, mixer, power, preamp
from neohw.checks import BlockResult, CheckResult
from neohw.ngspice import run_decks
from neohw.spice import Ac, Load, Op, node_name, to_deck
from neohw.touchstone import read_s2p

# rail loads (A), from datasheet currents: 5V_A = CS5361 VA 50 mA + 2 × LM4562 20 mA + preamp 22 mA;
# 3V3_RF = Si5351C 35 mA + logic; 3V3_D = ESP32-S3 on USB without radio (100 mA typ, 250 mA peaks)
# + CS5361 VL/VD + logic; 5V_MAIN = G6K-2F coils, 28 mA each (2 filter relays + preamp typ, + attenuator
# max); BIAS = active antenna (0 typ, 100 mA max)
LOADS_TYP = {"v5a": 0.092, "v3rf": 0.040, "v3d": 0.130, "main": 0.085, "bias5": 0.0}
LOADS_MAX = {"v5a": 0.110, "v3rf": 0.050, "v3d": 0.250, "main": 0.112, "bias5": 0.100}
USB2_LIMIT_A = 0.5
RAIL_MIN_V = {"v5a": 4.95, "v3rf": 3.25, "v3d": 3.135}   # 3V3_D: CS5361 VD minimum (datasheet 3.135 V)
LEAK_MAX_A = 1e-3
# thermal: θJA from the datasheets (MIC5205 SOT-23-5 220 °C/W; AMS1117 SOT-223 ≈ 90 °C/W); 50 °C ambient
AMBIENT_C, TJ_MAX_C = 50.0, 125.0
F_SW = 1.2e6                  # MT3608 (typical; PFM at light load)
BOOST_L = 10e-6
SPUR_MARGIN_DB = 20.0         # conducted spurs at least this far below MDS
MDS_DBM = -140.5
BANDS = {"160m": 1.9e6, "80m": 3.65e6, "60m": 5.35e6, "40m": 7.1e6, "30m": 10.12e6, "20m": 14.1e6, "17m": 18.1e6,
         "15m": 21.2e6, "12m": 24.94e6, "10m": 28.5e6, "6m": 50.1e6}
SCENARIOS = {
    "usb 4.75 V, typical load": (4.75, None, LOADS_TYP),
    "usb 4.75 V, maximum load": (4.75, None, LOADS_MAX),
    "usb 5.0 + ext 5.0 V, typical": (5.0, 5.0, LOADS_TYP),
    "usb 5.25 + ext 4.75 V, typical": (5.25, 4.75, LOADS_TYP),
    "ext 4.75 V only, maximum load": (None, 4.75, LOADS_MAX),
    "usb 5.0 V + ext reversed": (5.0, -5.0, LOADS_TYP),
}


def psrr_db(f):
    """MIC5205 ripple rejection: 75 dB at 100 Hz (datasheet); assumed −20 dB/decade above 1 kHz to a
    15 dB floor (no curve in the datasheet text)."""
    return np.clip(75 - 20 * np.log10(np.maximum(f, 1e3) / 1e3), 15, 75)


def _dc_overrides(blk):
    lb = next(p for p in blk.circuit.parts if getattr(p, "dc_open", False))
    return {lb.ref: lambda n: [f"R{lb.ref}_open {n['1']} {n['2']} 1e9"]}


def dc_scenarios(blk) -> dict:
    P = {k: node_name(v) for k, v in blk.ports.items()}
    ports = [Load(blk.ports["vbus"], "vbus_hz", 1e9), Load(blk.ports["ext_tip"], "tip_hz", 1e9)]
    decks = []
    for name, (vu, ve, loads) in SCENARIOS.items():
        lines = [f"VUSB {P['vbus']} 0 dc {vu}" if vu is not None else f"RUSB_open {P['vbus']} 0 1e6",
                 f"VEXT {P['ext_tip']} 0 dc {ve}" if ve is not None else f"REXT_open {P['ext_tip']} 0 1e6"]
        lines += [f"ILD_{k} {P[k]} 0 dc {i}" for k, i in loads.items()] + [".options gmin=1e-12 itl1=500"]
        decks.append(to_deck(blk.circuit, f"power {name}", ports, [Op()], extra_lines=lines,
                             overrides=_dc_overrides(blk))[0])
    out = {}
    for name, r in zip(SCENARIOS, run_decks(decks, ["a0.raw"])):
        d = r["a0.raw"][0].data
        out[name] = {k: float(d[f"v({P[k]})"][0].real) for k in ("main", "v7", "v5a", "v3rf", "v3d", "bias5")}
        out[name]["i_usb"] = float(-d["i(vusb)"][0].real) if "i(vusb)" in d else 0.0
        out[name]["i_ext"] = float(-d["i(vext)"][0].real) if "i(vext)" in d else 0.0
    return out


def junction_temps(dc: dict) -> dict:
    worst = dc["usb 4.75 V, maximum load"]
    hot = dc["usb 5.0 V + ext reversed"]          # highest 5V_MAIN of the maximum-loss cases
    p_mic_a = (worst["v7"] - worst["v5a"]) * LOADS_MAX["v5a"]
    p_ams = (max(hot["main"], worst["main"]) - worst["v3d"]) * LOADS_MAX["v3d"]
    return {"MIC5205-5.0": AMBIENT_C + 220 * p_mic_a, "AMS1117-3.3": AMBIENT_C + 90 * p_ams,
            "MIC5205-3.3": AMBIENT_C + 220 * (worst["main"] - worst["v3rf"]) * LOADS_MAX["v3rf"]}


def ripple_spurs(blk) -> dict:
    """Conducted boost ripple → RF path, per amateur band (the harmonic of F_SW nearest the band):
    the diode-current pulses into the boost output and the inductor's triangular input current are
    driven into the simulated filter network; the 5V_A ripple reaches the RF path through the
    preamp's and the mixer bias network's supply rejection, and 5V_MAIN ripple through the bias-T
    feed. Spurs are referred to the antenna (equivalent available power)."""
    P = {k: node_name(v) for k, v in blk.ports.items()}
    mt = next(p for p in blk.circuit.parts if p.value == "MT3608")
    ov = {**_dc_overrides(blk), mt.ref: lambda n: [f"R{mt.ref}_o {n['IN']} {n['GND']} 1e9"]}
    usb = [f"VUSB {P['vbus']}_src 0 dc 4.75", f"RUSB {P['vbus']}_src {P['vbus']}_l 0.2",
           f"LUSB {P['vbus']}_l {P['vbus']} 1u", f"REXT {P['ext_tip']} 0 1e6"]       # USB cable: 0.2 Ω, 1 µH
    ports = [Load(blk.ports["vbus"], "vb", 1e9), Load(blk.ports["ext_tip"], "tp", 1e9)]
    ac = Ac(0.5e6, 70e6, 200)
    decks = [to_deck(blk.circuit, "ripple out", ports, [ac], extra_lines=usb + [f"IRIP 0 {P['v7']} dc 0 ac 1"],
                     overrides=ov)[0],
             to_deck(blk.circuit, "ripple in", ports, [ac], extra_lines=usb + [f"IRIP 0 {P['boost_in']} dc 0 ac 1"],
                     overrides=ov)[0]]
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
    z_bias = tr(res[1], P["bias5"])
    t_pre = tr(res[2], node_name(pre.ports["rf_out"]))
    t_bias = tr(res[3], node_name(bt.ports["rf_out"]))
    t_mix = tr(res[4], node_name(mx.ports["a"]))
    v_in, v_out = 4.65, 7.0
    duty = 1 - v_in / v_out
    i_l = LOADS_MAX["v5a"] * 5.0 / v_out / (1 - duty)
    di = v_in * duty / (BOOST_L * F_SW)
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
        c_out = 2 * i_l * abs(math.sin(math.pi * n * (1 - duty))) / (math.pi * n)
        c_in = di * abs(math.sin(math.pi * n * duty)) / (math.pi ** 2 * n ** 2 * duty * (1 - duty))
        v5a = c_out * at(z_a, fn) * 10 ** (-float(psrr_db(fn)) / 20)
        paths = {"preamp": dbm(v5a * at(t_pre, fn) / front(fn)), "mixer": dbm(v5a * at(t_mix, fn) / front(fn)),
                 "bias_tee": dbm(c_in * at(z_bias, fn) * at(t_bias, fn))}
        out[band] = {"harmonic": n, "f_hz": fn, **paths,
                     "total_dbm": 10 * math.log10(sum(10 ** (p / 10) for p in paths.values()))}
    return out


def run(out_dir: Path, char_dir: Path) -> BlockResult:
    out_dir, char_dir = Path(out_dir), Path(char_dir)
    blk = power.build()
    dc = dc_scenarios(blk)
    tj = junction_temps(dc)
    spurs = ripple_spurs(blk)
    rail_min = {k: min(s[k] for s in dc.values()) for k in RAIL_MIN_V}
    usb_typ = dc["usb 4.75 V, typical load"]["i_usb"]
    leak_usb = dc["usb 5.25 + ext 4.75 V, typical"]["i_usb"]
    leak_rev = abs(dc["usb 5.0 V + ext reversed"]["i_ext"])
    worst_spur = max(s["total_dbm"] for s in spurs.values())
    checks = [CheckResult(f"{k}_min_v", v, f"≥ {RAIL_MIN_V[k]:g}", v >= RAIL_MIN_V[k], "V") for k, v in rail_min.items()]
    checks += [
        CheckResult("usb_current_typical_ma", usb_typ * 1e3, f"≤ {USB2_LIMIT_A * 1e3:g} (USB 2.0)", usb_typ <= USB2_LIMIT_A, "mA"),
        CheckResult("usb_current_max_ma", dc["usb 4.75 V, maximum load"]["i_usb"] * 1e3,
                    "report (active antenna needs the jack or a 1.5 A USB-C host)", True, "mA"),
        CheckResult("usb_draw_with_ext_ma", leak_usb * 1e3, f"≤ {LEAK_MAX_A * 1e3:g}", leak_usb <= LEAK_MAX_A, "mA"),
        CheckResult("reversed_ext_current_ma", leak_rev * 1e3, f"≤ {LEAK_MAX_A * 1e3:g}", leak_rev <= LEAK_MAX_A, "mA"),
    ]
    checks += [CheckResult(f"tj_{k}", t, f"≤ {TJ_MAX_C:g}", t <= TJ_MAX_C, "°C") for k, t in tj.items()]
    checks.append(CheckResult("conducted_spur_worst_dbm", worst_spur, f"≤ MDS − {SPUR_MARGIN_DB:g} = {MDS_DBM - SPUR_MARGIN_DB:g}",
                              worst_spur <= MDS_DBM - SPUR_MARGIN_DB, "dBm"))
    result = BlockResult("power", checks)
    data = availability.load()
    result.artefacts = {
        "kicad_netlist": export.kicad_netlist(blk.circuit, out_dir / "power.net"),
        "bom": export.write_bom(blk.circuit, out_dir / "power_bom.csv", data),
        "svg": export.svg_diagram(blk.circuit, out_dir / "power.svg"),
        "characterisation": charfile.write(char_dir / "power.json", "power", "pass" if result.passed else "fail", {
            "loads_typ_a": LOADS_TYP, "loads_max_a": LOADS_MAX, "scenarios": dc, "junction_c": tj,
            "spurs_dbm": spurs, "psrr_assumption": "MIC5205 75 dB at 100 Hz (datasheet), assumed -20 dB/dec above 1 kHz to 15 dB",
            "spur_note": "conducted only; radiated/layout coupling is checked in the layout phase",
            "checks": [c.__dict__ for c in checks]}),
    }
    return result
