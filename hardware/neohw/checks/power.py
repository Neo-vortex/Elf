"""Power check (phase 3 spec §3.2): input switch-over and back-feed, rail voltages at minimum input
and maximum load, regulator junction temperatures, conducted boost-ripple spurs per amateur band,
and the current budget. Writes power.json."""
from __future__ import annotations

import json
import math
from pathlib import Path

import numpy as np

from neohw import availability, charfile, export, parts
from neohw.blocks import bias_tee, mixer, power, preamp
from neohw.checks import BlockResult, CheckResult
from neohw.ngspice import run_decks
from neohw.spice import Ac, Load, Op, Tran, node_name, to_deck
from neohw.touchstone import read_s2p

# rail loads (A), from datasheet currents: 5V_A = CS5361 VA 50 mA + 2 × LM4562 20 mA + preamp 22 mA;
# 3V3_RF = Si5351C 35 mA + logic; 3V3_D continuous maximum = ESP32-S3 240 MHz dual core, radio off
# (≈ 107 mA) + CS5361 VL/VD 17 mA + 24.576 MHz oscillator 10 mA + 74HC595/ULN2803 inputs and LED 8 mA
# ≈ 0.15 A, with short peaks (boot, flash writes) to 0.25 A within the TLV74033's 0.3 A rating; 5V_MAIN = G6K-2F coils, 28 mA each (2 filter relays + preamp typ, + attenuator
# max); BIAS = active antenna (0 typ, 100 mA max)
LOADS_TYP = {"v5a": 0.092, "v3rf": 0.040, "v3d": 0.130, "main": 0.085, "bias5": 0.0}
LOADS_MAX = {"v5a": 0.110, "v3rf": 0.050, "v3d": 0.150, "main": 0.112, "bias5": 0.100}
USB2_LIMIT_A = 0.5
V3D_PEAK_A, TLV74033_RATING_A = 0.25, 0.30
RAIL_MIN_V = {"v5a": 4.95, "v3rf": 3.25, "v3d": 3.25}     # every rail in regulation (LDO output ±1.5 %)
FUSE_PULSE_MARGIN = 5.0       # inrush I²t at most 1/5 of the fuse's melting I²t (pulse derating)
HANDOVER_MIN_V3D = 3.1        # during a source hand-over: CS5361 VD minimum (ESP32-S3 3.0 V)
LEAK_MAX_A = 1e-3
# thermal: θJA from the datasheets (MIC5205 SOT-23-5 220 °C/W, TLV74033 SOT-23-5 231 °C/W); 50 °C ambient
AMBIENT_C, TJ_MAX_C = 50.0, 125.0
THETA_JA = {"MIC5205-5.0": 220.0, "MIC5205-3.3": 220.0, "TLV74033": 231.0}
# regulator: (input node, output node, load key)
REGULATORS = {"MIC5205-5.0": ("ldo_a_in", "v5a", "v5a"), "MIC5205-3.3": ("ldo_rf_in", "v3rf", "v3rf"),
              "TLV74033": ("main", "v3d", "v3d")}
F_SW = 1.2e6                  # MT3608 (typical; PFM at light load)
BOOST_L = 10e-6
SPUR_MARGIN_DB = 20.0         # conducted spurs at least this far below MDS
MDS_DBM = -140.5
BANDS = {"160m": 1.9e6, "80m": 3.65e6, "60m": 5.35e6, "40m": 7.1e6, "30m": 10.12e6, "20m": 14.1e6, "17m": 18.1e6,
         "15m": 21.2e6, "12m": 24.94e6, "10m": 28.5e6, "6m": 50.1e6}
# name: (USB volts or None, external volts or None, loads, extra deck lines)
SS34_HOT_LEAK_A = 2e-3       # SS34 reverse leakage, 50 °C ambient (0.5 mA max at 25 °C, about doubling per 10 °C)
SCENARIOS = {
    "usb 4.75 V, typical load": (4.75, None, LOADS_TYP, []),
    "usb 4.75 V, maximum load": (4.75, None, LOADS_MAX, []),
    "usb 5.25 V, maximum load": (5.25, None, LOADS_MAX, []),
    "usb 5.0 + ext 5.0 V, typical": (5.0, 5.0, LOADS_TYP, []),
    "usb 5.25 + ext 4.75 V, typical": (5.25, 4.75, LOADS_TYP, []),
    "usb 4.4 + ext 5.25 V, typical": (4.4, 5.25, LOADS_TYP, []),
    "ext 4.75 V only, maximum load": (None, 4.75, LOADS_MAX, []),
    "ext 5.25 V only, maximum load": (None, 5.25, LOADS_MAX, []),
    "usb 5.0 V + ext reversed": (5.0, -5.0, LOADS_TYP, []),
    "usb 4.75 V, maximum load, SS34 leaking 2 mA": (4.75, None, LOADS_MAX, ["ILEAK_ss34 {main} {ext_p} dc %g" % SS34_HOT_LEAK_A]),
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
    for name, (vu, ve, loads, extra) in SCENARIOS.items():
        lines = [f"VUSB {P['vbus']} 0 dc {vu}" if vu is not None else f"RUSB_open {P['vbus']} 0 1e6",
                 f"VEXT {P['ext_tip']} 0 dc {ve}" if ve is not None else f"REXT_open {P['ext_tip']} 0 1e6"]
        lines += [f"ILD_{k} {P[k]} 0 dc {i}" for k, i in loads.items()] + [x.format(**P) for x in extra]
        lines += [".options gmin=1e-12 itl1=500"]
        decks.append(to_deck(blk.circuit, f"power {name}", ports, [Op()], extra_lines=lines,
                             overrides=_dc_overrides(blk))[0])
    out = {}
    for name, r in zip(SCENARIOS, run_decks(decks, ["a0.raw"])):
        d = r["a0.raw"][0].data
        out[name] = {k: float(d[f"v({P[k]})"][0].real) for k in ("main", "v7", "ldo_a_in", "ldo_rf_in", "v5a", "v3rf", "v3d", "bias5", "ext_p")}
        out[name]["i_usb"] = float(-d["i(vusb)"][0].real) if "i(vusb)" in d else 0.0
        out[name]["i_ext"] = float(-d["i(vext)"][0].real) if "i(vext)" in d else 0.0
    return out


def junction_temps(dc: dict) -> dict:
    """Each regulator's junction at its worst maximum-load scenario: (own input − output) × load."""
    worst = [s for name, s in dc.items() if "maximum load" in name]
    return {reg: AMBIENT_C + THETA_JA[reg] * max(s[i] - s[o] for s in worst) * LOADS_MAX[load]
            for reg, (i, o, load) in REGULATORS.items()}


RAIL_NOMINAL_V = {"v5a": 5.0, "v3rf": 3.3, "v3d": 3.3, "main": 5.0, "bias5": 5.0}
MT3608_SOFT_START_S = 1e-3    # assumed (the datasheet states an internal soft start but no time)
MT3608_UVLO_V = 2.5
MT3608_SWITCH_LIMIT_A = 4.0   # datasheet switch current limit: the input current never exceeds it


def _transient(blk, title: str, vusb: str, vext: str, stop_s: float) -> dict:
    """Start-up and hand-over transients at maximum load (rails as resistors). The averaged MT3608
    gets an under-voltage lock-out and a soft start (an RC state node), so it does not demand
    full current the instant its input appears."""
    P = {k: node_name(v) for k, v in blk.ports.items()}
    mt = next(p for p in blk.circuit.parts if p.value == "MT3608")

    def boost(n):
        on = f"0.5 * (1 + tanh(20 * (v({n['IN']}) - {MT3608_UVLO_V:g})))"
        reg = f"max(0, min(3, 20 * (0.6 - v({n['FB']})))) * min(1, v({mt.ref}_ss))"
        return [f"B{mt.ref}_ssc 0 {mt.ref}_ss I = {on} * (1 - v({mt.ref}_ss)) * 1e-3",
                f"C{mt.ref}_ss {mt.ref}_ss 0 {MT3608_SOFT_START_S * 1e-3:g}", f"R{mt.ref}_ss {mt.ref}_ss 0 1e9",
                f"B{mt.ref}_reg {n['GND']} {n['SW']} I = {reg}",
                f"B{mt.ref}_in {n['IN']} {n['GND']} I = min({MT3608_SWITCH_LIMIT_A:g}, {reg} * v({n['SW']}) / ({0.85:g} * max(v({n['IN']}), 0.5)))",
                f"R{mt.ref}_en {n['EN']} {n['GND']} 1e9"]
    ports = [Load(blk.ports["vbus"], "vbus_hz", 1e9), Load(blk.ports["ext_tip"], "tip_hz", 1e9)]
    lines = [f"VUSB {P['vbus']}_src 0 {vusb}", f"RUSB {P['vbus']}_src {P['vbus']}_l 0.2",   # USB cable: 0.2 Ω, 1 µH
             f"LUSB {P['vbus']}_l {P['vbus']} 1u", f"VEXT {P['ext_tip']} 0 {vext}"]
    lines += [f"RLD_{k} {P[k]} 0 {RAIL_NOMINAL_V[k] / i:g}" for k, i in LOADS_MAX.items()]
    deck = to_deck(blk.circuit, title, ports, [Tran(20e-6, stop_s, 0.0, 2e-6)], extra_lines=lines,
                   overrides={**_dc_overrides(blk), mt.ref: boost})[0]
    d = run_decks([deck], ["a0.raw"])[0]["a0.raw"][0].data
    return {"t": d["time"].real, "i_usb": -d["i(vusb)"].real,
            **{k: d[f"v({P[k]})"].real for k in ("main", "v3d", "v5a", "v3rf")}}


def usb_inrush(blk) -> dict:
    """USB plugged in (VBUS 0 → 5 V in 50 µs at 0.1 ms), no jack: peak VBUS current and the time
    every rail is up."""
    w = _transient(blk, "usb plug-in", "pwl(0 0 0.1m 0 0.15m 5)", "dc 0", 30e-3)
    up = ("v5a", "v3rf", "v3d")
    t_ok = max(float(w["t"][np.argmax(w[k] >= RAIL_MIN_V[k])]) if (w[k] >= RAIL_MIN_V[k]).any() else math.inf for k in up)
    i2t = float(np.trapezoid(np.clip(w["i_usb"] - 0.5, 0, None) ** 2, w["t"]))     # above steady draw
    return {"peak_a": float(w["i_usb"].max()), "i2t_a2s": i2t, "rails_up_s": t_ok}


def ext_unplug(blk) -> dict:
    """Jack and USB both at 5 V, the jack pulled at 10 ms (5 → 0 V in 10 µs): 3V3_D must not dip
    below the CS5361/ESP32 minimum while USB takes over."""
    w = _transient(blk, "jack unplug", "dc 5", "pwl(0 5 10m 5 10.01m 0)", 25e-3)
    after = w["t"] >= 10e-3
    return {"v3d_min": float(w["v3d"][after].min()), "main_min": float(w["main"][after].min())}


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
    inrush, unplug = usb_inrush(blk), ext_unplug(blk)
    fuse_limit = parts.FUSE_1206_SLOW_I2T_MIN / FUSE_PULSE_MARGIN
    leak_usb = max(abs(dc[k]["i_usb"]) for k in ("usb 5.25 + ext 4.75 V, typical", "usb 4.4 + ext 5.25 V, typical"))
    leak_rev = abs(dc["usb 5.0 V + ext reversed"]["i_ext"])
    worst_spur = max(s["total_dbm"] for s in spurs.values())
    checks = [CheckResult(f"{k}_min_v", v, f"≥ {RAIL_MIN_V[k]:g}", v >= RAIL_MIN_V[k], "V") for k, v in rail_min.items()]
    checks += [
        CheckResult("usb_current_typical_ma", usb_typ * 1e3, f"≤ {USB2_LIMIT_A * 1e3:g} (USB 2.0)", usb_typ <= USB2_LIMIT_A, "mA"),
        CheckResult("usb_current_max_ma", dc["usb 4.75 V, maximum load"]["i_usb"] * 1e3,
                    "report (active antenna needs the jack or a 1.5 A USB-C host)", True, "mA"),
        CheckResult("usb_draw_with_ext_ma", leak_usb * 1e3, f"≤ {LEAK_MAX_A * 1e3:g}", leak_usb <= LEAK_MAX_A, "mA"),
        CheckResult("usb_inrush_peak_a", inrush["peak_a"], "report (hard switch; load switch is a Rev A decision)", True, "A"),
        CheckResult("usb_inrush_i2t_a2s", inrush["i2t_a2s"], f"≤ {fuse_limit:g} (time-lag fuse / {FUSE_PULSE_MARGIN:g})",
                    inrush["i2t_a2s"] <= fuse_limit, "A²s"),
        CheckResult("rails_up_after_plug_in_ms", inrush["rails_up_s"] * 1e3, "≤ 10", inrush["rails_up_s"] <= 10e-3, "ms"),
        CheckResult("v3d_min_at_jack_unplug_v", unplug["v3d_min"], f"≥ {HANDOVER_MIN_V3D:g}",
                    unplug["v3d_min"] >= HANDOVER_MIN_V3D, "V"),
        CheckResult("hot_leak_main_v", dc["usb 4.75 V, maximum load, SS34 leaking 2 mA"]["main"], "report", True, "V"),
        CheckResult("reversed_ext_current_ma", leak_rev * 1e3, f"≤ {LEAK_MAX_A * 1e3:g}", leak_rev <= LEAK_MAX_A, "mA"),
    ]
    checks.append(CheckResult("v3d_peak_a", V3D_PEAK_A, f"≤ {TLV74033_RATING_A:g} (TLV74033 rating)",
                              V3D_PEAK_A <= TLV74033_RATING_A, "A"))
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
            "usb_inrush": inrush, "jack_unplug": unplug,
            "spurs_dbm": spurs, "psrr_assumption": "MIC5205 75 dB at 100 Hz (datasheet), assumed -20 dB/dec above 1 kHz to 15 dB",
            "spur_note": "conducted only; radiated/layout coupling is checked in the layout phase",
            "checks": [c.__dict__ for c in checks]}),
    }
    return result
