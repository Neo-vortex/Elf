"""Preamp check (spec §4, amended after the final review of plan 2.3). Over every corner of
supply (4.75 / 5.0 / 5.25 V) × BFR92A hFE (65 / 94.7 / 150) × temperature (0 / 27 / 50 °C):
gain 14 ± 1 dB over 1–54 MHz, NF ≤ 5.0 dB (IEEE, 290 K source), OIP3 ≥ +22 dBm (two tones at 14 and 50 MHz),
K > 1 and |Δ| < 1 from 0.1 MHz to 6 GHz, collector current < 25 mA (BFR92A maximum);
input RL ≥ 12 dB at the nominal corner. Output RL, bypass loss and supply current reported.
Writes preamp.json (spec §6) from the nominal corner."""
from __future__ import annotations

import itertools
from pathlib import Path

import numpy as np

from neohw import availability, charfile, export, parts
from neohw.blocks import preamp
from neohw.checks import BlockResult, CheckResult, db
from neohw.ngspice import run_decks
from neohw.rfmath import noise_figure_db, stability, two_tone
from neohw.spice import Load, Noise, Op, Port, Source, Sp, Supply, Tran, node_name, to_deck

BAND = (1e6, 54e6)
# characterisation grid for the twin: down to the bank's 0.1 MHz and up to 5x 54 MHz (the mixer's
# 5th-harmonic response folds the preamp's output noise there into the window)
CHAR_BAND = (0.1e6, 300e6)
GAIN_DB, GAIN_TOL_DB = 14.0, 1.0
NF_MAX_DB = 5.0
OIP3_MIN_DBM = 22.0
RL_IN_MIN_DB = 12.0
IC_MAX_MA = parts.BFR92A_IC_MAX * 1e3
NOMINAL = (5.0, 94.733, 27.0)
CORNERS = list(itertools.product((4.75, 5.0, 5.25), (65.0, 94.733, 150.0), (0.0, 27.0, 50.0)))
STAB_SWEEP = Sp(0.1e6, 6e9, 40, "dec")
TONES_HZ = (14.0e6, 50.0e6)
DF = 0.1e6
TONE_DBM = -20.0
SETTLE_S, WINDOW_S, STEP_S = 10e-6, 20e-6, 0.25e-9
TRAN_OPTIONS = ".options reltol=1e-6 abstol=1e-15 vntol=1e-12"


def _supply(blk, volts):
    return Supply(blk.ports["vcc"], volts, "5v")


def _temp(t):
    return f".options temp={t:g}"


def _sparams(d):
    return [d[f"s_{a}_{b}"] for a, b in ((1, 1), (2, 1), (1, 2), (2, 2))]


def corner_decks(volts: float, bf: float, temp: float) -> dict:
    """Build (serially: SKiDL is not thread-safe) the S-parameter, noise and two-tone decks."""
    amp = preamp.build("amp", bf=bf)
    sp = to_deck(amp.circuit, "preamp sp", [Port(amp.ports["rf_in"], 1), Port(amp.ports["rf_out"], 2),
                                            _supply(amp, volts)], [Op(), STAB_SWEEP], extra_lines=[_temp(temp)])
    src = Source(amp.ports["rf_in"], "rf")
    nz = to_deck(amp.circuit, "preamp noise", [src, Load(amp.ports["rf_out"]), _supply(amp, volts)],
                 [Noise(amp.ports["rf_out"], src, CHAR_BAND[0], CHAR_BAND[1], 20)], extra_lines=[_temp(temp)])
    tt = []
    for f1 in TONES_HZ:
        tones = Source(amp.ports["rf_in"], "rf", tones=((f1, TONE_DBM), (f1 + DF, TONE_DBM)))
        tt.append(to_deck(amp.circuit, f"preamp two-tone {f1:g}",
                          [tones, Load(amp.ports["rf_out"]), _supply(amp, volts)],
                          [Tran(STEP_S, SETTLE_S + WINDOW_S, SETTLE_S)], extra_lines=[TRAN_OPTIONS, _temp(temp)]))
    names = {k: node_name(amp.ports[k]) for k in ("sense_hi", "sense_lo", "rf_out")}
    return {"corner": (volts, bf, temp), "sp": sp, "nz": nz, "tt": tt, "nodes": names}


def evaluate(corners=CORNERS) -> list[dict]:
    """All corners: decks built serially, every ngspice run in parallel."""
    built = [corner_decks(*c) for c in corners]
    sp_res = run_decks([b["sp"][0] for b in built], ["a0.raw", "a1.raw"])
    nz_res = run_decks([b["nz"][0] for b in built], ["a0.raw"])
    tt_res = run_decks([d for b in built for d, _ in b["tt"]], ["a0.raw"])
    rs = float(preamp.VALUES["rs"])
    n = int(round(WINDOW_S / STEP_S))
    out = []
    for i, b in enumerate(built):
        op, sp = sp_res[i]["a0.raw"][0].data, sp_res[i]["a1.raw"][0].data
        f = sp["frequency"].real
        s = _sparams(sp)
        k, delta = stability(*s)
        band = (f >= BAND[0]) & (f <= BAND[1])
        char = (f >= CHAR_BAND[0]) & (f <= CHAR_BAND[1])
        nodes = b["nodes"]
        supply_a = (op[f"v({nodes['sense_hi']})"][0].real - op[f"v({nodes['sense_lo']})"][0].real) / rs
        nz = nz_res[i]["a0.raw"][0].data
        nz_f = nz["frequency"].real
        nz_band = (nz_f >= BAND[0]) & (nz_f <= BAND[1])
        nf_all = noise_figure_db(nz["inoise_spectrum"].real, t_kelvin=b["corner"][2] + 273.15)
        oip3 = []
        for j, f1 in enumerate(TONES_HZ):
            d = tt_res[i * len(TONES_HZ) + j]["a0.raw"][0].data
            t, v = d["time"].real, d[f"v({nodes['rf_out']})"].real
            oip3.append(two_tone(v[t >= SETTLE_S - STEP_S / 2][:n], STEP_S, f1, f1 + DF)["oip3_dbm"])
        out.append({"corner": b["corner"], "f": f[band], "gain_db": db(s[1])[band],
                    "rl_in_db": -db(s[0])[band], "rl_out_db": -db(s[3])[band],
                    "k_min": float(k.min()), "delta_max": float(delta.max()),
                    # the sense resistor carries the collector current plus the BC857B's emitter current
                    "collector_current_ma": float(supply_a * 1e3),
                    "char_f": f[char], "char_gain_db": db(s[1])[char],
                    "nf_f_all": nz_f, "nf_db_all": nf_all,
                    "nf_f": nz_f[nz_band], "nf_db": nf_all[nz_band],
                    "oip3_dbm": min(oip3)})
    return out


def bypass_loss_db() -> float:
    byp = preamp.build("bypass")
    deck, raws = to_deck(byp.circuit, "preamp bypass", [Port(byp.ports["rf_in"], 1), Port(byp.ports["rf_out"], 2),
                                                         _supply(byp, 5.0)], [Sp(*BAND, 50, "dec")])
    (res,) = run_decks([deck], raws)
    return float(-db(res[raws[0]][0].data["s_2_1"]).max())


def run(out_dir: Path, char_dir: Path) -> BlockResult:
    out_dir, char_dir = Path(out_dir), Path(char_dir)
    corners = evaluate()
    nom = next(c for c in corners if c["corner"] == NOMINAL)
    g_lo = min(float(c["gain_db"].min()) for c in corners)
    g_hi = max(float(c["gain_db"].max()) for c in corners)
    nf_max = max(float(c["nf_db"].max()) for c in corners)
    oip3_min = min(c["oip3_dbm"] for c in corners)
    k_min = min(c["k_min"] for c in corners)
    d_max = max(c["delta_max"] for c in corners)
    ic_max = max(c["collector_current_ma"] for c in corners)
    rl_in = float(nom["rl_in_db"].min())
    byp = bypass_loss_db()
    checks = [
        CheckResult("gain_min_db", g_lo, f"≥ {GAIN_DB - GAIN_TOL_DB:g}", g_lo >= GAIN_DB - GAIN_TOL_DB),
        CheckResult("gain_max_db", g_hi, f"≤ {GAIN_DB + GAIN_TOL_DB:g}", g_hi <= GAIN_DB + GAIN_TOL_DB),
        CheckResult("nf_max_db", nf_max, f"≤ {NF_MAX_DB:g}", nf_max <= NF_MAX_DB),
        CheckResult("oip3_min_dbm", oip3_min, f"≥ {OIP3_MIN_DBM:g}", oip3_min >= OIP3_MIN_DBM, "dBm"),
        CheckResult("stability_k_min", k_min, "> 1", k_min > 1, ""),
        CheckResult("stability_delta_max", d_max, "< 1", d_max < 1, ""),
        CheckResult("collector_current_max_ma", ic_max, f"< {IC_MAX_MA:g}", ic_max < IC_MAX_MA, "mA"),
        CheckResult("input_rl_min_db", rl_in, f"≥ {RL_IN_MIN_DB:g}", rl_in >= RL_IN_MIN_DB),
        CheckResult("output_rl_min_db", float(nom["rl_out_db"].min()), "report", True),
        CheckResult("bypass_loss_max_db", byp, "report", True),
        CheckResult("supply_current_ma", nom["collector_current_ma"], "report", True, "mA"),
    ]
    fb = nom["f"]
    nf_on_grid = np.interp(fb, nom["nf_f"], nom["nf_db"])
    result = BlockResult("preamp", checks, plots={"gain (dB)": (fb, nom["gain_db"]), "NF (dB)": (fb, nf_on_grid),
                                                   "input RL (dB)": (fb, nom["rl_in_db"])}, logx=True)
    blk = preamp.build("amp")
    data = availability.load()
    result.artefacts = {
        "kicad_netlist": export.kicad_netlist(blk.circuit, out_dir / "preamp.net"),
        "bom": export.write_bom(blk.circuit, out_dir / "preamp_bom.csv", data),
        "svg": export.svg_diagram(blk.circuit, out_dir / "preamp.svg"),
        "characterisation": charfile.write(
            char_dir / "preamp.json", "preamp", "pass" if result.passed else "fail",
            {"freq_hz": nom["char_f"].tolist(), "gain_db": nom["char_gain_db"].tolist(),
             "nf_db": np.interp(nom["char_f"], nom["nf_f_all"], nom["nf_db_all"]).tolist(),
             "grid_note": "nominal corner; the checks cover 1–54 MHz, the grid extends to 0.1–300 MHz for the twin",
             "oip3_dbm": nom["oip3_dbm"], "oip3_min_corner_dbm": oip3_min, "bypass_loss_db": byp,
             "checks": [c.__dict__ for c in checks]}),
    }
    return result
