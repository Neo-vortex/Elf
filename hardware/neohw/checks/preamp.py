"""Preamp check (spec §4, NF amended): gain 14 ± 1 dB over 1–54 MHz, NF ≤ 4.5 dB, OIP3 ≥ +25 dBm,
unconditionally stable 0.1–500 MHz (also at 4.75 and 5.25 V), input RL ≥ 12 dB; output RL and
bypass loss reported. Writes preamp.json (spec §6)."""
from __future__ import annotations

from pathlib import Path

import numpy as np

from neohw import availability, charfile, export
from neohw.blocks import preamp
from neohw.checks import BlockResult, CheckResult, db
from neohw.ngspice import run_decks
from neohw.rfmath import noise_figure_db, stability, two_tone
from neohw.spice import Load, Noise, Op, Port, Source, Sp, Supply, Tran, node_name, to_deck

BAND = (1e6, 54e6)
GAIN_DB, GAIN_TOL_DB = 14.0, 1.0
NF_MAX_DB = 4.5
OIP3_MIN_DBM = 25.0
RL_IN_MIN_DB = 12.0
SUPPLY_V = (4.75, 5.0, 5.25)
STAB_SWEEP = Sp(0.1e6, 500e6, 40, "dec")
TONES_HZ = (14.0e6, 50.0e6)
DF = 0.1e6
TONE_DBM = -20.0
SETTLE_S, WINDOW_S, STEP_S = 10e-6, 20e-6, 0.25e-9
TRAN_OPTIONS = ".options reltol=1e-6 abstol=1e-15 vntol=1e-12"


def _ports(blk, volts):
    return [Port(blk.ports["rf_in"], 1), Port(blk.ports["rf_out"], 2), Supply(blk.ports["vcc"], volts, "5v")]


def _sparams(d):
    return [d[f"s_{a}_{b}"] for a, b in ((1, 1), (2, 1), (1, 2), (2, 2))]


def evaluate() -> dict:
    amp, byp = preamp.build("amp"), preamp.build("bypass")
    decks = [to_deck(amp.circuit, f"preamp {v} V", _ports(amp, v), [Op(), STAB_SWEEP]) for v in SUPPLY_V]
    decks.append(to_deck(byp.circuit, "preamp bypass", _ports(byp, 5.0), [Sp(*BAND, 50, "dec")]))
    src = Source(amp.ports["rf_in"], "rf")
    decks.append(to_deck(amp.circuit, "preamp noise",
                         [src, Load(amp.ports["rf_out"]), Supply(amp.ports["vcc"], 5.0, "5v")],
                         [Noise(amp.ports["rf_out"], src, BAND[0], BAND[1], 20)]))
    for f1 in TONES_HZ:
        tones = Source(amp.ports["rf_in"], "rf", tones=((f1, TONE_DBM), (f1 + DF, TONE_DBM)))
        decks.append(to_deck(amp.circuit, f"preamp two-tone {f1:g}",
                             [tones, Load(amp.ports["rf_out"]), Supply(amp.ports["vcc"], 5.0, "5v")],
                             [Tran(STEP_S, SETTLE_S + WINDOW_S, SETTLE_S)], extra_lines=[TRAN_OPTIONS]))
    out = run_decks([d for d, _ in decks[:3]], decks[0][1])
    corners = []
    for v, res in zip(SUPPLY_V, out):
        op, sp = res["a0.raw"][0].data, res["a1.raw"][0].data
        f = sp["frequency"].real
        s = _sparams(sp)
        k, delta = stability(*s)
        band = (f >= BAND[0]) & (f <= BAND[1])
        corners.append({"volts": v, "f": f, "s": s, "band": band, "k_min": float(k.min()),
                        "delta_max": float(delta.max()),
                        "current_ma": float(-op["i(vsup_5v)"][0].real * 1e3)})
    (byp_res,) = run_decks([decks[3][0]], decks[3][1])
    (nz_res,) = run_decks([decks[4][0]], decks[4][1])
    tt = run_decks([d for d, _ in decks[5:]], decks[5][1])
    bd = byp_res["a0.raw"][0].data
    nz = nz_res["a0.raw"][0].data
    oip3 = []
    for f1, res in zip(TONES_HZ, tt):
        d = res["a0.raw"][0].data
        t, vout = d["time"].real, d[f"v({node_name(amp.ports['rf_out'])})"].real
        n = int(round(WINDOW_S / STEP_S))
        oip3.append(two_tone(vout[t >= SETTLE_S - STEP_S / 2][:n], STEP_S, f1, f1 + DF)["oip3_dbm"])
    return {"corners": corners, "bypass_loss_db": float(-db(bd["s_2_1"]).min()),
            "bypass_loss_max_db": float(-db(bd["s_2_1"]).max()),
            "nf_f": nz["frequency"].real, "nf_db": noise_figure_db(nz["inoise_spectrum"].real),
            "oip3_dbm": dict(zip(TONES_HZ, oip3))}


def run(out_dir: Path, char_dir: Path) -> BlockResult:
    out_dir, char_dir = Path(out_dir), Path(char_dir)
    r = evaluate()
    nominal = next(c for c in r["corners"] if c["volts"] == 5.0)
    f, s, band = nominal["f"], nominal["s"], nominal["band"]
    gain = db(s[1])[band]
    rl_in, rl_out = -db(s[0])[band], -db(s[3])[band]
    nf_max = float(r["nf_db"].max())
    oip3_min = min(r["oip3_dbm"].values())
    k_min = min(c["k_min"] for c in r["corners"])
    d_max = max(c["delta_max"] for c in r["corners"])
    gains = [db(c["s"][1])[c["band"]] for c in r["corners"]]
    g_lo, g_hi = float(min(g.min() for g in gains)), float(max(g.max() for g in gains))
    checks = [
        CheckResult("gain_min_db", g_lo, f"≥ {GAIN_DB - GAIN_TOL_DB:g}", g_lo >= GAIN_DB - GAIN_TOL_DB),
        CheckResult("gain_max_db", g_hi, f"≤ {GAIN_DB + GAIN_TOL_DB:g}", g_hi <= GAIN_DB + GAIN_TOL_DB),
        CheckResult("nf_max_db", nf_max, f"≤ {NF_MAX_DB:g}", nf_max <= NF_MAX_DB),
        CheckResult("oip3_min_dbm", oip3_min, f"≥ {OIP3_MIN_DBM:g}", oip3_min >= OIP3_MIN_DBM, "dBm"),
        CheckResult("stability_k_min", k_min, "> 1", k_min > 1, ""),
        CheckResult("stability_delta_max", d_max, "< 1", d_max < 1, ""),
        CheckResult("input_rl_min_db", float(rl_in.min()), f"≥ {RL_IN_MIN_DB:g}", float(rl_in.min()) >= RL_IN_MIN_DB),
        CheckResult("output_rl_min_db", float(rl_out.min()), "report", True),
        CheckResult("bypass_loss_max_db", r["bypass_loss_max_db"], "report", True),
        CheckResult("supply_current_ma", nominal["current_ma"], "report", True, "mA"),
    ]
    fb = f[band]
    nf_on_grid = np.interp(fb, r["nf_f"], r["nf_db"])
    result = BlockResult("preamp", checks, plots={"gain (dB)": (fb, gain), "NF (dB)": (fb, nf_on_grid),
                                                   "input RL (dB)": (fb, rl_in)}, logx=True)
    blk = preamp.build("amp")
    data = availability.load()
    result.artefacts = {
        "kicad_netlist": export.kicad_netlist(blk.circuit, out_dir / "preamp.net"),
        "bom": export.write_bom(blk.circuit, out_dir / "preamp_bom.csv", data),
        "svg": export.svg_diagram(blk.circuit, out_dir / "preamp.svg"),
        "characterisation": charfile.write(
            char_dir / "preamp.json", "preamp", "pass" if result.passed else "fail",
            {"freq_hz": fb.tolist(), "gain_db": gain.tolist(), "nf_db": nf_on_grid.tolist(),
             "oip3_dbm": oip3_min, "bypass_loss_db": r["bypass_loss_max_db"],
             "checks": [c.__dict__ for c in checks]}),
    }
    return result
