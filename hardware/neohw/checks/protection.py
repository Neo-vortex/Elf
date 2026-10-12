"""Limiter check (spec §4): IMD3 products at −10 dBm two-tone ≤ −100 dBm; strong signals are
clamped (output peak ≤ 2.5 V at +27 dBm available); small-signal loss reported."""
from __future__ import annotations

from pathlib import Path

from neohw import availability, export
from neohw.blocks import protection
from neohw.checks import BlockResult, CheckResult, db
from neohw.ngspice import run_decks
from neohw.rfmath import two_tone
from neohw.spice import Load, Port, Source, Sp, Tran, to_deck

TONE_DBM = -10.0
F1, DF = 14.0e6, 0.1e6
SETTLE_S, WINDOW_S, STEP_S = 10e-6, 20e-6, 0.25e-9
IMD3_MAX_DBM = -100.0
CLAMP_TEST_DBM = 27.0
CLAMP_MAX_V = 2.5
TRAN_OPTIONS = ".options reltol=1e-6 abstol=1e-15 vntol=1e-12"


def imd3(blk=None) -> dict:
    blk = blk or protection.build()
    src = Source(blk.ports["rf_in"], "rf", tones=((F1, TONE_DBM), (F1 + DF, TONE_DBM)))
    deck, raws = to_deck(blk.circuit, "limiter two-tone", [src, Load(blk.ports["rf_out"])],
                         [Tran(STEP_S, SETTLE_S + WINDOW_S, SETTLE_S)], extra_lines=[TRAN_OPTIONS])
    (res,) = run_decks([deck], raws)
    d = res[raws[0]][0].data
    t, v = d["time"].real, d["v(rf)"].real
    n = int(round(WINDOW_S / STEP_S))
    return two_tone(v[t >= SETTLE_S - STEP_S / 2][:n], STEP_S, F1, F1 + DF)


def clamp_peak_v(level_dbm: float, blk=None) -> float:
    """Output peak voltage for a single 14 MHz tone of `level_dbm` available power."""
    blk = blk or protection.build()
    src = Source(blk.ports["rf_in"], "rf", tones=((F1, level_dbm),))
    deck, raws = to_deck(blk.circuit, "limiter clamp", [src, Load(blk.ports["rf_out"])],
                         [Tran(0.1e-9, 3e-6, 2e-6)])
    (res,) = run_decks([deck], raws)
    return float(abs(res[raws[0]][0].data["v(rf)"].real).max())


def run(out_dir: Path, char_dir: Path) -> BlockResult:
    out_dir = Path(out_dir)
    blk = protection.build()
    r = imd3(blk)
    peak = clamp_peak_v(CLAMP_TEST_DBM, blk)
    deck, raws = to_deck(blk.circuit, "limiter sp", [Port(blk.ports["rf_in"], 1), Port(blk.ports["rf_out"], 2)],
                         [Sp(0.1e6, 54e6, 50, "dec")])
    (res,) = run_decks([deck], raws)
    d = res[raws[0]][0].data
    il = float(-db(d["s_2_1"]).max())
    checks = [CheckResult("imd3_at_minus10dbm_dbm", r["imd3_dbm"], f"≤ {IMD3_MAX_DBM:g}", r["imd3_dbm"] <= IMD3_MAX_DBM, "dBm"),
              CheckResult("clamp_peak_at_27dbm_v", peak, f"≤ {CLAMP_MAX_V:g}", peak <= CLAMP_MAX_V, "V"),
              CheckResult("insertion_loss_max_db", il, "report", True)]
    result = BlockResult("protection", checks)
    data = availability.load()
    result.artefacts = {
        "kicad_netlist": export.kicad_netlist(blk.circuit, out_dir / "protection.net"),
        "bom": export.write_bom(blk.circuit, out_dir / "protection_bom.csv", data),
        "svg": export.svg_diagram(blk.circuit, out_dir / "protection.svg"),
    }
    return result
