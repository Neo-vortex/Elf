"""Clock and LO path check (phase 3 spec §3.1): frequency-planner coverage on the 1 kHz grid,
select-line skew budget from the datasheets, calibration-pad level and loading. Writes
clock.json."""
from __future__ import annotations

import math
from pathlib import Path

from neohw import availability, charfile, export, lo_plan
from neohw.blocks import cal_tone, clock
from neohw.checks import BlockResult, CheckResult, db
from neohw.ngspice import run_decks
from neohw.spice import Ac, Load, Port, Source, Sp, _check_nets, node_name, to_deck

CAL_TARGET_DB, CAL_TOL_DB = -70.0, 3.0
CAL_LOADING_MAX_DB = 0.05
CAL_FREQS = (0.1e6, 1e6, 14e6, 30e6, 54e6)
SI5351_ROUT = 50.0             # default high drive strength: 50 Ω output impedance (datasheet note 2, Table 6)


def net_check(blk) -> None:
    """No dangling nets: each net has ≥ 2 pins unless it is a block port."""
    _check_nets(blk.circuit, [], exempt=frozenset(node_name(n) for n in _port_nets(blk)))


def _port_nets(blk):
    for v in blk.ports.values():
        yield from (v if isinstance(v, tuple) else (v,))


def cal_pad_response() -> dict:
    """Attenuation from CLK3's open-circuit voltage to the RF line (terminated 50 Ω each way), and
    the RF line's extra insertion loss with the pad attached and CLK3 driven low."""
    blk = cal_tone.build()
    src = Source(blk.ports["clk"], "clk", z0=SI5351_ROUT)
    ac = to_deck(blk.circuit, "cal pad", [src, Load(blk.ports["rf"], "ant", 50.0), Load(blk.ports["rf"], "flt", 50.0)],
                 [Ac(0.1e6, 54e6, 50)])
    # loading: the RF line as a 2-port with the pad hanging off it (CLK3 low = its 50 Ω to ground)
    sp = to_deck(blk.circuit, "cal pad loading", [Port(blk.ports["rf"], 1), Port(blk.ports["rf"], 2),
                                                  Load(blk.ports["clk"], "clk", SI5351_ROUT)], [Sp(0.1e6, 54e6, 50, "dec")])
    res_ac, res_sp = run_decks([ac[0], sp[0]], ["a0.raw"])
    d = res_ac["a0.raw"][0].data
    f = d["frequency"].real
    h = d[f"v({node_name(blk.ports['rf'])})"]          # source is 1 V open circuit
    import numpy as np
    att = {fc: float(np.interp(math.log(fc), np.log(f), db(h))) for fc in CAL_FREQS}
    s21 = res_sp["a0.raw"][0].data["s_2_1"]
    return {"att_db": att, "rf_insertion_loss_db": float(-db(s21).min()), "freq_hz": f, "h_db": db(h)}


def run(out_dir: Path, char_dir: Path) -> BlockResult:
    out_dir, char_dir = Path(out_dir), Path(char_dir)
    cov = lo_plan.coverage(lo_plan.LO_MIN_HZ, lo_plan.LO_MAX_HZ, 1e3)
    pad = cal_pad_response()
    blk = clock.build()
    net_check(blk)
    att1 = pad["att_db"][1e6]
    worst = {path: sum(stages) for path, stages in clock.SKEW_STAGES_S.items()}
    checks = [
        CheckResult("lo_plan_points", cov["points"], "report", True, ""),
        CheckResult("lo_plan_failures", len(cov["failures"]), "= 0", not cov["failures"], ""),
        CheckResult("lo_plan_max_error_hz", cov["max_error_hz"], "≤ 1", cov["max_error_hz"] <= 1.0, "Hz"),
        CheckResult("direct_min_mhz", lo_plan.DIRECT_MIN_HZ / 1e6, "report (90° phase register limit)", True, "MHz"),
        CheckResult("skew_worst_direct_ns", worst["direct"] * 1e9, "report", True, "ns"),
        CheckResult("skew_worst_div4_ns", worst["div4"] * 1e9, "report", True, "ns"),
        CheckResult("cal_pad_att_1mhz_db", att1, f"{CAL_TARGET_DB:g} ± {CAL_TOL_DB:g}",
                    abs(att1 - CAL_TARGET_DB) <= CAL_TOL_DB),
        CheckResult("cal_pad_att_54mhz_db", pad["att_db"][54e6], "report", True),
        CheckResult("cal_pad_rf_loading_db", pad["rf_insertion_loss_db"], f"≤ {CAL_LOADING_MAX_DB:g}",
                    pad["rf_insertion_loss_db"] <= CAL_LOADING_MAX_DB),
    ]
    result = BlockResult("clock", checks, plots={"cal pad (dB)": (pad["freq_hz"], pad["h_db"])}, logx=True)
    data = availability.load()
    pad_blk = cal_tone.build()
    result.artefacts = {
        "kicad_netlist": export.kicad_netlist(blk.circuit, out_dir / "clock.net"),
        "bom": export.write_bom(blk.circuit, out_dir / "clock_bom.csv", data),
        "svg": export.svg_diagram(blk.circuit, out_dir / "clock.svg"),
        "cal_netlist": export.kicad_netlist(pad_blk.circuit, out_dir / "cal_tone.net"),
        "cal_bom": export.write_bom(pad_blk.circuit, out_dir / "cal_tone_bom.csv", data),
        "characterisation": charfile.write(char_dir / "clock.json", "clock", "pass" if result.passed else "fail", {
            "direct_min_hz": lo_plan.DIRECT_MIN_HZ,
            "skew_stages_s": {k: list(v) for k, v in clock.SKEW_STAGES_S.items()},
            "skew_source": "74LVC74A / 74LVC157A tsk(o) <= 1.0 ns (Nexperia, VCC 3.0-3.6 V) -> +-0.5 ns each; "
                           "Si5351 output skew not specified: +-0.2 ns assumed",
            "lo_plan": {"points": cov["points"], "failures": len(cov["failures"]), "max_error_hz": cov["max_error_hz"]},
            "cal_pad_att_db": {str(int(k)): v for k, v in pad["att_db"].items()},
            "cal_pad_rf_loading_db": pad["rf_insertion_loss_db"],
            "checks": [c.__dict__ for c in checks]}),
    }
    return result
