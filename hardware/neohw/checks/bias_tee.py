"""Bias-T check (spec §4): shunt impedance > 500 Ω and insertion loss < 0.3 dB over 0.1–54 MHz;
DC drop through the choke chain at 100 mA antenna current ≤ 0.5 V."""
from __future__ import annotations

from pathlib import Path

from neohw import availability, export
from neohw.blocks import bias_tee
from neohw.checks import BlockResult, CheckResult, db
from neohw.rfmath import shunt_impedance_at_port1
from neohw.ngspice import run_decks
from neohw.spice import Load, Op, Port, Sp, Supply, node_name, simulate, to_deck

SWEEP = Sp(0.1e6, 54e6, 100, "dec")
SHUNT_MIN_OHMS = 500.0
IL_MAX_DB = 0.3
DC_LOAD_A = 0.1
DC_DROP_MAX_V = 0.5


def dc_drop_v(load_a: float = DC_LOAD_A) -> float:
    """Feed voltage minus antenna-port voltage with `load_a` drawn at the connector."""
    blk = bias_tee.build()
    deck, raws = to_deck(blk.circuit, "bias-T DC", [Supply(blk.ports["bias_feed"], 5.0, "feed"),
                                                    Load(blk.ports["rf_out"])],
                         [Op()], extra_lines=[f"ILOAD {node_name(blk.ports['rf_in'])} 0 dc {load_a:g}"])
    (res,) = run_decks([deck], raws)
    return 5.0 - float(res[raws[0]][0].data[f"v({node_name(blk.ports['rf_in'])})"][0].real)


def run(out_dir: Path, char_dir: Path) -> BlockResult:
    out_dir = Path(out_dir)
    blk = bias_tee.build()
    (plot,) = simulate(blk.circuit, "bias-T", [Port(blk.ports["rf_in"], 1), Port(blk.ports["rf_out"], 2)], [SWEEP])
    d = plot.data
    f = d["frequency"].real
    s = [d[f"s_{a}_{b}"] for a, b in ((1, 1), (2, 1), (1, 2), (2, 2))]
    z = shunt_impedance_at_port1(*s)
    il = -db(s[1])
    checks = [
        CheckResult("shunt_impedance_min_ohm", float(z.min()), f"> {SHUNT_MIN_OHMS:g}", bool(z.min() > SHUNT_MIN_OHMS), "Ω"),
        CheckResult("insertion_loss_max_db", float(il.max()), f"< {IL_MAX_DB:g}", bool(il.max() < IL_MAX_DB)),
    ]
    drop = dc_drop_v()
    checks.append(CheckResult("dc_drop_at_100ma_v", drop, f"≤ {DC_DROP_MAX_V:g}", drop <= DC_DROP_MAX_V, "V"))
    result = BlockResult("bias_tee", checks, plots={"shunt |Z| (dB re 1 Ω)": (f, db(z)), "S21 (dB)": (f, db(s[1]))},
                         logx=True)
    data = availability.load()
    result.artefacts = {
        "kicad_netlist": export.kicad_netlist(blk.circuit, out_dir / "bias_tee.net"),
        "bom": export.write_bom(blk.circuit, out_dir / "bias_tee_bom.csv", data),
        "svg": export.svg_diagram(blk.circuit, out_dir / "bias_tee.svg"),
    }
    return result
