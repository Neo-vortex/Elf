"""Bias-T check (spec §4): shunt impedance > 500 Ω and insertion loss < 0.3 dB over 0.1–54 MHz."""
from __future__ import annotations

from pathlib import Path

from neohw import availability, export
from neohw.blocks import bias_tee
from neohw.checks import BlockResult, CheckResult, db
from neohw.rfmath import shunt_impedance_at_port1
from neohw.spice import Port, Sp, simulate

SWEEP = Sp(0.1e6, 54e6, 100, "dec")
SHUNT_MIN_OHMS = 500.0
IL_MAX_DB = 0.3


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
    result = BlockResult("bias_tee", checks, plots={"shunt |Z| (dB re 1 Ω)": (f, db(z)), "S21 (dB)": (f, db(s[1]))},
                         logx=True)
    data = availability.load()
    result.artefacts = {
        "kicad_netlist": export.kicad_netlist(blk.circuit, out_dir / "bias_tee.net"),
        "bom": export.write_bom(blk.circuit, out_dir / "bias_tee_bom.csv", data),
        "svg": export.svg_diagram(blk.circuit, out_dir / "bias_tee.svg"),
    }
    return result
