"""Attenuator check (spec §4): 20 ± 0.5 dB, RL ≥ 18 dB, bypass ≤ 0.2 dB over 0.1–54 MHz."""
from __future__ import annotations

from pathlib import Path

from neohw import availability, charfile, export
from neohw.blocks import attenuator
from neohw.checks import BlockResult, CheckResult, db
from neohw.spice import Port, Sp, simulate

SWEEP = Sp(0.1e6, 54e6, 201, "lin")


def _sparams(state: str):
    blk = attenuator.build(state)
    (plot,) = simulate(blk.circuit, f"attenuator {state}",
                       [Port(blk.ports["rf_in"], 1), Port(blk.ports["rf_out"], 2)], [SWEEP])
    return blk, plot.data["frequency"].real, plot.data["s_2_1"], plot.data["s_1_1"]


def run(out_dir: Path, char_dir: Path) -> BlockResult:
    out_dir, char_dir = Path(out_dir), Path(char_dir)
    pad_blk, f, s21_pad, s11_pad = _sparams("energised")
    _, _, s21_byp, _ = _sparams("released")
    att = -db(s21_pad)
    rl = -db(s11_pad)
    byp = -db(s21_byp)
    checks = [
        CheckResult("attenuation_min_db", float(att.min()), "≥ 19.5", bool(att.min() >= 19.5)),
        CheckResult("attenuation_max_db", float(att.max()), "≤ 20.5", bool(att.max() <= 20.5)),
        CheckResult("pad_return_loss_db", float(rl.min()), "≥ 18", bool(rl.min() >= 18)),
        CheckResult("bypass_loss_db", float(byp.max()), "≤ 0.2", bool(byp.max() <= 0.2)),
    ]
    result = BlockResult("attenuator", checks,
                         plots={"pad S21 (dB)": (f, db(s21_pad)), "pad S11 (dB)": (f, db(s11_pad)),
                                "bypass S21 (dB)": (f, db(s21_byp))})
    data = availability.load()
    result.artefacts = {
        "kicad_netlist": export.kicad_netlist(pad_blk.circuit, out_dir / "attenuator.net"),
        "bom": export.write_bom(pad_blk.circuit, out_dir / "attenuator_bom.csv", data),
        "svg": export.svg_diagram(pad_blk.circuit, out_dir / "attenuator.svg"),
        "characterisation": charfile.write(
            char_dir / "attenuator.json", "attenuator", "pass" if result.passed else "fail",
            {"freq_hz": f.tolist(), "pad_db": (-att).tolist(), "bypass_db": (-byp).tolist(),
             "checks": [c.__dict__ for c in checks]}),
    }
    return result
