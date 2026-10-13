import numpy as np

from neohw.checks import BlockResult, CheckResult
from neohw.report import write_report


def test_report_contains_checks_and_plot(tmp_path):
    svg = tmp_path / "d.svg"
    svg.write_text('<?xml version="1.0"?><svg xmlns="http://www.w3.org/2000/svg"><text>K1</text></svg>')
    bom = tmp_path / "b.csv"
    bom.write_text("refs,value,footprint,mpn,qty,availability\nR1,61.9Ω,fp,mpn,2,unknown\n")
    res = BlockResult("attenuator",
                      [CheckResult("attenuation_min_db", 19.9, "≥ 19.5", True),
                       CheckResult("bypass_loss_db", 0.3, "≤ 0.2", False)],
                      plots={"pad S21 (dB)": (np.array([1e6, 2e6]), np.array([-20.0, -20.1]))},
                      artefacts={"svg": svg, "bom": bom})
    html = write_report([res], tmp_path / "r.html").read_text()
    assert "attenuator" in html and "FAIL" in html and "PASS" in html
    assert "attenuation_min_db" in html and "19.9" in html
    assert "data:image/png;base64," in html and "<svg" in html and "unknown" in html


def test_report_with_log_axis(tmp_path):
    res = BlockResult("bpf_bank", [CheckResult("x", 1.0, "≤ 2", True)],
                      plots={"1": (np.array([1e5, 1e6, 1e7]), np.array([-60.0, -1.0, -60.0]))}, logx=True)
    assert "data:image/png;base64," in write_report([res], tmp_path / "r.html").read_text()


def test_report_plots_a_non_frequency_axis_given_as_lists(tmp_path):
    from neohw.checks import BlockResult, CheckResult
    from neohw.report import write_report
    r = BlockResult("controller", [CheckResult("x", 1.0, "report", True, "")],
                    plots={"sense (V vs mA)": ([0.0, 0.05, 0.1], [0.6, 0.9, 1.1])},
                    x_label="bias-T current (mA)", x_scale=1e-3)
    text = write_report([r], tmp_path / "r.html").read_text()
    assert "controller" in text and "data:image/png" in text


def test_controller_result_uses_a_current_axis():
    from neohw.checks import controller as c
    assert c.PLOT_X_LABEL == "bias-T current (mA)" and c.PLOT_X_SCALE == 1e-3
