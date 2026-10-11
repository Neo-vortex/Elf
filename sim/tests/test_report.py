import numpy as np

from neosim.adc import AdcStats
from neosim.channel import Exclusion
from neosim.report import write_report
from neosim.scenario import Scenario
from neosim.sweep import SweepPoint
from neosim.twin import ReceiveResult, ScenarioResult


def test_report_contains_results(tmp_path):
    scen = Scenario("demo scenario", 14_060_000.0, 15.0, 1, -100.0, (), ())
    res = ScenarioResult(
        scen, [Exclusion("bcast", "outside window")], AdcStats(0, -40.0, 1.0),
        [ReceiveResult("rx_ft8", "ft8", "CQ EP2ABC LL35", True, "ok"),
         ReceiveResult("rx_cw", "cw", "CQ TEXT", False, "CER 0.143")],
        np.linspace(-48_000, 48_000, 16), np.zeros(16))
    path = write_report([res], [SweepPoint(-10, 1.0, 1.0), SweepPoint(-20, 0.0, 0.0)], tmp_path / "r.html")
    html = path.read_text()
    assert "demo scenario" in html
    assert "FAIL" in html and "PASS" in html
    assert "bcast" in html and "outside window" in html
    assert "Implementation loss" in html
    assert html.count("data:image/png;base64,") == 2
