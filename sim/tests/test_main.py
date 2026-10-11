from pathlib import Path

import pytest

from neosim.__main__ import main
from neosim.tools import ToolNotFoundError, tool_path

SCENARIOS = Path(__file__).resolve().parents[1] / "scenarios"


def test_bad_scenario_file_returns_error(tmp_path, capsys):
    bad = tmp_path / "bad.yaml"
    bad.write_text("name: x\n")
    assert main(["run", str(bad), "--workdir", str(tmp_path), "--report", str(tmp_path / "r.html")]) == 2
    assert "lo_hz" in capsys.readouterr().err


@pytest.mark.integration
def test_run_writes_report(tmp_path):
    try:
        for name in ("gen_ft8", "decode_ft8", "neoradio-dsp"):
            tool_path(name)
    except ToolNotFoundError as e:
        pytest.skip(str(e))
    report = tmp_path / "report.html"
    code = main(["run", str(SCENARIOS / "20m_ft8_cw_ideal.yaml"), "--workdir", str(tmp_path), "--report", str(report)])
    assert code == 0
    assert report.exists()
