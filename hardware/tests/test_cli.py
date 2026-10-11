from neohw import __main__ as cli
from neohw.checks import BlockResult, CheckResult


def _fake(passed):
    def run(out_dir, char_dir):
        return BlockResult("fake", [CheckResult("x", 1.0, "≤ 0", passed)])
    return run


def test_cli_exit_code_reflects_failed_check(tmp_path, monkeypatch):
    monkeypatch.setattr(cli, "CHECKS", {"fake": _fake(False)})
    report = tmp_path / "r.html"
    assert cli.main(["run", "--out", str(tmp_path), "--characterized", str(tmp_path), "--report", str(report)]) == 1
    assert report.exists()


def test_cli_success(tmp_path, monkeypatch):
    monkeypatch.setattr(cli, "CHECKS", {"fake": _fake(True)})
    assert cli.main(["run", "--out", str(tmp_path), "--characterized", str(tmp_path),
                     "--report", str(tmp_path / "r.html")]) == 0


def test_cli_tool_error_is_exit_2(tmp_path, monkeypatch, capsys):
    from neohw.tools import ToolNotFoundError

    def broken(out_dir, char_dir):
        raise ToolNotFoundError("ngspice not found; install it with `sudo apt install ngspice`")
    monkeypatch.setattr(cli, "CHECKS", {"broken": broken})
    assert cli.main(["run", "--out", str(tmp_path), "--characterized", str(tmp_path),
                     "--report", str(tmp_path / "r.html")]) == 2
    assert "sudo apt install ngspice" in capsys.readouterr().err
