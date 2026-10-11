import stat

import pytest

from neohw import tools


def _exe(path):
    path.write_text("#!/bin/sh\nexit 0\n")
    path.chmod(path.stat().st_mode | stat.S_IXUSR)
    return path


def test_missing_ngspice_says_how_to_install(monkeypatch, tmp_path):
    monkeypatch.setenv("NEOHW_NGSPICE", str(tmp_path / "nope"))
    with pytest.raises(tools.ToolNotFoundError, match="sudo apt install ngspice"):
        tools.tool_path("ngspice")


def test_missing_dot_says_how_to_install(monkeypatch, tmp_path):
    monkeypatch.setenv("NEOHW_DOT", str(tmp_path / "nope"))
    with pytest.raises(tools.ToolNotFoundError, match="graphviz"):
        tools.tool_path("dot")


def test_env_override_is_used(monkeypatch, tmp_path):
    exe = _exe(tmp_path / "ngspice")
    monkeypatch.setenv("NEOHW_NGSPICE", str(exe))
    assert tools.tool_path("ngspice") == exe


def test_unknown_tool_is_key_error():
    with pytest.raises(KeyError):
        tools.tool_path("kicad")


def test_git_describe_names_last_commit_of_the_sources():
    import subprocess
    want = subprocess.run(["git", "log", "-1", "--format=%h", "--", "hardware/neohw"], cwd=tools.REPO_ROOT,
                          capture_output=True, text=True).stdout.strip()
    dirty = subprocess.run(["git", "status", "--porcelain", "--", "hardware/neohw"], cwd=tools.REPO_ROOT,
                           capture_output=True, text=True).stdout.strip()
    assert tools.git_describe() == (f"{want}-dirty" if dirty else want)


def test_git_describe_is_short_hash_or_unknown():
    d = tools.git_describe()
    assert d == "unknown" or len(d.removesuffix("-dirty")) >= 7
