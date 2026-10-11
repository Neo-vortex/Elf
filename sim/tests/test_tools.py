import stat

import pytest

from neosim import tools


def _make_exe(path):
    path.write_text("#!/bin/sh\nexit 0\n")
    path.chmod(path.stat().st_mode | stat.S_IXUSR)
    return path


def test_missing_tool_names_make_target(monkeypatch, tmp_path):
    monkeypatch.setenv("NEOSIM_DECODE_FT8", str(tmp_path / "nope"))
    with pytest.raises(tools.ToolNotFoundError) as err:
        tools.tool_path("decode_ft8")
    assert "make tools" in str(err.value)
    assert "nope" in str(err.value)


def test_existing_executable_is_returned(monkeypatch, tmp_path):
    exe = _make_exe(tmp_path / "fake")
    monkeypatch.setenv("NEOSIM_DECODE_FT8", str(exe))
    assert tools.tool_path("decode_ft8") == exe


def test_non_executable_file_is_rejected(monkeypatch, tmp_path):
    f = tmp_path / "plain"
    f.write_text("x")
    monkeypatch.setenv("NEOSIM_GEN_FT8", str(f))
    with pytest.raises(tools.ToolNotFoundError):
        tools.tool_path("gen_ft8")


def test_unknown_tool_name_is_key_error():
    with pytest.raises(KeyError):
        tools.tool_path("jt9")


def test_default_dsp_path_points_into_pi_build():
    assert tools.DEFAULT_PATHS["neoradio-dsp"].parts[-5:] == (
        "NeoRadio.Dsp.Cli", "bin", "Release", "net10.0", "neoradio-dsp")
