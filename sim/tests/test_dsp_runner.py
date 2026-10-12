import stat

import pytest

from neosim.dsp_runner import run_dsp


def test_run_dsp_failure_includes_stderr(monkeypatch, tmp_path):
    fake = tmp_path / "neoradio-dsp"
    fake.write_text("#!/bin/sh\necho 'input error: boom' >&2\nexit 3\n")
    fake.chmod(fake.stat().st_mode | stat.S_IXUSR)
    monkeypatch.setenv("NEOSIM_DSP_CLI", str(fake))
    with pytest.raises(RuntimeError) as err:
        run_dsp(tmp_path / "x.sigmf-meta", tmp_path / "x.wav", "usb", 1000.0)
    assert "exit 3" in str(err.value) and "boom" in str(err.value)


def test_run_dsp_passes_arguments(monkeypatch, tmp_path):
    log = tmp_path / "args.txt"
    fake = tmp_path / "neoradio-dsp"
    fake.write_text(f'#!/bin/sh\necho "$@" > {log}\n')
    fake.chmod(fake.stat().st_mode | stat.S_IXUSR)
    monkeypatch.setenv("NEOSIM_DSP_CLI", str(fake))
    run_dsp(tmp_path / "x.sigmf-meta", tmp_path / "x.wav", "cw", -15000.0)
    args = log.read_text().split()
    assert args[args.index("--mode") + 1] == "cw"
    assert args[args.index("--tune-offset") + 1] == "-15000.000"


def test_run_dsp_passes_the_iq_correction_only_when_given(monkeypatch, tmp_path):
    log = tmp_path / "args.txt"
    fake = tmp_path / "neoradio-dsp"
    fake.write_text(f'#!/bin/sh\necho "$@" > {log}\n')
    fake.chmod(fake.stat().st_mode | stat.S_IXUSR)
    monkeypatch.setenv("NEOSIM_DSP_CLI", str(fake))
    run_dsp(tmp_path / "x.sigmf-meta", tmp_path / "x.wav", "usb", 0.0)
    assert "--iq-gain-db" not in log.read_text()
    run_dsp(tmp_path / "x.sigmf-meta", tmp_path / "x.wav", "usb", 0.0, iq=(0.1608, -0.4273))
    args = log.read_text().split()
    assert args[args.index("--iq-gain-db") + 1] == "0.1608" and args[args.index("--iq-phase-deg") + 1] == "-0.4273"
