import numpy as np
import pytest

from neosim import sources
from neosim.scenario import FS_HZ
from neosim.tools import ToolNotFoundError, tool_path


def _peak_hz(x, fs):
    spec = np.abs(np.fft.fft(x))
    freqs = np.fft.fftfreq(len(x), 1 / fs)
    return freqs[np.argmax(spec)]


def test_cw_envelope_length_matches_keying():
    env = sources.cw_envelope("E", wpm=20, fs=12_000, start_s=0.5)
    dit = 1.2 / 20
    keyed = np.count_nonzero(env > 0.5) / 12_000
    assert keyed == pytest.approx(dit, abs=0.006)
    assert np.all(env[: int(0.49 * 12_000)] == 0)


def test_cw_baseband_has_unit_active_power_at_offset():
    x = sources.cw_baseband("TEST", 20, 2_000.0, FS_HZ * 3)
    active = np.abs(x) > 0.5
    assert np.mean(np.abs(x[active]) ** 2) == pytest.approx(1.0, rel=0.05)
    assert _peak_hz(x, FS_HZ) == pytest.approx(2_000.0, abs=2)


def test_carrier_is_unit_power_tone():
    x = sources.carrier_baseband(-7_000.0, FS_HZ)
    assert np.mean(np.abs(x) ** 2) == pytest.approx(1.0)
    assert _peak_hz(x, FS_HZ) == pytest.approx(-7_000.0, abs=1)


@pytest.mark.integration
def test_ft8_baseband_energy_sits_at_dial_offset_plus_audio():
    try:
        tool_path("gen_ft8")
    except ToolNotFoundError as e:
        pytest.skip(str(e))
    n = FS_HZ * 15
    x = sources.ft8_baseband("CQ EP2ABC LL35", 1_500.0, 14_000.0, n)
    assert len(x) == n
    spec = np.abs(np.fft.fft(x)) ** 2
    freqs = np.fft.fftfreq(n, 1 / FS_HZ)
    band = (freqs > 15_490) & (freqs < 15_560)
    assert spec[band].sum() / spec.sum() > 0.95
