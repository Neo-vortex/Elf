import numpy as np
import pytest

from neosim import channel
from neosim.scenario import FS_HZ, Scenario, Station


def scenario(stations):
    return Scenario("t", 14_060_000.0, 2.0, 1, -100.0, tuple(stations), ())


def test_awgn_power_gives_unit_noise_in_2500_hz():
    x = channel.awgn(FS_HZ * 4, np.random.default_rng(0))
    assert np.mean(np.abs(x) ** 2) == pytest.approx(FS_HZ / 2500, rel=0.02)


def test_compose_scales_station_to_snr():
    st = Station("c", "carrier", 14_065_000.0, 20.0)
    iq, excluded = channel.compose(scenario([st]), np.random.default_rng(1))
    t = np.arange(len(iq)) / FS_HZ
    amp = np.abs(np.mean(iq * np.exp(-2j * np.pi * 5_000.0 * t)))
    assert excluded == []
    assert 20 * np.log10(amp) == pytest.approx(20.0, abs=0.3)


def test_compose_excludes_station_outside_window():
    far = Station("bcast", "carrier", 15_400_000.0, 60.0)
    iq, excluded = channel.compose(scenario([far]), np.random.default_rng(2))
    assert [e.station_id for e in excluded] == ["bcast"]
    assert "outside" in excluded[0].reason
    assert np.mean(np.abs(iq) ** 2) == pytest.approx(FS_HZ / 2500, rel=0.03)


def test_window_edge_respects_station_bandwidth():
    lo = 14_060_000.0
    inside = Station("a", "carrier", lo + 43_199.0, 0.0)
    outside = Station("b", "cw", lo + 43_199.0, 0.0, text="E", wpm=20)
    assert channel.in_window(inside, lo)
    assert not channel.in_window(outside, lo)
