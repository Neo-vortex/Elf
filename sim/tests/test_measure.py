import math

import pytest

from neosim import measure, sweep
from neosim.hardware import Hardware
from neosim.scenario import SNR_BANDWIDTH_HZ
from tests.charfixture import write_char


def test_mds_measured_in_the_twin_matches_the_noise_budget(tmp_path):
    hw = Hardware.load(write_char(tmp_path, adc_dr_db=108.0))
    r = measure.measure_mds(hw, 14.1e6, preamp=True)
    assert r.measured_dbm == pytest.approx(r.predicted_dbm, abs=0.5)
    # flat synthetic chain: thermal −174 dBm/Hz × F_total in 500 Hz
    k_t0 = 1.380649e-23 * 290
    expected = 10 * math.log10((10 ** (-17.4) * 1e-3 + k_t0 * (10 ** (r.nf_total_db / 10) - 1)) * 500 / 1e-3)
    assert r.predicted_dbm == pytest.approx(expected, abs=1e-9)


def test_mds_without_the_preamp_is_worse(tmp_path):
    hw = Hardware.load(write_char(tmp_path))
    assert measure.measure_mds(hw, 14.1e6, preamp=False).predicted_dbm > measure.measure_mds(hw, 14.1e6, preamp=True).predicted_dbm


def test_sweep_scenario_for_the_hardware_uses_absolute_levels():
    sc = sweep.sweep_scenario(-15.0, seed=3, hardware=True)
    assert sc.receiver.model == "hardware" and not sc.receiver.preamp
    assert sc.noise_dbm_per_hz == sweep.BAND_NOISE_DBM_HZ
    assert sc.stations[0].level_dbm == pytest.approx(-15.0 + sweep.BAND_NOISE_DBM_HZ + 10 * math.log10(SNR_BANDWIDTH_HZ))
    assert sweep.sweep_scenario(-15.0, seed=3, hardware=False).receiver.model == "ideal"


def test_implementation_loss_of_the_hardware_curve():
    points = [sweep.SweepPoint(-10, 1.0, 1.0, hardware_rate=1.0), sweep.SweepPoint(-14, 1.0, 1.0, hardware_rate=0.0),
              sweep.SweepPoint(-18, 0.0, 0.0, hardware_rate=0.0)]
    assert sweep.implementation_loss_db(points, "hardware") == pytest.approx(4.0)
    assert sweep.implementation_loss_db(points) == pytest.approx(0.0)      # ideal twin = reference here


def test_mds_ignores_the_dc_offset_and_the_tone_image(tmp_path):
    """The real mixer has millivolts of DC offset at 0 Hz and a −40 dB image of the tone at
    −10 kHz; both lie within 20 kHz of the tone and must not count as noise."""
    hw = Hardware.load(write_char(tmp_path, dc=2.7e-3, imbalance=(0.16, 0.43)))
    r = measure.measure_mds(hw, 14.1e6, preamp=True)
    assert r.measured_dbm == pytest.approx(r.predicted_dbm, abs=0.5)
