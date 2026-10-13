"""Recorded-antenna input (spec 2026-10-13-recorded-antenna-design.md): recordings of known content
(synthetic SigMF and WAV files) placed on the antenna."""
import json
import math

import numpy as np
import pytest
from scipy.io import wavfile

from neosim import antenna, hardware
from neosim.scenario import FS_HZ, WINDOW_HZ

LO = 7_100_000.0


def _sigmf(tmp_path, name, iq, rate, center):
    np.asarray(iq, dtype="<c8").tofile(tmp_path / f"{name}.sigmf-data")
    (tmp_path / f"{name}.sigmf-meta").write_text(json.dumps({
        "global": {"core:datatype": "cf32_le", "core:sample_rate": rate, "core:version": "1.0.0"},
        "captures": [{"core:sample_start": 0, "core:frequency": center}], "annotations": []}))
    return tmp_path / f"{name}.sigmf-meta"


def _tones(rate, seconds, tones):
    t = np.arange(int(rate * seconds)) / rate
    return sum(a * np.exp(2j * np.pi * f * t) for f, a in tones)


def _antenna_file(tmp_path, recordings, noise=-174.0):
    import yaml
    p = tmp_path / "antenna.yaml"
    p.write_text(yaml.safe_dump({"name": "test", "duration_s": 1.0, "seed": 3, "noise": {"dbm_per_hz": noise},
                                 "recordings": recordings}))
    return p


def _power_at(iq, f):
    spec = np.abs(np.fft.fft(iq)) ** 2 / len(iq) ** 2
    return spec[int(round(f / FS_HZ * len(iq))) % len(iq)]


@pytest.fixture(scope="module")
def hw():
    return hardware.Hardware.load()


def test_full_scale_reference_sets_the_antenna_level(tmp_path):
    rec = antenna.load_recording({"file": str(_sigmf(tmp_path, "a", _tones(192000, 0.5, [(20e3, 0.1)]), 192000, LO)),
                                  "full_scale_dbm": -30.0}, tmp_path)
    assert rec.level_dbm == pytest.approx(-30.0 + 20 * math.log10(0.1), abs=0.05)     # −50 dBm


def test_carrier_reference_and_full_scale_agree(tmp_path):
    meta = _sigmf(tmp_path, "b", _tones(192000, 0.5, [(20e3, 0.1), (-30e3, 0.01)]), 192000, LO)
    by_fs = antenna.load_recording({"file": str(meta), "full_scale_dbm": -30.0}, tmp_path)
    by_ref = antenna.load_recording({"file": str(meta), "reference": {"hz": LO + 20e3, "dbm": -50.0}}, tmp_path)
    assert by_ref.level_dbm == pytest.approx(by_fs.level_dbm, abs=0.1)


def test_a_recording_without_a_level_reference_is_refused(tmp_path):
    meta = _sigmf(tmp_path, "c", _tones(96000, 0.1, [(1e3, 0.1)]), 96000, LO)
    with pytest.raises(antenna.AntennaError, match="level reference"):
        antenna.load_recording({"file": str(meta)}, tmp_path)


def test_wav_iq_needs_its_centre_frequency_from_the_sidecar(tmp_path):
    x = _tones(12000, 0.5, [(0.0, 0.2)])
    wavfile.write(tmp_path / "k.wav", 12000, (np.stack([x.real, x.imag], 1) * 32767).astype(np.int16))
    with pytest.raises(antenna.AntennaError, match="center_hz"):
        antenna.load_recording({"file": "k.wav", "full_scale_dbm": -20.0}, tmp_path)
    rec = antenna.load_recording({"file": "k.wav", "center_hz": LO + 15e3, "full_scale_dbm": -20.0}, tmp_path)
    assert rec.rate == 12000 and rec.center_hz == LO + 15e3


def test_tones_appear_at_their_offset_and_level_through_the_board(tmp_path, hw):
    # two tones 20 kHz above and 30 kHz below the recording's centre, which sits 5 kHz above the LO
    meta = _sigmf(tmp_path, "d", _tones(192000, 1.0, [(20e3, 0.1), (-30e3, 0.03)]), 192000, LO + 5e3)
    ant = antenna.Antenna.load(_antenna_file(tmp_path, [{"file": str(meta), "full_scale_dbm": -30.0}]))
    iq = ant.render(LO, hw, preamp=False, attenuator_db=0)
    for f, a in ((25e3, 0.1), (-25e3, 0.03)):
        want_dbm = -30.0 + 20 * math.log10(a)
        gain_db = 20 * math.log10(abs(complex(hw.front_gain(LO + f, LO, False, 0)) * hw.conv_gain(LO)
                                      * hw.bypass_correction(LO, np.array([f]))[0] * hw.baseband_response(np.array([f]))[0]))
        got_dbm = 10 * math.log10(_power_at(iq, f) * hw.adc_fs_peak_v ** 2 / (2 * hardware.R0) / 1e-3)
        assert got_dbm == pytest.approx(want_dbm + gain_db, abs=1.0), f


def test_a_recording_tuned_away_from_is_absent(tmp_path, hw):
    meta = _sigmf(tmp_path, "e", _tones(192000, 1.0, [(0.0, 0.1)]), 192000, LO)
    ant = antenna.Antenna.load(_antenna_file(tmp_path, [{"file": str(meta), "full_scale_dbm": -30.0}]))
    far = ant.render(LO + 200e3, hw, preamp=False, attenuator_db=0)
    near = ant.render(LO - 10e3, hw, preamp=False, attenuator_db=0)
    assert _power_at(near, 10e3) > 1e6 * max(_power_at(far, -200e3 % FS_HZ), 1e-30)
    assert np.max(np.abs(np.fft.fft(far))[1:]) ** 2 < 1e-6 * np.max(np.abs(np.fft.fft(near))[1:]) ** 2


def test_a_narrow_recording_leaves_the_rest_of_the_window_at_the_noise_floor(tmp_path, hw):
    x = _tones(12000, 1.0, [(0.0, 0.3)])
    wavfile.write(tmp_path / "n.wav", 12000, (np.stack([x.real, x.imag], 1) * 32767).astype(np.int16))
    ant = antenna.Antenna.load(_antenna_file(tmp_path, [{"file": "n.wav", "center_hz": LO + 10e3, "full_scale_dbm": -40.0}],
                                             noise=-150.0))
    iq = ant.render(LO, hw, preamp=False, attenuator_db=0)
    spec = np.abs(np.fft.fftshift(np.fft.fft(iq))) ** 2
    f = np.fft.fftshift(np.fft.fftfreq(len(iq), 1 / FS_HZ))
    inside = spec[np.abs(f - 10e3) < 50].max()
    floor = np.median(spec[(np.abs(f + 25e3) < 5e3)])                     # far from the recording, in the window
    assert inside > 1e6 * floor
    assert floor > 0                                                      # the configured band noise is there


def test_a_short_recording_loops_without_a_click(tmp_path):
    rate = 96000
    x = _tones(rate, 1.0, [(1234.5, 0.5)])                                  # not periodic in 1 s: a hard seam would step
    rec = antenna.load_recording({"file": str(_sigmf(tmp_path, "f", x, rate, LO)), "full_scale_dbm": -20.0}, tmp_path)
    y = rec.baseband(0.0, 3 * rate)
    step = np.max(np.abs(np.diff(y)))
    assert step < 1.5 * np.max(np.abs(np.diff(y[: rate // 2])))            # no jump beyond the signal's own


def test_antenna_file_loads_recordings_and_imaginary_stations(tmp_path):
    import yaml
    meta = _sigmf(tmp_path, "g", _tones(96000, 0.2, [(0.0, 0.1)]), 96000, LO)
    p = tmp_path / "mix.yaml"
    p.write_text(yaml.safe_dump({"name": "mix", "duration_s": 0.5, "seed": 1, "noise": {"dbm_per_hz": -160},
                                 "recordings": [{"file": str(meta), "full_scale_dbm": -30.0}],
                                 "stations": [{"id": "cw1", "mode": "cw", "rf_hz": LO + 20e3, "level_dbm": -90,
                                               "text": "TEST", "wpm": 20}]}))
    ant = antenna.Antenna.load(p)
    assert [s.mode for s in ant.stations] == ["recording", "cw"]
