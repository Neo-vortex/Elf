import json
import math

import numpy as np
import pytest

from neosim import hardware, sources
from neosim.hardware import Hardware, HardwareModelError, Imbalance
from neosim.scenario import FS_HZ, WINDOW_HZ, parse_scenario

K_T0 = hardware.K_B * hardware.T0


def _doc(block, **payload):
    return {"schema_version": 1, "block": block, "status": "pass", "generated_by": "test", **payload}


def write_char(d, *, il_db=1.0, pre_gain_db=14.0, pre_nf_db=4.0, mixer_nf_db=7.0, conv_gain_db=36.0,
               harmonics=None, imbalance=(0.0, 0.0), adc_dr_db=108.0, dc=0.0, bypass=1.0,
               tp_gain_db=None, tp_harmonics=None):
    """`tp_gain_db` / `tp_harmonics`: (value at the 1 MHz test point, value at 60 MHz); default
    both equal to the top-level (nominal) values."""
    """Flat synthetic characterisation: two filters (1–10 MHz, 10–60 MHz) with S21 = −il_db in
    band and −80 dB outside, flat preamp and attenuator, flat mixer response."""
    d.mkdir(parents=True, exist_ok=True)
    filters = [{"index": 1, "lo_hz": 1e6, "hi_hz": 10e6}, {"index": 2, "lo_hz": 10e6, "hi_hz": 60e6}]
    (d / "bpf_bank.json").write_text(json.dumps(_doc("bpf_bank", filters=filters)))
    for flt in filters:
        lines = ["# Hz S RI R 50"]
        for f in np.geomspace(0.05e6, 400e6, 400):
            inside = flt["lo_hz"] * 0.9 <= f <= flt["hi_hz"] * 1.1
            s21 = 10 ** ((-il_db if inside else -80.0) / 20)
            lines.append(f"{f:.6g} 0 0 {s21:.9g} 0 {s21:.9g} 0 0 0")
        (d / f"bpf_{flt['index']}.s2p").write_text("\n".join(lines) + "\n")
    grid = list(np.geomspace(0.1e6, 300e6, 50))
    (d / "preamp.json").write_text(json.dumps(_doc(
        "preamp", freq_hz=grid, gain_db=[pre_gain_db] * 50, nf_db=[pre_nf_db] * 50,
        bypass_loss_db=0.0, oip3_dbm=24.0)))
    (d / "attenuator.json").write_text(json.dumps(_doc(
        "attenuator", freq_hz=[0.1e6, 54e6], pad_db=[-20.0, -20.0], bypass_db=[0.0, 0.0])))
    offs = [-60e3, -3e3, 3e3, 60e3]
    harm = {"1": {"db": 0.0, "phase_deg": 0.0, "sideband": "upper"}}
    harm.update(harmonics or {})
    ir = {"p5_gain_err_db": imbalance[0], "p5_phase_err_deg": imbalance[1]}
    (d / "mixer.json").write_text(json.dumps(_doc(
        "mixer", conv_gain_db=conv_gain_db,
        test_points=[{"f_lo_hz": f, "conv_gain_db": g, "harmonics": h} for f, g, h in zip(
            (1e6, 60e6), tp_gain_db or (conv_gain_db, conv_gain_db), tp_harmonics or (harmonics or {}, harmonics or {}))],
        baseband_response={"offset_hz": offs, "re": [1.0] * 4, "im": [0.0] * 4},
        harmonic_responses=harm, image_rejection_db={"direct": ir, "div4": ir},
        nf_db=mixer_nf_db, adc_fs_peak_v=2.825, clip_peak_v=2.0,
        adc_noise={"dynamic_range_db": adc_dr_db, "bandwidth_hz": 40e3},
        dc_offset_v={"typical": dc, "worst": dc},
        bypass_correction={str(i): {"offset_hz": offs, "re": [bypass] * 4, "im": [0.0] * 4,
                                    "mean_db": 20 * math.log10(bypass)} for i in (1, 2)})))
    return d


def scenario(lo, stations=(), preamp=False, att=0, noise=-174.0, duration=1.0):
    return parse_scenario({
        "name": "t", "lo_hz": lo, "duration_s": duration, "noise": {"type": "awgn", "dbm_per_hz": noise},
        "receiver": {"model": "hardware", "preamp": "on" if preamp else "off", "attenuator": att},
        "stations": list(stations),
        "receive": [{"id": "r", "mode": "cw", "dial_hz": lo + 10e3, "decoder": "cw", "expect": {"text": "E"}}]})


def run(sc, hw, seed=1):
    return hardware.receive(sc, np.random.default_rng(seed), hw, sources.render_station, FS_HZ, WINDOW_HZ)


def tone_dbfs(iq, f):
    t = np.arange(len(iq)) / FS_HZ
    return 20 * math.log10(abs(np.mean(iq * np.exp(-2j * np.pi * f * t))))


def test_refuses_failed_or_foreign_files(tmp_path):
    d = write_char(tmp_path / "c")
    doc = json.loads((d / "mixer.json").read_text())
    doc["status"] = "fail"
    doc["checks"] = [{"name": "iip3_dbm", "passed": False}]
    (d / "mixer.json").write_text(json.dumps(doc))
    with pytest.raises(HardwareModelError, match="iip3_dbm"):
        Hardware.load(d)
    doc["status"], doc["schema_version"] = "pass", 2
    (d / "mixer.json").write_text(json.dumps(doc))
    with pytest.raises(HardwareModelError, match="schema_version"):
        Hardware.load(d)
    (d / "mixer.json").unlink()
    with pytest.raises(HardwareModelError, match="make hw"):
        Hardware.load(d)


def test_filter_selection_follows_the_bank_rule(tmp_path):
    hw = Hardware.load(write_char(tmp_path))
    assert hw.filter_index(1e6) == 1 and hw.filter_index(9.99e6) == 1
    assert hw.filter_index(10e6) == 2 and hw.filter_index(60e6) == 2
    with pytest.raises(HardwareModelError, match="outside"):
        hw.filter_index(0.5e6)


def test_tone_level_follows_the_gain_chain(tmp_path):
    """−80 dBm through 1 dB filter loss and 36 dB conversion gain, into a 2.825 V ADC."""
    hw = Hardware.load(write_char(tmp_path, adc_dr_db=200.0))
    st = {"id": "c", "mode": "carrier", "rf_hz": 14.01e6, "level_dbm": -80.0}
    iq, _, stats = run(scenario(14e6, [st], noise=-200.0), hw)
    a = math.sqrt(2 * 50 * 1e-3 * 10 ** (-80 / 10))
    expected = 20 * math.log10(a * 10 ** ((36 - 1) / 20) / 2.825)
    assert tone_dbfs(iq, 10e3) == pytest.approx(expected, abs=0.05)
    assert stats.filter_index == 2


def test_preamp_and_attenuator_scale_the_tone(tmp_path):
    hw = Hardware.load(write_char(tmp_path, adc_dr_db=200.0))
    st = {"id": "c", "mode": "carrier", "rf_hz": 14.01e6, "level_dbm": -90.0}
    base = tone_dbfs(run(scenario(14e6, [st], noise=-200.0), hw)[0], 10e3)
    pre = tone_dbfs(run(scenario(14e6, [st], preamp=True, noise=-200.0), hw)[0], 10e3)
    att = tone_dbfs(run(scenario(14e6, [st], att=20, noise=-200.0), hw)[0], 10e3)
    assert pre - base == pytest.approx(14.0, abs=0.05)
    assert att - base == pytest.approx(-20.0, abs=0.05)


def test_noise_floor_matches_the_cascade(tmp_path):
    """Thermal noise at the antenna: output density = kT0·F·gains (+ ADC noise, here negligible)."""
    hw = Hardware.load(write_char(tmp_path, adc_dr_db=200.0))
    iq, _, stats = run(scenario(14e6, preamp=True, duration=2.0), hw)
    budget = stats.noise
    f_expected = 10 ** (1 / 10) + (10 ** 0.4 - 1) / 10 ** (-1 / 10) + (10 ** 0.7 - 1) / 10 ** ((14 - 1) / 10)
    assert budget["f_cascade"] == pytest.approx(f_expected, rel=1e-6)
    density = 2 * 50 * K_T0 * budget["f_cascade"] * 10 ** ((36 - 1 + 14) / 10) / 2.825 ** 2
    measured = np.mean(np.abs(iq) ** 2) / FS_HZ
    assert 10 * math.log10(measured / density) == pytest.approx(0.0, abs=0.1)


def test_preamp_noise_folds_in_through_the_third_harmonic(tmp_path):
    harm = {"3": {"db": 20 * math.log10(1 / 3), "phase_deg": 0.0, "sideband": "lower"}}
    hw = Hardware.load(write_char(tmp_path, harmonics=harm))
    b = hw.noise_budget(14e6, True, 0)
    # preamp output noise kT0·G·F at 42 MHz, /9, referred back through filter + preamp gain
    expected = K_T0 * 10 ** (1.4) * 10 ** 0.4 / 9 / (10 ** (-0.1) * 10 ** 1.4)
    assert b["harmonic_noise_w_per_hz"] == pytest.approx(expected, rel=1e-6)
    assert hw.noise_budget(14e6, False, 0)["harmonic_noise_w_per_hz"] == 0.0


def test_station_near_3lo_folds_to_the_lower_sideband(tmp_path):
    harm = {"3": {"db": -9.54, "phase_deg": 0.0, "sideband": "lower"}}
    hw = Hardware.load(write_char(tmp_path, harmonics=harm, adc_dr_db=200.0))
    lo = 14e6
    st = {"id": "h", "mode": "carrier", "rf_hz": 3 * lo + 7e3, "level_dbm": -60.0}
    iq, excluded, stats = run(scenario(lo, [st], noise=-200.0), hw)
    assert not excluded and stats.folded == ("h",)
    a = math.sqrt(2 * 50 * 1e-3 * 10 ** (-60 / 10))
    # filter 2 at 42 MHz is in band (−1 dB) in this synthetic bank
    expected = 20 * math.log10(a * 10 ** ((36 - 1 - 9.54) / 20) / 2.825)
    assert tone_dbfs(iq, -7e3) == pytest.approx(expected, abs=0.05)
    assert tone_dbfs(iq, 7e3) < expected - 60


def test_out_of_window_station_is_excluded_with_a_reason(tmp_path):
    hw = Hardware.load(write_char(tmp_path))
    st = {"id": "far", "mode": "carrier", "rf_hz": 15.4e6, "level_dbm": -30.0}
    _, excluded, _ = run(scenario(14e6, [st]), hw)
    assert excluded[0][0] == "far" and "plan 2.6" in excluded[0][1]


@pytest.mark.parametrize("imb", [Imbalance(0.16, 0.43), Imbalance(-0.3, -1.0), Imbalance(0.0, 2.0)])
def test_imbalance_image_matches_its_formula_and_neohw_convention(imb):
    n = 9600
    t = np.arange(n) / FS_HZ
    z = np.exp(2j * np.pi * 12e3 * t)
    out = hardware.apply_imbalance(z, imb)
    wanted, image = (abs(np.mean(out * np.exp(-2j * np.pi * s * 12e3 * t))) for s in (1, -1))
    assert 20 * math.log10(wanted / image) == pytest.approx(hardware.image_rejection_db(imb), abs=0.01)
    # neohw's iq_imbalance convention: gain = |I|/|Q|, phase = angle(Q/I) − 90° for an upper-sideband tone
    i, q = out.real, -out.imag
    ph = np.exp(-2j * np.pi * 12e3 * t)
    pi, pq = np.mean(i * ph), np.mean(q * ph)
    assert 20 * math.log10(abs(pi) / abs(pq)) == pytest.approx(imb.gain_db, abs=1e-6)
    assert math.degrees(np.angle(pq / pi)) - 90 == pytest.approx(imb.phase_deg, abs=1e-6)


def test_clipping_at_the_op_amp_swing(tmp_path):
    hw = Hardware.load(write_char(tmp_path))
    st = {"id": "big", "mode": "carrier", "rf_hz": 14.01e6, "level_dbm": -5.0}
    iq, _, stats = run(scenario(14e6, [st]), hw)
    assert stats.clipped_samples > 0
    # the ADC's own noise comes after the op-amp swing limit
    assert np.max(np.abs(iq.real)) == pytest.approx(2.0 / 2.825, abs=1e-4)


def test_bypass_correction_scales_the_tone_and_the_budget(tmp_path):
    st = {"id": "c", "mode": "carrier", "rf_hz": 14.01e6, "level_dbm": -80.0}
    off, on = {}, {}
    for k in (1.0, 0.5):
        hw = Hardware.load(write_char(tmp_path / str(k), adc_dr_db=200.0, bypass=k))
        off[k] = tone_dbfs(run(scenario(14e6, [st], noise=-200.0), hw)[0], 10e3)
        on[k] = tone_dbfs(run(scenario(14e6, [st], preamp=True, noise=-200.0), hw)[0], 10e3)
    assert off[0.5] - off[1.0] == pytest.approx(-6.02, abs=0.05)
    assert on[0.5] == pytest.approx(on[1.0], abs=0.01)      # the preamp path has no correction
    b = Hardware.load(tmp_path / "0.5").noise_budget(14e6, False, 0)
    mixer_term = [r for r in b["stages"] if r["stage"] == "mixer"][0]["added_factor"]
    # mixer excess (10^0.7 − 1) referred through filter (−1 dB) and the 0.25 interface power gain
    assert mixer_term == pytest.approx((10 ** 0.7 - 1) / (10 ** -0.1 * 0.25), rel=1e-6)


def test_harmonic_station_beyond_the_filter_data_is_excluded(tmp_path):
    harm = {"5": {"db": -14.0, "phase_deg": 0.0, "sideband": "upper"}}
    hw = Hardware.load(write_char(tmp_path, harmonics=harm))
    st = {"id": "uhf", "mode": "carrier", "rf_hz": 5 * 50e6 + 3e3, "level_dbm": -40.0}   # 250 MHz > 208 MHz s2p data
    hw.s21 = {k: (f[f <= 208e6], s[f <= 208e6]) for k, (f, s) in hw.s21.items()}
    _, excluded, stats = run(scenario(50e6, [st]), hw)
    assert stats.folded == () and excluded[0][0] == "uhf" and "beyond the filter data" in excluded[0][1]


def test_lo_at_the_top_edge_uses_the_last_filter_and_beyond_is_an_error(tmp_path):
    hw = Hardware.load(write_char(tmp_path))
    assert run(scenario(60e6), hw)[2].filter_index == 2
    with pytest.raises(HardwareModelError, match="outside the filter bank"):
        run(scenario(61e6), hw)


def test_files_from_before_plan_2_5_are_refused_with_the_missing_fields(tmp_path):
    d = write_char(tmp_path)
    doc = json.loads((d / "mixer.json").read_text())
    del doc["nf_db"], doc["bypass_correction"]
    (d / "mixer.json").write_text(json.dumps(doc))
    with pytest.raises(HardwareModelError, match=r"lacks .*bypass_correction.*nf_db.*make hw"):
        Hardware.load(d)


def test_harmonic_responses_follow_the_lo(tmp_path):
    """mixer.json's top level is the 14.2 MHz point; the per-LO test points differ (band 1: 3rd
    harmonic −5 dB, not −9.5): interpolate over log f like the conversion gain."""
    lo_h = {"3": {"db": -5.0, "phase_deg": 10.0, "sideband": "lower"}}
    hi_h = {"3": {"db": -15.0, "phase_deg": -30.0, "sideband": "lower"}}
    nominal = {"3": {"db": -9.54, "phase_deg": 0.0, "sideband": "lower"}}
    hw = Hardware.load(write_char(tmp_path, harmonics=nominal, tp_harmonics=(lo_h, hi_h)))
    mid = math.sqrt(1e6 * 60e6)
    c, side = hw.harmonics(mid)[3]
    assert 20 * math.log10(abs(c)) == pytest.approx(-10.0) and math.degrees(np.angle(c)) == pytest.approx(-10.0)
    assert side == "lower"
    assert 20 * math.log10(abs(hw.harmonics(1e6)[3][0])) == pytest.approx(-5.0)


def test_mixer_noise_is_referred_at_the_lo_conversion_gain(tmp_path):
    """nf_db is referred at the nominal (top-level) conversion gain; the mixer's added noise is
    fixed at the ADC, so 3 dB less conversion gain doubles its input-referred excess."""
    hw = Hardware.load(write_char(tmp_path, conv_gain_db=36.0, tp_gain_db=(39.0, 33.0)))
    lo = math.sqrt(1e6 * 60e6)          # conversion gain 36 dB: the reference
    lo_low = 60e6 / 1.0001              # ≈ 33 dB
    m = lambda f: [r for r in hw.noise_budget(f, False, 0)["stages"] if r["stage"] == "mixer"][0]
    assert 10 ** (m(lo)["nf_db"] / 10) - 1 == pytest.approx(10 ** 0.7 - 1, rel=1e-6)
    assert (10 ** (m(lo_low)["nf_db"] / 10) - 1) / (10 ** 0.7 - 1) == pytest.approx(10 ** 0.3, rel=1e-3)


def test_budget_total_includes_the_adc_noise_and_matches_the_twin_floor(tmp_path):
    """With a noisy ADC (60 dB dynamic range) the twin's floor must match nf_total_db, not the
    analog cascade alone."""
    hw = Hardware.load(write_char(tmp_path, adc_dr_db=75.0))
    iq, _, stats = run(scenario(14e6, duration=2.0), hw)
    b = stats.noise
    assert b["nf_total_db"] > b["nf_db"] + 1.0          # the ADC matters at this dynamic range
    density = 2 * 50 * K_T0 * 10 ** (b["nf_total_db"] / 10) * 10 ** ((36 - 1) / 10) / 2.825 ** 2
    measured = np.mean(np.abs(iq) ** 2) / FS_HZ
    assert 10 * math.log10(measured / density) == pytest.approx(0.0, abs=0.1)
