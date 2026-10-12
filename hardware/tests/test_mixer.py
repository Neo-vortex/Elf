import math

import pytest

from neohw.blocks import mixer
from neohw.checks import mixer as check
from neohw.ngspice import run_decks
from neohw.tools import ToolNotFoundError, tool_path


@pytest.fixture
def tools():
    try:
        tool_path("ngspice"), tool_path("dot")
    except ToolNotFoundError as e:
        pytest.skip(str(e))


def test_test_points_cover_every_band_and_the_div4_crossover():
    pts = check.test_points()
    assert len(pts) == 12
    # ÷4 below 4.8 MHz (the Si5351 phase register's 90° limit): bands 1–4 and the 1.9 MHz point
    assert [p.path for p in pts[:4]] == ["div4"] * 4 and all(p.path == "direct" for p in pts[4:11])
    assert pts[-1].f_lo == 1.9e6 and pts[-1].path == "div4"


def test_full_scale_is_set_by_the_opamp_swing():
    # gain 1 V/V: the op-amps' ±1 V per pin (2 V differential) comes before the ADC's 2.825 V
    assert check.full_scale_dbm(1.0) == pytest.approx(10 * math.log10(2.0 ** 2 / 100 / 1e-3))


@pytest.mark.integration
def test_multitone_response_at_20m(tools):
    blk = mixer.build()
    f_lo = 14.2e6
    deck, raws, names = check._deck(blk, f_lo, check.multitone_tones(f_lo))
    (res,) = run_decks([deck], raws)
    r = check.analyse_multitone(f_lo, *check._spectrum(res, raws, names))
    assert r["flatness_db"] <= check.FLATNESS_DB
    assert abs(r["harmonics"][3]["db"] - 20 * math.log10(1 / 3)) <= check.HARMONIC_TOL_DB
    assert r["harmonics"][3]["sideband"] == "lower" and r["harmonics"][5]["sideband"] == "upper"
    assert r["harmonics"][2]["db"] <= check.TWO_F_MAX_DBC
    assert r["image_min_db"] > 50
    assert 35 < 20 * math.log10(r["ref_gain"]) < 38
    assert all(abs(v) < 0.05 for v in r["dc_offset_v"])   # outputs settle at the 2.5 V common mode


def test_system_rejection_adds_filter_attenuation(tmp_path):
    import numpy as np
    from neohw.touchstone import write_s2p
    f = np.array([1e6, 10e6, 20e6, 30e6])
    s21 = np.array([1.0, 1.0, 0.01, 0.001])          # 0 dB in band, -40 dB at 20 MHz, -60 dB at 30 MHz
    zero = np.zeros(4)
    write_s2p(tmp_path / "bpf_7.s2p", f, {"11": zero, "21": s21, "12": s21, "22": zero})
    # LO 10 MHz: 2f at 20 MHz (-40 dB filter) with a -45 dBc mixer response -> 85 dB
    assert check.system_rejection_db(tmp_path, 7, 10e6, 2, -45.0) == pytest.approx(85.0)
    with pytest.raises(FileNotFoundError, match="bpf_bank"):
        check.system_rejection_db(tmp_path, 8, 10e6, 2, -45.0)


def test_monte_carlo_varies_sampling_caps_and_baseband_parts():
    import numpy as np
    blk = mixer.build()
    var, duty, skew = check.mc_variation(blk, np.random.default_rng(1), "direct")
    values = {p.ref: p.value for p in blk.circuit.parts}
    varied = {values[r] for r in var}
    assert {"5.6nF", "2.2kΩ", "240Ω", "100Ω", "2.7nF"} <= varied
    assert sum(1 for r in var if values[r] == "5.6nF") == 4
    assert abs(skew) <= sum(check.LO_SKEW_STAGES_S["direct"]) and all(abs(d) <= check.LO_DUTY_S["direct"] for d in duty)


def test_iq_imbalance_fit_recovers_known_errors():
    import numpy as np
    t = np.arange(4000) * 0.25e-6
    w = 2 * np.pi * 12e3
    i = 1.02 * np.cos(w * t)
    q = -np.sin(w * t + np.radians(1.5))
    g, ph = check.iq_imbalance(i, q, 12e3, 0.25e-6)
    assert g == pytest.approx(20 * math.log10(1.02), abs=0.01) and ph == pytest.approx(1.5, abs=0.05)


def test_harmonic_phase_is_relative_to_the_fundamental():
    import numpy as np
    n = 4000
    freqs = np.fft.fftfreq(n, check.OUT_STEP_S)
    spec = np.zeros(n, dtype=complex)
    a = check._amp_in(check.TONE_DBM)
    h0 = 0.8 * np.exp(1j * 0.3)
    for o in check.OFFSETS_HZ:
        spec[np.argmin(abs(freqs - o))] = a * h0 * np.exp(2j * np.pi * o * check.SETTLE_S)
    c3 = h0 / 3 * np.exp(1j * np.radians(40))                  # 3f: 1/3 of the fundamental, +40°, lower sideband
    spec[np.argmin(abs(freqs + 18e3))] = a * c3 * np.exp(2j * np.pi * -18e3 * check.SETTLE_S)
    r = check.analyse_multitone(14.2e6, freqs, spec, np.zeros(n), np.zeros(n))
    assert r["harmonics"][3]["sideband"] == "lower"
    assert r["harmonics"][3]["db"] == pytest.approx(20 * math.log10(1 / 3), abs=0.01)
    assert r["harmonics"][3]["phase_deg"] == pytest.approx(40.0, abs=0.1)


def test_dc_offset_is_bounded_by_the_opamp_datasheet():
    typ, worst = check.dc_offset_bound_v()
    gain = 1 + 2 * 2200 / 240
    assert worst == pytest.approx(2 * 0.7e-3 * gain, rel=1e-6) and typ < worst


def test_system_rejection_uses_band_edges(tmp_path):
    import numpy as np
    from neohw.touchstone import write_s2p
    f = np.array([1e6, 10e6, 20e6, 30e6, 40e6])
    s21 = np.array([1.0, 1.0, 0.5, 0.001, 0.001])          # filter falls off between 20 and 30 MHz
    zero = np.zeros(5)
    write_s2p(tmp_path / "bpf_7.s2p", f, {"11": zero, "21": s21, "12": s21, "22": zero})
    lo, hi = 10e6, 15e6                                     # 2f spans 20–30 MHz: worst at the lower edge
    worst = check.system_rejection_band_db(tmp_path, 7, lo, hi, 2, lambda f_lo: -45.0)
    assert worst == pytest.approx(-20 * math.log10(0.5) + 45.0, abs=0.01)
