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
    assert len(pts) == 11
    assert [p.path for p in pts[:3]] == ["div4"] * 3 and all(p.path == "direct" for p in pts[3:10])
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
