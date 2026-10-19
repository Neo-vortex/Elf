import math

import pytest

from neohw import inductors


def test_every_filter_value_has_a_china_and_an_iran_datasheet_row():
    from neohw.blocks import bpf
    for index in range(1, 12):
        for pos in bpf.design(index).positions:
            if pos.kind.startswith("L"):
                for h in pos.parts:
                    assert inductors.row(inductors.china_series(h), h).q_min > 0
                    assert inductors.row(inductors.IRAN, h).q_min > 0


def test_china_uses_1812_from_1_uh_and_1210_below():
    assert inductors.china_series(1e-6) == inductors.CHINA_LARGE
    assert inductors.china_series(0.82e-6) == inductors.CHINA_SMALL


def test_rows_are_the_datasheet_numbers():
    r = inductors.row(inductors.CHINA_LARGE, 39e-6)          # TDK NL453232T-390J-PF
    assert (r.mpn, r.q_min, r.f_test_hz, r.srf_min_hz, r.dcr_max) == ("NL453232T-390J-PF", 50, 2.52e6, 10e6, 4.5)
    r = inductors.row(inductors.IRAN, 39e-6)                 # Bourns 78F390J-RC
    assert (r.mpn, r.q_min, r.f_test_hz, r.srf_min_hz, r.dcr_max) == ("78F390J-RC", 50, 2.5e6, 9.5e6, 1.70)


def test_q_is_the_guaranteed_minimum_at_and_above_the_test_frequency():
    r = inductors.row(inductors.CHINA_LARGE, 10e-6)
    for f in (r.f_test_hz, 2 * r.f_test_hz, 10 * r.f_test_hz):
        assert inductors.q_at(r, f) == pytest.approx(r.q_min)


def test_loss_never_falls_below_the_dc_resistance():
    r = inductors.row(inductors.CHINA_LARGE, 39e-6)
    r_t = 2 * math.pi * r.f_test_hz * r.henries / r.q_min
    assert inductors.r_series(r, 1.0) == pytest.approx(r.dcr_max, rel=1e-6)
    assert r.dcr_max < inductors.r_series(r, 0.3e6) < r_t
    assert inductors.q_at(r, 0.3e6) < r.q_min                 # lower Q below the test frequency


def test_parallel_capacitance_gives_the_minimum_self_resonance():
    r = inductors.row(inductors.IRAN, 1e-6)
    assert 1 / (2 * math.pi * math.sqrt(r.henries * inductors.c_par(r))) == pytest.approx(r.srf_min_hz)


def test_unknown_value_is_rejected():
    with pytest.raises(ValueError, match="not in"):
        inductors.row(inductors.IRAN, 1.23e-6)
