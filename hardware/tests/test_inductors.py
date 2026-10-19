import math

import numpy as np
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


def test_robust_resistance_is_the_bound_with_25_percent_margin():
    r = inductors.row(inductors.FILTER_LARGE, 39e-6)        # B78108S1393K000: Q 50 at 2.52 MHz
    r_t = 2 * math.pi * r.f_test_hz * r.henries / (0.75 * r.q_min)
    assert inductors.robust_r(r, 0.3e6) == pytest.approx(r_t)               # flat below f_t
    assert inductors.robust_r(r, 2 * r.f_test_hz) == pytest.approx(2 * r_t)  # Q held above f_t
    f = np.array([0.1e6, r.f_test_hz, 10e6])
    assert np.allclose(inductors.robust_r(r, f), [r_t, r_t, r_t * 10e6 / r.f_test_hz])


def test_filter_series_and_lead_inductance():
    assert inductors.filter_series(1e-6) == "tdk_bc" and inductors.filter_series(0.82e-6) == "bourns_78f"
    assert inductors.lead_l("tdk_bc") == pytest.approx((12.7 - 9.2 + 2.0) * 1e-9)
    assert inductors.lead_l("bourns_78f") == pytest.approx((10.16 - 7.11 + 2.0) * 1e-9)


def test_robust_z_of_a_table_value_matches_its_row():
    h, f = 10e-6, np.array([1e6])
    r = inductors.row("tdk_bc", h)
    z = inductors.robust_z(h, f)
    w = 2 * math.pi * f
    z_series = inductors.robust_r(r, f) + 1j * w * (h + inductors.lead_l("tdk_bc"))
    want = z_series / (1 + 1j * w * inductors.c_par(r) * z_series)
    assert np.allclose(z, want)


def test_bc_rows_reach_1500_uh_and_tolerances_follow_population():
    vals = inductors.available("tdk_bc")
    assert vals[0] == pytest.approx(1e-6) and vals[-1] == pytest.approx(1500e-6)
    r = inductors.row("tdk_bc", 39e-6)
    assert inductors.tolerance(r, "china") == 0.10 and inductors.tolerance(r, "iran") == 0.03
    assert inductors.tolerance(inductors.row("tdk_bc", 47e-6), "china") == 0.05
    assert inductors.row("tdk_bc", 1500e-6).mpn == "B78108S1155J000"
    assert inductors.tolerance(inductors.row("tdk_bc", 1500e-6), "china") == 0.05


def test_nearest_row_is_geometric_and_robust_z_scales_r_with_henries():
    assert inductors.nearest_row("tdk_bc", 10.5e-6).henries == pytest.approx(10e-6)
    f = np.array([1e6])
    lo, hi = inductors.robust_z(10e-6, f), inductors.robust_z(10.4e-6, f)
    assert hi.real > lo.real and hi.imag > lo.imag
