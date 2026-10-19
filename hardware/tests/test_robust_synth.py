import math

import numpy as np
import pytest

from neohw.synth import ladder, robust


def test_elliptic_seed_has_tanks_with_zeros_in_the_3f_band():
    br = robust.seed(robust.Topology(3, "ell5"), 16e6, 22e6)
    tanks = [b for b in br if b.form == "LC_par"]
    assert len(tanks) == 2
    for b in tanks:
        fz = 1 / (2 * math.pi * math.sqrt(b.l * b.c))
        assert 3 * 16e6 * 0.9 <= fz <= 3 * 22e6 * 1.5


def test_targets_carry_selectivity_floors():
    t = robust.targets(1)
    assert t.rej_min is None and t.att_2hi_min >= 20 and t.il_max == 8.0
    assert robust.targets(8).rej_min == 53.0 and robust.targets(8).il_max == 2.0


def test_selectivity_floors_come_from_the_today_selectivity_seam(monkeypatch):
    curve = {"f_lo_hz": (1e6, 2e6), "floor_lo_db": (40.0, 30.0), "f_hi_hz": (20e6, 40e6), "floor_hi_db": (35.0, 60.0)}
    monkeypatch.setattr(robust, "today_selectivity", lambda index: curve)
    t = robust.targets(5)
    assert t.stop_f == (1e6, 2e6, 20e6, 40e6) and t.stop_floor == (40.0, 30.0, 35.0, 60.0)
    assert t.att_half_lo_min == 30.0 and t.att_2hi_min == 35.0


def test_today_selectivity_is_todays_design_in_the_robust_model():
    from neohw.blocks import bpf
    d = bpf.design(6)
    s = robust.today_selectivity(6)
    for side in ("lo", "hi"):
        f = np.array(s[f"f_{side}_hz"])
        att = -ladder.db(ladder.s21_s11(ladder.from_positions(d.positions), f,
                                        ladder.robust_losses(d.centre_hz), ladder.bank_bus())[0])
        assert np.allclose(s[f"floor_{side}_db"], np.clip(att - 3.0, 20.0, 60.0))


def test_stop_grids_hold_the_decided_points():
    f_lo, f_hi = robust.stop_grids(7.5e6, 12e6)
    assert len(f_lo) == 30 and f_lo[0] == pytest.approx(7.5e6 / 20) and f_lo[-1] == pytest.approx(3.75e6)
    assert len(f_hi) == 60 and f_hi[0] == pytest.approx(24e6) and f_hi[-1] == pytest.approx(200e6)
    assert robust.stop_grids(0.1e6, 0.5e6)[0][0] == pytest.approx(10e3)
    fine_lo, fine_hi = robust.stop_grids(7.5e6, 12e6, 4)
    assert np.allclose(fine_lo[::4], f_lo) and np.allclose(fine_hi[::4], f_hi)


@pytest.mark.parametrize("index", range(1, 12))
def test_floor_is_capped_at_60_and_at_least_20(index):
    s = robust.today_selectivity(index)
    fl = np.array(s["floor_lo_db"] + s["floor_hi_db"])
    assert fl.max() <= 60.0 and fl.min() >= 20.0
    assert robust.floor_db([10.0, 50.0, 90.0]).tolist() == [20.0, 47.0, 60.0]


def test_pruning_removes_small_shunt_c_shorts_big_series_c_and_untanks():
    br = [ladder.Branch("series", "C", c=100e-9), ladder.Branch("shunt", "L", l=1e-6),
          ladder.Branch("shunt", "C", c=2e-12), ladder.Branch("series", "LC_par", l=1e-6, c=1e-12),
          ladder.Branch("shunt", "C", c=50e-12), ladder.Branch("series", "LC_par", l=1e-6, c=10e-12)]
    out, done = robust._prune(br)
    assert [(b.place, b.form) for b in out] == [("shunt", "L"), ("series", "L"), ("shunt", "C"), ("series", "LC_par")]
    assert out[1].l == 1e-6 and len(done) == 3


@pytest.mark.integration
def test_optimised_filter_8_meets_its_constraints_with_margin():
    r = robust.best(8)
    assert r.feasible, r.violations
    m = r.metrics
    assert m["il_db"] <= 2.0 and m["rl_db"] >= 15.5 and m["rej3f_db"] >= 53.0
    t = robust.targets(8)
    assert m["att_2hi_db"] >= t.att_2hi_min


@pytest.mark.integration
def test_filter_1_cannot_buy_il_by_losing_selectivity():
    r = robust.best(1)
    t = robust.targets(1)
    assert r.metrics["att_2hi_db"] >= t.att_2hi_min - 1e-6
    if t.att_half_lo_min is not None:
        assert r.metrics["att_half_lo_db"] >= t.att_half_lo_min - 1e-6


@pytest.mark.integration
@pytest.mark.parametrize("index", (2, 6))
def test_stopband_floor_holds_on_a_dense_grid(index):
    r = robust.best(index)
    t = robust.targets(index)
    assert r.feasible, r.violations
    losses, bus = ladder.robust_losses(math.sqrt(t.lo * t.hi)), ladder.bank_bus()
    for g in robust.stop_grids(t.lo, t.hi):
        f = np.geomspace(g[0], g[-1], 1001)
        att = -ladder.db(ladder.s21_s11(list(r.branches), f, losses, bus)[0])
        assert np.min(att - robust.floor_db(robust.today_attenuation(index, f))) >= -0.1


@pytest.mark.integration
@pytest.mark.parametrize("index", range(1, 12))
def test_best_designs_are_feasible_with_notches_above_band_and_no_tiny_caps(index):
    r = robust.best(index)
    hi = robust.targets(index).hi
    assert r.feasible, r.violations
    assert r.metrics["stop_margin_db"] >= -1e-6
    for b in r.branches:
        if b.form == "LC_par":
            assert 1 / (2 * math.pi * math.sqrt(b.l * b.c)) >= 1.3 * hi * (1 - 1e-9)
        if b.form != "L":
            assert b.c >= 3e-12


def test_infeasible_targets_report_violations():
    t = robust.targets(8)
    impossible = robust.Targets(t.index, t.lo, t.hi, 0.01, 40.0, 120.0, t.att_half_lo_min, t.att_2hi_min)
    r = robust.optimise(robust.Topology(3, "ell5"), impossible)
    assert not r.feasible and r.violations and all(v > 0 for v in r.violations.values())
