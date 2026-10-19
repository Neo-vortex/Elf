import math

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
    monkeypatch.setattr(robust, "today_selectivity", lambda index: {"att_half_lo_db": 70.0, "att_2hi_db": 21.0})
    t = robust.targets(5)
    assert t.att_half_lo_min == pytest.approx(67.0) and t.att_2hi_min == pytest.approx(20.0)


def test_today_selectivity_is_todays_design_in_the_robust_model():
    from neohw.blocks import bpf
    d = bpf.design(6)
    m = ladder.metrics(ladder.from_positions(d.positions), d.lo_hz, d.hi_hz,
                       ladder.robust_losses(d.centre_hz), ladder.bank_bus())
    s = robust.today_selectivity(6)
    assert s == {"att_half_lo_db": m["att_half_lo_db"], "att_2hi_db": m["att_2hi_db"]}


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


def test_infeasible_targets_report_violations():
    t = robust.targets(8)
    impossible = robust.Targets(t.index, t.lo, t.hi, 0.01, 40.0, 120.0, t.att_half_lo_min, t.att_2hi_min)
    r = robust.optimise(robust.Topology(3, "ell5"), impossible)
    assert not r.feasible and r.violations and all(v > 0 for v in r.violations.values())
