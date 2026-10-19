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


def test_snap_uses_one_choke_when_close_and_reports_the_unreachable():
    assert robust.snap_l(39e-6) == (39e-6,)
    parts_ = robust.snap_l(50e-6)
    assert abs(sum(parts_) / 50e-6 - 1) <= 0.02
    with pytest.raises(ValueError, match="cannot realise"):
        robust.snap_l(5000e-6)                     # above the largest choke pair


def test_snap_takes_parts_from_each_values_own_series():
    from neohw import inductors
    for target in (0.101e-6, 0.95e-6, 1.1e-6, 13.7e-6, 600e-6):
        parts_ = robust.snap_l(target)
        assert 1 <= len(parts_) <= 2 and abs(sum(parts_) / target - 1) <= 0.02
        assert all(p in inductors.available(inductors.filter_series(p)) for p in parts_)
    assert robust.snap_l(0.105e-6) == (0.1e-6,)    # no single or pair within 2 %: closest within 5 %
    with pytest.raises(ValueError, match="cannot realise"):
        robust.snap_l(0.09e-6)                     # 10 % from the smallest choke


def _realised_8():
    return robust.realise(robust.best(8), robust.targets(8, floor_margin=robust.FLOOR_MARGIN_DB))


@pytest.mark.integration
def test_realisation_fixes_inductors_and_snaps_capacitors_per_population():
    from neohw import eseries
    from neohw.blocks.bpf import C_MIN
    r = robust.best(8)
    real = _realised_8()
    for k, b in enumerate(r.branches):
        if b.form != "C":
            parts_ = real.l_parts[k]
            assert parts_ == robust.snap_l(b.l)
            assert real.l_deviation[k] == pytest.approx(sum(parts_) / b.l - 1)
            assert real.branches_china[k].l == pytest.approx(sum(parts_))
            assert real.branches_iran[k].l == pytest.approx(b.l)
            assert [n for n, _ in real.branches_china[k].parts] == list(parts_)
    for pop, caps, series in (("china", real.c_china, eseries.E24), ("iran", real.c_iran, eseries.E12)):
        brs = getattr(real, f"branches_{pop}")
        assert set(caps) == {k for k, b in enumerate(brs) if b.form != "L"}
        for k, parts_ in caps.items():
            assert brs[k].c == pytest.approx(sum(parts_))
            assert 1 <= len(parts_) <= 2 and all(p >= C_MIN and p in eseries.values(series) for p in parts_)
        m = real.metrics[pop]
        assert m["stop_margin_db"] == pytest.approx(robust.dense_stop_margin_db(brs, robust.targets(8)))
    assert [(b.place, b.form) for b in real.branches_china] == [(b.place, b.form) for b in r.branches]


@pytest.mark.integration
def test_realised_filter_8_holds_the_floor_on_a_dense_grid():
    real = _realised_8()
    for pop in ("china", "iran"):
        assert real.metrics[pop]["stop_margin_db"] >= 0, pop


def _two_part_ladder():
    return [ladder.Branch("series", "L", l=50.3e-6, parts=((47e-6, 47e-6), (3.3e-6, 3.3e-6))),
            ladder.Branch("shunt", "C", c=105e-12), ladder.Branch("series", "LC_par", l=0.33e-6, c=20e-12)]


def test_draws_scale_every_part_within_its_tolerance_and_repeat():
    from neohw import inductors
    br, caps = _two_part_ladder(), {1: (100e-12, 5e-12), 2: (20e-12,)}
    for pop, ctol in (("china", 0.01), ("iran", 0.02)):
        scale = robust._scale(br, pop, np.random.default_rng(0), 500, caps)
        kl, kc = scale[0]
        for i, (n, _) in enumerate(br[0].parts):
            tol = inductors.tolerance(inductors.row(inductors.filter_series(n), n), pop)
            assert np.abs(kl[:, i] - 1).max() <= tol and np.abs(kl[:, i] - 1).max() > 0.9 * tol
        assert np.abs(scale[1][1] - 1).max() <= ctol and scale[0][1] is None and scale[1][0] is None
        assert np.abs(scale[2][0] - 1).max() <= inductors.tolerance(inductors.nearest_row("bourns_78f", 0.33e-6), pop)
        again = robust._scale(br, pop, np.random.default_rng(0), 500, caps)
        assert all(np.array_equal(a, b) for x, y in zip(scale, again) for a, b in zip(x, y) if a is not None)


def test_draw_metrics_are_the_worst_of_with_and_without_the_choke_capacitance():
    t = robust.targets(8)
    br = _two_part_ladder()
    br[2] = ladder.Branch("series", "LC_par", l=0.33e-6, c=20e-12, parts=((0.33e-6, 0.33e-6),))
    scale = robust._scale(br, "china", np.random.default_rng(1), 4, {})
    il, rl, rej = robust._mc(br, t, scale)
    for d in range(4):
        one = []
        for b, (kl, kc) in zip(br, scale):
            parts = tuple((n, a * kl[d, i]) for i, (n, a) in enumerate(b.parts))
            one.append(ladder.Branch(b.place, b.form, l=sum(a for _, a in parts), parts=parts,
                                     c=b.c * kc[d] if kc is not None else 0.0))
        ms = [ladder.metrics(one, t.lo, t.hi, ladder.robust_losses(math.sqrt(t.lo * t.hi), cpar), ladder.bank_bus())
              for cpar in (True, False)]
        assert il[d] == pytest.approx(max(m["il_db"] for m in ms))
        assert rl[d] == pytest.approx(min(m["rl_db"] for m in ms))
        assert rej[d] == pytest.approx(min(m["rej3f_db"] for m in ms))


def test_nominal_metrics_are_the_worst_of_with_and_without_the_choke_capacitance():
    t = robust.targets(8)
    br = _two_part_ladder()
    ms = [ladder.metrics(br, t.lo, t.hi, ladder.robust_losses(math.sqrt(t.lo * t.hi), cpar), ladder.bank_bus())
          for cpar in (True, False)]
    m = robust._metrics(br, t)
    assert m["il_db"] == max(x["il_db"] for x in ms) and m["rl_db"] == min(x["rl_db"] for x in ms)
    assert m["rej3f_db"] == min(x["rej3f_db"] for x in ms) and m["att_2hi_db"] == min(x["att_2hi_db"] for x in ms)


def test_fast_yield_judges_the_real_targets_not_the_margins(monkeypatch):
    real = robust.Realised((), (), {}, {}, {}, None, {}, {}, {})
    monkeypatch.setattr(robust, "_mc", lambda br, t, scale: (np.array([1.9, 1.9]), np.array([14.1, 13.9]),
                                                             np.array([50.1, 50.1])))
    assert robust.fast_yield(real, robust.targets(8, 18.0, 60.0), "china", runs=2) == 50.0


@pytest.mark.integration
def test_yield_penalty_is_deterministic_and_grows_as_a_design_loses_margin():
    t = robust.targets(8)
    br = list(robust.best(8).branches)
    draws = robust._penalty_draws(br, ("china", "iran"))
    p0 = robust._fail_fraction(br, t, draws)
    assert p0 == robust._fail_fraction(br, t, robust._penalty_draws(br, ("china", "iran")))
    k = next(k for k, b in enumerate(br) if b.form == "L" and b.place == "series")
    worse = list(br)
    worse[k] = ladder.Branch("series", "L", l=br[k].l * 0.93)
    assert robust._fail_fraction(worse, t, draws) > p0 >= 0


@pytest.mark.integration
def test_best_filter_8_relies_on_no_self_resonance():
    r = robust.best(8)
    t = robust.targets(8)
    m = ladder.metrics(list(r.branches), t.lo, t.hi, ladder.robust_losses(math.sqrt(t.lo * t.hi), False), ladder.bank_bus())
    assert m["rl_db"] >= 15.5 - 1e-3 and m["rej3f_db"] >= 53.0 - 1e-3


def test_design_retries_with_raised_margins_then_reports_the_numbers(monkeypatch):
    calls = []
    monkeypatch.setattr(robust, "best", lambda i, rl, rej, fm: calls.append((rl, rej, fm)) or "result")
    real = robust.Realised((), (), {}, {}, {}, None, {"china": {"stop_margin_db": 0.2}, "iran": {"stop_margin_db": 0.1}},
                           {}, {})
    monkeypatch.setattr(robust, "realise", lambda r, t: real)
    monkeypatch.setattr(robust, "fast_yield", lambda real, t, pop: 96.5 if pop == "china" else 99.0)
    with pytest.raises(robust.DesignError, match="96.5"):
        robust.design(8)
    assert calls == [(15.5, 53.0, 0.5), (16.5, 55.0, 1.0), (17.5, 57.0, 1.5), (18.5, 59.0, 2.0)]


def test_design_retries_when_a_realised_nominal_misses_the_floor(monkeypatch):
    calls = []
    monkeypatch.setattr(robust, "best", lambda i, rl, rej, fm: calls.append(fm) or fm)

    def realise(r, t):
        miss = -0.1 if r < 1.0 else 0.05
        return robust.Realised((), (), {}, {}, {}, None,
                               {"china": {"stop_margin_db": miss}, "iran": {"stop_margin_db": 0.3}}, {}, {})
    monkeypatch.setattr(robust, "realise", realise)
    monkeypatch.setattr(robust, "fast_yield", lambda real, t, pop: 100.0)
    real = robust.design(8)
    assert calls == [0.5, 1.0] and real.yield_pct == {"china": 100.0, "iran": 100.0}
    assert real.margins["retries"] == 1 and real.margins["floor_margin_db"] == 1.0
    assert real.margins["rl_min"] == 16.5 and real.margins["rej_min"] == 55.0


def test_china_yield_uses_china_tolerances(monkeypatch):
    """A design whose margins only survive ±3 % must fail the China yield (±10 % chokes).
    The yield penalty (spec §4) makes even zero-margin targets yield, so it is switched off here
    to get such a design."""
    from neohw import inductors
    monkeypatch.setattr(robust, "YIELD_WEIGHT_DB", 0.0)
    t = robust.targets(8, 14.0, 50.0)                  # no margin at all
    r = robust.optimise(robust.Topology(3, "cheb7"), t, starts=1)
    real = robust.realise(r, t)
    china, iran = (robust.fast_yield(real, robust.targets(8), p) for p in ("china", "iran"))
    assert china <= iran                       # ±10 % chokes never yield better than ±3 %
    assert china < 97                          # and a zero-margin design does not pass the China gate


@pytest.mark.integration
def test_design_of_filter_8_holds_yield_in_both_populations():
    real = robust.design(8)
    assert real.yield_pct["china"] >= 97 and real.yield_pct["iran"] >= 97
    for pop in ("china", "iran"):
        m = real.metrics[pop]
        assert m["il_db"] <= 2.0 and m["rl_db"] >= 14 and m["rej3f_db"] >= 50
