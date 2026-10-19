import json

import pytest

from neohw.checks import bpf as check
from neohw.tools import ToolNotFoundError, tool_path
from neohw.touchstone import read_s2p


def _need_tools():
    try:
        tool_path("ngspice"), tool_path("dot")
    except ToolNotFoundError as e:
        pytest.skip(str(e))


def test_rejection_target_only_where_3f_is_out_of_band():
    assert [check.has_rejection_target(i) for i in range(1, 12)] == [False, False] + [True] * 9


@pytest.mark.integration
def test_filter_8_meets_targets_with_monte_carlo():
    _need_tools()
    r = check.evaluate(8, mc_runs=10)
    il, rl, rej = r["nominal"]
    assert il <= 2.0 and rl >= 14.0 and rej >= 50.0
    assert r["mc_passed"] == 10


@pytest.mark.integration
def test_muted_bank_isolates():
    _need_tools()
    assert check.mute_isolation_db() >= check.MUTE_MIN_DB


@pytest.mark.integration
def test_run_writes_s2p_and_bank_file(tmp_path):
    _need_tools()
    res = check.run(tmp_path / "out", tmp_path / "char", mc_runs=4, filters=[8])
    f, s = read_s2p(tmp_path / "char" / "bpf_8.s2p")
    assert f[0] == pytest.approx(0.05e6) and f[-1] >= 200e6
    doc = json.loads((tmp_path / "char" / "bpf_bank.json").read_text())
    (entry,) = doc["filters"]
    assert entry["index"] == 8 and entry["s2p"] == "bpf_8.s2p" and entry["inductors"]
    assert doc["selection_rule"] == check.SELECTION_RULE
    assert {"bpf8_il_db", "bpf8_rl_db", "bpf8_rej3f_db", "bpf8_mc_pass_pct",
            "bpf8_iran_il_db", "bpf8_iran_rl_db", "bpf8_iran_rej3f_db", "bpf8_iran_mc_pass_pct",
            "bank_mute_isolation_db"} == {c.name for c in res.checks}
    assert set(entry["populations"]) == {"china", "iran"}
    assert entry["il_db"] == entry["populations"]["china"]["il_db"]
    assert entry["populations"]["iran"]["mc_runs"] == 4
    for key in ("kicad_netlist", "bom", "bom_iran", "svg"):
        assert res.artefacts[key].exists()


@pytest.mark.integration
def test_monte_carlo_is_reproducible():
    _need_tools()
    a, b = check.evaluate(9, mc_runs=5), check.evaluate(9, mc_runs=5)
    assert a["nominal"] == b["nominal"] and a["mc_passed"] == b["mc_passed"]


def test_build_sheet_names_refs_and_targets():
    sheet = check.build_sheet(None)
    refs = [r for e in sheet for r in e["refs"]]
    assert len(refs) == len(set(refs)) > 0
    for e in sheet:
        assert 1 <= len(e["refs"]) == len(e["buy_h"]) <= 2
        assert sum(e["buy_h"]) == pytest.approx(e["target_h"], rel=0.10)
        assert e["select"] == "sum within ±3 % of target_h"


@pytest.mark.integration
def test_worst_case_q_loss_is_reported_and_higher():
    # the worst case evaluates every inductor's datasheet loss at the upper band edge instead of
    # the centre, in both populations
    _need_tools()
    for population in ("china", "iran"):
        r = check.evaluate(8, mc_runs=0, population=population)
        assert r["il_worst_q_db"] > r["nominal"][0], population


@pytest.mark.integration
def test_the_system_limited_filter_reports_its_noise_floor_rise(tmp_path):
    _need_tools()
    res = check.run(tmp_path / "out", tmp_path / "char", mc_runs=2, filters=[1])
    names = {c.name: c for c in res.checks}
    for prefix in ("", "iran_"):
        rise = names[f"bpf1_{prefix}noise_rise_db"]
        assert rise.limit == f"≤ {check.NOISE_RISE_MAX_DB:g}" and rise.passed
        assert names[f"bpf1_{prefix}il_db"].limit == "≤ 8"


@pytest.mark.integration
def test_filter_8_meets_targets_in_both_populations():
    _need_tools()
    for p in ("china", "iran"):
        r = check.evaluate(8, mc_runs=20, population=p)
        il, rl, rej = r["nominal"]
        assert il <= check.il_limit_db(8) and rl >= check.RL_MIN_DB, p
        assert r["mc_passed"] >= 19, p


def test_a_variants_tolerance_drives_the_monte_carlo_variation():
    import numpy as np
    from skidl import Circuit, Net

    from neohw import parts, population as pop
    ckt = Circuit()
    with ckt:
        a, b = Net("a"), Net("b")
        c = parts.capacitor(100e-12, tol=0.05)
        pop.set_variants(c, pop.Variant("100pF", "100 pF ±1 %", tol=0.01), pop.Variant("100pF", "100 pF ±2 %", tol=0.02))
        r = pop.only(parts.resistor(50), "iran")
        for p in (c, r):
            p[1] += a
            p[2] += b
    rng = np.random.default_rng(1)
    draws = [check._variation(ckt, rng, "china") for _ in range(200)]
    assert all(set(d) == {c.ref} for d in draws)          # the Iran-only resistor is not fitted
    ks = [d[c.ref] for d in draws]
    assert all(0.99 <= k <= 1.01 for k in ks) and max(ks) - min(ks) > 0.015


def test_only_the_system_limited_filter_gets_the_8_db_cap():
    assert set(check.SYSTEM_LIMITED) == {1}
    assert check.il_limit_db(1) == check.IL_SYSTEM_CAP_DB == 8.0
    assert check.il_limit_db(2) == 3.0 and check.il_limit_db(11) == 2.0


def test_noise_floor_rise_follows_p372_residential():
    import math
    # 8 dB of filter loss at 0.5 MHz, where residential noise is ≈ 81 dB above kT0: no measurable rise
    assert check.noise_floor_rise_db(8.0, 0.5e6) < 1e-3
    # the formula itself: F = L·F_rest against Fa
    fa_db = 72.5 - 27.7 * math.log10(30.0)
    want = 10 * math.log10(1 + (10 ** ((3.0 + check.REST_NF_DB) / 10) - 1) / 10 ** (fa_db / 10))
    assert check.noise_floor_rise_db(3.0, 30e6) == pytest.approx(want)


def test_rest_noise_figure_is_not_below_the_characterised_mixer():
    import json
    from pathlib import Path
    mixer = json.loads((Path(__file__).resolve().parents[1] / "characterized/mixer.json").read_text())
    nf = next(c["value"] for c in mixer["checks"] if c["name"] == "noise_figure_db")
    assert check.REST_NF_DB >= nf
