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
            "bank_mute_isolation_db"} == {c.name for c in res.checks}
    for key in ("kicad_netlist", "bom", "svg"):
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
    _need_tools()
    r = check.evaluate(8, mc_runs=2)
    assert r["il_worst_q_db"] > r["nominal"][0]
