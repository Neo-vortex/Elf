import json

import pytest

from neohw.blocks import preamp
from neohw.spice import Op, Port, Supply, to_deck
from neohw.checks import preamp as check
from neohw.tools import ToolNotFoundError, tool_path


@pytest.fixture
def tools():
    try:
        tool_path("ngspice"), tool_path("dot")
    except ToolNotFoundError as e:
        pytest.skip(str(e))


def _deck(blk):
    sup = [Supply(blk.ports["vcc"], 5.0)] if "vcc" in blk.ports else []
    return to_deck(blk.circuit, blk.name, [Port(blk.ports["rf_in"], 1), Port(blk.ports["rf_out"], 2)] + sup,
                   [Op()])[0]


@pytest.mark.parametrize("state, relay", [("amp", "energised"), ("bypass", "released")])
def test_preamp_states_drive_the_bypass_relay(state, relay):
    blk = preamp.build(state)
    (k,) = [p for p in blk.circuit.parts if p.ref.startswith("K")]
    assert k.state == relay
    assert "BFR92P" in _deck(blk)


def test_preamp_rejects_unknown_state():
    with pytest.raises(ValueError):
        preamp.build("off")


@pytest.mark.integration
def test_preamp_meets_targets_and_writes_characterisation(tools, tmp_path):
    res = check.run(tmp_path / "o", tmp_path / "c")
    assert res.passed, [(c.name, c.value) for c in res.checks]
    doc = json.loads((tmp_path / "c" / "preamp.json").read_text())
    assert doc["block"] == "preamp" and doc["status"] == "pass"
    assert len(doc["freq_hz"]) == len(doc["gain_db"]) == len(doc["nf_db"]) > 50
    assert doc["oip3_dbm"] >= check.OIP3_MIN_DBM and doc["bypass_loss_db"] < 0.3
    assert 3.0 < min(doc["nf_db"]) and max(doc["nf_db"]) <= check.NF_MAX_DB   # Source/Load noise, not ports
    ic = {c.name: c for c in res.checks}["collector_current_max_ma"]
    assert ic.passed and ic.value < check.IC_MAX_MA


def test_preamp_uses_active_bias():
    blk = preamp.build("amp")
    values = [p.value for p in blk.circuit.parts]
    assert "BC857B" in values and "BFR92A" in values


def test_corners_cover_supply_gain_and_temperature():
    volts = {c[0] for c in check.CORNERS}
    gains = {c[1] for c in check.CORNERS}
    temps = {c[2] for c in check.CORNERS}
    assert volts == {4.75, 5.0, 5.25} and min(gains) <= 65 and max(gains) >= 150 and temps == {0.0, 27.0, 50.0}
