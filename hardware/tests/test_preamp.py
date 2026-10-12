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
