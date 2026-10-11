import json

import pytest

from neohw.blocks import attenuator
from neohw.checks import attenuator as check
from neohw.spice import to_deck, Op, Port
from neohw.tools import ToolNotFoundError, tool_path


def test_block_has_ports_and_expected_parts():
    blk = attenuator.build("energised")
    assert set(blk.ports) == {"rf_in", "rf_out"}
    values = sorted(p.value for p in blk.circuit.parts)
    assert values == ["249Ω", "61.9Ω", "61.9Ω", "G6K-2F"]


def test_released_state_closes_bypass_contacts():
    blk = attenuator.build("released")
    deck, _ = to_deck(blk.circuit, "t", [Port(blk.ports["rf_in"], 1), Port(blk.ports["rf_out"], 2)], [Op()])
    assert "_NC1 rf_in" in deck and " 0.05" in deck


@pytest.mark.integration
def test_attenuator_check_passes_and_writes_artefacts(tmp_path):
    try:
        tool_path("ngspice"), tool_path("dot")
    except ToolNotFoundError as e:
        pytest.skip(str(e))
    res = check.run(tmp_path / "out", tmp_path / "char")
    details = {c.name: (c.value, c.limit, c.passed) for c in res.checks}
    assert res.passed, details
    assert {c.name for c in res.checks} == {"attenuation_min_db", "attenuation_max_db",
                                            "pad_return_loss_db", "bypass_loss_db"}
    doc = json.loads((tmp_path / "char" / "attenuator.json").read_text())
    assert doc["status"] == "pass" and len(doc["freq_hz"]) == 201
    for key in ("kicad_netlist", "bom", "svg"):
        assert res.artefacts[key].exists()
