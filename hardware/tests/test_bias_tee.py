import pytest

from neohw.blocks import bias_tee
from neohw.spice import Op, Port, Supply, to_deck
from neohw.checks import bias_tee as check
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


def test_bias_tee_has_gdt_bleed_block_and_damped_choke_chain():
    blk = bias_tee.build()
    values = sorted(p.value for p in blk.circuit.parts)
    assert values == sorted(["GDT 90V", "1MΩ", "1uF", "4.7uH", "47uH", "1mH", "2.2kΩ", "100nF", "10uF"])
    assert "bias_feed" in _deck(blk)


@pytest.mark.integration
def test_bias_tee_meets_targets(tools, tmp_path):
    res = check.run(tmp_path / "o", tmp_path / "c")
    assert res.passed, [(c.name, c.value) for c in res.checks]
