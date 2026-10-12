import pytest

from neohw.blocks import protection
from neohw.spice import Op, Port, Supply, to_deck
from neohw.checks import protection as check
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


def test_limiter_diodes_sit_between_the_bias_rails():
    blk = protection.build()
    (d,) = [p for p in blk.circuit.parts if p.value == "BAV99"]
    assert (d["A"].net.name, d["COM"].net.name, d["K"].net.name) == ("lim_lo", "lim_y", "lim_hi")
    _deck(blk)


@pytest.mark.integration
def test_limiter_imd3_below_target(tools):
    r = check.imd3()
    assert r["pout_dbm"] == pytest.approx(-10.0, abs=0.1)
    assert r["imd3_dbm"] <= check.IMD3_MAX_DBM
