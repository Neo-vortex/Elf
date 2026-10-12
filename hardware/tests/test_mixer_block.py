import pytest

from neohw.blocks import mixer
from neohw.ngspice import run_decks
from neohw.spice import Load, Op, QuadratureLO, Supply, to_deck
from neohw.tools import ToolNotFoundError, tool_path


@pytest.fixture
def tools():
    try:
        tool_path("ngspice"), tool_path("dot")
    except ToolNotFoundError as e:
        pytest.skip(str(e))


def test_block_wires_gray_code_phases_to_i_and_q():
    blk = mixer.build()
    (u,) = [p for p in blk.circuit.parts if p.value == "FST3253"]
    amps = [p for p in blk.circuit.parts if p.value == "LM4562"]
    pairs = sorted((a["+INA"].net.name, a["+INB"].net.name) for a in amps)
    # (S1, S0) = 00, 01, 11, 10 -> B0, B1, B3, B2: I = B0 - B3, Q = B1 - B2
    assert pairs == [("mix_b0", "mix_b3"), ("mix_b1", "mix_b2")]
    assert u["VCC"].net.name == "+5V_A" and u["/2OE"].net.name == "+5V_A"


@pytest.mark.integration
def test_bias_point_is_mid_supply(tools):
    blk = mixer.build()
    deck, raws = to_deck(blk.circuit, "op", [Load(blk.ports["rf_in"], "rf"), Supply(blk.ports["vcc"], 5.0),
                                             QuadratureLO(blk.ports["s0"], blk.ports["s1"], 14e6)], [Op()],
                         extra_lines=[".options reltol=1e-4 method=gear"])
    (res,) = run_decks([deck], raws)
    d = res[raws[0]][0].data
    # with the LO frozen at t = 0 only B0 is connected; the other caps float in a DC solve
    assert d["v(mix_a)"][0].real == pytest.approx(2.5, abs=0.05)
    assert d["v(mix_b0)"][0].real == pytest.approx(2.5, abs=0.05)
