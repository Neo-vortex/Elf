import pytest

from neohw.blocks import bpf, bpf_bank
from neohw.spice import Op, Port, to_deck


@pytest.mark.parametrize("index", range(1, 11))
def test_design_has_hpf_then_lpf_with_buyable_parts(index):
    d = bpf.design(index)
    kinds = [p.kind for p in d.positions]
    assert len(kinds) == bpf.HPF_ORDER + bpf.LPF_ORDER[index - 1]
    assert kinds[:2] == ["C_series", "L_shunt"] and kinds[5:7] == ["C_shunt", "L_series"]
    for pos in d.positions:
        assert 1 <= len(pos.parts) <= 2
        assert sum(pos.parts) == pytest.approx(pos.ideal, rel=0.10)  # a selected pair reaches it
        if pos.kind.startswith("L"):
            assert min(pos.parts) >= bpf.L_MIN


def test_design_rejects_bad_index():
    with pytest.raises(ValueError):
        bpf.design(11)


def test_bank_has_one_relay_per_filter_and_valid_deck():
    blk = bpf_bank.build(3)
    relays = [p for p in blk.circuit.parts if p.ref.startswith("K")]
    assert len(relays) == 10 and sum(p.state == "energised" for p in relays) == 1
    deck, _ = to_deck(blk.circuit, "t", [Port(blk.ports["rf_in"], 1), Port(blk.ports["rf_out"], 2)], [Op()])
    assert "f3_in" in deck and "f10_out" in deck


@pytest.mark.parametrize("freq, index", [(0.1e6, 1), (0.499e6, 1), (0.5e6, 2), (14.074e6, 7),
                                         (50.313e6, 10), (54e6, 10)])
def test_selection_rule(freq, index):
    assert bpf_bank.select(freq) == index


@pytest.mark.parametrize("freq", [0.05e6, 54.1e6])
def test_selection_outside_range_rejected(freq):
    with pytest.raises(ValueError):
        bpf_bank.select(freq)
