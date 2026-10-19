import pytest

from neohw.blocks import bpf, bpf_bank
from neohw.spice import Op, Port, to_deck


@pytest.mark.parametrize("index", range(1, 12))
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
        bpf.design(12)


def test_bank_has_one_relay_per_filter_and_valid_deck():
    blk = bpf_bank.build(3)
    relays = [p for p in blk.circuit.parts if p.ref.startswith("K")]
    assert len(relays) == 22 and sum(p.state == "energised" for p in relays) == 2
    deck, _ = to_deck(blk.circuit, "t", [Port(blk.ports["rf_in"], 1), Port(blk.ports["rf_out"], 2)], [Op()])
    assert "f3_in" in deck and "f10_out" in deck


@pytest.mark.parametrize("freq, index", [(0.1e6, 1), (0.499e6, 1), (0.5e6, 2), (14.074e6, 7),
                                         (39.9e6, 10), (40e6, 11), (50.313e6, 11), (54e6, 11)])
def test_selection_rule(freq, index):
    assert bpf_bank.select(freq) == index


@pytest.mark.parametrize("freq", [0.05e6, 54.1e6])
def test_selection_outside_range_rejected(freq):
    with pytest.raises(ValueError):
        bpf_bank.select(freq)


def test_bus_in_and_bus_out_never_share_a_relay():
    blk = bpf_bank.build(5)
    relays = [p for p in blk.circuit.parts if p.ref.startswith("K")]
    assert len(relays) == 22
    for k in relays:
        nets = {pin.net.name for pin in k.pins if pin.net is not None}
        assert len(nets & {"rf_in", "rf_out"}) == 1, (k.ref, nets)


def test_bank_inductors_carry_selection_groups():
    blk = bpf_bank.build(None)
    inds = [p for p in blk.circuit.parts if p.ref.startswith("L")]
    assert inds and all(p.select_group and p.select_target > 0 for p in inds)
    assert all("build sheet" in p.bom_note for p in inds)
    caps = [p for p in blk.circuit.parts if p.ref.startswith("C")]
    assert all("LCR meter" in p.bom_note for p in caps)


def test_worst_case_q_reference_is_the_upper_edge():
    nominal = bpf_bank.build(1)
    worst = bpf_bank.build(1, q_ref="upper")
    r = lambda blk: sorted(l for p in blk.circuit.parts if p.ref.startswith("L")
                           for l in p.spice_lines({"1": "a", "2": "b"}) if l.startswith("R"))
    assert r(nominal) != r(worst)
    with pytest.raises(ValueError):
        bpf_bank.build(1, q_ref="lower")


def test_band_10_split_keeps_2f_of_its_lower_edge_out_of_the_passband():
    # final review of plan 2.4: with one 30–54 MHz filter, 2·30 MHz sat just above its cutoff
    assert bpf.BANDS[9] == (30e6, 40e6) and bpf.BANDS[10] == (40e6, 54e6)
    for lo, hi in bpf.BANDS[9:]:
        assert 2 * lo > 1.4 * hi


from neohw import population as pop


def test_every_bank_inductor_has_an_smd_china_variant_and_an_axial_iran_variant():
    bank = bpf_bank.build(7)
    inductors = [p for p in bank.circuit.parts if p.ref.startswith("L")]
    assert inductors
    from neohw.units import parse_si
    for p in inductors:
        chip = "1812" if parse_si(p.value[:-1]) >= 1e-6 else "1210"
        assert p.footprint == f"NeoRadio:L_{chip}_Axial_P10.16mm"
        china, iran = pop.variant(p, "china"), pop.variant(p, "iran")
        assert f"{chip} wirewound" in china.ident and "TDK" in china.ident and china.tol == 0.05
        assert "axial" in iran.ident and "Bourns 78F" in iran.ident and iran.tol == 0.03
        assert "Q≥" in china.ident and "Q≥" in iran.ident and "I≥20 mA" in iran.ident


def test_iran_capacitors_use_e12_values_only_and_china_e24():
    from neohw import eseries
    e12 = {round(v, 15) for v in eseries.values(eseries.E12, 1e-13, 1e-6)}
    for index in range(1, 12):
        for pos in bpf.design(index).positions:
            if pos.kind.startswith("C"):
                assert all(round(v, 15) in e12 for v in pos.iran_parts), (index, pos)


def test_capacitor_slots_fit_each_populations_realisation():
    from neohw.units import parse_si
    blk = bpf.build(1)
    caps = [p for p in blk.circuit.parts if p.ref.startswith("C")]
    for name in pop.POPULATIONS:
        fitted = sorted(parse_si(pop.variant(c, name).value[:-1]) for c in caps if pop.fitted(c, name))
        want = sorted(v for pos in bpf.design(1).positions if pos.kind.startswith("C")
                      for v in (pos.parts if name == "china" else pos.iran_parts))
        assert fitted == pytest.approx(want, rel=1e-9), name
    for c in caps:
        for name in pop.POPULATIONS:
            v = pop.variant(c, name)
            if v is not None:
                assert v.tol == bpf.CAP_TOL[name] and f"±{bpf.CAP_TOL[name] * 100:g} %" in v.ident
