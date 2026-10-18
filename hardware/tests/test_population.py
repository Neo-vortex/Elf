import pytest
from skidl import Circuit, Net

from neohw import parts, population as pop
from neohw.spice import Port, Sp, SpiceError, to_deck


def _divider(fit_lower_in: str | None):
    ckt = Circuit()
    with ckt:
        a, mid, gnd = Net("a"), Net("mid"), Net("GND")
        r1 = parts.resistor("1k")
        r1[1] += a
        r1[2] += mid
        r2 = parts.resistor("1k")
        r2[1] += mid
        r2[2] += gnd
        if fit_lower_in:
            pop.only(r2, fit_lower_in)
    return ckt, a, mid, r2


def _elements(deck: str) -> list[str]:
    return [line.split()[0] for line in deck.splitlines() if line.startswith("R")]


def test_unfitted_part_emits_no_spice_and_its_nets_are_not_dangling():
    ckt, a, mid, r2 = _divider("iran")
    ports, sweep = [Port(a, 1), Port(mid, 2)], [Sp(1e6, 2e6, 3, "lin")]
    element = f"R{r2.ref}"
    deck, _ = to_deck(ckt, "t", ports, sweep, population="china")
    assert element not in _elements(deck)
    deck_iran, _ = to_deck(ckt, "t", ports, sweep, population="iran")
    assert element in _elements(deck_iran) and len(_elements(deck_iran)) == 2


def test_a_really_dangling_net_is_still_reported():
    ckt = Circuit()
    with ckt:
        a, b = Net("a"), Net("b")
        r = parts.resistor("1k")
        r[1] += a
        r[2] += b
    with pytest.raises(SpiceError, match="single connection"):
        to_deck(ckt, "t", [Port(a, 1)], [Sp(1e6, 2e6, 3, "lin")])


def test_variant_model_and_tolerance_follow_the_population():
    with Circuit():
        c = parts.capacitor("100p", tol=0.02)
        pop.set_variants(c, pop.Variant("100 pF", "100 pF ±1 % C0G 0805 50 V", tol=0.01),
                         pop.Variant("100 pF", "100 pF ±2 % C0G 0805 50 V (LCR-selected)", tol=0.02))
        assert pop.tol(c, "china") == 0.01 and pop.tol(c, "iran") == 0.02
        assert pop.fitted(c, "china") and pop.fitted(c, "iran")


def test_variant_spice_lines_replace_the_part_model():
    ckt, a, mid, r2 = _divider(None)
    with ckt:
        pop.set_variants(r2, pop.Variant("2k", "x", spice_lines=lambda n, k=1.0: [f"R{r2.ref} {n['1']} {n['2']} 2k"]), None)
    deck, _ = to_deck(ckt, "t", [Port(a, 1), Port(mid, 2)], [Sp(1e6, 2e6, 3, "lin")])
    assert f"R{r2.ref} mid 0 2k" in deck


def test_unknown_population_is_rejected():
    with Circuit():
        r = parts.resistor("1k")
        with pytest.raises(ValueError, match="population"):
            pop.fitted(r, "germany")
