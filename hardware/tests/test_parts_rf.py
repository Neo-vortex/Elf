import pytest
from skidl import Circuit, Net

from neohw import parts
from neohw.spice import Op, Port, to_deck


def test_bfr92a_pins_and_subckt_call():
    ckt = Circuit()
    with ckt:
        q = parts.npn_bfr92a()
    assert {p.name: p.num for p in q.pins} == {"B": "1", "E": "2", "C": "3"}
    assert q.spice_lines({"C": "c", "B": "b", "E": "0"}) == [f"X{q.ref} c b 0 BFR92P"]
    assert "IMST" in q.spice_models[0] and ".ends BFR92P" in q.spice_models[0]


def test_bav99_is_two_series_diodes():
    ckt = Circuit()
    with ckt:
        d = parts.diode_pair_bav99()
    assert d.spice_lines({"A": "lo", "COM": "y", "K": "hi"}) == [
        f"D{d.ref}a lo y D1N4148", f"D{d.ref}b y hi D1N4148"]
    assert ".model D1N4148" in d.spice_models[0]


def test_gdt_is_small_capacitance():
    ckt = Circuit()
    with ckt:
        g = parts.gdt()
    assert g.spice_lines({"1": "ant", "2": "0"})[0] == f"C{g.ref} ant 0 1.5e-12"


def test_inductor_parallel_c_can_be_set():
    ckt = Circuit()
    with ckt:
        ind = parts.inductor("1m", 40, 0.8e6, c_par=6.3e-12)
    assert ind.spice_lines({"1": "a", "2": "b"})[2] == f"C{ind.ref}_p a b 6.3e-12"


def test_missing_model_file_names_the_path(monkeypatch, tmp_path):
    monkeypatch.setattr(parts, "MODELS_DIR", tmp_path)
    with pytest.raises(FileNotFoundError, match="bfr92p.lib"):
        parts.npn_bfr92a()


def test_part_models_are_emitted_once_before_elements():
    ckt = Circuit()
    with ckt:
        c, b, e = Net("c"), Net("b"), Net("GND")
        for _ in range(2):
            q = parts.npn_bfr92a(); q["C"] += c; q["B"] += b; q["E"] += e
    deck, _ = to_deck(ckt, "t", [Port(c, 1), Port(b, 2)], [Op()])
    assert deck.count(".subckt BFR92P") == 1
    assert deck.index(".subckt BFR92P") < deck.index("XQ1 ")
