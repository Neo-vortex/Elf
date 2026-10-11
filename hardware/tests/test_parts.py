import pytest
from skidl import Circuit, Net

from neohw import parts


def test_resistor_carries_metadata_and_spice():
    ckt = Circuit()
    with ckt:
        r = parts.resistor("61.9", mpn="RC0805FR-0761R9L", torob="0805 resistor")
        r[1] += Net("a")
        r[2] += Net("GND")
    assert r.value == "61.9Ω"
    assert r.footprint == "Resistor_SMD:R_0805_2012Metric"
    assert r.mpn == "RC0805FR-0761R9L"
    assert r.spice_lines({"1": "a", "2": "0"}) == [f"R{r.ref} a 0 61.9"]


def test_capacitor_with_esr_adds_internal_node():
    ckt = Circuit()
    with ckt:
        c = parts.capacitor("33n", esr=0.1)
    lines = c.spice_lines({"1": "x", "2": "0"})
    assert lines == [f"R{c.ref}_esr x {c.ref}_esr 0.1", f"C{c.ref} {c.ref}_esr 0 3.3e-08"]


@pytest.mark.parametrize("state, closed, open_", [
    ("released", ("COM1", "NC1"), ("COM1", "NO1")),
    ("energised", ("COM1", "NO1"), ("COM1", "NC1")),
])
def test_relay_contacts_follow_state(state, closed, open_):
    ckt = Circuit()
    with ckt:
        k = parts.relay_dpdt(state)
    nodes = {p: p.lower() for p in ["COM1", "NC1", "NO1", "COM2", "NC2", "NO2", "COIL+", "COIL-"]}
    lines = k.spice_lines(nodes)
    assert f"R{k.ref}_{closed[1]} {closed[0].lower()} {closed[1].lower()} 0.05" in lines
    assert f"C{k.ref}_{open_[1]} {open_[0].lower()} {open_[1].lower()} 5e-13" in lines
    assert f"R{k.ref}_{open_[1]}_ins {open_[0].lower()} {open_[1].lower()} 1e+09" in lines
    assert len(lines) == 6  # closed R, open C || R_ins per pole; coil emits nothing


def test_relay_pins_use_datasheet_pad_numbers():
    # G6K-2F terminal arrangement: 1 COIL+, 2 NC1, 3 COM1, 4 NO1, 5 NO2, 6 COM2, 7 NC2, 8 COIL-
    ckt = Circuit()
    with ckt:
        k = parts.relay_dpdt("released")
    assert {p.name: p.num for p in k.pins} == {
        "COIL+": "1", "NC1": "2", "COM1": "3", "NO1": "4",
        "NO2": "5", "COM2": "6", "NC2": "7", "COIL-": "8"}


def test_relay_rejects_unknown_state():
    with pytest.raises(ValueError, match="state"):
        parts.relay_dpdt("half")


def test_non_positive_value_rejected():
    with pytest.raises(ValueError):
        parts.resistor("0")
