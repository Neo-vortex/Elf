from skidl import Circuit

from neohw import parts


def test_fst3253_pins_follow_the_datasheet_and_section_2_may_stay_open():
    ckt = Circuit()
    with ckt:
        u = parts.fst3253()
    assert {p.name: p.num for p in u.pins}["1A"] == "7" and {p.name: p.num for p in u.pins}["VCC"] == "16"
    assert set(u.nc_pins) == {"2A", "2B0", "2B1", "2B2", "2B3"}
    assert "VCC must be 4.0-5.5 V" in u.spice_models[0]


def test_lm4562_dual_is_two_macromodel_instances():
    ckt = Circuit()
    with ckt:
        u = parts.lm4562_dual()
    n = {p.name: p.name.lower() for p in u.pins}
    assert u.spice_lines(n) == [f"X{u.ref}a +ina -ina outa v+ v- LM4562", f"X{u.ref}b +inb -inb outb v+ v- LM4562"]


def test_cs5361_input_is_7k5_differential_with_vq():
    ckt = Circuit()
    with ckt:
        u = parts.cs5361()
    n = {p.name: p.name.lower() for p in u.pins}
    lines = u.spice_lines(n)
    assert f"R{u.ref}_l ainl+ ainl- 7500" in lines and f"V{u.ref}_vq {u.ref}_vqi gnd dc 2.5" in lines
    assert {p.name: p.num for p in u.pins}["AINL+"] == "16"
