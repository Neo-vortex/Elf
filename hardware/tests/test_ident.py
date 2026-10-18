from skidl import Circuit, Net

from neohw import ident, parts, population as pop
from neohw.blocks import board


def test_passives_carry_a_full_specification():
    with Circuit():
        c = parts.capacitor("47p", tol=0.01, note="filter")
        r = parts.resistor("61.9")
        assert ident.of(c) == "47 pF ±1 % C0G 0805 50 V"
        assert ident.of(r) == "61.9 Ω ±1 % 0805"


def test_dielectric_and_voltage_follow_the_note():
    with Circuit():
        c = parts.capacitor("22u", footprint="Capacitor_SMD:C_1206_3216Metric", note="X5R 16 V bulk")
        l = parts.inductor(10e-6, 20.0, 1e6, c_par=2.5e-12, dcr=0.3, tol=0.2,
                           footprint="Inductor_SMD:L_1210_3225Metric", note="filter, rated ≥ 300 mA")
        assert ident.of(c) == "22 µF ±10 % X5R 1206 16 V"
        assert ident.of(l) == "10 µH ±20 % 1210 wirewound, Q≥20 at 1 MHz, SRF≥31.8 MHz, I≥300 mA"


def test_catalogue_numbers_are_rejected():
    ckt = Circuit()
    with ckt:
        u = parts.lm75a()
        u.mpn = "C21101"
        for pin in u.pins:
            pin += Net(f"n_{pin.name}")
    assert any("catalogue" in v for v in ident.violations(ckt))


def test_variant_ident_wins_and_unfitted_parts_are_skipped():
    ckt = Circuit()
    with ckt:
        r = parts.resistor("1k")
        pop.set_variants(r, pop.Variant("1kΩ", "1 kΩ ±1 % 0603"), None)
    assert ident.of(r, "china") == "1 kΩ ±1 % 0603" and ident.of(r, "iran") == r.spec
    assert ident.violations(ckt) == []


def test_the_whole_board_is_identified_generically():
    assert ident.violations(board.build().circuit) == []


def test_a_crystal_with_only_a_specification_is_reported():
    ckt = Circuit()
    with ckt:
        x = parts.crystal_25mhz()
        x.mpn = None
        x.spec = "25 MHz crystal, 3.2 × 2.5 mm, CL 8 pF, ±20 ppm"
    assert any("manufacturer part number" in v for v in ident.violations(ckt))


def test_crystal_oscillator_jack_and_protection_carry_an_mpn():
    with Circuit():
        for p in (parts.crystal_25mhz(), parts.oscillator_24m576(), parts.barrel_jack(), parts.gdt(),
                  parts.ptc_resettable(), parts.usb_c_16p(), parts.relay_dpdt("released")):
            assert p.mpn and not hasattr(p, "spec"), p.value
    ckt = Circuit()
    with ckt:
        parts.led_0805()
        parts.button()
    assert ident.violations(ckt) == []


def test_explicit_dielectric_and_buyable_voltages():
    with Circuit():
        assert ident.of(parts.capacitor("10u", note="C0G is wrong here, X5R", dielectric="X5R")) == "10 µF ±10 % X5R 0805 16 V"
        assert ident.of(parts.capacitor("47u", note="x")).endswith("10 V")
        assert ident.of(parts.capacitor("10u", note="X5R", voltage="25 V")).endswith("25 V")
