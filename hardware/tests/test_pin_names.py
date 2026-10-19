"""SKiDL resolves part["2"] as pad number 2 before pin name "2", so a pin whose name is a digit must
sit on the pad with that number (the bug that shorted the crystal's XB to ground)."""
from skidl import Circuit

from neohw import parts

CONSTRUCTORS = [
    lambda: parts.resistor("1k"), lambda: parts.capacitor("1n"), lambda: parts.inductor(1e-6, 30, 1e6),
    parts.si5351c, parts.crystal_25mhz, parts.lvc74, parts.lvc157, parts.ss36, parts.smbj20a, lambda: parts.tps7a20(5.0), parts.tlv75733, parts.lm2576hv, parts.ptc_resettable,
    parts.barrel_jack, lambda: parts.mic5205(5.0), parts.ams1117_33,
    parts.esp32s3_wroom1, parts.usb_c_16p, parts.usblc6, parts.lm75a, parts.hc595, parts.uln2803,
    lambda: parts.sy6280(17.4e3), parts.oscillator_24m576, parts.led_0805, parts.button, parts.fst3253,
    parts.lm4562_dual, parts.cs5361, parts.pnp_bc857b, parts.npn_bfr92a, parts.diode_pair_bav99, parts.gdt,
    lambda: parts.relay_dpdt("released"),
]


def test_digit_pin_names_sit_on_the_pad_with_that_number():
    with Circuit():
        for make in CONSTRUCTORS:
            p = make()
            for pin in p.pins:
                if pin.name.isdigit():
                    assert pin.num == pin.name, (p.value, pin.name, pin.num)
