from skidl import Circuit, Net

from neohw import parts
from neohw.blocks import board, controller
from neohw.checks import board as check


def test_the_whole_board_has_no_open_pins_and_every_supply_on_its_rail():
    f = check.findings(board.build().circuit)
    assert f == {"single_pin_nets": [], "open_pins": [], "rail_errors": [], "missing_footprints": []}


def test_the_rail_check_catches_a_chip_on_the_wrong_rail():
    ckt = Circuit()
    with ckt:
        s = parts.hc595()
        rf, gnd = Net("+3V3_RF"), Net("GND")
        s["VCC"] += rf               # should be +3V3_D
        s["GND"] += gnd
    errors = check.findings(ckt)["rail_errors"]
    assert any("74HC595.VCC on +3V3_RF, expected +3V3_D" in e for e in errors)


def test_the_rail_check_catches_a_ground_pin_off_ground():
    ckt = Circuit()
    with ckt:
        u = parts.lm75a()
        u["GND"] += Net("+3V3_RF")
    assert any("LM75A.GND on +3V3_RF, expected GND" in e for e in check.findings(ckt)["rail_errors"])


def test_relay_chain_drives_every_relay_once():
    b = board.build()
    coils = {}
    for p in b.circuit.parts:
        if p.value == "G6K-2F":
            coils.setdefault(check._net(p["COIL-"]), []).append(p.ref)
    assert len(coils) == 24 and all(len(v) == 1 for v in coils.values())
    assert set(controller.RELAYS) == {n.removeprefix("coil_") for n in coils}


def _part(ckt, value):
    found = [p for p in ckt.parts if p.value == value]
    assert len(found) == 1, (value, len(found))
    return found[0]


def _net(pin):
    return check._net(pin)


def _other_end(ckt, net, value_prefix="R"):
    """Nets reached from `net` through one resistor."""
    ends = set()
    for p in ckt.parts:
        if p.ref.startswith(value_prefix) and p.ref[1:].isdigit():
            a, b = _net(p[1]), _net(p[2])
            if a == net:
                ends.add(b)
            elif b == net:
                ends.add(a)
    return ends


def test_power_up_holds_every_relay_released_and_the_bias_feed_off():
    ckt = board.build().circuit
    assert "+3V3_D" in _other_end(ckt, "sr_oe")       # 74HC595 outputs disabled until the firmware drives /OE low
    assert "GND" in _other_end(ckt, "bias_en")        # SY6280 off
    assert "GND" in _other_end(ckt, "adc_rst")        # CS5361 held in reset


def test_usb_data_lines_reach_the_esp32_usb_pads_unswapped():
    ckt = board.build().circuit
    u, j = _part(ckt, "ESP32-S3-WROOM-1"), _part(ckt, "USB-C 16P")
    assert _net(u["IO20"]) == _net(j["A6"]) == _net(j["B6"])      # D+
    assert _net(u["IO19"]) == _net(j["A7"]) == _net(j["B7"])      # D−


def test_i2c_pullups_go_to_the_si5351_core_rail():
    ckt = board.build().circuit
    for line in ("i2c_sda", "i2c_scl"):
        assert _other_end(ckt, line) == {"+3V3_RF"}


def test_cs5361_mode_pins_select_double_speed_master_clock_divide_by_two():
    ckt = board.build().circuit
    adc = _part(ckt, "CS5361")
    assert _other_end(ckt, _net(adc["M0"])) == {"+3V3_D"}
    assert _other_end(ckt, _net(adc["M1"])) == {"GND"}
    assert _other_end(ckt, _net(adc["MDIV"])) == {"+3V3_D"}
    osc = _part(ckt, "24.576MHz")
    assert _net(adc["MCLK"]) in _other_end(ckt, _net(osc["OUT"]))


def test_relay_chain_bit_order_matches_the_firmware_table():
    """Bit k of the chain (74HC595 #k//8, output Q(k%8)) drives the coil RELAYS[k]."""
    ckt = board.build().circuit
    regs = {_net(p["SER"]): p for p in ckt.parts if p.value == "74HC595"}
    drivers = [p for p in ckt.parts if p.value == "ULN2803"]
    ser, k = "sr_ser", 0
    while ser in regs:
        reg = regs[ser]
        for q in "ABCDEFGH":
            link = _net(reg[f"Q{q}"])
            drv, ch = next((d, c) for d in drivers for c in range(1, 9) if _net(d[f"I{c}"]) == link)
            if k < len(controller.RELAYS):
                assert _net(drv[f"O{ch}"]) == f"coil_{controller.RELAYS[k]}", k
            k += 1
        ser = _net(reg["QHS"])
    assert k == 24


def test_the_board_checks_every_cs5361_pin_it_wires():
    adc = _part(board.build().circuit, "CS5361")
    assert tuple(getattr(adc, "nc_pins", ())) == ()


def test_an_unconnected_ground_pin_is_an_error():
    ckt = Circuit()
    with ckt:
        t = parts.lm75a()
        for pin in ("SDA", "SCL", "A0", "A1", "A2", "VCC"):
            t[pin] += Net(f"n_{pin}")                 # GND left open
    assert any("LM75A.GND on None, expected GND" in e for e in check.findings(ckt)["rail_errors"])


def test_bias_t_chokes_are_rated_for_the_hardware_current_limit():
    from neohw.blocks import bias_tee, power
    assert all("0.4 A" in note for *_, note in bias_tee.CHOKES)
    assert power.BIAS_FILTER_NOTE.endswith("rated ≥ 0.4 A")
