import numpy as np
import pytest
from skidl import Circuit, Net

from neohw import parts
from neohw.spice import Ac, Op, Port, SpiceError, Sp, node_name, simulate, to_deck
from neohw.tools import ToolNotFoundError, tool_path


def divider():
    ckt = Circuit()
    with ckt:
        a, mid, gnd = Net("RF in"), Net("mid"), Net("GND")
        r1 = parts.resistor("1k"); r1[1] += a; r1[2] += mid
        r2 = parts.resistor("2k"); r2[1] += mid; r2[2] += gnd
    divider.gnd = gnd
    return ckt, a, mid


def test_node_names():
    ckt, a, mid = divider()
    assert node_name(a) == "rf_in"
    assert node_name(divider.gnd) == "0"


def test_deck_contains_parts_ports_and_analyses():
    ckt, a, mid = divider()
    deck, raws = to_deck(ckt, "div", [Port(a, 1), Port(mid, 2)], [Op(), Ac(1e3, 1e6)])
    assert "R" in deck and " rf_in mid 1000" in deck
    assert "portnum 1 z0 50" in deck and "dc 0 ac 1 portnum 1" in deck
    assert "dc 0 ac 0 portnum 2" in deck
    assert "\nop\nwrite a0.raw all" in deck and "ac dec 20 1000 1e+06\nwrite a1.raw all" in deck
    assert raws == ["a0.raw", "a1.raw"]


def test_unconnected_pin_is_rejected():
    ckt = Circuit()
    with ckt:
        r = parts.resistor("1k")
        r[1] += Net("a")
    with pytest.raises(SpiceError, match="unconnected"):
        to_deck(ckt, "x", [], [Op()])


def test_relay_coil_pins_may_be_unconnected():
    ckt = Circuit()
    with ckt:
        k = parts.relay_dpdt("released")
        nets = [Net(pin.lower()) for pin in ["COM1", "NC1", "NO1", "COM2", "NC2", "NO2"]]
        for pin, net in zip(["COM1", "NC1", "NO1", "COM2", "NC2", "NO2"], nets):
            k[pin] += net
    deck, _ = to_deck(ckt, "k", [Port(net, i + 1) for i, net in enumerate(nets)], [Op()])
    assert "com1 nc1 0.05" in deck


@pytest.mark.integration
def test_simulated_divider_s21_matches_theory():
    try:
        tool_path("ngspice")
    except ToolNotFoundError as e:
        pytest.skip(str(e))
    ckt, a, mid = divider()
    (sp,) = simulate(ckt, "div", [Port(a, 1), Port(mid, 2)], [Sp(1e6, 2e6, points=3)])
    # 50 Ω source, 1k series, 2k || 50 Ω load
    zl = 1 / (1 / 2000 + 1 / 50)
    s21 = 2 * zl / (50 + 1000 + zl)
    np.testing.assert_allclose(np.abs(sp.data["s_2_1"]), s21, rtol=1e-3)


def test_single_pin_net_is_rejected():
    ckt, a, mid = divider()
    with ckt:
        r3 = parts.resistor("1k"); r3[1] += mid; r3[2] += Net("stub")
    with pytest.raises(SpiceError, match="stub"):
        to_deck(ckt, "x", [Port(a, 1), Port(mid, 2)], [Op()])


def test_port_net_may_have_a_single_pin():
    ckt, a, mid = divider()
    deck, _ = to_deck(ckt, "x", [Port(a, 1)], [Op()])
    assert "VPORT1 rf_in 0" in deck


def test_nets_that_map_to_the_same_node_are_rejected():
    ckt = Circuit()
    with ckt:
        r = parts.resistor("1k"); r[1] += Net("RF in"); r[2] += Net("rf_in")
        r2 = parts.resistor("1k"); r2[1] += r[1].net; r2[2] += r[2].net
    with pytest.raises(SpiceError, match="same SPICE node"):
        to_deck(ckt, "x", [], [Op()])


def test_ground_name_is_case_insensitive():
    ckt = Circuit()
    with ckt:
        assert node_name(Net("gnd")) == "0"
