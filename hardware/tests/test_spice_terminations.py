import math

from skidl import Circuit, Net

from neohw import parts
from neohw.spice import Load, Noise, Op, Port, Source, Supply, Tran, to_deck


def amp_like():
    ckt = Circuit()
    with ckt:
        a, b, vcc, gnd = Net("in"), Net("out"), Net("+5V"), Net("GND")
        r1 = parts.resistor("1k"); r1[1] += a; r1[2] += b
        r2 = parts.resistor("1k"); r2[1] += vcc; r2[2] += b
        r3 = parts.resistor("1k"); r3[1] += b; r3[2] += gnd
    return ckt, a, b, vcc


def test_source_has_noisy_series_resistor_and_ac_drive():
    ckt, a, b, vcc = amp_like()
    src = Source(a, "rf")
    assert src.lines() == ["VS_rf0 s_rf0 0 dc 0 ac 1", "RS_rf s_rf0 in 50"]
    assert src.vname == "VS_rf0"


def test_two_tone_source_stacks_sines_at_available_power():
    src = Source(Net("x"), "rf", tones=((14e6, -10.0), (14.1e6, -10.0)))
    lines = src.lines()
    amp = 2 * math.sqrt(2 * 50 * 1e-3 * 0.1)     # open-circuit peak for -10 dBm available
    assert lines[0] == f"VS_rf0 s_rf0 0 dc 0 ac 1 sin(0 {amp:.9g} 14000000)"
    assert lines[1] == f"VS_rf1 s_rf1 s_rf0 dc 0 sin(0 {amp:.9g} 14100000)"
    assert lines[2] == "RS_rf s_rf1 x 50"


def test_load_supply_and_analyses_render():
    ckt, a, b, vcc = amp_like()
    src = Source(a, "rf")
    deck, raws = to_deck(ckt, "t", [src, Load(b), Supply(vcc, 5.0)],
                         [Op(), Noise(b, src, 1e6, 54e6, 10), Tran(1e-9, 2e-6, 1e-6)])
    assert "RL_out out 0 50" in deck and "VSUP_vcc _5v 0 dc 5" in deck
    assert "noise v(out) VS_rf0 dec 10 1e+06 5.4e+07\nsetplot noise1\nwrite a1.raw all" in deck
    assert "tran 1e-09 2e-06 1e-06 1e-09\nlinearize\nwrite a2.raw all" in deck
    assert raws == ["a0.raw", "a1.raw", "a2.raw"]


def test_terminated_nets_count_as_connected():
    ckt, a, b, vcc = amp_like()
    to_deck(ckt, "t", [Source(a), Load(b), Supply(vcc, 5.0)], [Op()])   # no dangling-net error
