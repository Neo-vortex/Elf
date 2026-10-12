import pytest

from neohw import availability, parts
from neohw.blocks import cal_tone, clock
from neohw.checks import clock as check
from neohw.tools import ToolNotFoundError, tool_path


def pin_net(part, name):
    return part[name].net.name


def by_value(blk, value):
    return [p for p in blk.circuit.parts if p.value == value]


def test_si5351c_gm1_is_the_16_pin_four_output_part():
    u = clock.build().circuit.parts
    si = next(p for p in u if p.value == "Si5351C")
    assert si.footprint.startswith("Package_DFN_QFN:QFN-16")
    assert {p.num for p in si.pins} == {str(k) for k in range(1, 18)}      # 16 pins + exposed pad (17)
    assert si.mpn == "Si5351C-B-GM1"


def test_johnson_counter_gives_quadrature_from_clk2():
    blk = clock.build()
    ff = by_value(blk, "74LVC74")[0]
    si = by_value(blk, "Si5351C")[0]
    assert pin_net(ff, "1CP") == pin_net(ff, "2CP") == pin_net(si, "CLK2")
    assert pin_net(ff, "2D") == pin_net(ff, "1Q")          # FF2 follows FF1
    assert pin_net(ff, "1D") == pin_net(ff, "/2Q")         # FF1 takes FF2's complement: ÷4 Johnson
    for p in ("/1RD", "/1SD", "/2RD", "/2SD"):
        assert pin_net(ff, p) == "+3V3_RF"


def test_path_mux_selects_direct_on_low_and_div4_on_high():
    blk = clock.build()
    mux, ff, si = by_value(blk, "74LVC157")[0], by_value(blk, "74LVC74")[0], by_value(blk, "Si5351C")[0]
    assert pin_net(mux, "S") == blk.ports["lo_path"].name and pin_net(mux, "/E") == "GND"
    assert pin_net(mux, "1I0") == pin_net(si, "CLK0") and pin_net(mux, "2I0") == pin_net(si, "CLK1")
    assert pin_net(mux, "1I1") == pin_net(ff, "1Q") and pin_net(mux, "2I1") == pin_net(ff, "2Q")
    for unused in ("3I0", "3I1", "4I0", "4I1"):
        assert pin_net(mux, unused) == "GND"


def test_every_supply_pin_is_on_3v3_rf_and_the_block_has_no_dangling_net():
    blk = clock.build()
    for p in blk.circuit.parts:
        for pin in p.pins:
            if pin.name.startswith("VDD") or pin.name == "VCC":
                assert pin.net.name == "+3V3_RF", (p.ref, pin.name)
    check.net_check(blk)          # raises on a single-pin net


def test_skew_budget_from_the_datasheets():
    # tsk(o) ≤ 1.0 ns for 74LVC74A and 74LVC157A (Nexperia, VCC 3.0–3.6 V): ±0.5 ns per stage
    assert clock.SKEW_STAGES_S["div4"] == pytest.approx((0.5e-9, 0.5e-9))
    assert clock.SKEW_STAGES_S["direct"] == pytest.approx((0.2e-9, 0.5e-9))


def test_cal_pad_values_and_ports():
    blk = cal_tone.build()
    assert sorted(p.value for p in blk.circuit.parts) == sorted(["100Ω", "51Ω", "22kΩ"])
    assert set(blk.ports) >= {"clk", "rf"}


def test_availability_reads_both_surveys():
    data = availability.load()
    assert "SI5351C" in data and "74LVC157" in data and "SI5351A" in data


@pytest.mark.integration
def test_cal_pad_attenuation_and_loading():
    try:
        tool_path("ngspice")
    except ToolNotFoundError as e:
        pytest.skip(str(e))
    r = check.cal_pad_response()
    assert r["att_db"][1e6] == pytest.approx(-70.0, abs=3.0)
    assert r["rf_insertion_loss_db"] <= 0.05
