import pytest

from neohw import parts
from neohw.blocks import power
from neohw.checks import power as check
from neohw.checks.clock import net_check
from neohw.tools import ToolNotFoundError, tool_path


def by_value(blk, value):
    return [p for p in blk.circuit.parts if p.value == value]


@pytest.fixture
def ngspice():
    try:
        tool_path("ngspice")
    except ToolNotFoundError as e:
        pytest.skip(str(e))


def test_usb_path_is_two_back_to_back_mosfets_gated_by_the_external_supply():
    blk = power.build()
    q = by_value(blk, "AO3401")
    usb = [p for p in q if p["G"].net.name == "ext_p"]
    assert len(usb) == 2 and usb[0]["S"].net is usb[1]["S"].net        # common source: body diodes oppose
    assert {p["D"].net.name for p in usb} == {"vbus_f", "+5V_MAIN"}


def test_external_input_is_blocked_before_the_tvs_when_reversed():
    blk = power.build()
    q2 = next(p for p in by_value(blk, "AO3401") if p["G"].net.name == "GND")
    tvs = by_value(blk, "SMBJ5.0A")[0]
    assert q2["D"].net.name == "ext_raw" and q2["S"].net.name == "ext_p" and tvs["K"].net.name == "ext_p"


def test_boost_feedback_sets_7_volts_and_every_net_is_connected():
    blk = power.build()
    top, bottom = (float(v.rstrip("k")) for v in (power.BOOST_FB_TOP, power.BOOST_FB_BOTTOM))
    assert 0.6 * (1 + top / bottom) == pytest.approx(7.02, abs=0.01)
    assert {p.value for p in blk.circuit.parts} >= {"MT3608", "MIC5205-5.0", "MIC5205-3.3", "TLV74033", "SS34"}
    net_check(blk)


def test_psrr_assumption_is_bounded():
    assert check.psrr_db(100.0) == 75.0 and check.psrr_db(1e6) == 15.0 and check.psrr_db(30e6) == 15.0


@pytest.mark.integration
def test_switch_over_rails_and_back_feed(ngspice):
    dc = check.dc_scenarios(power.build())
    assert dc["usb 5.25 + ext 4.75 V, typical"]["i_usb"] < 1e-3               # the external supply wins
    assert abs(dc["usb 4.4 + ext 5.25 V, typical"]["i_usb"]) < 1e-3            # and never back-feeds the host
    assert abs(dc["usb 5.0 V + ext reversed"]["i_ext"]) < 1e-3                 # reversed plug: nothing flows
    assert dc["usb 4.75 V, typical load"]["i_usb"] == pytest.approx(0.426, abs=0.01)
    for s in dc.values():
        assert s["v5a"] >= 4.95 and s["v3rf"] >= 3.25 and s["v3d"] >= 3.135


@pytest.mark.integration
def test_conducted_spurs_are_far_below_mds(ngspice):
    spurs = check.ripple_spurs(power.build())
    assert set(spurs) == set(check.BANDS)
    assert max(s["total_dbm"] for s in spurs.values()) <= check.MDS_DBM - check.SPUR_MARGIN_DB


def test_3v3_d_uses_a_low_dropout_regulator():
    values = {p.value for p in power.build().circuit.parts}
    assert "TLV74033" in values and "AMS1117-3.3" not in values


def test_junction_temperature_uses_each_regulators_own_input_at_its_worst_maximum_load_case():
    base = {"main": 4.6, "v7": 7.0, "ldo_a_in": 6.9, "ldo_rf_in": 4.5, "v5a": 5.0, "v3rf": 3.3, "v3d": 3.3}
    hot = {**base, "main": 5.1, "ldo_rf_in": 5.0}
    dc = {"usb 4.75 V, maximum load": base, "usb 5.25 V, maximum load": hot, "usb 5.0 + ext 5.0 V, typical": {
        **base, "main": 9.0, "ldo_rf_in": 9.0}}          # a typical-load case never sets the thermal worst case
    tj = check.junction_temps(dc)
    assert tj["TLV74033"] == pytest.approx(50 + check.THETA_JA["TLV74033"] * (5.1 - 3.3) * check.LOADS_MAX["v3d"])
    assert tj["MIC5205-3.3"] == pytest.approx(50 + 220 * (5.0 - 3.3) * check.LOADS_MAX["v3rf"])
    assert tj["MIC5205-5.0"] == pytest.approx(50 + 220 * (6.9 - 5.0) * check.LOADS_MAX["v5a"])


@pytest.mark.integration
def test_usb_path_stays_on_with_hot_schottky_leakage(ngspice):
    s = check.dc_scenarios(power.build())["usb 4.75 V, maximum load, SS34 leaking 2 mA"]
    assert s["main"] >= 4.4 and s["v3d"] >= 3.25


@pytest.mark.integration
def test_usb_plug_in_brings_every_rail_up_and_reports_the_inrush(ngspice):
    r = check.usb_inrush(power.build())
    assert r["rails_up_s"] < 10e-3 and 1.0 < r["peak_a"] < 10.0
    assert r["i2t_a2s"] <= parts.FUSE_1206_SLOW_I2T_MIN / check.FUSE_PULSE_MARGIN


@pytest.mark.integration
def test_unplugging_the_jack_hands_over_to_usb_without_a_brown_out(ngspice):
    assert check.ext_unplug(power.build())["v3d_min"] >= check.HANDOVER_MIN_V3D
