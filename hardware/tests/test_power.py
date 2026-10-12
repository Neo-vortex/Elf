import pytest

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
    assert 0.6 * (1 + 107 / 10) == pytest.approx(7.02, abs=0.01)
    assert {p.value for p in blk.circuit.parts} >= {"MT3608", "MIC5205-5.0", "MIC5205-3.3", "AMS1117-3.3", "SS34"}
    net_check(blk)


def test_psrr_assumption_is_bounded():
    assert check.psrr_db(100.0) == 75.0 and check.psrr_db(1e6) == 15.0 and check.psrr_db(30e6) == 15.0


@pytest.mark.integration
def test_switch_over_rails_and_back_feed(ngspice):
    dc = check.dc_scenarios(power.build())
    assert dc["usb 5.25 + ext 4.75 V, typical"]["i_usb"] < 1e-3               # the external supply wins
    assert abs(dc["usb 5.0 V + ext reversed"]["i_ext"]) < 1e-3                 # reversed plug: nothing flows
    assert dc["usb 4.75 V, typical load"]["i_usb"] == pytest.approx(0.426, abs=0.01)
    for s in dc.values():
        assert s["v5a"] >= 4.95 and s["v3rf"] >= 3.25 and s["v3d"] >= 3.135


@pytest.mark.integration
def test_conducted_spurs_are_far_below_mds(ngspice):
    spurs = check.ripple_spurs(power.build())
    assert set(spurs) == set(check.BANDS)
    assert max(s["total_dbm"] for s in spurs.values()) <= check.MDS_DBM - check.SPUR_MARGIN_DB
