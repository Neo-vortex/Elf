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


@pytest.fixture(scope="module")
def dc():
    try:
        tool_path("ngspice")
    except ToolNotFoundError as e:
        pytest.skip(str(e))
    return check.dc_scenarios(power.build())


def test_usb_powers_nothing():
    blk = power.build()
    vbus = blk.ports["vbus"]
    on_vbus = {p.value for p in blk.circuit.parts for pin in p.pins if pin.net is not None and pin.net.name == vbus.name}
    assert on_vbus == {"1MΩ"}                        # only the sense divider (the USB-C pads are in the controller)


def test_input_chain_is_ptc_diode_then_tvs():
    blk = power.build()
    ptc = next(p for p in blk.circuit.parts if p.value.startswith("PTC"))
    d = next(p for p in by_value(blk, "SS36") if p["K"].net.name == "ext_p")
    tvs = by_value(blk, "SMBJ30A")[0]
    assert ptc["1"].net.name == "ext_tip" and ptc["2"].net is d["A"].net and tvs["K"].net.name == "ext_p"


def test_buck_feedback_sets_5v5_and_every_net_is_connected():
    blk = power.build()
    top, bottom = (float(v.rstrip("k")) for v in (power.BUCK_FB_TOP, power.BUCK_FB_BOTTOM))
    assert parts.LM2576_VREF * (1 + top / bottom) == pytest.approx(5.5, abs=0.02)
    assert {p.value for p in blk.circuit.parts} >= {"LM2576HV-ADJ", "MIC5205-5.0", "MIC5205-3.3", "AMS1117-3.3", "SS36", "SMBJ30A"}
    assert not {p.value for p in blk.circuit.parts} & {"MT3608", "AO3401", "TLV74033", "SS34"}
    net_check(blk)


def test_psrr_assumption_is_bounded():
    assert check.psrr_db(100.0) == 75.0 and check.psrr_db(1e6) == 15.0 and check.psrr_db(30e6) == 15.0


def test_junction_temperature_uses_each_regulators_own_input_at_its_worst_maximum_load_case():
    base = {"main": 5.46, "ldo_a_in": 5.43, "ldo_rf_in": 5.45, "v5a": 5.0, "v3rf": 3.3, "v3d": 3.3}
    hot = {**base, "main": 5.6, "ldo_rf_in": 5.6}
    dc = {"jack 9 V, maximum load": base, "jack 30 V, maximum load": hot,
          "jack 12 V, typical load": {**base, "main": 9.0, "ldo_rf_in": 9.0}}   # a typical case never sets the worst
    tj = check.junction_temps(dc)
    assert tj["AMS1117-3.3"] == pytest.approx(50 + 90 * (5.6 - 3.3) * check.LOADS_MAX["v3d"])
    assert tj["MIC5205-3.3"] == pytest.approx(50 + 220 * (5.6 - 3.3) * check.LOADS_MAX["v3rf"])
    p_out = 5.6 * sum(check.LOADS_MAX.values())
    assert tj["LM2576HV"] == pytest.approx(50 + 50 * p_out * (1 / parts.LM2576_EFFICIENCY - 1))


def test_ext_sense_divider_keeps_30_volts_inside_the_adc_range():
    assert 30.0 / power.SENSE_DIVIDERS["ext"] <= 3.1


@pytest.mark.integration
def test_rails_hold_from_9_to_30_volts(dc):
    for name, s in dc.items():
        if "reversed" in name:
            continue
        assert check.MAIN_V[0] <= s["main"] <= check.MAIN_V[1], name
        assert s["v5a"] >= 4.95 and s["v3rf"] >= 3.25 and s["v3d"] >= 3.25, name
        assert abs(s["i_usb"]) < 1e-4, name                    # only the 200 kΩ divider: 25 µA


@pytest.mark.integration
def test_a_reversed_plug_draws_nothing(dc):
    s = dc["jack reversed (−12 V)"]
    assert abs(s["i_ext"]) < 1e-3 and s["main"] > -0.3


@pytest.mark.integration
def test_surge_clamp_stays_under_the_buck_rating(ngspice):
    assert check.surge_clamp(power.build()) <= check.BUCK_VIN_MAX_V


@pytest.mark.integration
@pytest.mark.parametrize("volts", [8.55, 9.0, 30.0])          # 8.55 V: a 9 V adapter at −5 %
def test_hot_plug_starts_the_board_and_stays_in_regulation(ngspice, volts):
    r = check.plug_in(power.build(), volts)
    assert r["rails_up_s"] < 20e-3
    assert check.MAIN_V[0] <= r["main_end_v"] <= check.MAIN_V[1]          # still in regulation at the end
    assert r["buck_in_peak_v"] <= check.BUCK_VIN_MAX_V
    assert r["peak_a"] <= parts.SS36_IFSM_A / 2                            # the reverse diode's surge rating


@pytest.mark.integration
def test_conducted_spurs_are_far_below_mds(ngspice):
    spurs = check.ripple_spurs(power.build())
    assert set(spurs) == set(check.BANDS)
    assert max(s["total_dbm"] for s in spurs.values()) <= check.MDS_DBM - check.SPUR_MARGIN_DB


def test_input_is_protected_by_a_resettable_fuse_not_an_inrush_sensitive_one():
    blk = power.build()
    values = {p.value for p in blk.circuit.parts}
    assert any(v.startswith("PTC") for v in values) and not any(v.startswith("NTC") or "fuse" in v for v in values)
    assert parts.PTC_V_MAX >= 33.0 and parts.PTC_HOLD_A >= 0.6          # above 30 V in; above the 0.45 A input


def test_buck_output_capacitor_meets_the_lm2576_stability_rule():
    # C_OUT ≥ 13 300 · V_IN(max) / (V_OUT · L[µH]) (LM2576 adjustable, datasheet)
    need_uf = 13300 * 30.0 / (5.5 * power.BUCK_L * 1e6)
    assert power.BUCK_C_OUT_UF >= need_uf


def test_buck_inductor_is_rated_for_start_up_current():
    blk = power.build()
    lb = next(p for p in blk.circuit.parts if p.ref.startswith("L") and "buck inductor" in p.bom_note)
    assert "Isat ≥ 2 A" in lb.bom_note and power.BUCK_L_DCR >= 0.15


def test_usb_cannot_back_power_an_unpowered_board():
    """With the jack out, VBUS (and CC) must not drive more than a few µA into an ESP32 pin."""
    top, bottom = power.VBUS_DIVIDER
    assert top * bottom / (top + bottom) >= 150e3          # Thevenin source behind the pin ≥ 150 kΩ
    from neohw.blocks import controller
    assert "cc1" not in controller.GPIO and "cc2" not in controller.GPIO
