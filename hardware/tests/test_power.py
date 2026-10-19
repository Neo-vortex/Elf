import pytest

from neohw import parts
from neohw.blocks import power
from neohw.checks import power as check
from neohw import population as pop
from neohw.checks.clock import net_check
from neohw.checks.controller import ADC_MAX_V
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
    tvs = by_value(blk, "SMBJ20A")[0]
    assert ptc["1"].net.name == "ext_tip" and ptc["2"].net is d["A"].net and tvs["K"].net.name == "ext_p"


def test_buck_feedback_sets_5v5_and_every_net_is_connected():
    blk = power.build()
    top, bottom = (float(v.rstrip("k")) for v in (power.BUCK_FB_TOP, power.BUCK_FB_BOTTOM))
    assert parts.LM2576_VREF * (1 + top / bottom) == pytest.approx(5.5, abs=0.02)
    assert {p.value for p in blk.circuit.parts} >= {"LM2576HV-ADJ", "TPS7A20-5.0", "TPS7A20-3.3", "AMS1117-3.3",
                                                    "SS36", "SMBJ20A"}
    assert not {p.value for p in blk.circuit.parts} & {"MT3608", "AO3401", "TLV74033", "SS34", "SMBJ30A"}
    net_check(blk)


def test_psrr_assumption_is_bounded():
    assert check.psrr_db(100.0) == 75.0 and check.psrr_db(1e6) == 15.0 and check.psrr_db(30e6) == 15.0


def test_junction_temperature_uses_each_regulators_own_input_at_its_worst_maximum_load_case():
    base = {"main": 5.46, "ldo_a_in": 5.43, "ldo_rf_in": 5.45, "v5a": 5.0, "v3rf": 3.3, "v3d": 3.3}
    hot = {**base, "main": 5.6, "ldo_rf_in": 5.6}
    dc = {"jack 9 V, maximum load": base, "jack 20 V, maximum load": hot,
          "jack 12 V, typical load": {**base, "main": 9.0, "ldo_rf_in": 9.0}}   # a typical case never sets the worst
    blk = power.build()
    p_out = 5.6 * sum(check.LOADS_MAX.values())
    for population, rf, d_reg in (("china", "TPS7A20-3.3", "AMS1117-3.3"), ("iran", "MIC5205-3.3", "AMS1117-3.3")):
        tj = check.junction_temps(dc, blk, population)
        assert set(tj) == {"TPS7A20-5.0" if population == "china" else "MIC5205-5.0", rf, d_reg, "LM2576HV"}
        rf_theta = 187.1 if population == "china" else 220.0
        d_theta = 90.0
        assert tj[rf] == pytest.approx(50 + rf_theta * (5.6 - 3.3) * check.LOADS_MAX["v3rf"])
        assert tj[d_reg] == pytest.approx(50 + d_theta * (5.6 - 3.3) * check.LOADS_MAX["v3d"])
        assert tj["LM2576HV"] == pytest.approx(50 + 50 * p_out * (1 / parts.LM2576_EFFICIENCY - 1))


def test_ext_sense_divider_keeps_20_volts_inside_the_adc_range():
    assert check.INPUT_V[1] / power.SENSE_DIVIDERS["ext"] <= ADC_MAX_V == 3.3


@pytest.mark.integration
def test_rails_hold_from_9_to_20_volts(dc):
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
@pytest.mark.parametrize("volts", [8.55, 9.0, 20.0])          # 8.55 V: a 9 V adapter at −5 %
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
    assert parts.PTC_V_MAX >= 33.0 and parts.PTC_HOLD_A >= 0.6          # above the TVS clamp and the 20 V in; above the 0.45 A input


def test_buck_output_capacitor_meets_the_lm2576_stability_rule():
    # C_OUT ≥ 13 300 · V_IN(max) / (V_OUT · L[µH]) (LM2576 adjustable, datasheet): 484 µF at 20 V in
    need_uf = 13300 * check.INPUT_V[1] / (5.5 * power.BUCK_L * 1e6)
    assert need_uf == pytest.approx(483.6, abs=0.1)
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


def test_rf_ldos_share_pads_and_swap_by_population():
    blk = power.build()
    rf = [p for p in blk.circuit.parts if pop.variant(p, "china") and "TPS7A20" in pop.variant(p, "china").ident]
    assert len(rf) == 2 and all(p.footprint.endswith("SOT-23-5") for p in rf)
    assert all("MIC5205" in pop.variant(p, "iran").ident for p in rf)
    assert {pop.variant(p, "china").ident for p in rf} == {"TPS7A2050PDBVR", "TPS7A2033PDBVR"}
    byp = [p for p in blk.circuit.parts if p.value == "470pF"]
    assert len(byp) == 2 and all(pop.fitted(c, "iran") and not pop.fitted(c, "china") for c in byp)


def test_3v3_d_is_ams1117_in_both_populations_and_no_tlv75733_exists():
    blk = power.build()
    ams = by_value(blk, "AMS1117-3.3")
    assert len(ams) == 1 and all(pop.fitted(ams[0], p) for p in pop.POPULATIONS)
    assert "LM1117" in ams[0].bom_note and not by_value(blk, "TLV75733")
    tant = [p for p in blk.circuit.parts if p.value == "22uF" and "tantalum" in p.spec]
    assert len(tant) == 1 and all(pop.fitted(tant[0], p) for p in pop.POPULATIONS)


@pytest.mark.parametrize("population", ["china", "iran"])
def test_every_rail_in_regulation_and_every_junction_cool_enough(ngspice, population):
    blk = power.build()
    dc = check.dc_scenarios(blk, population)
    for name, s in dc.items():
        if "reversed" in name:
            continue
        for rail, v_min in check.RAIL_MIN_V.items():
            assert s[rail] >= v_min, (population, name, rail, s[rail])
    assert max(check.junction_temps(dc, blk, population).values()) <= check.TJ_MAX_C


def test_ldo_pinouts_match_the_datasheets():
    # TPS7A20 SBVS338H Figure 4-4 / Pin Functions: DBV 1 IN 2 GND 3 EN 4 NC 5 OUT
    from skidl import Circuit
    with Circuit():
        for p in (parts.tps7a20(5.0), parts.tps7a20(3.3)):
            assert {pin.name: pin.num for pin in p.pins} == {"IN": "1", "GND": "2", "EN": "3", "NC": "4", "OUT": "5"}
            assert p.nc_pins == ("NC",) and p.footprint.endswith("SOT-23-5")
        assert parts.tps7a20(5.0).mpn == "TPS7A2050PDBVR" and parts.tps7a20(3.3).mpn == "TPS7A2033PDBVR"


def test_ldo_thermal_and_psrr_numbers_come_from_the_datasheets():
    from skidl import Circuit
    with Circuit():
        tps = parts.tps7a20(5.0)
        ams, mic = parts.ams1117_33(), parts.mic5205(5.0)
    assert (tps.theta_ja, ams.theta_ja, mic.theta_ja) == (187.1, 90.0, 220.0)
    assert (tps.dropout_v, ams.dropout_v, mic.dropout_v) == (0.145, 1.3, 0.165)
    for f, db in ((1e3, 92), (1e5, 60), (1e6, 40)):                    # TPS7A20 Table 5.5, 300 mA
        assert tps.psrr_db(f) == pytest.approx(db)
    assert tps.psrr_db(10e3) == pytest.approx(76, abs=1)               # the table's 75 dB at 10 kHz


def test_input_side_capacitors_are_25_volts_or_more_and_rail_capacitors_10_volts_or_more():
    import re
    blk = power.build()
    volts = lambda c: float(re.search(r"(\d+(?:\.\d+)?) V$", c.spec).group(1))
    side = {"ext_p": 25, "buck_in": 25}
    for c in (p for p in blk.circuit.parts if p.ref.startswith("C")):
        nets = {pin.net.name for pin in c.pins}
        need = 25 if nets & {"ext_p", "buck_in", "ext_tip", "ext_fused"} else 10
        assert volts(c) >= need, (c.ref, c.spec, nets)
