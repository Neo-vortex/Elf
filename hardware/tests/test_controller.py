import pytest

from neohw.blocks import controller
from neohw.checks import controller as ctl_check
from neohw.tools import ToolNotFoundError, tool_path


def test_gpio_map_follows_the_esp32_s3_rules():
    assert ctl_check.gpio_findings() == []


def test_gpio_rules_flag_a_psram_pin(monkeypatch):
    monkeypatch.setitem(controller.GPIO, "status_led", "IO35")
    assert any("octal PSRAM" in x for x in ctl_check.gpio_findings())


@pytest.mark.integration
@pytest.mark.parametrize("pop", ["china", "iran"])
def test_bias_sense_is_monotonic_stable_and_within_the_adc_range(pop):
    try:
        tool_path("ngspice")
    except ToolNotFoundError as e:
        pytest.skip(str(e))
    bs = ctl_check.bias_sense(pop)
    for tab in bs["table"].values():
        assert all(b[1] > a[1] for a, b in zip(tab, tab[1:]))
    assert all(0.3 <= s["i_a"] <= 0.5 and s["sense_v"] <= ctl_check.ADC_MAX_V for s in bs["short"].values())


def test_the_adc_limit_is_the_stm32f411_range():
    assert ctl_check.ADC_MAX_V == 3.3


def test_both_bias_sense_circuits_are_on_the_board_one_per_population():
    from neohw import population as pop
    from neohw.blocks import bias_switch
    blk = bias_switch.build()
    ina = [p for p in blk.circuit.parts if p.value == "INA180A1"]
    mirror = [p for p in blk.circuit.parts if p.value == "BC857B"]
    assert len(ina) == 1 and len(mirror) == 2
    assert pop.fitted(ina[0], "china") and not pop.fitted(ina[0], "iran")
    assert all(pop.fitted(q, "iran") and not pop.fitted(q, "china") for q in mirror)


def test_ina180_pinout_and_identification():
    from neohw import parts
    p = parts.ina180a1()
    assert {n: p[n].num for n in ("OUT", "GND", "IN+", "IN-", "VS")} == {
        "OUT": "1", "GND": "2", "IN+": "3", "IN-": "4", "VS": "5"}
    assert p.mpn == "INA180A1IDBVR"


@pytest.mark.integration
def test_ina180_sense_reads_3v_at_the_cutoff_and_never_exceeds_the_adc_range():
    try:
        tool_path("ngspice")
    except ToolNotFoundError as e:
        pytest.skip(str(e))
    bs = ctl_check.bias_sense("china")
    tab = bs["table"][27.0]
    near150 = min(tab, key=lambda iv: abs(iv[0] - 0.15))
    assert 2.85 <= near150[1] <= 3.15
    assert max(v for t in bs["table"].values() for _, v in t) <= ctl_check.ADC_MAX_V
    assert tab[0][1] < 0.05                         # zero reads ≈ 0 V: the firmware tells it from the mirror


@pytest.mark.integration
def test_the_two_zero_readings_are_split_by_the_threshold():
    try:
        tool_path("ngspice")
    except ToolNotFoundError as e:
        pytest.skip(str(e))
    z = {p: ctl_check.bias_sense(p)["table"][27.0][0][1] for p in ("china", "iran")}
    assert z["china"] < ctl_check.ZERO_SPLIT_V < z["iran"]


def test_relay_coil_voltage_is_reported_for_each_supply_corner():
    r = ctl_check.relay_coil_margin({"low": 4.30, "high": 5.46})
    assert r["low"]["coil_v"] == pytest.approx(4.30 - ctl_check.ULN2803_VCE_SAT_V)
    assert r["low"]["need_v"] == pytest.approx(ctl_check.RELAY_MUST_OPERATE_V * ctl_check.coil_heat_factor(50.0))
    assert r["low"]["margin_v"] < 0 < r["high"]["margin_v"]


def test_relays_pull_in_at_every_supply_corner_with_the_5v5_rail():
    assert min(m["margin_v"] for m in ctl_check.relay_coil_margin(ctl_check._supply_corners()).values()) > 0


def test_a_pull_up_marks_a_board_with_no_sense_circuit_fitted():
    from neohw import population as pop
    from neohw.blocks import bias_switch
    blk = bias_switch.build()
    pull = [p for p in blk.circuit.parts if getattr(p, "spec", "").startswith("1 MΩ")]
    assert len(pull) == 1
    assert all(pop.fitted(pull[0], x) for x in pop.POPULATIONS)
    nets = {pin.net.name for pin in pull[0].pins}
    assert nets == {"bias_isense_raw", "+3V3_D"}
