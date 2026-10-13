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
def test_bias_sense_is_monotonic_stable_and_within_the_adc_range():
    try:
        tool_path("ngspice")
    except ToolNotFoundError as e:
        pytest.skip(str(e))
    bs = ctl_check.bias_sense()
    for tab in bs["table"].values():
        assert all(b[1] > a[1] for a, b in zip(tab, tab[1:]))
    assert all(0.3 <= s["i_a"] <= 0.5 and s["sense_v"] <= 3.1 for s in bs["short"].values())


def test_relay_coil_voltage_is_reported_for_each_supply_corner():
    r = ctl_check.relay_coil_margin({"usb 4.75 V": 4.62, "ext 4.75 V": 4.30})
    assert r["ext 4.75 V"]["coil_v"] == pytest.approx(4.30 - ctl_check.ULN2803_VCE_SAT_V)
    assert r["ext 4.75 V"]["need_v"] == pytest.approx(ctl_check.RELAY_MUST_OPERATE_V * ctl_check.coil_heat_factor(50.0))
    assert r["usb 4.75 V"]["margin_v"] > r["ext 4.75 V"]["margin_v"]
