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
