from pathlib import Path

import pytest

from neosim.scenario import load_scenario
from neosim.tools import ToolNotFoundError, tool_path
from neosim.twin import run_scenario

SCENARIOS = Path(__file__).resolve().parents[1] / "scenarios"


@pytest.mark.integration
def test_20m_ideal_scenario_passes_end_to_end(tmp_path):
    try:
        for name in ("gen_ft8", "decode_ft8", "neoradio-dsp"):
            tool_path(name)
    except ToolNotFoundError as e:
        pytest.skip(str(e))
    result = run_scenario(load_scenario(SCENARIOS / "20m_ft8_cw_ideal.yaml"), tmp_path)
    details = {r.id: (r.passed, r.output, r.detail) for r in result.receives}
    assert result.passed, details
    assert [e.station_id for e in result.excluded] == ["bcast"]
    assert result.adc.clipped_samples == 0
    assert result.psd_freqs_hz.shape == result.psd_db.shape


@pytest.mark.integration
@pytest.mark.parametrize("name, band, folded_kt0", [("20m_ft8_cw_hardware.yaml", 7, 0.0),
                                                     ("6m_cw_hardware.yaml", 11, 0.1)])
def test_hardware_scenarios_decode_end_to_end(tmp_path, name, band, folded_kt0):
    try:
        for tool in ("gen_ft8", "decode_ft8", "neoradio-dsp"):
            tool_path(tool)
    except ToolNotFoundError as e:
        pytest.skip(str(e))
    result = run_scenario(load_scenario(SCENARIOS / name), tmp_path)
    details = {r.id: (r.passed, r.output, r.detail) for r in result.receives}
    assert result.passed, details
    assert result.hardware.filter_index == band and result.adc.clipped_samples == 0
    assert result.hardware.noise["harmonic_noise_w_per_hz"] / (1.380649e-23 * 290) >= folded_kt0
