"""The committed characterisation files carry what the twin (neosim/hardware.py) reads."""
import json
from pathlib import Path

CHAR = Path(__file__).resolve().parents[1] / "characterized"


def load(name):
    return json.loads((CHAR / name).read_text())


def test_mixer_json_has_the_plan_2_5_fields():
    m = load("mixer.json")
    for key in ("nf_db", "nf_definition", "ron_ohms", "noise_v2_per_hz", "adc_fs_peak_v", "clip_peak_v",
                "adc_noise", "bypass_correction"):
        assert key in m, key
    assert sorted(m["bypass_correction"], key=int) == [str(b) for b in range(1, 12)]
    for c in m["bypass_correction"].values():
        assert len(c["offset_hz"]) == len(c["re"]) == len(c["im"]) == 14
        assert -3.0 < c["mean_db"] < 0.5
    assert m["clip_peak_v"] < m["adc_fs_peak_v"]


def test_preamp_grid_reaches_5x_the_top_band():
    p = load("preamp.json")
    assert p["freq_hz"][0] <= 0.1e6 * 1.001 and p["freq_hz"][-1] >= 270e6
    assert len(p["freq_hz"]) == len(p["gain_db"]) == len(p["nf_db"])
