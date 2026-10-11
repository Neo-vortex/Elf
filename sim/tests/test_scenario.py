import pytest

from neosim.scenario import ScenarioError, parse_scenario


def base():
    return {
        "name": "t",
        "lo_hz": 14_060_000,
        "stations": [
            {"id": "a", "mode": "ft8", "dial_hz": 14_074_000, "audio_hz": 1500, "snr_db": -10, "msg": "CQ EP2ABC LL35"},
            {"id": "b", "mode": "cw", "rf_hz": 14_045_600, "snr_db": 3, "text": "CQ TEST", "wpm": 20},
            {"id": "c", "mode": "carrier", "rf_hz": 15_400_000, "snr_db": 60},
        ],
        "receive": [
            {"id": "r1", "mode": "usb", "dial_hz": 14_074_000, "decoder": "ft8", "expect": {"contains": ["CQ EP2ABC LL35"]}},
            {"id": "r2", "mode": "cw", "dial_hz": 14_045_000, "decoder": "cw", "expect": {"text": "CQ TEST"}},
        ],
    }


def test_valid_scenario_with_defaults():
    s = parse_scenario(base())
    assert s.duration_s == 15.0 and s.seed == 0 and s.noise_dbfs == -100.0
    ft8 = s.stations[0]
    assert ft8.rf_hz == 14_075_500 and ft8.audio_hz == 1500
    assert s.stations[1].bandwidth_hz == 100.0
    assert s.receive[1].expect == {"text": "CQ TEST", "max_cer": 0.05}


def test_out_of_window_station_is_allowed():
    # stations outside the window are excluded at composition time, not rejected here
    assert parse_scenario(base()).stations[2].rf_hz == 15_400_000


@pytest.mark.parametrize(
    "mutate, message",
    [
        (lambda d: d["stations"].append(dict(d["stations"][0])), "duplicate station id"),
        (lambda d: d["stations"][0].pop("msg"), "needs 'msg'"),
        (lambda d: d["stations"][1].update(text="CQ #"), "no Morse code"),
        (lambda d: d["stations"][0].update(mode="psk31"), "unknown station mode"),
        (lambda d: d["receive"][0].update(dial_hz=14_200_000), "outside the receiver window"),
        (lambda d: d["receive"][0].update(decoder="rtty"), "unknown decoder"),
        (lambda d: d.update(noise={"type": "itu_p372_rural"}), "only awgn"),
        (lambda d: d.update(duration_s=0), "duration_s"),
        (lambda d: d["receive"][0].update(expect={}), "expect.contains"),
        (lambda d: d["stations"][0].update(audio_hz=5000), "audio_hz"),
    ],
)
def test_invalid_scenarios_raise(mutate, message):
    d = base()
    mutate(d)
    with pytest.raises(ScenarioError, match=message):
        parse_scenario(d)


@pytest.mark.parametrize(
    "text, message",
    [
        ("", "must be a mapping"),
        ("- a\n- b\n", "must be a mapping"),
        ("name: [unclosed\n", "invalid YAML"),
    ],
)
def test_load_scenario_rejects_bad_documents(tmp_path, text, message):
    from neosim.scenario import load_scenario
    f = tmp_path / "s.yaml"
    f.write_text(text)
    with pytest.raises(ScenarioError, match=message):
        load_scenario(f)


def test_non_numeric_value_is_scenario_error():
    d = base()
    d["stations"][0]["snr_db"] = "abc"
    with pytest.raises(ScenarioError, match="snr_db"):
        parse_scenario(d)


def test_station_entry_must_be_mapping():
    d = base()
    d["stations"].append("oops")
    with pytest.raises(ScenarioError, match="mapping"):
        parse_scenario(d)
