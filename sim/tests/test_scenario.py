import math

import pytest

from neosim.scenario import Receiver, ScenarioError, parse_scenario


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


def absolute(model="hardware", **receiver):
    d = base()
    d["noise"] = {"type": "awgn", "dbm_per_hz": -155}
    for st in d["stations"]:
        st["level_dbm"] = st.pop("snr_db") - 121.0
    d["receiver"] = {"model": model, **receiver}
    return d


def test_absolute_scenario_keeps_levels_and_derives_snr():
    s = parse_scenario(absolute(preamp="on", attenuator=20))
    assert s.noise_dbm_per_hz == -155 and s.receiver == Receiver("hardware", True, 20)
    a = s.stations[0]
    assert a.level_dbm == -131.0
    # SNR in 2500 Hz: −131 − (−155 + 33.98)
    assert a.snr_db == pytest.approx(-131 + 155 - 10 * math.log10(2500))


def test_yaml_on_off_booleans_are_accepted_for_the_preamp():
    assert parse_scenario(absolute(preamp=True)).receiver.preamp is True
    assert parse_scenario(absolute(preamp=False)).receiver.preamp is False


def test_default_receiver_is_ideal_and_snr_mode():
    s = parse_scenario(base())
    assert s.receiver == Receiver() and s.noise_dbm_per_hz is None and s.stations[0].level_dbm is None


@pytest.mark.parametrize(
    "data, message",
    [
        (lambda: {**base(), "receiver": {"model": "hardware"}}, "needs absolute levels"),
        (lambda: {**absolute(), "adc": {"noise_dbfs": -90}}, "ideal receiver only"),
        (lambda: absolute(model="ideal", preamp="on"), "hardware model only"),
        (lambda: absolute(attenuator=10), "0 or 20"),
        (lambda: absolute(preamp="maybe"), "on or off"),
        (lambda: absolute(model="analog"), "unknown model"),
        (lambda: {**absolute(), "receiver": {"model": "hardware", "lna": "on"}}, "unknown key"),
        (lambda: {**absolute(), "noise": {"type": "awgn", "dbm_per_hz": -20}}, "-200..-50"),
    ],
)
def test_receiver_and_level_errors(data, message):
    with pytest.raises(ScenarioError, match=message):
        parse_scenario(data())


def test_snr_and_level_cannot_be_mixed():
    d = absolute()
    d["stations"][1]["snr_db"] = 3
    with pytest.raises(ScenarioError, match="not snr_db"):
        parse_scenario(d)
    d = base()
    d["stations"][0]["level_dbm"] = -100
    with pytest.raises(ScenarioError, match="needs absolute noise"):
        parse_scenario(d)
