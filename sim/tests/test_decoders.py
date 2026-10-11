import numpy as np
import pytest
from scipy.io import wavfile

from neosim import decoders, sources
from neosim.tools import ToolNotFoundError, tool_path

REAL_OUTPUT = """Sample rate 12000 Hz, 180000 samples, 15.000 seconds
Block size = 1920
Subblock size = 960
N_FFT = 3840
#############################################################################################
Max magnitude: -6.0 dB
Decoded 2 messages, callsign hashtable size 1
000000 +16.5 +1.36 1500 ~  CQ EP2ABC LL35
000000 -12.0 +0.80 2100 ~  EP2ABC DL1XYZ JO62
"""


def test_parse_ft8_output_extracts_messages():
    assert decoders.parse_ft8_output(REAL_OUTPUT) == [
        decoders.Ft8Decode(1500, "CQ EP2ABC LL35"),
        decoders.Ft8Decode(2100, "EP2ABC DL1XYZ JO62"),
    ]


def test_parse_ft8_output_empty():
    assert decoders.parse_ft8_output("Decoded 0 messages\n") == []


def test_cer():
    assert decoders.cer("CQ TEST", "CQ TEST") == 0.0
    assert decoders.cer("CQ TEXT", "CQ TEST") == pytest.approx(1 / 7)
    assert decoders.cer(" cq   test ", "CQ TEST") == 0.0
    assert decoders.cer("", "CQ") == 1.0


def _cw_audio(text, wpm, snr_db_500hz, seed):
    fs = 12_000
    env = sources.cw_envelope(text, wpm, fs)
    t = np.arange(len(env)) / fs
    tone = env * np.sin(2 * np.pi * 600 * t)
    rng = np.random.default_rng(seed)
    # noise power in 500 Hz relative to key-down tone power (0.5)
    noise = rng.normal(size=len(env)) * np.sqrt(0.5 / 10 ** (snr_db_500hz / 10) * (fs / 2) / 500)
    return tone + noise, fs


def test_decode_cw_clean():
    audio, fs = _cw_audio("CQ TEST DE EP2ABC", 20, 60, 0)
    assert decoders.decode_cw(audio, fs) == "CQ TEST DE EP2ABC"


def test_decode_cw_noisy_has_low_cer():
    audio, fs = _cw_audio("CQ TEST DE EP2ABC", 22, 12, 1)
    assert decoders.cer(decoders.decode_cw(audio, fs), "CQ TEST DE EP2ABC") <= 0.05


def test_decode_cw_silence_is_empty():
    assert decoders.decode_cw(np.random.default_rng(2).normal(size=24_000) * 1e-3, 12_000) == ""


@pytest.mark.integration
def test_decode_ft8_on_generated_audio(tmp_path):
    try:
        tool_path("gen_ft8"), tool_path("decode_ft8")
    except ToolNotFoundError as e:
        pytest.skip(str(e))
    audio = sources.ft8_audio("CQ EP2ABC LL35", 1500)
    wav = tmp_path / "a.wav"
    wavfile.write(wav, 12_000, (audio * 32767).astype(np.int16))
    assert [d.message for d in decoders.decode_ft8(wav)] == ["CQ EP2ABC LL35"]
