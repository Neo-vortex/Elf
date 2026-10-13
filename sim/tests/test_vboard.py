from fractions import Fraction

import numpy as np
import pytest

from neosim import hardware, vboard

ORDER = [f"bank{k}_{e}" for k in range(1, 12) for e in ("in", "out")] + ["atten", "preamp"]


def _params(x: Fraction) -> list[int]:
    a, b, c = x.numerator // x.denominator, x.numerator % x.denominator, x.denominator
    f = 128 * b // c
    p1, p2, p3 = 128 * a + f - 512, 128 * b - c * f, c
    return [p3 >> 8 & 0xFF, p3 & 0xFF, p1 >> 16 & 3, p1 >> 8 & 0xFF, p1 & 0xFF,
            (p3 >> 12 & 0xF0) | (p2 >> 16 & 0x0F), p2 >> 8 & 0xFF, p2 & 0xFF]


def _regs_14mhz() -> bytes:
    r = bytearray(256)
    r[3] = 0xFC
    r[16:24] = [0x4F, 0x4F] + [0x80] * 6
    r[26:34] = _params(Fraction(28))                 # PLLA 700 MHz
    r[42:50] = r[50:58] = _params(Fraction(50))      # 14 MHz
    r[166] = 50
    return bytes(r)


def _bits(*names: str) -> int:
    return sum(1 << ORDER.index(n) for n in names)


@pytest.fixture(scope="module")
def hw():
    return hardware.Hardware.load()


def test_a_correct_state_decodes_to_lo_filter_and_front_end(hw):
    s = vboard.decode_state(_regs_14mhz(), _bits("bank7_in", "bank7_out", "preamp"), False, ORDER, hw)
    assert (s.lo_hz, s.filter, s.preamp, s.attenuator, s.reason) == (14e6, 7, True, False, "")


@pytest.mark.parametrize("relays, path, why", [
    (_bits("bank7_in"), False, "no single filter"),                                   # output relay missing
    (_bits("bank7_in", "bank7_out", "bank6_in", "bank6_out"), False, "no single filter"),
    (_bits("bank6_in", "bank6_out"), False, "filter 6 selected for an LO in filter 7"),
    (_bits("bank7_in", "bank7_out"), True, "CLK2 is off"),                            # wrong path select
])
def test_a_wrong_firmware_state_blocks_the_stations(hw, relays, path, why):
    s = vboard.decode_state(_regs_14mhz(), relays, path, ORDER, hw)
    assert why in s.reason


def test_a_blocked_state_renders_noise_only(hw, tmp_path):
    st = tmp_path / "st.yaml"
    st.write_text("name: t\nduration_s: 1.0\nseed: 3\nnoise: {type: awgn, dbm_per_hz: -160}\nstations:\n"
                  "  - {id: c, mode: carrier, rf_hz: 14_010_000, level_dbm: -60}\n")
    twin = vboard.Twin(st, hw)
    good = vboard.decode_state(_regs_14mhz(), _bits("bank7_in", "bank7_out"), False, ORDER, hw)
    bad = vboard.decode_state(_regs_14mhz(), _bits("bank6_in", "bank6_out"), False, ORDER, hw)

    def at_10khz(frames):                            # power in the carrier's bin (LO + 10 kHz), I − jQ
        iq = frames[:, 0].astype(float) - 1j * frames[:, 1]
        spec = np.abs(np.fft.fft(iq)) ** 2
        return spec[int(round(10e3 / 96e3 * len(iq)))]

    assert at_10khz(twin.render(good)) > 1e6 * at_10khz(twin.render(bad))   # 60 dB: the carrier is gone


def test_the_twin_plays_a_recorded_antenna_for_the_state_the_firmware_set(hw, tmp_path):
    import json, yaml
    rate, n = 96000, 96000
    t = np.arange(n) / rate
    iq = 0.1 * np.exp(2j * np.pi * 20e3 * t)                       # a tone 20 kHz above the recording's centre
    np.asarray(iq, dtype="<c8").tofile(tmp_path / "r.sigmf-data")
    (tmp_path / "r.sigmf-meta").write_text(json.dumps({
        "global": {"core:datatype": "cf32_le", "core:sample_rate": rate, "core:version": "1.0.0"},
        "captures": [{"core:sample_start": 0, "core:frequency": 14_000_000}], "annotations": []}))
    (tmp_path / "ant.yaml").write_text(yaml.safe_dump({"name": "rec", "duration_s": 1.0, "seed": 2,
                                                       "noise": {"dbm_per_hz": -170},
                                                       "recordings": [{"file": "r.sigmf-meta", "full_scale_dbm": -40.0}]}))
    twin = vboard.Twin(tmp_path / "ant.yaml", hw)
    good = vboard.decode_state(_regs_14mhz(), _bits("bank7_in", "bank7_out"), False, ORDER, hw)
    frames = twin.render(good)
    x = frames[:, 0].astype(float) - 1j * frames[:, 1]
    spec = np.abs(np.fft.fft(x)) ** 2
    assert int(np.argmax(spec)) == int(round(20e3 / 96e3 * len(x)))      # 14.020 MHz − LO 14.000 MHz
    bad = vboard.decode_state(_regs_14mhz(), _bits("bank6_in", "bank6_out"), False, ORDER, hw)
    y = twin.render(bad)
    yb = y[:, 0].astype(float) - 1j * y[:, 1]
    k = int(round(20e3 / 96e3 * len(yb)))
    assert spec[k] > 1e6 * (np.abs(np.fft.fft(yb)) ** 2)[k]               # a wrong filter blocks the recording too
