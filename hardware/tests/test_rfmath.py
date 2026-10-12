import math

import numpy as np
import pytest

from neohw import rfmath


def test_noise_figure_of_a_matched_resistor_source_is_0_db():
    e = math.sqrt(4 * rfmath.K_B * rfmath.T_SIM * 50)
    assert rfmath.noise_figure_db(np.array([e]))[0] == pytest.approx(0.0, abs=1e-9)
    assert rfmath.noise_figure_db(np.array([e * math.sqrt(2)]))[0] == pytest.approx(3.0103, abs=1e-3)


def test_two_tone_recovers_known_intercept():
    h, n = 1e-9, 20000                  # 20 µs: whole periods of 50 kHz multiples
    t = np.arange(n) * h
    a = math.sqrt(2 * 50 * 1e-3)        # 0 dBm per tone in 50 Ω
    a3 = math.sqrt(2 * 50 * 1e-3 * 10 ** (-60 / 10))
    v = a * (np.sin(2 * np.pi * 14.0e6 * t) + np.sin(2 * np.pi * 14.1e6 * t)) \
        + a3 * np.sin(2 * np.pi * 13.9e6 * t)
    r = rfmath.two_tone(v, h, 14.0e6, 14.1e6)
    assert r["pout_dbm"] == pytest.approx(0.0, abs=1e-6)
    assert r["imd3_dbm"] == pytest.approx(-60.0, abs=1e-6)
    assert r["oip3_dbm"] == pytest.approx(30.0, abs=1e-6)


def test_off_grid_tone_rejected():
    with pytest.raises(ValueError, match="FFT grid"):
        rfmath.tone_powers_dbm(np.zeros(1000), 1e-9, [1.234567e6])


def test_stability_of_a_matched_attenuator():
    s = np.array([0.0]), np.array([0.1]), np.array([0.1]), np.array([0.0])
    k, d = rfmath.stability(*s)
    assert k[0] == pytest.approx((1 + 0.01 ** 2) / 0.02) and d[0] == pytest.approx(0.01)


def _s_from_abcd(a, b, c, d, z0=50.0):
    den = a + b / z0 + c * z0 + d
    return ((a + b / z0 - c * z0 - d) / den, 2 / den, 2 * (a * d - b * c) / den,
            (-a + b / z0 - c * z0 + d) / den)


def test_shunt_impedance_of_a_pi_network():
    # shunt 500 ohm at port 1, then a 10 ohm series resistor (ABCD = shunt @ series)
    rp, rs = 500.0, 10.0
    a, b, c, d = 1.0, rs, 1 / rp, 1 + rs / rp
    s11, s21, s12, s22 = (np.array([x]) for x in _s_from_abcd(a, b, c, d))
    assert rfmath.shunt_impedance_at_port1(s11, s21, s12, s22)[0] == pytest.approx(rp, rel=1e-9)
