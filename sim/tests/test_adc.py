import numpy as np
import pytest

from neosim.adc import ideal_adc


def test_noise_lands_at_requested_dbfs():
    rng = np.random.default_rng(0)
    x = (rng.normal(size=200_000) + 1j * rng.normal(size=200_000)) * np.sqrt(38.4 / 2)
    y, stats = ideal_adc(x, noise_dbfs=-100.0, noise_power=38.4)
    assert 10 * np.log10(np.mean(np.abs(y) ** 2)) == pytest.approx(-100.0, abs=0.2)
    assert stats.clipped_samples == 0


def test_output_is_on_24_bit_grid():
    y, _ = ideal_adc(np.array([0.123456789 + 0.5j]), noise_dbfs=0.0, noise_power=1.0)
    lsb = 2.0 ** -23
    assert y.real[0] / lsb == pytest.approx(round(y.real[0] / lsb), abs=1e-6)


def test_adc_clips_without_wrapping_and_counts():
    x = np.array([10 + 0j, -10 + 0j, 0.1 + 10j, 0.1 - 10j, 0.1 + 0.1j])
    y, stats = ideal_adc(x, noise_dbfs=0.0, noise_power=1.0)
    hi = 1.0 - 2.0 ** -23
    assert stats.clipped_samples == 4
    assert y[0].real == hi and y[1].real == -1.0
    assert y[2].imag == hi and y[3].imag == -1.0
    assert np.all(np.abs(y.real) <= 1.0) and np.all(np.abs(y.imag) <= 1.0)
    assert stats.peak_dbfs == pytest.approx(0.0, abs=1e-5)
