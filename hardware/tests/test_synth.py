import math

import pytest

from neohw.synth.filters import chebyshev_g, highpass, lowpass


def test_chebyshev_matches_matthaei_table_0p1db():
    # Matthaei, Young & Jones, Table 4.05-2(a), 0.1 dB ripple
    assert chebyshev_g(5, 0.1) == pytest.approx([1.1468, 1.3712, 1.9750, 1.3712, 1.1468], abs=1e-4)
    assert chebyshev_g(7, 0.1)[:4] == pytest.approx([1.1812, 1.4228, 2.0967, 1.5734], abs=1e-4)


def test_odd_orders_are_symmetric():
    g = chebyshev_g(9, 0.02)
    assert g == pytest.approx(g[::-1])


@pytest.mark.parametrize("order, ripple", [(4, 0.1), (0, 0.1), (5, 0.0)])
def test_invalid_prototype_rejected(order, ripple):
    with pytest.raises(ValueError):
        chebyshev_g(order, ripple)


def test_lowpass_at_unit_scale_equals_prototype():
    els = lowpass(1 / (2 * math.pi), 5, 0.1, r0=1.0)
    assert [e.kind for e in els] == ["C_shunt", "L_series", "C_shunt", "L_series", "C_shunt"]
    assert [e.value for e in els] == pytest.approx(chebyshev_g(5, 0.1))


def test_highpass_is_the_reciprocal_transform():
    els = highpass(1 / (2 * math.pi), 5, 0.1, r0=1.0)
    assert [e.kind for e in els] == ["C_series", "L_shunt", "C_series", "L_shunt", "C_series"]
    assert [e.value for e in els] == pytest.approx([1 / g for g in chebyshev_g(5, 0.1)])


def test_lowpass_scales_to_50_ohm_and_frequency():
    (c1, l2, *_) = lowpass(10e6, 5, 0.1)
    w = 2 * math.pi * 10e6
    assert c1.value == pytest.approx(1.1468 / (50 * w), rel=1e-4)
    assert l2.value == pytest.approx(1.3712 * 50 / w, rel=1e-4)
