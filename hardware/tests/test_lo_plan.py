from fractions import Fraction

import pytest

from neohw import lo_plan as lp


def test_direct_quadrature_uses_one_even_integer_divider_with_a_90_degree_offset():
    p = lp.plan(14.074e6)
    assert p.path == "direct"
    c0, c1 = p.outputs["CLK0"], p.outputs["CLK1"]
    assert c0.pll == c1.pll == "A" and c0.ms == c1.ms and c0.ms.denominator == 1 and c0.ms % 2 == 0
    assert c0.phoff == 0 and c1.phoff == c0.ms <= lp.PHOFF_MAX
    assert lp.VCO_MIN_HZ <= p.vco_hz["A"] <= lp.VCO_MAX_HZ
    assert c0.actual_hz == Fraction(14_074_000)


def test_below_the_crossover_clk2_runs_at_four_times_the_lo():
    p = lp.plan(3.573e6)
    assert p.path == "div4"
    c2 = p.outputs["CLK2"]
    assert c2.pll == "B" and c2.actual_hz == 4 * Fraction(3_573_000)
    assert "CLK0" not in p.outputs and "CLK1" not in p.outputs


def test_the_crossover_follows_the_phase_register_limit():
    # 90° needs an even integer divider ≤ 126 with the VCO ≥ 600 MHz: 600/126 = 4.76 MHz
    assert lp.DIRECT_MIN_HZ == 4.8e6
    assert lp.plan(4.8e6).path == "direct" and lp.plan(4.79e6).path == "div4"


def test_cal_tone_shares_pllb_and_lands_on_its_frequency():
    for f_lo in (1.9e6, 14.074e6, 50.1e6):
        p = lp.plan(f_lo, cal_offset_hz=10e3)
        c3 = p.outputs["CLK3"]
        assert c3.pll == "B"
        assert abs(float(c3.actual_hz) - (f_lo + 10e3)) <= 1.0


def test_low_frequencies_use_the_r_divider():
    p = lp.plan(0.1e6, cal_offset_hz=10e3)
    assert p.outputs["CLK2"].actual_hz == Fraction(400_000)
    assert p.outputs["CLK3"].r_div > 1          # 110 kHz needs MS × R > 1800


def test_every_lo_from_0_1_to_54_mhz_has_an_exact_plan():
    """The planner's coverage check (spec phase 3 §1 item 4), at 10 kHz steps in the unit test;
    the hardware check runs the full 1 kHz grid."""
    stats = lp.coverage(0.1e6, 54e6, 10e3)
    assert stats["points"] == 5391 and stats["failures"] == []
    assert stats["max_error_hz"] <= 1.0


@pytest.mark.parametrize("f", [0.05e6, 55e6])
def test_outside_the_receiver_range_is_an_error(f):
    with pytest.raises(ValueError, match="0.1–54 MHz"):
        lp.plan(f)
