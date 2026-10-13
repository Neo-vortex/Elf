from fractions import Fraction

from neosim import si5351_decode as d


def _params(x: Fraction, rdiv_log2: int = 0) -> list[int]:
    a, b, c = x.numerator // x.denominator, x.numerator % x.denominator, x.denominator
    f = 128 * b // c
    p1, p2, p3 = 128 * a + f - 512, 128 * b - c * f, c
    return [p3 >> 8 & 0xFF, p3 & 0xFF, rdiv_log2 << 4 | p1 >> 16 & 3, p1 >> 8 & 0xFF, p1 & 0xFF,
            (p3 >> 12 & 0xF0) | (p2 >> 16 & 0x0F), p2 >> 8 & 0xFF, p2 & 0xFF]


def _direct(ms: int = 50, phoff: int = 50, pll=Fraction(28, 1)) -> bytearray:
    """14 MHz direct quadrature: PLLA 700 MHz, MS0 = MS1 = 50."""
    r = bytearray(256)
    r[3] = 0xFC
    r[16:24] = [0x4F, 0x4F] + [0x80] * 6
    r[26:34] = _params(pll)
    r[42:50] = _params(Fraction(ms))
    r[50:58] = _params(Fraction(ms))
    r[166] = phoff
    return r


def test_direct_quadrature_decodes_to_the_lo():
    s = d.lo_state(bytes(_direct()), lo_path_high=False)
    assert s.lo_hz == 14_000_000 and s.path == "direct" and s.cal_hz is None


def test_a_wrong_phase_offset_gives_no_lo():
    s = d.lo_state(bytes(_direct(phoff=49)), lo_path_high=False)
    assert s.lo_hz is None and "quarter period" in s.reason


def test_the_mux_select_must_match_the_running_outputs():
    s = d.lo_state(bytes(_direct()), lo_path_high=True)      # ÷4 selected, CLK2 off
    assert s.lo_hz is None and "CLK2" in s.reason


def test_a_disabled_output_gives_no_lo():
    r = _direct()
    r[3] = 0xFE                                              # CLK1 disabled
    assert d.lo_state(bytes(r), lo_path_high=False).lo_hz is None


def test_r_divider_and_fractional_multisynth():
    r = bytearray(256)
    r[3] = 0xFB                                              # CLK2 on
    r[16:24] = [0x80, 0x80, 0x6F, 0x80, 0x80, 0x80, 0x80, 0x80]   # CLK2: PLLB, integer
    r[34:42] = _params(Fraction(30))                         # 750 MHz
    r[58:66] = _params(Fraction(1500, 7), rdiv_log2=3)       # ÷(1500/7) ÷8
    s = d.lo_state(bytes(r), lo_path_high=True)
    assert s.lo_hz == Fraction(750_000_000 * 7, 1500 * 8 * 4)
