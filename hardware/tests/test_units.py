import pytest

from neohw.units import format_si, parse_si


@pytest.mark.parametrize("text, value", [
    ("61.9", 61.9), ("33n", 33e-9), ("4.7u", 4.7e-6), ("4.7µ", 4.7e-6), ("1m", 1e-3),
    ("1M", 1e6), ("2.2k", 2.2e3), ("10p", 10e-12), (249, 249.0), ("1e-3", 1e-3),
])
def test_parse_si(text, value):
    assert parse_si(text) == pytest.approx(value)


@pytest.mark.parametrize("bad", ["", "abc", "1x", "k"])
def test_parse_si_rejects_garbage(bad):
    with pytest.raises(ValueError):
        parse_si(bad)


def test_format_si():
    assert format_si(61.9, "Ω") == "61.9Ω"
    assert format_si(3.3e-8, "F") == "33nF"
    assert format_si(4.7e-6, "H") == "4.7uH"
    assert format_si(1e6, "Hz") == "1MHz"
