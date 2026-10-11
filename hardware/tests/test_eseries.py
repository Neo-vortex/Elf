import pytest

from neohw.eseries import E12, E24, combo, snap, values


def test_values_are_exact_decimals():
    assert 4.7e-06 in values(E12)
    assert 3.3e-07 in values(E12)
    assert values(E24, 1e-12, 1e-11)[:3] == [1e-12, 1.1e-12, 1.2e-12]


@pytest.mark.parametrize("value, series, expected", [
    (4.6e-6, E12, 4.7e-6), (1.04e-9, E24, 1e-9), (9.6e-8, E12, 1e-7), (2.0e-10, E24, 2.0e-10),
])
def test_snap_to_nearest_on_log_scale(value, series, expected):
    assert snap(value, series) == expected


def test_snap_rejects_non_positive():
    with pytest.raises(ValueError):
        snap(0.0, E12)


def test_combo_uses_one_part_when_close_enough():
    assert combo(1.005e-9, E24) == (1e-9,)


def test_combo_pairs_two_parts_that_add_up():
    assert combo(3.62e-6, E12) == (3.3e-06, 3.3e-07)
    assert sum(combo(1.07e-9, E24)) == pytest.approx(1.07e-9, rel=0.01)


def test_combo_respects_minimum_part_value():
    parts = combo(4.99e-7, E12, minimum=1e-7)
    assert all(p >= 1e-7 for p in parts)
    assert sum(parts) == pytest.approx(4.99e-7, rel=0.05)
