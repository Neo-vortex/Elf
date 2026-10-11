import pytest

from neosim import morse


def test_letters_and_gaps():
    assert morse.keying_units("ET") == [(True, 1), (False, 3), (True, 3)]


def test_word_gap():
    assert morse.keying_units("E T") == [(True, 1), (False, 7), (True, 3)]


def test_intra_character_gap():
    assert morse.keying_units("A") == [(True, 1), (False, 1), (True, 3)]


def test_unknown_character_raises():
    with pytest.raises(ValueError, match="no Morse code"):
        morse.keying_units("A#")


def test_reverse_table_round_trips():
    assert all(morse.REVERSE[code] == ch for ch, code in morse.MORSE.items())
