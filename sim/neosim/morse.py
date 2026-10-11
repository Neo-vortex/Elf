"""International Morse code table and keying in dit units."""
from __future__ import annotations

MORSE: dict[str, str] = {
    "A": ".-", "B": "-...", "C": "-.-.", "D": "-..", "E": ".", "F": "..-.", "G": "--.", "H": "....",
    "I": "..", "J": ".---", "K": "-.-", "L": ".-..", "M": "--", "N": "-.", "O": "---", "P": ".--.",
    "Q": "--.-", "R": ".-.", "S": "...", "T": "-", "U": "..-", "V": "...-", "W": ".--", "X": "-..-",
    "Y": "-.--", "Z": "--..",
    "0": "-----", "1": ".----", "2": "..---", "3": "...--", "4": "....-", "5": ".....", "6": "-....",
    "7": "--...", "8": "---..", "9": "----.",
    "/": "-..-.", "?": "..--..", ".": ".-.-.-", ",": "--..--", "=": "-...-",
}
REVERSE: dict[str, str] = {code: ch for ch, code in MORSE.items()}


def keying_units(text: str) -> list[tuple[bool, int]]:
    """Return (key_down, length_in_dits) segments for text; no leading/trailing gaps."""
    words = text.upper().split()
    segments: list[tuple[bool, int]] = []
    for w_index, word in enumerate(words):
        if w_index:
            segments.append((False, 7))
        for c_index, ch in enumerate(word):
            if ch not in MORSE:
                raise ValueError(f"no Morse code for {ch!r}")
            if c_index:
                segments.append((False, 3))
            for e_index, element in enumerate(MORSE[ch]):
                if e_index:
                    segments.append((False, 1))
                segments.append((True, 1 if element == "." else 3))
    return segments
