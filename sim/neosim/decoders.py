"""Decoders used as test oracles: ft8_lib for FT8, a simple timing decoder for CW."""
from __future__ import annotations

import re
import subprocess
from dataclasses import dataclass
from pathlib import Path

import numpy as np
from scipy.signal import butter, hilbert, sosfiltfilt

from neosim import morse
from neosim.tools import tool_path

_FT8_LINE = re.compile(r"^\d{6}\s+[+-]?\d+(?:\.\d+)?\s+[+-]?\d+(?:\.\d+)?\s+(\d+)\s+~\s+(.+?)\s*$")


@dataclass(frozen=True)
class Ft8Decode:
    audio_hz: int
    message: str


def parse_ft8_output(text: str) -> list[Ft8Decode]:
    out = []
    for line in text.splitlines():
        m = _FT8_LINE.match(line.strip())
        if m:
            out.append(Ft8Decode(int(m.group(1)), m.group(2)))
    return out


def decode_ft8(wav_path: Path) -> list[Ft8Decode]:
    result = subprocess.run([str(tool_path("decode_ft8")), str(wav_path)], capture_output=True, text=True)
    if result.returncode != 0:
        raise RuntimeError(f"decode_ft8 failed on {wav_path} (exit {result.returncode}): {result.stderr.strip()}")
    return parse_ft8_output(result.stdout)


def _normalise(s: str) -> str:
    return " ".join(s.upper().split())


def cer(decoded: str, expected: str) -> float:
    a, b = _normalise(decoded), _normalise(expected)
    prev = list(range(len(b) + 1))
    for i, ca in enumerate(a, 1):
        cur = [i]
        for j, cb in enumerate(b, 1):
            cur.append(min(prev[j] + 1, cur[j - 1] + 1, prev[j - 1] + (ca != cb)))
        prev = cur
    return prev[-1] / max(1, len(b))


def decode_cw(audio: np.ndarray, fs: int, pitch_hz: float = 600.0, bandwidth_hz: float = 300.0) -> str:
    sos = butter(4, [pitch_hz - bandwidth_hz / 2, pitch_hz + bandwidth_hz / 2], btype="bandpass", fs=fs, output="sos")
    env = np.abs(hilbert(sosfiltfilt(sos, np.asarray(audio, dtype=float))))
    k = max(1, int(0.01 * fs))
    env = np.convolve(env, np.ones(k) / k, mode="same")
    low, high = np.percentile(env, [10, 99])
    if high < 4 * max(low, 1e-12):
        return ""
    keyed = env > (low + high) / 2
    edges = np.flatnonzero(np.diff(keyed.astype(np.int8))) + 1
    bounds = np.concatenate(([0], edges, [len(keyed)]))
    runs = [(bool(keyed[s]), int(e - s)) for s, e in zip(bounds[:-1], bounds[1:])]
    marks = np.array([length for on, length in runs if on], dtype=float)
    if marks.size == 0:
        return ""
    min_len = int(0.012 * fs)  # shorter marks are noise glitches (a dit at 60 wpm is 20 ms)
    marks = marks[marks >= min_len]
    if marks.size == 0:
        return ""
    rough = np.percentile(marks, 10)
    dit = float(np.median(marks[marks < 2 * rough]))

    text: list[str] = []
    symbol = ""

    def flush():
        nonlocal symbol
        if symbol:
            text.append(morse.REVERSE.get(symbol, "?"))
            symbol = ""

    for on, length in runs:
        units = length / dit
        if on:
            if length < min_len:
                continue
            symbol += "." if units < 2 else "-"
        elif units >= 5:
            flush()
            text.append(" ")
        elif units >= 2:
            flush()
    flush()
    return _normalise("".join(text))
