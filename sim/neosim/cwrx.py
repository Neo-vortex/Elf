"""Receive side of the virtual-board acceptance (phase 4 spec §4): raw UAC2 I/Q from the board
(S24_3LE, stereo, L = I, R = Q) → the shipping C# DSP in CW mode → the twin's CW decoder."""
from __future__ import annotations

from pathlib import Path

import numpy as np
from scipy.io import wavfile

from neosim import decoders
from neosim.dsp_runner import run_dsp
from neosim.scenario import FS_HZ
from neosim.sigmf import write_sigmf

PITCH_HZ = 600.0          # the DSP's CW pitch: the dial sits 600 Hz below the station
LO_OFFSET_HZ = 10_000.0   # the LO sits 10 kHz below the station (clear of DC and the I/Q image)


def iq_from_s24le(raw: bytes) -> np.ndarray:
    """Frames of 3-byte little-endian I then Q → complex I/Q in the twin's convention (I − jQ),
    full scale 1.0."""
    b = np.frombuffer(raw[: len(raw) // 6 * 6], dtype=np.uint8).reshape(-1, 3).astype(np.int32)
    v = b[:, 0] | b[:, 1] << 8 | b[:, 2] << 16
    v = np.where(v >= 1 << 23, v - (1 << 24), v).astype(np.float64) / 2 ** 23
    return v[0::2] - 1j * v[1::2]


def decode(iq: np.ndarray, lo_hz: float, station_hz: float, workdir: Path) -> str:
    workdir = Path(workdir)
    meta = write_sigmf(workdir / "capture", iq, FS_HZ, lo_hz)
    wav = workdir / "cw.wav"
    run_dsp(meta, wav, "cw", station_hz - PITCH_HZ - lo_hz)
    rate, audio = wavfile.read(wav)
    return decoders.decode_cw(audio.astype(np.float64) / 32768.0, rate, pitch_hz=PITCH_HZ)


def best_cer(decoded: str, expected: str) -> float:
    """CER of the best-matching stretch of the decoded text: a looped recording starts and ends
    mid-message, so the full message appears somewhere inside it."""
    words = decoded.split()
    n = len(expected.split())
    best = decoders.cer(decoded, expected)
    for i in range(len(words)):
        for j in range(i + 1, min(len(words), i + n + 2) + 1):
            best = min(best, decoders.cer(" ".join(words[i:j]), expected))
    return best
