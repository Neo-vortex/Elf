"""FT8 decode rate vs SNR: audio-only reference vs full digital twin."""
from __future__ import annotations

import shutil
from dataclasses import dataclass
from pathlib import Path

import numpy as np
from scipy.io import wavfile

from neosim import decoders, sources
from neosim.scenario import parse_scenario
from neosim.twin import run_scenario

MESSAGE = "CQ EP2ABC LL35"
AUDIO_HZ = 1500
LO_HZ = 14_060_000
DIAL_HZ = 14_074_000


@dataclass(frozen=True)
class SweepPoint:
    snr_db: float
    reference_rate: float
    twin_rate: float
    trials: int = 1


def wilson_interval(successes: int, trials: int, z: float = 1.96) -> tuple[float, float]:
    """95 % Wilson score interval for a binomial decode rate."""
    if trials <= 0:
        return 0.0, 1.0
    p = successes / trials
    denom = 1 + z * z / trials
    centre = (p + z * z / (2 * trials)) / denom
    half = z * ((p * (1 - p) / trials + z * z / (4 * trials * trials)) ** 0.5) / denom
    return max(0.0, centre - half), min(1.0, centre + half)


def _reference_trial(audio: np.ndarray, snr_db: float, rng: np.random.Generator, wav: Path) -> bool:
    active = np.abs(audio) > 1e-3
    signal_power = np.mean(audio[active] ** 2)
    # real noise at 12 kHz: density spread over 6000 Hz; scale so noise in 2500 Hz sets the SNR
    sigma2 = signal_power / 10 ** (snr_db / 10) * 6000 / 2500
    x = audio + rng.normal(scale=np.sqrt(sigma2), size=len(audio))
    x = x / np.max(np.abs(x)) * 0.9
    wavfile.write(wav, 12_000, (x * 32767).astype(np.int16))
    return any(d.message == MESSAGE for d in decoders.decode_ft8(wav))


def _twin_trial(snr_db: float, seed: int, workdir: Path) -> bool:
    scenario = parse_scenario({
        "name": f"sweep {snr_db:+.1f} dB", "lo_hz": LO_HZ, "seed": seed,
        "stations": [{"id": "s", "mode": "ft8", "dial_hz": DIAL_HZ, "audio_hz": AUDIO_HZ,
                      "snr_db": snr_db, "msg": MESSAGE}],
        "receive": [{"id": "rx", "mode": "usb", "dial_hz": DIAL_HZ, "decoder": "ft8",
                     "expect": {"contains": [MESSAGE]}}],
    })
    try:
        return run_scenario(scenario, workdir).passed
    finally:
        shutil.rmtree(workdir, ignore_errors=True)  # an 11.5 MB capture per trial adds up fast


def sweep_ft8(snrs: list[float], trials: int, workdir: Path, seed: int = 1) -> list[SweepPoint]:
    workdir = Path(workdir)
    workdir.mkdir(parents=True, exist_ok=True)
    audio = sources.ft8_audio(MESSAGE, AUDIO_HZ)
    rng = np.random.default_rng(seed)
    points = []
    for snr in snrs:
        ref_ok = twin_ok = 0
        for trial in range(trials):
            ref_ok += _reference_trial(audio, snr, rng, workdir / "reference.wav")
            twin_ok += _twin_trial(snr, seed * 1000 + trial, workdir / f"twin_{snr:+.1f}_{trial}")
        points.append(SweepPoint(float(snr), ref_ok / trials, twin_ok / trials, trials))
    return points


def threshold_snr(snrs: list[float], rates: list[float], level: float = 0.5) -> float | None:
    pairs = sorted(zip(snrs, rates), reverse=True)  # strongest first
    for (s_hi, r_hi), (s_lo, r_lo) in zip(pairs, pairs[1:]):
        if r_hi >= level > r_lo:
            return s_hi + (r_hi - level) / (r_hi - r_lo) * (s_lo - s_hi)
    return None  # never crosses inside the swept range: the threshold is unknown


def implementation_loss_db(points: list[SweepPoint]) -> float | None:
    snrs = [p.snr_db for p in points]
    ref = threshold_snr(snrs, [p.reference_rate for p in points])
    twin = threshold_snr(snrs, [p.twin_rate for p in points])
    if ref is None or twin is None:
        return None
    return twin - ref
