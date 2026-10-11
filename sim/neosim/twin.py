"""Digital twin: scenario → I/Q → ideal ADC → shipping DSP → decoders."""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np
from scipy.io import wavfile
from scipy.signal import welch

from neosim import decoders
from neosim.adc import AdcStats, ideal_adc
from neosim.channel import NOISE_POWER, Exclusion, compose
from neosim.dsp_runner import run_dsp
from neosim.scenario import FS_HZ, Receive, Scenario
from neosim.sigmf import write_sigmf


@dataclass(frozen=True)
class ReceiveResult:
    id: str
    decoder: str
    output: str
    passed: bool
    detail: str


@dataclass(frozen=True)
class ScenarioResult:
    scenario: Scenario
    excluded: list[Exclusion]
    adc: AdcStats
    receives: list[ReceiveResult]
    psd_freqs_hz: np.ndarray
    psd_db: np.ndarray

    @property
    def passed(self) -> bool:
        return all(r.passed for r in self.receives)


def _evaluate(rx: Receive, wav: Path) -> ReceiveResult:
    if rx.decoder == "ft8":
        messages = [d.message for d in decoders.decode_ft8(wav)]
        missing = [m for m in rx.expect["contains"] if m not in messages]
        detail = "all expected messages decoded" if not missing else f"missing: {missing}"
        return ReceiveResult(rx.id, rx.decoder, " | ".join(messages), not missing, detail)
    rate, data = wavfile.read(wav)
    text = decoders.decode_cw(data.astype(np.float64) / 32768.0, rate)
    error = decoders.cer(text, rx.expect["text"])
    passed = error <= rx.expect["max_cer"]
    return ReceiveResult(rx.id, rx.decoder, text, passed, f"CER {error:.3f} (limit {rx.expect['max_cer']})")


def run_scenario(scenario: Scenario, workdir: Path) -> ScenarioResult:
    workdir = Path(workdir)
    workdir.mkdir(parents=True, exist_ok=True)
    rng = np.random.default_rng(scenario.seed)
    iq, excluded = compose(scenario, rng)
    iq_q, adc = ideal_adc(iq, scenario.noise_dbfs, NOISE_POWER)
    meta = write_sigmf(workdir / "capture", iq_q, FS_HZ, scenario.lo_hz)

    receives = []
    for rx in scenario.receive:
        wav = workdir / f"{rx.id}.wav"
        run_dsp(meta, wav, rx.mode, rx.dial_hz - scenario.lo_hz)
        receives.append(_evaluate(rx, wav))

    freqs, psd = welch(iq_q, fs=FS_HZ, nperseg=8192, return_onesided=False)
    order = np.argsort(freqs)
    return ScenarioResult(scenario, excluded, adc, receives, freqs[order],
                          10 * np.log10(np.maximum(psd[order], 1e-30)))
