"""Compose stations and noise into the receiver's 96 kHz window."""
from __future__ import annotations

from dataclasses import dataclass
from typing import Callable

import numpy as np

from neosim import sources
from neosim.scenario import FS_HZ, WINDOW_HZ, Scenario, Station

NOISE_POWER = FS_HZ / 2500  # complex noise power per sample: noise in 2500 Hz == 1


@dataclass(frozen=True)
class Exclusion:
    station_id: str
    reason: str


def in_window(st: Station, lo_hz: float) -> bool:
    return abs(st.rf_hz - lo_hz) + st.bandwidth_hz / 2 <= WINDOW_HZ


def awgn(n: int, rng: np.random.Generator) -> np.ndarray:
    return (rng.normal(size=n) + 1j * rng.normal(size=n)) * np.sqrt(NOISE_POWER / 2)


def compose(scenario: Scenario, rng: np.random.Generator,
            render: Callable[[Station, float, int], np.ndarray] = sources.render_station,
            ) -> tuple[np.ndarray, list[Exclusion]]:
    n = int(round(scenario.duration_s * FS_HZ))
    iq = awgn(n, rng)
    excluded: list[Exclusion] = []
    for st in scenario.stations:
        if not in_window(st, scenario.lo_hz):
            excluded.append(Exclusion(
                st.id,
                f"{st.rf_hz / 1e6:.6f} MHz is outside the receiver window "
                f"{scenario.lo_hz / 1e6:.6f} MHz ±{WINDOW_HZ / 1e3:.1f} kHz"))
            continue
        iq += render(st, scenario.lo_hz, n) * np.sqrt(10 ** (st.snr_db / 10))
    return iq, excluded
