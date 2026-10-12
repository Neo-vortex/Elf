"""Hardware-model measurements for the report (phase 2 spec §1 items 3–4, §7): MDS from the
twin's I/Q and the blocker scenario's decode result and noise-floor degradation."""
from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from neosim import hardware, sources
from neosim.scenario import FS_HZ, WINDOW_HZ, load_scenario, parse_scenario
from neosim.twin import ScenarioResult, run_scenario

MDS_BW_HZ = 500.0
THERMAL_DBM_HZ = -174.0
MDS_TONE_OFFSET_HZ = 10e3
MDS_TONE_DBM = -110.0
MDS_TARGET_DBM = -130.0
MDS_AGREEMENT_DB = 1.0
MDS_LO_HZ = (14.1e6, 50.1e6)          # spec §1 item 4, preamp on
BLOCKER_SCENARIO = Path(__file__).resolve().parents[1] / "scenarios" / "20m_blocker_hardware.yaml"


@dataclass(frozen=True)
class MdsResult:
    f_lo_hz: float
    preamp: bool
    measured_dbm: float
    predicted_dbm: float
    nf_total_db: float

    @property
    def passed(self) -> bool:
        return self.measured_dbm <= MDS_TARGET_DBM and abs(self.measured_dbm - self.predicted_dbm) <= MDS_AGREEMENT_DB


def measure_mds(hw: hardware.Hardware, f_lo: float, preamp: bool = True, seed: int = 1,
                duration_s: float = 4.0) -> MdsResult:
    """MDS: the CW level whose power equals the noise in 500 Hz (3 dB (S+N)/N), with only thermal
    noise at the antenna. A carrier well above the floor is received through the hardware model;
    the tone power and the noise density next to it (2–20 kHz from it, both sides, without the DC
    offset and the tone's image) are read from the I/Q, and MDS = level − 10·log10(S / N_500)."""
    sc = parse_scenario({
        "name": "mds", "lo_hz": f_lo, "duration_s": duration_s, "seed": seed,
        "noise": {"type": "awgn", "dbm_per_hz": THERMAL_DBM_HZ},
        "receiver": {"model": "hardware", "preamp": "on" if preamp else "off"},
        "stations": [{"id": "cw", "mode": "carrier", "rf_hz": f_lo + MDS_TONE_OFFSET_HZ, "level_dbm": MDS_TONE_DBM}],
        "receive": [{"id": "r", "mode": "cw", "dial_hz": f_lo + MDS_TONE_OFFSET_HZ, "decoder": "cw",
                     "expect": {"text": "E"}}]})
    iq, _, stats = hardware.receive(sc, np.random.default_rng(seed), hw, sources.render_station, FS_HZ, WINDOW_HZ)
    n = len(iq)
    spec = np.abs(np.fft.fft(iq)) ** 2
    f = np.fft.fftfreq(n, 1 / FS_HZ)
    tone = spec[int(np.argmin(np.abs(f - MDS_TONE_OFFSET_HZ)))] / n ** 2
    d = np.abs(f - MDS_TONE_OFFSET_HZ)
    # not noise: the op-amps' DC offset at 0 Hz and the tone's I/Q image at −offset
    guard = (np.abs(f) < 500) | (np.abs(f + MDS_TONE_OFFSET_HZ) < 500)
    density = np.mean(spec[(d >= 2e3) & (d <= 20e3) & ~guard]) / (n * FS_HZ)
    snr = tone / (density * MDS_BW_HZ)
    k_t0 = hardware.K_B * hardware.T0
    n_total = 10 ** (THERMAL_DBM_HZ / 10) * 1e-3 + k_t0 * (10 ** (stats.noise["nf_total_db"] / 10) - 1)
    return MdsResult(f_lo, preamp, MDS_TONE_DBM - 10 * math.log10(snr),
                     10 * math.log10(n_total * MDS_BW_HZ / 1e-3), stats.noise["nf_total_db"])


@dataclass(frozen=True)
class BlockerResult:
    with_blocker: ScenarioResult
    without_blocker: ScenarioResult
    blocker_ids: tuple[str, ...]
    degradation_db: float           # noise floor in the window, with minus without (median of the PSD)

    @property
    def overload(self) -> tuple[str, ...]:
        """Stations above full scale at the mixer input (out-of-window clipping is not simulated)."""
        return self.with_blocker.hardware.overload if self.with_blocker.hardware else ()

    @property
    def passed(self) -> bool:
        return self.with_blocker.passed and not self.overload


def blocker_check(path: Path, workdir: Path, hw: hardware.Hardware | None = None) -> BlockerResult:
    """Run a blocker scenario as written and again without its out-of-window stations; report
    whether it still decodes and how far the noise floor rose."""
    sc = load_scenario(path)
    blockers = tuple(st.id for st in sc.stations if abs(st.rf_hz - sc.lo_hz) > WINDOW_HZ)
    if not blockers:
        raise ValueError(f"{path}: the blocker scenario has no station outside the window")
    from dataclasses import replace
    quiet = replace(sc, stations=tuple(st for st in sc.stations if st.id not in blockers))
    with_b = run_scenario(sc, Path(workdir) / "with", hw)
    without = run_scenario(quiet, Path(workdir) / "without", hw)
    return BlockerResult(with_b, without, blockers,
                         float(np.median(with_b.psd_db) - np.median(without.psd_db)))


@dataclass(frozen=True)
class HardwareChecks:
    mds: list[MdsResult]
    blocker: BlockerResult

    @property
    def passed(self) -> bool:
        return all(m.passed for m in self.mds) and self.blocker.passed
