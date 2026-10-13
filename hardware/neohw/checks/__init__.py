"""Block checks: simulations + spec targets."""
from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path

import numpy as np


@dataclass(frozen=True)
class CheckResult:
    name: str
    value: float
    limit: str
    passed: bool
    unit: str = "dB"


@dataclass
class BlockResult:
    block: str
    checks: list[CheckResult]
    plots: dict[str, tuple[np.ndarray, np.ndarray]] = field(default_factory=dict)
    artefacts: dict[str, Path] = field(default_factory=dict)
    logx: bool = False
    x_label: str = "frequency (MHz)"      # plot x axis: label, and the unit the values are divided by
    x_scale: float = 1e6

    @property
    def passed(self) -> bool:
        return all(c.passed for c in self.checks)


def db(x) -> np.ndarray:
    return 20 * np.log10(np.maximum(np.abs(x), 1e-30))
