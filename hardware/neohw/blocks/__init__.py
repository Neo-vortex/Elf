"""Circuit blocks: functions that build a SKiDL circuit with named ports."""
from __future__ import annotations

from dataclasses import dataclass, field


@dataclass
class Block:
    name: str
    circuit: object
    ports: dict = field(default_factory=dict)
