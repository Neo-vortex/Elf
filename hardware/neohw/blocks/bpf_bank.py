"""The 10-filter bank on common input/output buses (parent spec §4.5)."""
from __future__ import annotations

from skidl import Circuit, Net

from neohw.blocks import Block, bpf


def build(selected: int | None, q_ref: str = "centre") -> Block:
    """`selected` is the one energised filter, or None for the muted safe state."""
    if selected is not None and not 1 <= selected <= len(bpf.BANDS):
        raise ValueError(f"selected must be 1..{len(bpf.BANDS)} or None, got {selected}")
    ckt = Circuit()
    with ckt:
        rf_in, rf_out, gnd = Net("rf_in"), Net("rf_out"), Net("GND")
        for index in range(1, len(bpf.BANDS) + 1):
            bpf.add_switched(index, "energised" if index == selected else "released", rf_in, rf_out, gnd, q_ref)
    return Block("bpf_bank", ckt, {"rf_in": rf_in, "rf_out": rf_out})


def select(freq_hz: float) -> int:
    """Filter index for an LO frequency: lo <= f < hi, and 54 MHz itself -> filter 10."""
    for index, (lo, hi) in enumerate(bpf.BANDS, 1):
        if lo <= freq_hz < hi or (index == len(bpf.BANDS) and freq_hz == hi):
            return index
    raise ValueError(f"{freq_hz} Hz is outside the bank's 0.1–54 MHz range")
