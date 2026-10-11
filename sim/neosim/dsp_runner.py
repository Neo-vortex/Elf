"""Run the shipping C# DSP (neoradio-dsp) on a SigMF capture."""
from __future__ import annotations

import subprocess
from pathlib import Path

from neosim.tools import tool_path


def run_dsp(meta: Path, wav: Path, mode: str, tune_offset_hz: float) -> None:
    cmd = [str(tool_path("neoradio-dsp")), "--input", str(meta), "--output", str(wav),
           "--mode", mode, "--tune-offset", f"{tune_offset_hz:.3f}"]
    result = subprocess.run(cmd, capture_output=True, text=True)
    if result.returncode != 0:
        raise RuntimeError(f"neoradio-dsp failed (exit {result.returncode}): {result.stderr.strip()}")
