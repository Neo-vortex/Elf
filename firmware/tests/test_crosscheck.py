"""The firmware against the design's Python references (phase 4 spec §1 item 2): the C LO
planner equals hardware/neohw/lo_plan.py on the whole 1 kHz grid, and every Si5351 register
image the firmware builds decodes, through the independent AN619 decoder in sim/neosim, to
exactly the LO and calibration tone asked for."""
from __future__ import annotations

import os
import subprocess
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
BUILD = Path(os.environ.get("ELF_FW_BUILD", ROOT / "build" / "firmware"))
sys.path[:0] = [str(ROOT / "hardware"), str(ROOT / "sim")]

from neohw import lo_plan  # noqa: E402
from neosim import si5351_decode  # noqa: E402

GRID = ("100000", "54000000", "1000")


def _run(tool: str, *args: str) -> list[str]:
    exe = BUILD / tool
    if not exe.exists():
        pytest.skip(f"{exe} not built (make fw)")
    return subprocess.run([str(exe), *args], check=True, capture_output=True, text=True).stdout.splitlines()


def _fr(x) -> str:
    return f"{x.numerator}/{x.denominator}"


def _python_line(f: int, cal: int) -> str:
    try:
        p = lo_plan.plan(f, cal)
    except ValueError:
        return f"{f} error"
    parts = [str(f), p.path] + [_fr(p.pll_mult[k]) if k in p.pll_mult else "-" for k in ("A", "B")]
    for c in ("CLK0", "CLK1", "CLK2", "CLK3"):
        o = p.outputs.get(c)
        parts += ["-"] if o is None else [o.pll, _fr(o.ms), str(o.r_div), str(o.phoff)]
    return " ".join(parts)


def test_c_planner_equals_the_python_planner_on_the_whole_grid():
    c_lines = _run("lo_plan_dump", *GRID, "10000")
    start, stop, step = (int(x) for x in GRID)
    py_lines = [_python_line(f, 10000) for f in range(start, stop + 1, step)]
    assert len(c_lines) == len(py_lines) == 53901
    diff = [(c, p) for c, p in zip(c_lines, py_lines) if c != p]
    assert diff == [], diff[:3]


@pytest.mark.parametrize("cal", [0, 10000])
def test_register_images_decode_to_the_requested_lo_and_tone(cal):
    bad = []
    for line in _run("si5351_dump", *GRID, str(cal)):
        f, path, hx = line.split()
        s = si5351_decode.lo_state(bytes.fromhex(hx), path == "1")
        want_cal = int(f) + cal if cal else None
        if s.lo_hz != int(f) or s.cal_hz != want_cal:
            bad.append((f, s))
    assert bad == [], bad[:3]
