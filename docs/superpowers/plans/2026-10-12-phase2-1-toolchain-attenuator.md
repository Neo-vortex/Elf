# Phase 2.1: Circuit-as-Code Toolchain and Attenuator Block (Implementation Plan)

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** `make hw` builds the 20 dB attenuator block as SKiDL code and simulates its S-parameters in ngspice (pad and bypass states). It checks the block against its targets and writes `hardware/characterized/attenuator.json`, a KiCad 9 netlist, a BOM with Iranian availability, an SVG diagram and an HTML report.

**Architecture:** This adds a new Python package, `neohw`, in `hardware/`. Each part constructor creates a SKiDL part that carries a footprint, an MPN, a Torob key and a function returning its SPICE element lines. Our own writer turns a SKiDL `Circuit` plus simulation ports and analyses into an ngspice deck. ngspice runs in batch mode and writes ASCII rawfiles, which we parse. Block checks compare results with spec targets and emit characterisation files. This plan exercises the whole pipeline on one simple block. Plans 2.2–2.4 add filters, preamp, mixer and twin integration on top of it.

**Tech Stack:** Python 3.11+, SKiDL 2.3.0, numpy, matplotlib, pytest, ngspice 45 (batch mode), Graphviz `dot`.

**Spec:** `docs/superpowers/specs/2026-10-12-phase2-frontend-mixer-design.md` (§3 structure, §4 attenuator row, §6 characterisation contract). Parent: `docs/superpowers/specs/2026-10-12-neoradio-hf-receiver-design.md`.

## Global Constraints

- The KiCad netlist format is **KiCad 9** (`skidl.KICAD9`). Parts are created with `tool=SKIDL` templates, so no KiCad libraries are needed. Every part has a footprint.
- The SPICE simulator is **ngspice ≥ 45**, invoked as `ngspice -b deck.cir`. Results come from `set filetype=ascii` plus `write <file> all`, one rawfile per analysis.
- The net named `GND` maps to SPICE node `0`.
- Simulation ports are not parts. They are added by the SPICE writer as `V<n> <node> 0 dc 0 ac <1|0> portnum <n> z0 <z0>`.
- Characterisation files carry `schema_version` (integer, starting at 1), `generated_by` (git short hash, with `-dirty` appended when the tree has uncommitted changes), `block` and `status` (`"pass"` | `"fail"`).
- Attenuator targets (spec §4), over 0.1–54 MHz: attenuation 20 ± 0.5 dB, return loss ≥ 18 dB in pad state, bypass loss ≤ 0.2 dB.
- SKiDL writes `skidl.erc`, `skidl.log` and `skidl_sklib.py` into the working directory. All SKiDL netlist generation must run inside a temporary directory.
- Iranian availability data: `docs/superpowers/specs/2026-10-12-parts-availability-torob.json`. It maps a query to `{"ok": bool, "hits": [{"name","price","shop","instock"}]}`.

## Review Focus

1. **An ngspice deck that fails** (bad syntax, singular matrix, missing model) must raise an error that includes the ngspice log tail and the deck path, not an empty result or a parser crash. → Task 3 `test_failed_simulation_reports_log_tail`.
2. **ngspice not installed** must produce an error that says how to install it (`sudo apt install ngspice`). → Task 1 `test_missing_ngspice_says_how_to_install`.
3. **A part whose MPN has no Torob entry, or only out-of-stock hits,** must appear in the BOM as `unknown` or `out_of_stock`, not silently `available`. → Task 6 `test_bom_flags_unknown_and_out_of_stock`.
4. **A check that misses its target** must mark the characterisation file `status: "fail"`, make `python -m neohw run` exit 1, and still write the report. → Task 8 `test_cli_exit_code_reflects_failed_check`.
5. **Unconnected nets or a part pin with no net** must be reported before simulation, not passed to ngspice as a floating node with a cryptic "singular matrix". → Task 5 `test_unconnected_pin_is_rejected`.

---

## File Structure

```
hardware/
  pyproject.toml
  neohw/
    __init__.py
    tools.py          locate ngspice and dot; git describe for generated_by
    units.py          parse/format SI values ("61.9", "33n", "4.7u")
    rawfile.py        parse ngspice ASCII rawfiles into Plot objects
    ngspice.py        run a deck in a temp dir; return plots; NgspiceError
    parts.py          SKiDL templates + constructors: resistor, capacitor, relay_dpdt
    spice.py          circuit → deck text (ports, analyses, control block); simulate()
    export.py         KiCad netlist, BOM CSV with availability, Graphviz SVG
    availability.py   Torob JSON lookup → available / out_of_stock / unknown
    charfile.py       write characterisation JSON with schema/version metadata
    blocks/__init__.py
    blocks/attenuator.py
    checks/__init__.py
    checks/attenuator.py
    report.py         HTML report for block results
    __main__.py       python -m neohw run
  tests/              test_*.py per module
  characterized/      (output; attenuator.json committed)
Makefile              + hw target, test target runs hardware tests
```

---

### Task 1: Package scaffold and tool locator

**Files:**
- Create: `hardware/pyproject.toml`, `hardware/neohw/__init__.py`, `hardware/neohw/tools.py`
- Test: `hardware/tests/test_tools.py`

**Interfaces:**
- Produces:
  - `neohw.tools.REPO_ROOT: Path`
  - `neohw.tools.ToolNotFoundError(RuntimeError)`
  - `neohw.tools.tool_path(name: str) -> Path` for `"ngspice"` and `"dot"`, with env overrides `NEOHW_NGSPICE` and `NEOHW_DOT`. Lookup order: env, then `shutil.which`.
  - `neohw.tools.git_describe() -> str`: short hash, plus `-dirty` if `git status --porcelain` is non-empty; `"unknown"` outside git.

- [ ] **Step 1: Install dependencies and create the package skeleton**

Run: `python3 -m pip install skidl==2.3.0`
Expected: `Successfully installed skidl-2.3.0 …` (or "Requirement already satisfied").

`hardware/pyproject.toml`:

```toml
[project]
name = "neohw"
version = "0.1.0"
requires-python = ">=3.11"
dependencies = ["skidl==2.3.0", "numpy", "matplotlib"]

[project.optional-dependencies]
dev = ["pytest"]

[tool.pytest.ini_options]
pythonpath = ["."]
testpaths = ["tests"]
markers = ["integration: needs ngspice / dot installed"]
```

`hardware/neohw/__init__.py`:

```python
"""NeoRadio hardware: circuits as code (SKiDL) + ngspice simulation."""
```

- [ ] **Step 2: Write the failing tests**

`hardware/tests/test_tools.py`:

```python
import stat

import pytest

from neohw import tools


def _exe(path):
    path.write_text("#!/bin/sh\nexit 0\n")
    path.chmod(path.stat().st_mode | stat.S_IXUSR)
    return path


def test_missing_ngspice_says_how_to_install(monkeypatch, tmp_path):
    monkeypatch.setenv("NEOHW_NGSPICE", str(tmp_path / "nope"))
    with pytest.raises(tools.ToolNotFoundError, match="sudo apt install ngspice"):
        tools.tool_path("ngspice")


def test_missing_dot_says_how_to_install(monkeypatch, tmp_path):
    monkeypatch.setenv("NEOHW_DOT", str(tmp_path / "nope"))
    with pytest.raises(tools.ToolNotFoundError, match="graphviz"):
        tools.tool_path("dot")


def test_env_override_is_used(monkeypatch, tmp_path):
    exe = _exe(tmp_path / "ngspice")
    monkeypatch.setenv("NEOHW_NGSPICE", str(exe))
    assert tools.tool_path("ngspice") == exe


def test_unknown_tool_is_key_error():
    with pytest.raises(KeyError):
        tools.tool_path("kicad")


def test_git_describe_is_short_hash_or_unknown():
    d = tools.git_describe()
    assert d == "unknown" or len(d.removesuffix("-dirty")) >= 7
```

- [ ] **Step 3: Run the tests to verify they fail**

Run: `cd hardware && python3 -m pytest tests/test_tools.py -q`
Expected: FAIL with `ImportError: cannot import name 'tools'`.

- [ ] **Step 4: Implement `hardware/neohw/tools.py`**

```python
"""Locate external tools and describe the git revision."""
from __future__ import annotations

import os
import shutil
import subprocess
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]

_ENV = {"ngspice": "NEOHW_NGSPICE", "dot": "NEOHW_DOT"}
_HINT = {
    "ngspice": "install it with `sudo apt install ngspice`",
    "dot": "install graphviz with `sudo apt install graphviz`",
}


class ToolNotFoundError(RuntimeError):
    """An external tool is missing or not executable."""


def tool_path(name: str) -> Path:
    if name not in _ENV:
        raise KeyError(name)
    override = os.environ.get(_ENV[name])
    candidate = override if override else shutil.which(name)
    if candidate:
        path = Path(candidate)
        if path.is_file() and os.access(path, os.X_OK):
            return path
    raise ToolNotFoundError(f"{name} not found (looked at {candidate or 'PATH'}); {_HINT[name]}")


def git_describe() -> str:
    try:
        head = subprocess.run(["git", "rev-parse", "--short", "HEAD"], cwd=REPO_ROOT,
                              capture_output=True, text=True, check=True).stdout.strip()
        dirty = subprocess.run(["git", "status", "--porcelain"], cwd=REPO_ROOT,
                               capture_output=True, text=True, check=True).stdout.strip()
    except (OSError, subprocess.CalledProcessError):
        return "unknown"
    return f"{head}-dirty" if dirty else head
```

- [ ] **Step 5: Run the tests to verify they pass**

Run: `cd hardware && python3 -m pytest tests/test_tools.py -q`
Expected: `5 passed`.

- [ ] **Step 6: Commit**

```bash
git add hardware
git commit -m "build(hw): scaffold neohw package with tool locator"
```

---

### Task 2: SI units and ngspice rawfile parser

**Files:**
- Create: `hardware/neohw/units.py`, `hardware/neohw/rawfile.py`
- Test: `hardware/tests/test_units.py`, `hardware/tests/test_rawfile.py`

**Interfaces:**
- Produces:
  - `units.parse_si(text: str | float) -> float`. Accepts suffixes f p n u µ m k M G (case-sensitive: `m` = milli, `M` = mega) and a plain number. Raises `ValueError`.
  - `units.format_si(value: float, unit: str = "") -> str`, e.g. `format_si(61.9, "Ω") == "61.9Ω"`, `format_si(3.3e-8, "F") == "33nF"`.
  - `rawfile.Plot(name: str, variables: list[str], data: dict[str, np.ndarray])`. Data are complex for complex plots and float for real plots.
  - `rawfile.parse_ascii(text: str) -> list[Plot]`, raising `RawfileError(ValueError)` on malformed input.

- [ ] **Step 1: Write the failing tests**

`hardware/tests/test_units.py`:

```python
import pytest

from neohw.units import format_si, parse_si


@pytest.mark.parametrize("text, value", [
    ("61.9", 61.9), ("33n", 33e-9), ("4.7u", 4.7e-6), ("4.7µ", 4.7e-6), ("1m", 1e-3),
    ("1M", 1e6), ("2.2k", 2.2e3), ("10p", 10e-12), (249, 249.0), ("1e-3", 1e-3),
])
def test_parse_si(text, value):
    assert parse_si(text) == pytest.approx(value)


@pytest.mark.parametrize("bad", ["", "abc", "1x", "k"])
def test_parse_si_rejects_garbage(bad):
    with pytest.raises(ValueError):
        parse_si(bad)


def test_format_si():
    assert format_si(61.9, "Ω") == "61.9Ω"
    assert format_si(3.3e-8, "F") == "33nF"
    assert format_si(4.7e-6, "H") == "4.7uH"
    assert format_si(1e6, "Hz") == "1MHz"
```

`hardware/tests/test_rawfile.py`:

```python
import numpy as np
import pytest

from neohw.rawfile import RawfileError, parse_ascii

COMPLEX = """Title: rc test
Date: Mon Oct 12 01:26:11  2026
Command: ngspice-45.2, Build Fri Sep 12 11:58:13 UTC 2025
Plotname: AC Analysis
Flags: complex
No. Variables: 3
No. Points: 2
Variables:
\t0\tfrequency\tfrequency grid=3
\t1\tv(in)\tvoltage
\t2\tv(out)\tvoltage
Values:
 0\t1.000000000000000e+02,0.000000000000000e+00
\t1.000000000000000e+00,0.000000000000000e+00
\t9.900996188197071e-01,-9.900688679469646e-02

 1\t1.000000000000000e+03,0.000000000000000e+00
\t1.000000000000000e+00,0.000000000000000e+00
\t5.000155294223400e-01,-4.999999997588371e-01

"""

REAL = """Title: rc test
Date: Mon Oct 12 01:26:11  2026
Plotname: Operating Point
Flags: real
No. Variables: 2
No. Points: 1
Variables:
\t0\tv(in)\tvoltage
\t1\tv(out)\tvoltage
Values:
 0\t0.000000000000000e+00
\t2.500000000000000e+00

"""


def test_parse_complex_plot():
    (plot,) = parse_ascii(COMPLEX)
    assert plot.name == "AC Analysis"
    assert plot.variables == ["frequency", "v(in)", "v(out)"]
    np.testing.assert_allclose(plot.data["frequency"].real, [100.0, 1000.0])
    assert plot.data["v(out)"][1] == pytest.approx(0.5000155294223400 - 0.4999999997588371j)


def test_parse_real_plot():
    (plot,) = parse_ascii(REAL)
    assert plot.data["v(out)"][0] == pytest.approx(2.5)
    assert not np.iscomplexobj(plot.data["v(out)"])


def test_parse_two_plots_in_one_file():
    plots = parse_ascii(COMPLEX + REAL)
    assert [p.name for p in plots] == ["AC Analysis", "Operating Point"]


def test_truncated_values_raise():
    with pytest.raises(RawfileError, match="expected 2 points"):
        parse_ascii(COMPLEX.split(" 1\t")[0])


def test_missing_header_raises():
    with pytest.raises(RawfileError):
        parse_ascii("garbage\n")
```

- [ ] **Step 2: Run the tests to verify they fail**

Run: `cd hardware && python3 -m pytest tests/test_units.py tests/test_rawfile.py -q`
Expected: FAIL with `ModuleNotFoundError: No module named 'neohw.units'`.

- [ ] **Step 3: Implement**

`hardware/neohw/units.py`:

```python
"""SI value parsing and formatting."""
from __future__ import annotations

import math
import re

_PREFIX = {"f": 1e-15, "p": 1e-12, "n": 1e-9, "u": 1e-6, "µ": 1e-6, "m": 1e-3,
           "k": 1e3, "M": 1e6, "G": 1e9}
_RE = re.compile(r"^\s*([+-]?(?:\d+\.?\d*|\.\d+)(?:[eE][+-]?\d+)?)\s*([fpnuµmkMG]?)\s*$")


def parse_si(text: str | float) -> float:
    if isinstance(text, (int, float)):
        return float(text)
    m = _RE.match(text)
    if not m:
        raise ValueError(f"not an SI value: {text!r}")
    return float(m.group(1)) * _PREFIX.get(m.group(2), 1.0)


def format_si(value: float, unit: str = "") -> str:
    if value == 0:
        return f"0{unit}"
    exp = int(math.floor(math.log10(abs(value)) / 3) * 3)
    exp = max(-15, min(9, exp))
    prefix = {-15: "f", -12: "p", -9: "n", -6: "u", -3: "m", 0: "", 3: "k", 6: "M", 9: "G"}[exp]
    mantissa = value / 10 ** exp
    return f"{mantissa:.4g}{prefix}{unit}"
```

`hardware/neohw/rawfile.py`:

```python
"""Parse ngspice ASCII rawfiles (set filetype=ascii; write file all)."""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np


class RawfileError(ValueError):
    """The rawfile is malformed or truncated."""


@dataclass
class Plot:
    name: str
    variables: list[str]
    data: dict[str, np.ndarray]


def _header(lines: list[str], i: int, key: str) -> tuple[str, int]:
    while i < len(lines) and not lines[i].startswith(key):
        if lines[i].startswith("Values:"):
            raise RawfileError(f"missing header {key!r}")
        i += 1
    if i >= len(lines):
        raise RawfileError(f"missing header {key!r}")
    return lines[i].split(":", 1)[1].strip(), i + 1


def _number(token: str, is_complex: bool) -> complex | float:
    if is_complex:
        re_part, im_part = token.split(",")
        return complex(float(re_part), float(im_part))
    return float(token)


def parse_ascii(text: str) -> list[Plot]:
    lines = text.splitlines()
    plots: list[Plot] = []
    i = 0
    while i < len(lines):
        if not lines[i].startswith("Title:"):
            if lines[i].strip() == "":
                i += 1
                continue
            raise RawfileError(f"line {i + 1}: expected 'Title:', got {lines[i][:40]!r}")
        name, i = _header(lines, i, "Plotname:")
        flags, i = _header(lines, i, "Flags:")
        nvars, i = _header(lines, i, "No. Variables:")
        npoints, i = _header(lines, i, "No. Points:")
        _, i = _header(lines, i, "Variables:")
        is_complex = "complex" in flags
        nvars_i, npoints_i = int(nvars), int(npoints)
        variables = []
        for _ in range(nvars_i):
            parts = lines[i].split()
            variables.append(parts[1])
            i += 1
        if not lines[i].startswith("Values:"):
            raise RawfileError(f"line {i + 1}: expected 'Values:'")
        i += 1
        tokens: list[str] = []
        needed = nvars_i * npoints_i
        while len(tokens) < needed * 2 and i < len(lines) and not lines[i].startswith("Title:"):
            tokens.extend(lines[i].split())
            i += 1
        # every point starts with its index followed by nvars values
        values = []
        j = 0
        for point in range(npoints_i):
            if j >= len(tokens):
                raise RawfileError(f"{name}: expected {npoints_i} points, got {point}")
            j += 1  # point index
            row = tokens[j:j + nvars_i]
            if len(row) < nvars_i:
                raise RawfileError(f"{name}: expected {npoints_i} points, got {point}")
            values.append([_number(t, is_complex) for t in row])
            j += nvars_i
        arr = np.array(values, dtype=complex if is_complex else float)
        plots.append(Plot(name, variables, {v: arr[:, k] for k, v in enumerate(variables)}))
    if not plots:
        raise RawfileError("no plots found")
    return plots
```

- [ ] **Step 4: Run the tests to verify they pass**

Run: `cd hardware && python3 -m pytest tests/test_units.py tests/test_rawfile.py -q`
Expected: `20 passed`.

- [ ] **Step 5: Commit**

```bash
git add hardware/neohw/units.py hardware/neohw/rawfile.py hardware/tests/test_units.py hardware/tests/test_rawfile.py
git commit -m "feat(hw): SI value helpers and ngspice ASCII rawfile parser"
```

---

### Task 3: ngspice runner

**Files:**
- Create: `hardware/neohw/ngspice.py`
- Test: `hardware/tests/test_ngspice.py`

**Interfaces:**
- Consumes: `tools.tool_path("ngspice")`, `rawfile.parse_ascii`.
- Produces:
  - `ngspice.NgspiceError(RuntimeError)`, whose message includes the deck path and the last 20 log lines.
  - `ngspice.run_deck(deck: str, rawfiles: list[str], keep_dir: Path | None = None) -> dict[str, list[Plot]]`. It writes `deck.cir` into a temp dir (or `keep_dir`), runs `ngspice -b deck.cir`, then parses each named rawfile. It raises `NgspiceError` when ngspice exits non-zero, the log contains `Error`, or a rawfile is missing.

- [ ] **Step 1: Write the failing tests**

`hardware/tests/test_ngspice.py`:

```python
import numpy as np
import pytest

from neohw.ngspice import NgspiceError, run_deck
from neohw.tools import ToolNotFoundError, tool_path


def _need_ngspice():
    try:
        tool_path("ngspice")
    except ToolNotFoundError as e:
        pytest.skip(str(e))


RC = """rc
V1 in 0 dc 0 ac 1
R1 in out 1k
C1 out 0 159.155n
.control
set filetype=ascii
ac dec 10 100 10k
write ac.raw all
.endc
.end
"""


@pytest.mark.integration
def test_rc_lowpass_is_3db_down_at_corner():
    _need_ngspice()
    plots = run_deck(RC, ["ac.raw"])
    (ac,) = plots["ac.raw"]
    f = ac.data["frequency"].real
    gain = np.abs(ac.data["v(out)"])
    k = np.argmin(np.abs(f - 1000))
    assert 20 * np.log10(gain[k]) == pytest.approx(-3.01, abs=0.05)


@pytest.mark.integration
def test_failed_simulation_reports_log_tail(tmp_path):
    _need_ngspice()
    bad = RC.replace("R1 in out 1k", "R1 in out")
    with pytest.raises(NgspiceError) as err:
        run_deck(bad, ["ac.raw"], keep_dir=tmp_path)
    assert "deck.cir" in str(err.value)
    assert "log" in str(err.value).lower()


@pytest.mark.integration
def test_missing_rawfile_is_error():
    _need_ngspice()
    with pytest.raises(NgspiceError, match="nothere.raw"):
        run_deck(RC, ["nothere.raw"])
```

- [ ] **Step 2: Run the tests to verify they fail**

Run: `cd hardware && python3 -m pytest tests/test_ngspice.py -q`
Expected: FAIL with `ModuleNotFoundError: No module named 'neohw.ngspice'`.

- [ ] **Step 3: Implement `hardware/neohw/ngspice.py`**

```python
"""Run ngspice in batch mode and collect rawfiles."""
from __future__ import annotations

import subprocess
import tempfile
from pathlib import Path

from neohw.rawfile import Plot, RawfileError, parse_ascii
from neohw.tools import tool_path


class NgspiceError(RuntimeError):
    """ngspice failed or produced unusable output."""


def _fail(reason: str, deck_path: Path, log: str) -> NgspiceError:
    tail = "\n".join(log.strip().splitlines()[-20:])
    return NgspiceError(f"{reason} (deck: {deck_path})\n--- ngspice log tail ---\n{tail}")


def run_deck(deck: str, rawfiles: list[str], keep_dir: Path | None = None) -> dict[str, list[Plot]]:
    ngspice = tool_path("ngspice")
    tmp = None
    if keep_dir is None:
        tmp = tempfile.TemporaryDirectory(prefix="neohw-")
        workdir = Path(tmp.name)
    else:
        workdir = Path(keep_dir)
        workdir.mkdir(parents=True, exist_ok=True)
    try:
        deck_path = workdir / "deck.cir"
        deck_path.write_text(deck)
        proc = subprocess.run([str(ngspice), "-b", "deck.cir"], cwd=workdir, capture_output=True, text=True)
        log = proc.stdout + proc.stderr
        (workdir / "ngspice.log").write_text(log)
        if proc.returncode != 0 or "Error" in log or "error:" in log.lower():
            raise _fail(f"ngspice failed (exit {proc.returncode})", deck_path, log)
        results: dict[str, list[Plot]] = {}
        for name in rawfiles:
            path = workdir / name
            if not path.exists():
                raise _fail(f"ngspice did not write {name}", deck_path, log)
            try:
                results[name] = parse_ascii(path.read_text())
            except RawfileError as e:
                raise _fail(f"could not parse {name}: {e}", deck_path, log) from e
        return results
    finally:
        if tmp is not None:
            tmp.cleanup()
```

- [ ] **Step 4: Run the tests to verify they pass**

Run: `cd hardware && python3 -m pytest tests/test_ngspice.py -q`
Expected: `3 passed`. If `test_failed_simulation_reports_log_tail` fails because ngspice exits 0 without the word "Error", read `tmp_path/ngspice.log`. Then extend the failure detection in `run_deck` to match the actual message (keep the test unchanged).

- [ ] **Step 5: Commit**

```bash
git add hardware/neohw/ngspice.py hardware/tests/test_ngspice.py
git commit -m "feat(hw): ngspice batch runner with log-tail errors"
```

---

### Task 4: Part layer (SKiDL templates with SPICE models)

**Files:**
- Create: `hardware/neohw/parts.py`
- Test: `hardware/tests/test_parts.py`

**Interfaces:**
- Consumes: `units.parse_si`, `units.format_si`.
- Produces:
  - `parts.resistor(value, footprint="Resistor_SMD:R_0805_2012Metric", mpn=None, torob=None) -> skidl.Part`. Pins 1, 2. SPICE line `R<ref> n1 n2 <ohms>`.
  - `parts.capacitor(value, footprint="Capacitor_SMD:C_0805_2012Metric", mpn=None, torob=None, esr=0.0) -> Part`. SPICE line `C<ref> n1 n2 <farads>`. When `esr > 0`, it emits a series R and an internal node `<ref>_esr`.
  - `parts.relay_dpdt(state: str, contact_r=0.05, open_c=0.5e-12, footprint="Relay_SMD:Relay_DPDT_Omron_G6K-2F", mpn="G6K-2F-5VDC", torob="G6K-2F") -> Part`.
    - Pins: `COM1, NC1, NO1, COM2, NC2, NO2, COIL+, COIL-`.
    - `state` is `"released"` (COM–NC closed) or `"energised"` (COM–NO closed).
    - SPICE: a closed contact is a resistor `contact_r`; an open contact is a capacitor `open_c`. Coil pins emit nothing.
  - Every part carries the attributes `mpn: str | None`, `torob: str | None` and `spice_lines: Callable[[dict[str, str]], list[str]]`. The callable receives a pin-name → SPICE-node map and returns element lines. The element name is the part ref with a type letter prefix.
  - Raises `ValueError` for an unknown relay state or a non-positive value.

- [ ] **Step 1: Write the failing tests**

`hardware/tests/test_parts.py`:

```python
import pytest
from skidl import Circuit, Net

from neohw import parts


def test_resistor_carries_metadata_and_spice():
    ckt = Circuit()
    with ckt:
        r = parts.resistor("61.9", mpn="RC0805FR-0761R9L", torob="0805 resistor")
        r[1] += Net("a")
        r[2] += Net("GND")
    assert r.value == "61.9Ω"
    assert r.footprint == "Resistor_SMD:R_0805_2012Metric"
    assert r.mpn == "RC0805FR-0761R9L"
    assert r.spice_lines({"1": "a", "2": "0"}) == [f"R{r.ref} a 0 61.9"]


def test_capacitor_with_esr_adds_internal_node():
    ckt = Circuit()
    with ckt:
        c = parts.capacitor("33n", esr=0.1)
    lines = c.spice_lines({"1": "x", "2": "0"})
    assert lines == [f"R{c.ref}_esr x {c.ref}_esr 0.1", f"C{c.ref} {c.ref}_esr 0 3.3e-08"]


@pytest.mark.parametrize("state, closed, open_", [
    ("released", ("COM1", "NC1"), ("COM1", "NO1")),
    ("energised", ("COM1", "NO1"), ("COM1", "NC1")),
])
def test_relay_contacts_follow_state(state, closed, open_):
    ckt = Circuit()
    with ckt:
        k = parts.relay_dpdt(state)
    nodes = {p: p.lower() for p in ["COM1", "NC1", "NO1", "COM2", "NC2", "NO2", "COIL+", "COIL-"]}
    lines = k.spice_lines(nodes)
    assert f"R{k.ref}_{closed[1]} {closed[0].lower()} {closed[1].lower()} 0.05" in lines
    assert f"C{k.ref}_{open_[1]} {open_[0].lower()} {open_[1].lower()} 5e-13" in lines
    assert len(lines) == 4  # two contacts per pole, coil emits nothing


def test_relay_rejects_unknown_state():
    with pytest.raises(ValueError, match="state"):
        parts.relay_dpdt("half")


def test_non_positive_value_rejected():
    with pytest.raises(ValueError):
        parts.resistor("0")
```

- [ ] **Step 2: Run the tests to verify they fail**

Run: `cd hardware && python3 -m pytest tests/test_parts.py -q`
Expected: FAIL with `ImportError: cannot import name 'parts'`.

- [ ] **Step 3: Implement `hardware/neohw/parts.py`**

```python
"""Part constructors: SKiDL parts that also know their SPICE model."""
from __future__ import annotations

from skidl import SKIDL, TEMPLATE, Part, Pin

from neohw.units import format_si, parse_si


def _template(name: str, prefix: str, pins: list[str], footprint: str) -> Part:
    return Part(tool=SKIDL, name=name, ref_prefix=prefix, dest=TEMPLATE, footprint=footprint,
                pins=[Pin(num=p, name=p) for p in pins])


def _positive(value) -> float:
    v = parse_si(value)
    if v <= 0:
        raise ValueError(f"value must be positive, got {value!r}")
    return v


def _num(v: float) -> str:
    return f"{v:.6g}"


def resistor(value, footprint: str = "Resistor_SMD:R_0805_2012Metric", mpn: str | None = None,
             torob: str | None = None) -> Part:
    ohms = _positive(value)
    p = _template("R", "R", ["1", "2"], footprint)(value=format_si(ohms, "Ω"))
    p.mpn, p.torob = mpn, torob
    p.spice_lines = lambda n: [f"R{p.ref} {n['1']} {n['2']} {_num(ohms)}"]
    return p


def capacitor(value, footprint: str = "Capacitor_SMD:C_0805_2012Metric", mpn: str | None = None,
              torob: str | None = None, esr: float = 0.0) -> Part:
    farads = _positive(value)
    p = _template("C", "C", ["1", "2"], footprint)(value=format_si(farads, "F"))
    p.mpn, p.torob = mpn, torob

    def lines(n):
        if esr > 0:
            mid = f"{p.ref}_esr"
            return [f"R{p.ref}_esr {n['1']} {mid} {_num(esr)}", f"C{p.ref} {mid} {n['2']} {_num(farads)}"]
        return [f"C{p.ref} {n['1']} {n['2']} {_num(farads)}"]

    p.spice_lines = lines
    return p


RELAY_PINS = ["COM1", "NC1", "NO1", "COM2", "NC2", "NO2", "COIL+", "COIL-"]


def relay_dpdt(state: str, contact_r: float = 0.05, open_c: float = 0.5e-12,
               footprint: str = "Relay_SMD:Relay_DPDT_Omron_G6K-2F", mpn: str = "G6K-2F-5VDC",
               torob: str = "G6K-2F") -> Part:
    if state not in ("released", "energised"):
        raise ValueError(f"relay state must be 'released' or 'energised', got {state!r}")
    p = _template("G6K-2F", "K", RELAY_PINS, footprint)(value="G6K-2F")
    p.mpn, p.torob, p.state = mpn, torob, state

    def lines(n):
        out = []
        for pole in ("1", "2"):
            com = n[f"COM{pole}"]
            closed, open_ = ("NC", "NO") if state == "released" else ("NO", "NC")
            out.append(f"R{p.ref}_{closed}{pole} {com} {n[closed + pole]} {_num(contact_r)}")
            out.append(f"C{p.ref}_{open_}{pole} {com} {n[open_ + pole]} {_num(open_c)}")
        return out

    p.spice_lines = lines
    return p
```

- [ ] **Step 4: Run the tests to verify they pass**

Run: `cd hardware && python3 -m pytest tests/test_parts.py -q`
Expected: `6 passed`. If SKiDL rejects assigning the custom attributes (`mpn`, `torob`, `spice_lines`, `state`), note it in the ledger and store them in a dict attribute `p.neohw = {...}` instead. Keep the test expectations (adapt the attribute access in the tests only if the API forces it).

- [ ] **Step 5: Commit**

```bash
git add hardware/neohw/parts.py hardware/tests/test_parts.py
git commit -m "feat(hw): SKiDL part layer with SPICE models (R, C, DPDT relay)"
```

---

### Task 5: SPICE writer and `simulate()`

**Files:**
- Create: `hardware/neohw/spice.py`
- Test: `hardware/tests/test_spice.py`

**Interfaces:**
- Consumes: `ngspice.run_deck`; parts with a `spice_lines` attribute.
- Produces:
  - `spice.Port(net: skidl.Net, number: int, z0: float = 50.0)`
  - `spice.Ac(start_hz, stop_hz, points_per_decade=20)` → `ac dec …`
  - `spice.Sp(start_hz, stop_hz, points=201, sweep="lin")` → `sp lin …`
  - `spice.Op()` → `op`
  - `spice.node_name(net) -> str`: `GND` → `0`; other names are lower-cased with any character outside `[a-z0-9_]` replaced by `_`.
  - `spice.to_deck(circuit, title: str, ports: list[Port], analyses: list, extra_lines: list[str] = ()) -> tuple[str, list[str]]` returns the deck text and the rawfile names (`a0.raw`, `a1.raw`, …, one per analysis, in order).
    - Port 1 drives `ac 1`; the other ports drive `ac 0`.
    - It raises `SpiceError(ValueError)` if any part pin is unconnected (unless the pin name starts with `COIL`), or if a part has no `spice_lines`.
  - `spice.simulate(circuit, title, ports, analyses, keep_dir=None) -> list[Plot]` returns one plot per analysis, in order.

- [ ] **Step 1: Write the failing tests**

`hardware/tests/test_spice.py`:

```python
import numpy as np
import pytest
from skidl import Circuit, Net

from neohw import parts
from neohw.spice import Ac, Op, Port, SpiceError, Sp, node_name, simulate, to_deck
from neohw.tools import ToolNotFoundError, tool_path


def divider():
    ckt = Circuit()
    with ckt:
        a, mid, gnd = Net("RF in"), Net("mid"), Net("GND")
        r1 = parts.resistor("1k"); r1[1] += a; r1[2] += mid
        r2 = parts.resistor("2k"); r2[1] += mid; r2[2] += gnd
    divider.gnd = gnd
    return ckt, a, mid


def test_node_names():
    ckt, a, mid = divider()
    assert node_name(a) == "rf_in"
    assert node_name(divider.gnd) == "0"


def test_deck_contains_parts_ports_and_analyses():
    ckt, a, mid = divider()
    deck, raws = to_deck(ckt, "div", [Port(a, 1), Port(mid, 2)], [Op(), Ac(1e3, 1e6)])
    assert "R" in deck and " rf_in mid 1000" in deck
    assert "portnum 1 z0 50" in deck and "dc 0 ac 1 portnum 1" in deck
    assert "dc 0 ac 0 portnum 2" in deck
    assert "\nop\nwrite a0.raw all" in deck and "ac dec 20 1000 1e+06\nwrite a1.raw all" in deck
    assert raws == ["a0.raw", "a1.raw"]


def test_unconnected_pin_is_rejected():
    ckt = Circuit()
    with ckt:
        r = parts.resistor("1k")
        r[1] += Net("a")
    with pytest.raises(SpiceError, match="unconnected"):
        to_deck(ckt, "x", [], [Op()])


def test_relay_coil_pins_may_be_unconnected():
    ckt = Circuit()
    with ckt:
        k = parts.relay_dpdt("released")
        for pin in ["COM1", "NC1", "NO1", "COM2", "NC2", "NO2"]:
            k[pin] += Net(pin.lower())
    deck, _ = to_deck(ckt, "k", [], [Op()])
    assert "com1 nc1 0.05" in deck


@pytest.mark.integration
def test_simulated_divider_s21_matches_theory():
    try:
        tool_path("ngspice")
    except ToolNotFoundError as e:
        pytest.skip(str(e))
    ckt, a, mid = divider()
    (sp,) = simulate(ckt, "div", [Port(a, 1), Port(mid, 2)], [Sp(1e6, 2e6, points=3)])
    # 50 Ω source, 1k series, 2k || 50 Ω load
    zl = 1 / (1 / 2000 + 1 / 50)
    s21 = 2 * zl / (50 + 1000 + zl)
    np.testing.assert_allclose(np.abs(sp.data["s_2_1"]), s21, rtol=1e-3)
```

- [ ] **Step 2: Run the tests to verify they fail**

Run: `cd hardware && python3 -m pytest tests/test_spice.py -q`
Expected: FAIL with `ModuleNotFoundError: No module named 'neohw.spice'`.

- [ ] **Step 3: Implement `hardware/neohw/spice.py`**

```python
"""Turn a SKiDL circuit into an ngspice deck and simulate it."""
from __future__ import annotations

import re
from dataclasses import dataclass
from pathlib import Path

from neohw.ngspice import run_deck
from neohw.rawfile import Plot


class SpiceError(ValueError):
    """The circuit cannot be turned into a valid deck."""


@dataclass(frozen=True)
class Port:
    net: object
    number: int
    z0: float = 50.0


@dataclass(frozen=True)
class Ac:
    start_hz: float
    stop_hz: float
    points_per_decade: int = 20

    def command(self) -> str:
        return f"ac dec {self.points_per_decade} {self.start_hz:g} {self.stop_hz:g}"


@dataclass(frozen=True)
class Sp:
    start_hz: float
    stop_hz: float
    points: int = 201
    sweep: str = "lin"

    def command(self) -> str:
        return f"sp {self.sweep} {self.points} {self.start_hz:g} {self.stop_hz:g}"


@dataclass(frozen=True)
class Op:
    def command(self) -> str:
        return "op"


def node_name(net) -> str:
    if net.name == "GND":
        return "0"
    return re.sub(r"[^a-z0-9_]", "_", net.name.lower())


def to_deck(circuit, title: str, ports: list[Port], analyses: list, extra_lines=()) -> tuple[str, list[str]]:
    lines = [title]
    for part in circuit.parts:
        if not hasattr(part, "spice_lines"):
            raise SpiceError(f"{part.ref} has no SPICE model")
        nodes = {}
        for pin in part.pins:
            if pin.net is None or not pin.is_connected():
                if pin.name.upper().startswith("COIL"):
                    continue
                raise SpiceError(f"{part.ref} pin {pin.name} is unconnected")
            nodes[pin.name] = node_name(pin.net)
        lines.extend(part.spice_lines(nodes))
    for port in ports:
        drive = 1 if port.number == 1 else 0
        lines.append(f"VPORT{port.number} {node_name(port.net)} 0 dc 0 ac {drive} "
                     f"portnum {port.number} z0 {port.z0:g}")
    lines.extend(extra_lines)
    raws = []
    lines += [".control", "set filetype=ascii"]
    for k, analysis in enumerate(analyses):
        raw = f"a{k}.raw"
        lines += [analysis.command(), f"write {raw} all"]
        raws.append(raw)
    lines += [".endc", ".end", ""]
    return "\n".join(lines), raws


def simulate(circuit, title: str, ports: list[Port], analyses: list, keep_dir: Path | None = None) -> list[Plot]:
    deck, raws = to_deck(circuit, title, ports, analyses)
    results = run_deck(deck, raws, keep_dir=keep_dir)
    return [results[r][0] for r in raws]
```

- [ ] **Step 4: Run the tests to verify they pass**

Run: `cd hardware && python3 -m pytest tests/test_spice.py -q`
Expected: `5 passed`. If SKiDL's `Pin` has no `is_connected()`, use `pin.net is None` alone and record a ledger ruling.

- [ ] **Step 5: Commit**

```bash
git add hardware/neohw/spice.py hardware/tests/test_spice.py
git commit -m "feat(hw): SPICE deck writer with ports and analyses"
```

---

### Task 6: Exports (KiCad netlist, BOM with availability, SVG)

**Files:**
- Create: `hardware/neohw/availability.py`, `hardware/neohw/export.py`
- Test: `hardware/tests/test_availability.py`, `hardware/tests/test_export.py`

**Interfaces:**
- Consumes: parts' `mpn`, `torob`, `value`, `footprint`; `tools.tool_path("dot")`.
- Produces:
  - `availability.lookup(key: str | None, data: dict) -> str`, returning `"available"` (at least one in-stock hit), `"out_of_stock"` (hits exist but none in stock), or `"unknown"` (no key, key not in data, or no hits).
  - `availability.load(path=DEFAULT_PATH) -> dict`.
  - `export.kicad_netlist(circuit, path: Path) -> Path`: runs `circuit.generate_netlist(tool=KICAD9, file_=…)` inside a temp working directory, so no SKiDL clutter files are left behind.
  - `export.bom_rows(circuit, data) -> list[dict]` with keys `refs, value, footprint, mpn, qty, availability`. Rows are grouped by (value, footprint, mpn).
  - `export.write_bom(circuit, path, data) -> Path` (CSV).
  - `export.svg_diagram(circuit, path: Path) -> Path`: Graphviz graph where parts are boxes labelled `ref\nvalue`, nets are points, and an edge joins each part to its pin's net. GND is drawn once.

- [ ] **Step 1: Write the failing tests**

`hardware/tests/test_availability.py`:

```python
from neohw.availability import lookup

DATA = {
    "G6K-2F": {"ok": True, "hits": [{"name": "G6K-2F", "price": "x", "shop": "y", "instock": True}]},
    "RD16HHF1": {"ok": True, "hits": [{"name": "RD16HHF1", "price": "", "shop": "y", "instock": False}]},
    "nothing": {"ok": True, "hits": []},
}


def test_bom_flags_unknown_and_out_of_stock():
    assert lookup("G6K-2F", DATA) == "available"
    assert lookup("RD16HHF1", DATA) == "out_of_stock"
    assert lookup("nothing", DATA) == "unknown"
    assert lookup("never-searched", DATA) == "unknown"
    assert lookup(None, DATA) == "unknown"
```

`hardware/tests/test_export.py`:

```python
import csv
import os

import pytest
from skidl import Circuit, Net

from neohw import export, parts
from neohw.tools import ToolNotFoundError, tool_path


def small():
    ckt = Circuit()
    with ckt:
        a, gnd = Net("a"), Net("GND")
        for _ in range(2):
            r = parts.resistor("61.9", torob="G6K-2F")
            r[1] += a
            r[2] += gnd
        c = parts.capacitor("10n")
        c[1] += a
        c[2] += gnd
    return ckt


def test_bom_groups_identical_parts():
    rows = export.bom_rows(small(), {"G6K-2F": {"ok": True, "hits": [{"instock": True}]}})
    by_value = {r["value"]: r for r in rows}
    assert by_value["61.9Ω"]["qty"] == 2
    assert by_value["61.9Ω"]["availability"] == "available"
    assert by_value["10nF"]["availability"] == "unknown"


def test_write_bom_csv(tmp_path):
    path = export.write_bom(small(), tmp_path / "bom.csv", {})
    rows = list(csv.DictReader(path.open()))
    assert {r["value"] for r in rows} == {"61.9Ω", "10nF"}


def test_kicad_netlist_leaves_no_clutter(tmp_path, monkeypatch):
    monkeypatch.chdir(tmp_path)
    out = export.kicad_netlist(small(), tmp_path / "out" / "block.net")
    text = out.read_text()
    assert "(export" in text and "Resistor_SMD:R_0805_2012Metric" in text
    assert sorted(os.listdir(tmp_path)) == ["out"]


@pytest.mark.integration
def test_svg_diagram(tmp_path):
    try:
        tool_path("dot")
    except ToolNotFoundError as e:
        pytest.skip(str(e))
    svg = export.svg_diagram(small(), tmp_path / "d.svg").read_text()
    assert svg.lstrip().startswith("<?xml") and "61.9Ω" in svg
```

- [ ] **Step 2: Run the tests to verify they fail**

Run: `cd hardware && python3 -m pytest tests/test_availability.py tests/test_export.py -q`
Expected: FAIL with `ModuleNotFoundError`.

- [ ] **Step 3: Implement**

`hardware/neohw/availability.py`:

```python
"""Iranian-market availability from the Torob survey."""
from __future__ import annotations

import json
from pathlib import Path

from neohw.tools import REPO_ROOT

DEFAULT_PATH = REPO_ROOT / "docs" / "superpowers" / "specs" / "2026-10-12-parts-availability-torob.json"


def load(path: Path = DEFAULT_PATH) -> dict:
    return json.loads(Path(path).read_text(encoding="utf-8"))


def lookup(key: str | None, data: dict) -> str:
    if not key or key not in data:
        return "unknown"
    hits = data[key].get("hits") or []
    if not hits:
        return "unknown"
    return "available" if any(h.get("instock") for h in hits) else "out_of_stock"
```

`hardware/neohw/export.py`:

```python
"""KiCad netlist, BOM and diagram exports for a SKiDL circuit."""
from __future__ import annotations

import contextlib
import csv
import html
import subprocess
import tempfile
from pathlib import Path

from skidl import KICAD9

from neohw.availability import lookup
from neohw.tools import tool_path


def kicad_netlist(circuit, path: Path) -> Path:
    path = Path(path).resolve()
    path.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix="neohw-skidl-") as tmp, contextlib.chdir(tmp):
        circuit.generate_netlist(tool=KICAD9, file_=str(path))
    return path


def bom_rows(circuit, data: dict) -> list[dict]:
    groups: dict[tuple, dict] = {}
    for part in circuit.parts:
        mpn = getattr(part, "mpn", None) or ""
        key = (part.value, part.footprint, mpn)
        row = groups.setdefault(key, {"refs": [], "value": part.value, "footprint": part.footprint,
                                      "mpn": mpn, "qty": 0,
                                      "availability": lookup(getattr(part, "torob", None), data)})
        row["refs"].append(part.ref)
        row["qty"] += 1
    rows = list(groups.values())
    for row in rows:
        row["refs"] = " ".join(sorted(row["refs"]))
    return sorted(rows, key=lambda r: r["refs"])


def write_bom(circuit, path: Path, data: dict) -> Path:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    rows = bom_rows(circuit, data)
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=["refs", "value", "footprint", "mpn", "qty", "availability"])
        writer.writeheader()
        writer.writerows(rows)
    return path


def svg_diagram(circuit, path: Path) -> Path:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    out = ["graph G {", "  rankdir=LR; node [fontname=Helvetica fontsize=10];"]
    for part in circuit.parts:
        label = html.escape(f"{part.ref}\n{part.value}")
        out.append(f'  "{part.ref}" [shape=box label="{label}"];')
    for net in circuit.get_nets():
        out.append(f'  "net:{net.name}" [shape=point xlabel="{html.escape(net.name)}"];')
    for part in circuit.parts:
        for pin in part.pins:
            if pin.net is not None:
                out.append(f'  "{part.ref}" -- "net:{pin.net.name}" [taillabel="{html.escape(pin.name)}" fontsize=8];')
    out.append("}")
    dot = tool_path("dot")
    proc = subprocess.run([str(dot), "-Tsvg", "-o", str(path)], input="\n".join(out), text=True,
                          capture_output=True)
    if proc.returncode != 0:
        raise RuntimeError(f"dot failed: {proc.stderr.strip()}")
    return path
```

- [ ] **Step 4: Run the tests to verify they pass**

Run: `cd hardware && python3 -m pytest tests/test_availability.py tests/test_export.py -q`
Expected: `5 passed`.

- [ ] **Step 5: Commit**

```bash
git add hardware/neohw/availability.py hardware/neohw/export.py hardware/tests/test_availability.py hardware/tests/test_export.py
git commit -m "feat(hw): KiCad netlist, BOM with Iranian availability, SVG diagram"
```

---

### Task 7: Attenuator block, check and characterisation file

**Files:**
- Create: `hardware/neohw/blocks/__init__.py`, `hardware/neohw/blocks/attenuator.py`, `hardware/neohw/checks/__init__.py`, `hardware/neohw/checks/attenuator.py`, `hardware/neohw/charfile.py`
- Test: `hardware/tests/test_charfile.py`, `hardware/tests/test_attenuator.py`

**Interfaces:**
- Consumes: `parts.resistor`, `parts.relay_dpdt`, `spice.simulate`, `Port`, `Sp`, `tools.git_describe`.
- Produces:
  - `blocks.attenuator.build(state: str) -> Block`, where `Block` is a dataclass `(name: str, circuit, ports: dict[str, Net])` with ports `rf_in` and `rf_out`.
    - Topology: input → K1 pole 1 (COM1 = rf_in) → NC1 = bypass trace → NC2 → K1 pole 2 COM2 = rf_out. The pad path runs NO1 → π-pad → NO2.
    - π-pad: 61.9 Ω shunt at each side to GND, 249 Ω series.
    - `state="released"` → bypass; `"energised"` → pad.
  - `checks.CheckResult(name: str, value: float, limit: str, passed: bool, unit: str)`
  - `checks.BlockResult(block: str, checks: list[CheckResult], plots: dict[str, tuple[np.ndarray, np.ndarray]], artefacts: dict[str, Path])` with property `passed`.
  - `checks.attenuator.run(out_dir: Path, char_dir: Path) -> BlockResult`
    - Simulates both states with `Sp(0.1e6, 54e6, 201, "lin")`.
    - Checks: attenuation min and max (20 ± 0.5 dB), worst return loss in pad state (≥ 18 dB), worst bypass loss (≤ 0.2 dB).
    - Writes the KiCad netlist (pad state), BOM, SVG and `char_dir/attenuator.json`.
  - `charfile.write(path, block: str, status: str, payload: dict) -> Path` adds `schema_version: 1` and `generated_by: git_describe()`.
  - `charfile.read(path, expected_block: str) -> dict` raises `ValueError` on an unsupported `schema_version` or a block mismatch.

- [ ] **Step 1: Write the failing tests**

`hardware/tests/test_charfile.py`:

```python
import json

import pytest

from neohw import charfile


def test_write_and_read_round_trip(tmp_path):
    p = charfile.write(tmp_path / "a.json", "attenuator", "pass", {"freq_hz": [1.0]})
    doc = json.loads(p.read_text())
    assert doc["schema_version"] == 1 and doc["block"] == "attenuator" and doc["status"] == "pass"
    assert doc["generated_by"]
    assert charfile.read(p, "attenuator")["freq_hz"] == [1.0]


def test_read_rejects_wrong_block_or_schema(tmp_path):
    p = charfile.write(tmp_path / "a.json", "attenuator", "pass", {})
    with pytest.raises(ValueError, match="block"):
        charfile.read(p, "preamp")
    doc = json.loads(p.read_text())
    doc["schema_version"] = 99
    p.write_text(json.dumps(doc))
    with pytest.raises(ValueError, match="schema_version"):
        charfile.read(p, "attenuator")


def test_status_must_be_pass_or_fail(tmp_path):
    with pytest.raises(ValueError):
        charfile.write(tmp_path / "a.json", "attenuator", "maybe", {})
```

`hardware/tests/test_attenuator.py`:

```python
import json

import pytest

from neohw.blocks import attenuator
from neohw.checks import attenuator as check
from neohw.spice import to_deck, Op, Port
from neohw.tools import ToolNotFoundError, tool_path


def test_block_has_ports_and_expected_parts():
    blk = attenuator.build("energised")
    assert set(blk.ports) == {"rf_in", "rf_out"}
    values = sorted(p.value for p in blk.circuit.parts)
    assert values == ["249Ω", "61.9Ω", "61.9Ω", "G6K-2F"]


def test_released_state_closes_bypass_contacts():
    blk = attenuator.build("released")
    deck, _ = to_deck(blk.circuit, "t", [Port(blk.ports["rf_in"], 1), Port(blk.ports["rf_out"], 2)], [Op()])
    assert "_NC1 rf_in" in deck and " 0.05" in deck


@pytest.mark.integration
def test_attenuator_check_passes_and_writes_artefacts(tmp_path):
    try:
        tool_path("ngspice"), tool_path("dot")
    except ToolNotFoundError as e:
        pytest.skip(str(e))
    res = check.run(tmp_path / "out", tmp_path / "char")
    details = {c.name: (c.value, c.limit, c.passed) for c in res.checks}
    assert res.passed, details
    assert {c.name for c in res.checks} == {"attenuation_min_db", "attenuation_max_db",
                                            "pad_return_loss_db", "bypass_loss_db"}
    doc = json.loads((tmp_path / "char" / "attenuator.json").read_text())
    assert doc["status"] == "pass" and len(doc["freq_hz"]) == 201
    for key in ("kicad_netlist", "bom", "svg"):
        assert res.artefacts[key].exists()
```

- [ ] **Step 2: Run the tests to verify they fail**

Run: `cd hardware && python3 -m pytest tests/test_charfile.py tests/test_attenuator.py -q`
Expected: FAIL with `ModuleNotFoundError`.

- [ ] **Step 3: Implement**

`hardware/neohw/charfile.py`:

```python
"""Characterisation files: the contract between neohw and the digital twin."""
from __future__ import annotations

import json
from pathlib import Path

from neohw.tools import git_describe

SCHEMA_VERSION = 1


def write(path: Path, block: str, status: str, payload: dict) -> Path:
    if status not in ("pass", "fail"):
        raise ValueError(f"status must be 'pass' or 'fail', got {status!r}")
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    doc = {"schema_version": SCHEMA_VERSION, "block": block, "status": status,
           "generated_by": git_describe(), **payload}
    path.write_text(json.dumps(doc, indent=1))
    return path


def read(path: Path, expected_block: str) -> dict:
    doc = json.loads(Path(path).read_text())
    if doc.get("schema_version") != SCHEMA_VERSION:
        raise ValueError(f"{path}: unsupported schema_version {doc.get('schema_version')}, expected {SCHEMA_VERSION}")
    if doc.get("block") != expected_block:
        raise ValueError(f"{path}: block is {doc.get('block')!r}, expected {expected_block!r}")
    return doc
```

`hardware/neohw/blocks/__init__.py`:

```python
"""Circuit blocks: functions that build a SKiDL circuit with named ports."""
from __future__ import annotations

from dataclasses import dataclass, field


@dataclass
class Block:
    name: str
    circuit: object
    ports: dict = field(default_factory=dict)
```

`hardware/neohw/blocks/attenuator.py`:

```python
"""20 dB π-pad switched by a G6K-2F DPDT relay (spec §4)."""
from __future__ import annotations

from skidl import Circuit, Net

from neohw import parts
from neohw.blocks import Block

SHUNT_OHMS = "61.9"   # E96, ideal 61.11 Ω for 20 dB at 50 Ω
SERIES_OHMS = "249"   # E96, ideal 247.5 Ω


def build(state: str) -> Block:
    ckt = Circuit()
    with ckt:
        rf_in, rf_out, gnd = Net("rf_in"), Net("rf_out"), Net("GND")
        bypass, pad_a, pad_b = Net("bypass"), Net("pad_a"), Net("pad_b")
        k = parts.relay_dpdt(state)
        k["COM1"] += rf_in
        k["COM2"] += rf_out
        k["NC1"] += bypass
        k["NC2"] += bypass
        k["NO1"] += pad_a
        k["NO2"] += pad_b
        r_a = parts.resistor(SHUNT_OHMS, mpn="RC0805FR-0761R9L")
        r_a[1] += pad_a
        r_a[2] += gnd
        r_s = parts.resistor(SERIES_OHMS, mpn="RC0805FR-07249RL")
        r_s[1] += pad_a
        r_s[2] += pad_b
        r_b = parts.resistor(SHUNT_OHMS, mpn="RC0805FR-0761R9L")
        r_b[1] += pad_b
        r_b[2] += gnd
    return Block("attenuator", ckt, {"rf_in": rf_in, "rf_out": rf_out})
```

`hardware/neohw/checks/__init__.py`:

```python
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

    @property
    def passed(self) -> bool:
        return all(c.passed for c in self.checks)


def db(x) -> np.ndarray:
    return 20 * np.log10(np.maximum(np.abs(x), 1e-30))
```

`hardware/neohw/checks/attenuator.py`:

```python
"""Attenuator check (spec §4): 20 ± 0.5 dB, RL ≥ 18 dB, bypass ≤ 0.2 dB over 0.1–54 MHz."""
from __future__ import annotations

from pathlib import Path

from neohw import availability, charfile, export
from neohw.blocks import attenuator
from neohw.checks import BlockResult, CheckResult, db
from neohw.spice import Port, Sp, simulate

SWEEP = Sp(0.1e6, 54e6, 201, "lin")


def _sparams(state: str):
    blk = attenuator.build(state)
    (plot,) = simulate(blk.circuit, f"attenuator {state}",
                       [Port(blk.ports["rf_in"], 1), Port(blk.ports["rf_out"], 2)], [SWEEP])
    return blk, plot.data["frequency"].real, plot.data["s_2_1"], plot.data["s_1_1"]


def run(out_dir: Path, char_dir: Path) -> BlockResult:
    out_dir, char_dir = Path(out_dir), Path(char_dir)
    pad_blk, f, s21_pad, s11_pad = _sparams("energised")
    _, _, s21_byp, _ = _sparams("released")
    att = -db(s21_pad)
    rl = -db(s11_pad)
    byp = -db(s21_byp)
    checks = [
        CheckResult("attenuation_min_db", float(att.min()), "≥ 19.5", bool(att.min() >= 19.5)),
        CheckResult("attenuation_max_db", float(att.max()), "≤ 20.5", bool(att.max() <= 20.5)),
        CheckResult("pad_return_loss_db", float(rl.min()), "≥ 18", bool(rl.min() >= 18)),
        CheckResult("bypass_loss_db", float(byp.max()), "≤ 0.2", bool(byp.max() <= 0.2)),
    ]
    result = BlockResult("attenuator", checks,
                         plots={"pad S21 (dB)": (f, db(s21_pad)), "pad S11 (dB)": (f, db(s11_pad)),
                                "bypass S21 (dB)": (f, db(s21_byp))})
    data = availability.load()
    result.artefacts = {
        "kicad_netlist": export.kicad_netlist(pad_blk.circuit, out_dir / "attenuator.net"),
        "bom": export.write_bom(pad_blk.circuit, out_dir / "attenuator_bom.csv", data),
        "svg": export.svg_diagram(pad_blk.circuit, out_dir / "attenuator.svg"),
        "characterisation": charfile.write(
            char_dir / "attenuator.json", "attenuator", "pass" if result.passed else "fail",
            {"freq_hz": f.tolist(), "pad_db": (-att).tolist(), "bypass_db": (-byp).tolist(),
             "checks": [c.__dict__ for c in checks]}),
    }
    return result
```

- [ ] **Step 4: Run the tests to verify they pass**

Run: `cd hardware && python3 -m pytest tests/test_charfile.py tests/test_attenuator.py -q`
Expected: `6 passed`. If the attenuation target fails because the 0.5 pF open-contact model leaks, report the worst frequency. Then check the G6K-2F datasheet isolation figure and correct `open_c` with the source noted in a code comment. Do not loosen the spec targets.

- [ ] **Step 5: Commit**

```bash
git add hardware/neohw/charfile.py hardware/neohw/blocks hardware/neohw/checks hardware/tests/test_charfile.py hardware/tests/test_attenuator.py
git commit -m "feat(hw): attenuator block with S-parameter check and characterisation file"
```

---

### Task 8: Hardware report, CLI and `make hw`

**Files:**
- Create: `hardware/neohw/report.py`, `hardware/neohw/__main__.py`
- Modify: `Makefile` (add `hw`, extend `test`)
- Create (generated, committed): `hardware/characterized/attenuator.json`
- Test: `hardware/tests/test_report.py`, `hardware/tests/test_cli.py`

**Interfaces:**
- Consumes: `checks.attenuator.run`, `BlockResult`, `CheckResult`.
- Produces:
  - `report.write_report(results: list[BlockResult], path: Path) -> Path`: a self-contained HTML file. Each block gets a PASS/FAIL badge, a check table, a matplotlib PNG of its plots (base64), the inline SVG diagram and the BOM table.
  - `python -m neohw run [--out DIR] [--characterized DIR] [--report FILE]` runs all registered checks (currently `attenuator`). It exits 0 when all pass, 1 when any check fails (report still written), and 2 on a tool or ngspice error (message on stderr).
  - `neohw.__main__.main(argv) -> int`; `neohw.__main__.CHECKS: dict[str, Callable[[Path, Path], BlockResult]]`.
  - `make hw` runs it with `--out build/hw --characterized hardware/characterized --report build/hw/report.html`.

- [ ] **Step 1: Write the failing tests**

`hardware/tests/test_report.py`:

```python
import numpy as np

from neohw.checks import BlockResult, CheckResult
from neohw.report import write_report


def test_report_contains_checks_and_plot(tmp_path):
    svg = tmp_path / "d.svg"
    svg.write_text('<?xml version="1.0"?><svg xmlns="http://www.w3.org/2000/svg"><text>K1</text></svg>')
    bom = tmp_path / "b.csv"
    bom.write_text("refs,value,footprint,mpn,qty,availability\nR1,61.9Ω,fp,mpn,2,unknown\n")
    res = BlockResult("attenuator",
                      [CheckResult("attenuation_min_db", 19.9, "≥ 19.5", True),
                       CheckResult("bypass_loss_db", 0.3, "≤ 0.2", False)],
                      plots={"pad S21 (dB)": (np.array([1e6, 2e6]), np.array([-20.0, -20.1]))},
                      artefacts={"svg": svg, "bom": bom})
    html = write_report([res], tmp_path / "r.html").read_text()
    assert "attenuator" in html and "FAIL" in html and "PASS" in html
    assert "attenuation_min_db" in html and "19.9" in html
    assert "data:image/png;base64," in html and "<svg" in html and "unknown" in html
```

`hardware/tests/test_cli.py`:

```python
from neohw import __main__ as cli
from neohw.checks import BlockResult, CheckResult


def _fake(passed):
    def run(out_dir, char_dir):
        return BlockResult("fake", [CheckResult("x", 1.0, "≤ 0", passed)])
    return run


def test_cli_exit_code_reflects_failed_check(tmp_path, monkeypatch):
    monkeypatch.setattr(cli, "CHECKS", {"fake": _fake(False)})
    report = tmp_path / "r.html"
    assert cli.main(["run", "--out", str(tmp_path), "--characterized", str(tmp_path), "--report", str(report)]) == 1
    assert report.exists()


def test_cli_success(tmp_path, monkeypatch):
    monkeypatch.setattr(cli, "CHECKS", {"fake": _fake(True)})
    assert cli.main(["run", "--out", str(tmp_path), "--characterized", str(tmp_path),
                     "--report", str(tmp_path / "r.html")]) == 0


def test_cli_tool_error_is_exit_2(tmp_path, monkeypatch, capsys):
    from neohw.tools import ToolNotFoundError

    def broken(out_dir, char_dir):
        raise ToolNotFoundError("ngspice not found; install it with `sudo apt install ngspice`")
    monkeypatch.setattr(cli, "CHECKS", {"broken": broken})
    assert cli.main(["run", "--out", str(tmp_path), "--characterized", str(tmp_path),
                     "--report", str(tmp_path / "r.html")]) == 2
    assert "sudo apt install ngspice" in capsys.readouterr().err
```

- [ ] **Step 2: Run the tests to verify they fail**

Run: `cd hardware && python3 -m pytest tests/test_report.py tests/test_cli.py -q`
Expected: FAIL with `ModuleNotFoundError`.

- [ ] **Step 3: Implement**

`hardware/neohw/report.py`:

```python
"""Self-contained HTML report for hardware block checks."""
from __future__ import annotations

import base64
import csv
import html
import io
from datetime import datetime
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

from neohw.checks import BlockResult  # noqa: E402

_STYLE = """
body{font-family:system-ui,sans-serif;max-width:1100px;margin:2rem auto;padding:0 16px;color:#222}
table{border-collapse:collapse;margin:.5rem 0 1.5rem}td,th{border:1px solid #ccc;padding:.3rem .5rem;font-size:.9rem}
.pass{color:#0a7a2f;font-weight:600}.fail{color:#b00020;font-weight:600}img,svg{max-width:100%}
"""


def _badge(ok: bool) -> str:
    return '<span class="pass">PASS</span>' if ok else '<span class="fail">FAIL</span>'


def _plot_png(plots: dict) -> str:
    fig, ax = plt.subplots(figsize=(9, 3.2))
    for label, (x, y) in plots.items():
        ax.plot(x / 1e6, y, label=label)
    ax.set_xlabel("frequency (MHz)")
    ax.grid(alpha=0.3)
    ax.legend(fontsize=8)
    buf = io.BytesIO()
    fig.savefig(buf, format="png", dpi=110, bbox_inches="tight")
    plt.close(fig)
    return "data:image/png;base64," + base64.b64encode(buf.getvalue()).decode()


def _block_html(r: BlockResult) -> str:
    e = html.escape
    rows = "".join(f"<tr><td>{e(c.name)}</td><td>{c.value:.3f} {e(c.unit)}</td><td>{e(c.limit)}</td>"
                   f"<td>{_badge(c.passed)}</td></tr>" for c in r.checks)
    out = [f"<h2>{e(r.block)} {_badge(r.passed)}</h2>",
           f"<table><tr><th>check</th><th>value</th><th>limit</th><th>result</th></tr>{rows}</table>"]
    if r.plots:
        out.append(f'<img alt="plots" src="{_plot_png(r.plots)}">')
    svg = r.artefacts.get("svg")
    if svg and Path(svg).exists():
        text = Path(svg).read_text()
        out.append("<h3>Connections</h3>" + text[text.find("<svg"):])
    bom = r.artefacts.get("bom")
    if bom and Path(bom).exists():
        rows_bom = list(csv.DictReader(Path(bom).open(encoding="utf-8")))
        if rows_bom:
            head = "".join(f"<th>{e(k)}</th>" for k in rows_bom[0])
            body = "".join("<tr>" + "".join(f"<td>{e(str(v))}</td>" for v in row.values()) + "</tr>"
                           for row in rows_bom)
            out.append(f"<h3>BOM</h3><table><tr>{head}</tr>{body}</table>")
    return "".join(out)


def write_report(results: list[BlockResult], path: Path) -> Path:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    overall = all(r.passed for r in results)
    body = "".join(_block_html(r) for r in results)
    path.write_text(f"<!doctype html><html><head><meta charset='utf-8'><title>NeoRadio Hardware Report</title>"
                    f"<style>{_STYLE}</style></head><body><h1>NeoRadio hardware report {_badge(overall)}</h1>"
                    f"<p>Generated {datetime.now():%Y-%m-%d %H:%M}</p>{body}</body></html>", encoding="utf-8")
    return path
```

`hardware/neohw/__main__.py`:

```python
"""Command line: python -m neohw run"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

from neohw.checks import attenuator as attenuator_check
from neohw.ngspice import NgspiceError
from neohw.report import write_report
from neohw.tools import ToolNotFoundError

CHECKS = {"attenuator": attenuator_check.run}


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(prog="neohw")
    sub = parser.add_subparsers(dest="command", required=True)
    run = sub.add_parser("run", help="run block checks, write characterisation files and a report")
    run.add_argument("--out", type=Path, default=Path("build/hw"))
    run.add_argument("--characterized", type=Path, default=Path("characterized"))
    run.add_argument("--report", type=Path, default=Path("build/hw/report.html"))
    args = parser.parse_args(argv)

    results = []
    try:
        for name, check in CHECKS.items():
            result = check(args.out, args.characterized)
            print(f"{'PASS' if result.passed else 'FAIL'}  {name}")
            results.append(result)
    except (ToolNotFoundError, NgspiceError) as e:
        print(f"error: {e}", file=sys.stderr)
        return 2
    report = write_report(results, args.report)
    print(f"report: {report}")
    return 0 if all(r.passed for r in results) else 1


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
```

Modify the `Makefile`: add `hw` to `.PHONY`, add the target below, and append the hardware tests to `test`.

```make
hw:
	cd hardware && $(PYTHON) -m neohw run --out ../build/hw --characterized characterized --report ../build/hw/report.html
```

In the `test` recipe, after the `cd sim && …` line, add:

```make
	cd hardware && $(PYTHON) -m pytest -q
```

- [ ] **Step 4: Run the tests and the pipeline**

Run: `cd hardware && python3 -m pytest -q`
Expected: all hardware tests pass (54 tests).

Run: `make hw`
Expected: prints `PASS  attenuator` and `report: ../build/hw/report.html`; creates `hardware/characterized/attenuator.json` with `"status": "pass"`.

Run: `make test`
Expected: the .NET, sim and hardware suites all pass.

- [ ] **Step 5: Commit (including the generated characterisation file)**

```bash
git add Makefile hardware/neohw/report.py hardware/neohw/__main__.py hardware/tests/test_report.py hardware/tests/test_cli.py hardware/characterized/attenuator.json
git commit -m "feat(hw): hardware report, neohw CLI and make hw"
```
