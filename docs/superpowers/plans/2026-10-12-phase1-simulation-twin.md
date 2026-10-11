# Phase 1 — Simulation Environment and Ideal Digital Twin — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** One command (`make sim`) generates imaginary FT8/CW stations as 96 kHz I/Q, passes them through an *ideal* receiver model and a 24-bit ADC, runs the **shipping C# DSP** (`neoradio-dsp`) on the result, decodes the audio with real decoders, and writes an HTML report including an FT8 decode-vs-SNR sweep and the receiver's implementation loss.

**Architecture:** A Python package `neosim` (in `sim/`) builds scenarios from YAML, renders stations at complex baseband around the receiver LO, adds AWGN, quantises like the ADC, and writes SigMF. A C# solution (in `pi/`) holds `NeoRadio.Dsp` (pure DSP library) and `NeoRadio.Dsp.Cli` (`neoradio-dsp`, SigMF in → 12 kHz WAV out). `neosim` calls `neoradio-dsp` and the ft8_lib binaries as subprocesses. Later phases swap the ideal blocks for real hardware models without touching the interfaces.

**Tech Stack:** Python 3.11+ (numpy, scipy, PyYAML, matplotlib, pytest); C# / .NET 10 (xUnit); ft8_lib (C, git submodule); GNU make.

**Spec:** `docs/superpowers/specs/2026-10-12-neoradio-hf-receiver-design.md` (§1 success criterion 1, §8 Pi software layout, §9 L1/L4 simulation, §11 repository layout).

## Global Constraints

- .NET projects target `net10.0` with `<Nullable>enable</Nullable>` and `<TreatWarningsAsErrors>true</TreatWarningsAsErrors>`.
- Python ≥ 3.11; runtime deps numpy, scipy, PyYAML, matplotlib only.
- I/Q sample rate is **96 000 Hz complex**; ADC is **24-bit**; decoder audio is **12 000 Hz, 16-bit mono WAV**.
- I/Q interchange format is **SigMF `cf32_le`** (`.sigmf-meta` + `.sigmf-data`).
- **SNR definition everywhere:** station power relative to noise power in a **2500 Hz** bandwidth (WSJT convention). For a key-down CW or carrier, power is measured while key-down.
- Usable receiver window: `|f − LO| ≤ 0.45 × 96 000 = 43 200 Hz`.
- ft8_lib is pinned to commit `9fec6ca39886edbf96f4f5e71edc76da5074e871`.
- `neoradio-dsp` exit codes: `0` success, `2` usage/configuration error, `3` input-file error; messages go to stderr.
- Network: GitHub (submodule) and nuget.org (test packages) must be reachable. If either times out, ask the user to switch the VPN on and retry.

## Review Focus

1. **A station outside the 96 kHz window** (e.g. a broadcast station 1.3 MHz away) must be listed as *excluded* with a reason and must not appear in the I/Q — never silently aliased. → Task 6 test `test_compose_excludes_station_outside_window`.
2. **ADC overload** (very strong in-window signal) must saturate at full scale, never wrap sign, and the clipped-sample count must be reported. → Task 7 test `test_adc_clips_without_wrapping_and_counts`.
3. **Malformed SigMF input** (wrong datatype, missing sample rate, missing data file, truncated data) must exit with code 3 and a message naming the problem. → Task 4 tests.
4. **Tuning outside the usable window or an impossible rate ratio** must be rejected with exit code 2 and a clear message, not produce garbage audio. → Task 3 `Config_validation_rejects_*` and Task 4 `Tune_offset_outside_window_is_usage_error`.
5. **A missing or failing external tool** (ft8_lib not built, `neoradio-dsp` not built or crashing) must raise an error naming the tool and how to fix it (`make tools`), including the tool's stderr. → Task 1 `test_missing_tool_names_make_target`, Task 9 `test_run_dsp_failure_includes_stderr`.

---

## File Structure

```
Makefile                                  build tools, test, sim targets
.gitignore
.gitmodules
third_party/ft8_lib/                      submodule (gen_ft8, decode_ft8)
pi/
  Directory.Build.props                   shared .NET settings
  NeoRadio.slnx
  src/NeoRadio.Dsp/
    FirDesign.cs                          windowed-sinc lowpass design
    ComplexFir.cs                         streaming complex FIR
    Decimator.cs                          streaming decimating FIR
    Nco.cs                                phase-continuous frequency shifter
    Agc.cs                                peak-following AGC
    ReceiverConfig.cs                     Sideband enum + validated config
    Receiver.cs                           SSB/CW demod pipeline
  src/NeoRadio.Dsp.Cli/
    CliApp.cs                             argument handling + exit codes
    CliOptions.cs                         option parsing
    SigMfReader.cs                        cf32_le SigMF reader
    WavWriter.cs                          16-bit mono WAV writer
    Metrics.cs                            metrics JSON record
    Program.cs                            Main → CliApp.Run
  tests/NeoRadio.Dsp.Tests/               FirDesignTests, NcoTests, FilterTests, ReceiverTests, AgcTests
  tests/NeoRadio.Dsp.Cli.Tests/           CliTests
sim/
  pyproject.toml
  neosim/
    __init__.py
    __main__.py                           CLI: run scenarios (+ optional FT8 sweep) → report
    tools.py                              locate external binaries
    scenario.py                           YAML → validated dataclasses
    morse.py                              Morse table + keying units
    sources.py                            FT8 / CW / carrier baseband generators
    channel.py                            window check, AWGN, composition
    adc.py                                ideal 24-bit ADC
    sigmf.py                              SigMF writer
    dsp_runner.py                         run neoradio-dsp
    decoders.py                           ft8_lib wrapper, CW oracle decoder, CER
    twin.py                               scenario → result
    sweep.py                              FT8 decode-vs-SNR sweep + implementation loss
    report.py                             HTML report
  scenarios/20m_ft8_cw_ideal.yaml
  tests/                                  one test_*.py per module
```

---

### Task 1: Repository scaffolding, ft8_lib, tool locator

**Files:**
- Create: `.gitignore`, `Makefile`, `.gitmodules` (via git), `third_party/ft8_lib` (submodule)
- Create: `pi/Directory.Build.props`, `pi/NeoRadio.slnx` and four projects (via `dotnet new`)
- Create: `sim/pyproject.toml`, `sim/neosim/__init__.py`, `sim/neosim/tools.py`
- Test: `sim/tests/test_tools.py`

**Interfaces:**
- Produces: `neosim.tools.REPO_ROOT: Path`, `neosim.tools.ToolNotFoundError(RuntimeError)`, `neosim.tools.tool_path(name: str) -> Path` for names `"gen_ft8"`, `"decode_ft8"`, `"neoradio-dsp"` (env overrides `NEOSIM_GEN_FT8`, `NEOSIM_DECODE_FT8`, `NEOSIM_DSP_CLI`). Makefile targets `tools`, `dsp`, `test`, `sim`. C# assembly name `neoradio-dsp` built to `pi/src/NeoRadio.Dsp.Cli/bin/Release/net10.0/neoradio-dsp`.

- [ ] **Step 1: Add ft8_lib as a pinned submodule**

```bash
cd /home/neo/projects/personal/NeoRadio
git submodule add https://github.com/kgoba/ft8_lib.git third_party/ft8_lib
git -C third_party/ft8_lib checkout 9fec6ca39886edbf96f4f5e71edc76da5074e871
make -C third_party/ft8_lib -j8
./third_party/ft8_lib/gen_ft8 "CQ EP2ABC LL35" /tmp/neo_t.wav 1500 && ./third_party/ft8_lib/decode_ft8 /tmp/neo_t.wav | tail -1
```
Expected last line: `000000 +16.5 +1.36 1500 ~  CQ EP2ABC LL35` (SNR/time columns may differ slightly).

- [ ] **Step 2: Create the .NET solution and projects**

```bash
mkdir -p pi && cd pi
dotnet new sln -n NeoRadio
dotnet new classlib -n NeoRadio.Dsp -o src/NeoRadio.Dsp -f net10.0
dotnet new console  -n NeoRadio.Dsp.Cli -o src/NeoRadio.Dsp.Cli -f net10.0
dotnet new xunit    -n NeoRadio.Dsp.Tests -o tests/NeoRadio.Dsp.Tests -f net10.0
dotnet new xunit    -n NeoRadio.Dsp.Cli.Tests -o tests/NeoRadio.Dsp.Cli.Tests -f net10.0
dotnet sln NeoRadio.slnx add src/NeoRadio.Dsp src/NeoRadio.Dsp.Cli tests/NeoRadio.Dsp.Tests tests/NeoRadio.Dsp.Cli.Tests
dotnet add src/NeoRadio.Dsp.Cli reference src/NeoRadio.Dsp
dotnet add tests/NeoRadio.Dsp.Tests reference src/NeoRadio.Dsp
dotnet add tests/NeoRadio.Dsp.Cli.Tests reference src/NeoRadio.Dsp.Cli src/NeoRadio.Dsp
rm src/NeoRadio.Dsp/Class1.cs tests/NeoRadio.Dsp.Tests/UnitTest1.cs tests/NeoRadio.Dsp.Cli.Tests/UnitTest1.cs
```

Create `pi/Directory.Build.props`:

```xml
<Project>
  <PropertyGroup>
    <TargetFramework>net10.0</TargetFramework>
    <Nullable>enable</Nullable>
    <ImplicitUsings>enable</ImplicitUsings>
    <TreatWarningsAsErrors>true</TreatWarningsAsErrors>
    <LangVersion>latest</LangVersion>
  </PropertyGroup>
</Project>
```

In `pi/src/NeoRadio.Dsp.Cli/NeoRadio.Dsp.Cli.csproj`, inside the first `<PropertyGroup>`, add:

```xml
    <AssemblyName>neoradio-dsp</AssemblyName>
    <RootNamespace>NeoRadio.Dsp.Cli</RootNamespace>
```

Replace `pi/src/NeoRadio.Dsp.Cli/Program.cs` with a temporary stub (Task 4 replaces it):

```csharp
return 0;
```

Run: `dotnet build pi/NeoRadio.slnx -c Release`
Expected: `Build succeeded` with 0 warnings, and `pi/src/NeoRadio.Dsp.Cli/bin/Release/net10.0/neoradio-dsp` exists.

- [ ] **Step 3: Create the Python package skeleton**

`sim/pyproject.toml`:

```toml
[project]
name = "neosim"
version = "0.1.0"
requires-python = ">=3.11"
dependencies = ["numpy", "scipy", "pyyaml", "matplotlib"]

[project.optional-dependencies]
dev = ["pytest"]

[tool.pytest.ini_options]
pythonpath = ["."]
testpaths = ["tests"]
markers = ["integration: needs built external tools (make tools)"]
```

`sim/neosim/__init__.py`:

```python
"""NeoRadio simulation framework (digital twin)."""
```

- [ ] **Step 4: Write the failing tool-locator tests**

`sim/tests/test_tools.py`:

```python
import stat

import pytest

from neosim import tools


def _make_exe(path):
    path.write_text("#!/bin/sh\nexit 0\n")
    path.chmod(path.stat().st_mode | stat.S_IXUSR)
    return path


def test_missing_tool_names_make_target(monkeypatch, tmp_path):
    monkeypatch.setenv("NEOSIM_DECODE_FT8", str(tmp_path / "nope"))
    with pytest.raises(tools.ToolNotFoundError) as err:
        tools.tool_path("decode_ft8")
    assert "make tools" in str(err.value)
    assert "nope" in str(err.value)


def test_existing_executable_is_returned(monkeypatch, tmp_path):
    exe = _make_exe(tmp_path / "fake")
    monkeypatch.setenv("NEOSIM_DECODE_FT8", str(exe))
    assert tools.tool_path("decode_ft8") == exe


def test_non_executable_file_is_rejected(monkeypatch, tmp_path):
    f = tmp_path / "plain"
    f.write_text("x")
    monkeypatch.setenv("NEOSIM_GEN_FT8", str(f))
    with pytest.raises(tools.ToolNotFoundError):
        tools.tool_path("gen_ft8")


def test_unknown_tool_name_is_key_error():
    with pytest.raises(KeyError):
        tools.tool_path("jt9")


def test_default_dsp_path_points_into_pi_build():
    assert tools.DEFAULT_PATHS["neoradio-dsp"].parts[-5:] == (
        "NeoRadio.Dsp.Cli", "bin", "Release", "net10.0", "neoradio-dsp")
```

- [ ] **Step 5: Run tests to verify they fail**

Run: `cd sim && python -m pytest tests/test_tools.py -q`
Expected: FAIL — `ImportError: cannot import name 'tools'`.

- [ ] **Step 6: Implement `sim/neosim/tools.py`**

```python
"""Locate external binaries used by the simulation."""
from __future__ import annotations

import os
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]

DEFAULT_PATHS: dict[str, Path] = {
    "gen_ft8": REPO_ROOT / "third_party" / "ft8_lib" / "gen_ft8",
    "decode_ft8": REPO_ROOT / "third_party" / "ft8_lib" / "decode_ft8",
    "neoradio-dsp": REPO_ROOT / "pi" / "src" / "NeoRadio.Dsp.Cli" / "bin" / "Release" / "net10.0" / "neoradio-dsp",
}

ENV_OVERRIDES: dict[str, str] = {
    "gen_ft8": "NEOSIM_GEN_FT8",
    "decode_ft8": "NEOSIM_DECODE_FT8",
    "neoradio-dsp": "NEOSIM_DSP_CLI",
}


class ToolNotFoundError(RuntimeError):
    """An external tool is missing or not executable."""


def tool_path(name: str) -> Path:
    """Return the path of an external tool, honouring env overrides."""
    if name not in DEFAULT_PATHS:
        raise KeyError(name)
    path = Path(os.environ.get(ENV_OVERRIDES[name], DEFAULT_PATHS[name]))
    if not (path.is_file() and os.access(path, os.X_OK)):
        raise ToolNotFoundError(
            f"{name} not found or not executable at {path}; run `make tools` in {REPO_ROOT}")
    return path
```

- [ ] **Step 7: Run tests to verify they pass**

Run: `cd sim && python -m pytest tests/test_tools.py -q`
Expected: `5 passed`.

- [ ] **Step 8: Add `.gitignore` and `Makefile`**

`.gitignore`:

```
bin/
obj/
build/
__pycache__/
*.pyc
.pytest_cache/
```

`Makefile` (recipe lines start with a TAB):

```make
FT8_DIR := third_party/ft8_lib
SLN     := pi/NeoRadio.slnx
PYTHON  ?= python3

.PHONY: tools dsp ft8 test sim clean

tools: ft8 dsp

ft8:
	$(MAKE) -C $(FT8_DIR) -j8

dsp:
	dotnet build $(SLN) -c Release

test: tools
	dotnet test $(SLN) -c Release
	cd sim && $(PYTHON) -m pytest -q

sim: tools
	cd sim && $(PYTHON) -m neosim run scenarios/*.yaml --workdir ../build/sim --report ../build/sim/report.html --sweep-ft8 -8 -24 2 --trials 4

clean:
	rm -rf build
	dotnet clean $(SLN) -c Release
```

Run: `make tools && make -n sim`
Expected: tools build succeeds; `make -n sim` prints the `python3 -m neosim run …` line (the module itself arrives in Task 10).

- [ ] **Step 9: Commit**

```bash
git add .gitignore .gitmodules Makefile third_party/ft8_lib pi sim
git commit -m "build: scaffold sim package, .NET solution and pinned ft8_lib"
```

---

### Task 2: C# DSP primitives — FIR design, complex FIR, decimator, NCO

**Files:**
- Create: `pi/src/NeoRadio.Dsp/FirDesign.cs`, `ComplexFir.cs`, `Decimator.cs`, `Nco.cs`
- Test: `pi/tests/NeoRadio.Dsp.Tests/FirDesignTests.cs`, `NcoTests.cs`, `FilterTests.cs`

**Interfaces:**
- Produces (namespace `NeoRadio.Dsp`):
  - `static double[] FirDesign.Lowpass(double cutoffHz, double transitionHz, double sampleRate)` — Blackman windowed sinc, odd length `ceil(5.5·fs/transition)`, unity DC gain; `cutoffHz` is the −6 dB point, passband flat to `cutoffHz − transitionHz/2`.
  - `sealed class ComplexFir(double[] taps)` with `void Push(Complex x)`, `Complex Output()`, `Complex Step(Complex x)`, `void Process(ReadOnlySpan<Complex> input, Span<Complex> output)` (in-place allowed).
  - `sealed class Decimator(int factor, double[] taps)` with `Complex[] Process(ReadOnlySpan<Complex> input)`; emits one output per `factor` inputs, state carried across calls.
  - `sealed class Nco(double frequencyHz, double sampleRate)` with `void Mix(Span<Complex> data)` (multiplies by `e^{j2πfn/fs}`, phase continuous across calls).

- [ ] **Step 1: Write the failing tests**

`pi/tests/NeoRadio.Dsp.Tests/FirDesignTests.cs`:

```csharp
using System.Numerics;
using NeoRadio.Dsp;
using Xunit;

namespace NeoRadio.Dsp.Tests;

public class FirDesignTests
{
    internal static double Gain(double[] taps, double f, double fs)
    {
        Complex acc = Complex.Zero;
        for (int i = 0; i < taps.Length; i++)
            acc += taps[i] * Complex.FromPolarCoordinates(1, -2 * Math.PI * f * i / fs);
        return acc.Magnitude;
    }

    [Fact]
    public void Lowpass_has_unity_dc_gain() =>
        Assert.Equal(1.0, Gain(FirDesign.Lowpass(1000, 200, 12000), 0, 12000), 6);

    [Fact]
    public void Lowpass_passes_passband_and_rejects_stopband()
    {
        var taps = FirDesign.Lowpass(1000, 200, 12000);
        Assert.True(Gain(taps, 800, 12000) > 0.999);
        Assert.True(Gain(taps, 1200, 12000) < 1e-3);
        Assert.True(Gain(taps, 3000, 12000) < 1e-3);
    }

    [Fact]
    public void Lowpass_tap_count_is_odd_and_sized_by_transition()
    {
        var taps = FirDesign.Lowpass(1000, 200, 12000);
        Assert.Equal(1, taps.Length % 2);
        Assert.InRange(taps.Length, 330, 332);
    }

    [Theory]
    [InlineData(0, 100)]
    [InlineData(6000, 100)]
    [InlineData(1000, 0)]
    public void Lowpass_rejects_invalid_arguments(double cutoff, double transition) =>
        Assert.Throws<ArgumentOutOfRangeException>(() => FirDesign.Lowpass(cutoff, transition, 12000));
}
```

`pi/tests/NeoRadio.Dsp.Tests/NcoTests.cs`:

```csharp
using System.Numerics;
using NeoRadio.Dsp;
using Xunit;

namespace NeoRadio.Dsp.Tests;

public class NcoTests
{
    [Fact]
    public void Mix_rotates_by_requested_frequency()
    {
        var data = Enumerable.Repeat(Complex.One, 9600).ToArray();
        new Nco(1000, 96000).Mix(data);
        for (int n = 0; n < data.Length; n += 97)
        {
            var expected = Complex.FromPolarCoordinates(1, 2 * Math.PI * 1000 * n / 96000);
            Assert.True((data[n] - expected).Magnitude < 1e-9, $"sample {n}");
        }
    }

    [Fact]
    public void Mix_is_phase_continuous_across_calls()
    {
        var whole = Enumerable.Repeat(Complex.One, 5000).ToArray();
        new Nco(-1234.5, 96000).Mix(whole);

        var split = Enumerable.Repeat(Complex.One, 5000).ToArray();
        var nco = new Nco(-1234.5, 96000);
        nco.Mix(split.AsSpan(0, 1700));
        nco.Mix(split.AsSpan(1700));

        for (int i = 0; i < whole.Length; i++)
            Assert.True((whole[i] - split[i]).Magnitude < 1e-9, $"sample {i}");
    }
}
```

`pi/tests/NeoRadio.Dsp.Tests/FilterTests.cs`:

```csharp
using System.Numerics;
using NeoRadio.Dsp;
using Xunit;

namespace NeoRadio.Dsp.Tests;

public class FilterTests
{
    internal static Complex[] Tone(double f, double fs, int n, double amp = 1.0) =>
        Enumerable.Range(0, n).Select(i => Complex.FromPolarCoordinates(amp, 2 * Math.PI * f * i / fs)).ToArray();

    static double TailRms(Complex[] x, int skip) =>
        Math.Sqrt(x.Skip(skip).Average(c => c.Magnitude * c.Magnitude));

    [Fact]
    public void ComplexFir_passes_inband_and_blocks_outband()
    {
        var taps = FirDesign.Lowpass(1000, 200, 12000);
        var pass = Tone(500, 12000, 4000);
        new ComplexFir(taps).Process(pass, pass);
        var stop = Tone(-2000, 12000, 4000);
        new ComplexFir(taps).Process(stop, stop);
        Assert.InRange(TailRms(pass, taps.Length), 0.99, 1.01);
        Assert.True(TailRms(stop, taps.Length) < 1e-3);
    }

    [Fact]
    public void ComplexFir_streaming_equals_one_shot()
    {
        var taps = FirDesign.Lowpass(1000, 200, 12000);
        var x = Tone(700, 12000, 3000);
        var once = new Complex[x.Length];
        new ComplexFir(taps).Process(x, once);
        var parts = new Complex[x.Length];
        var fir = new ComplexFir(taps);
        fir.Process(x.AsSpan(0, 1111), parts.AsSpan(0, 1111));
        fir.Process(x.AsSpan(1111), parts.AsSpan(1111));
        for (int i = 0; i < x.Length; i++) Assert.True((once[i] - parts[i]).Magnitude < 1e-12);
    }

    [Fact]
    public void Decimator_outputs_one_sample_per_factor_and_keeps_inband_tone()
    {
        var taps = FirDesign.Lowpass(4800, 2400, 96000);
        var dec = new Decimator(8, taps);
        var x = Tone(1000, 96000, 96000);
        var y1 = dec.Process(x.AsSpan(0, 50001));
        var y2 = dec.Process(x.AsSpan(50001));
        Assert.Equal(12000, y1.Length + y2.Length);
        var y = y1.Concat(y2).ToArray();
        Assert.InRange(TailRms(y, taps.Length / 8 + 1), 0.99, 1.01);
        var step = y[5000] * Complex.Conjugate(y[4999]);
        Assert.Equal(2 * Math.PI * 1000 / 12000, step.Phase, 3);
    }

    [Fact]
    public void Decimator_rejects_band_that_would_alias()
    {
        var taps = FirDesign.Lowpass(4800, 2400, 96000);
        var y = new Decimator(8, taps).Process(Tone(13000, 96000, 48000));
        Assert.True(TailRms(y, taps.Length / 8 + 1) < 1e-3);
    }

    [Fact]
    public void Decimator_rejects_factor_below_one() =>
        Assert.Throws<ArgumentOutOfRangeException>(() => new Decimator(0, new double[] { 1 }));
}
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `dotnet test pi/NeoRadio.slnx -c Release`
Expected: FAIL — compile errors `The name 'FirDesign' does not exist`, etc.

- [ ] **Step 3: Implement the primitives**

`pi/src/NeoRadio.Dsp/FirDesign.cs`:

```csharp
namespace NeoRadio.Dsp;

/// <summary>FIR filter design helpers.</summary>
public static class FirDesign
{
    /// <summary>
    /// Blackman-windowed sinc lowpass. <paramref name="cutoffHz"/> is the −6 dB point; the passband is
    /// flat to cutoff − transition/2 and the stopband (≈ −74 dB) starts at cutoff + transition/2.
    /// </summary>
    public static double[] Lowpass(double cutoffHz, double transitionHz, double sampleRate)
    {
        if (cutoffHz <= 0 || cutoffHz >= sampleRate / 2)
            throw new ArgumentOutOfRangeException(nameof(cutoffHz), cutoffHz, "cutoff must be between 0 and fs/2");
        if (transitionHz <= 0)
            throw new ArgumentOutOfRangeException(nameof(transitionHz), transitionHz, "transition must be positive");

        int n = (int)Math.Ceiling(5.5 * sampleRate / transitionHz);
        if (n % 2 == 0) n++;
        int m = n - 1;
        double fc = cutoffHz / sampleRate;
        var taps = new double[n];
        double sum = 0;
        for (int i = 0; i < n; i++)
        {
            double k = i - m / 2.0;
            double sinc = k == 0 ? 2 * fc : Math.Sin(2 * Math.PI * fc * k) / (Math.PI * k);
            double window = 0.42 - 0.5 * Math.Cos(2 * Math.PI * i / m) + 0.08 * Math.Cos(4 * Math.PI * i / m);
            taps[i] = sinc * window;
            sum += taps[i];
        }
        for (int i = 0; i < n; i++) taps[i] /= sum;
        return taps;
    }
}
```

`pi/src/NeoRadio.Dsp/ComplexFir.cs`:

```csharp
using System.Numerics;

namespace NeoRadio.Dsp;

/// <summary>Streaming FIR filter with real taps and complex samples.</summary>
public sealed class ComplexFir
{
    private readonly double[] _taps;
    private readonly Complex[] _history;
    private int _pos;

    public ComplexFir(double[] taps)
    {
        if (taps.Length == 0) throw new ArgumentException("taps must not be empty", nameof(taps));
        _taps = taps;
        _history = new Complex[taps.Length];
    }

    public void Push(Complex x)
    {
        _pos = _pos + 1 == _history.Length ? 0 : _pos + 1;
        _history[_pos] = x;
    }

    public Complex Output()
    {
        Complex acc = Complex.Zero;
        int idx = _pos;
        for (int i = 0; i < _taps.Length; i++)
        {
            acc += _history[idx] * _taps[i];
            idx = idx == 0 ? _history.Length - 1 : idx - 1;
        }
        return acc;
    }

    public Complex Step(Complex x)
    {
        Push(x);
        return Output();
    }

    public void Process(ReadOnlySpan<Complex> input, Span<Complex> output)
    {
        if (output.Length < input.Length) throw new ArgumentException("output too short", nameof(output));
        for (int i = 0; i < input.Length; i++) output[i] = Step(input[i]);
    }
}
```

`pi/src/NeoRadio.Dsp/Decimator.cs`:

```csharp
using System.Numerics;

namespace NeoRadio.Dsp;

/// <summary>Streaming FIR decimator: computes only every <c>factor</c>-th output.</summary>
public sealed class Decimator
{
    private readonly ComplexFir _fir;
    private readonly int _factor;
    private int _phase;

    public Decimator(int factor, double[] taps)
    {
        if (factor < 1) throw new ArgumentOutOfRangeException(nameof(factor), factor, "factor must be >= 1");
        _factor = factor;
        _fir = new ComplexFir(taps);
    }

    public Complex[] Process(ReadOnlySpan<Complex> input)
    {
        var output = new List<Complex>(input.Length / _factor + 1);
        foreach (var x in input)
        {
            _fir.Push(x);
            if (_phase == 0) output.Add(_fir.Output());
            _phase = _phase + 1 == _factor ? 0 : _phase + 1;
        }
        return output.ToArray();
    }
}
```

`pi/src/NeoRadio.Dsp/Nco.cs`:

```csharp
using System.Numerics;

namespace NeoRadio.Dsp;

/// <summary>Phase-continuous complex frequency shifter.</summary>
public sealed class Nco
{
    private readonly Complex _step;
    private Complex _phasor = Complex.One;
    private int _sinceNormalize;

    public Nco(double frequencyHz, double sampleRate)
    {
        double w = 2 * Math.PI * frequencyHz / sampleRate;
        _step = new Complex(Math.Cos(w), Math.Sin(w));
    }

    public void Mix(Span<Complex> data)
    {
        for (int i = 0; i < data.Length; i++)
        {
            data[i] *= _phasor;
            _phasor *= _step;
            if (++_sinceNormalize == 1024)
            {
                _phasor /= _phasor.Magnitude;
                _sinceNormalize = 0;
            }
        }
    }
}
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `dotnet test pi/NeoRadio.slnx -c Release`
Expected: all tests in `FirDesignTests`, `NcoTests`, `FilterTests` pass (13 tests).

- [ ] **Step 5: Commit**

```bash
git add pi
git commit -m "feat(dsp): FIR design, streaming FIR, decimator and NCO"
```

---

### Task 3: C# receiver — SSB/CW demodulation and AGC

**Files:**
- Create: `pi/src/NeoRadio.Dsp/Agc.cs`, `ReceiverConfig.cs`, `Receiver.cs`
- Test: `pi/tests/NeoRadio.Dsp.Tests/AgcTests.cs`, `ReceiverTests.cs`

**Interfaces:**
- Consumes: `FirDesign.Lowpass`, `ComplexFir`, `Decimator`, `Nco` (Task 2).
- Produces:
  - `enum Sideband { Upper, Lower }`
  - `sealed record ReceiverConfig(Sideband Sideband, double InputRate, double AudioRate, double TuneOffsetHz, double LowCutHz, double HighCutHz, bool AgcEnabled = true)` with `void Validate()` throwing `ArgumentException`; `const double UsableFraction = 0.45`.
  - `sealed class Agc(double sampleRate, double decaySeconds = 0.5, double target = 0.3)` with `void Process(Span<float> audio)` — instant-attack peak follower, so `|output| ≤ target` always.
  - `sealed class Receiver(ReceiverConfig config)` with `float[] Process(ReadOnlySpan<Complex> iq)`; output length = input length / (InputRate/AudioRate).
  - Semantics: `TuneOffsetHz` = dial frequency − LO. Upper sideband: RF `dial + a` → audio `a`. Lower: RF `dial − a` → audio `a`. Audio passband `[LowCutHz, HighCutHz]`.

- [ ] **Step 1: Write the failing tests**

`pi/tests/NeoRadio.Dsp.Tests/AgcTests.cs`:

```csharp
using NeoRadio.Dsp;
using Xunit;

namespace NeoRadio.Dsp.Tests;

public class AgcTests
{
    static float[] Sine(double amp, double f, double fs, int n) =>
        Enumerable.Range(0, n).Select(i => (float)(amp * Math.Sin(2 * Math.PI * f * i / fs))).ToArray();

    [Theory]
    [InlineData(0.001)]
    [InlineData(1.0)]
    public void Settles_to_target_peak(double amplitude)
    {
        var x = Sine(amplitude, 600, 12000, 24000);
        new Agc(12000).Process(x);
        float peak = x.Skip(12000).Max(v => Math.Abs(v));
        Assert.InRange(peak, 0.29f, 0.3001f);
    }

    [Fact]
    public void Output_never_exceeds_target_even_at_start()
    {
        var x = Sine(0.5, 600, 12000, 100);
        new Agc(12000).Process(x);
        Assert.All(x, v => Assert.InRange(v, -0.3001f, 0.3001f));
    }
}
```

`pi/tests/NeoRadio.Dsp.Tests/ReceiverTests.cs`:

```csharp
using System.Numerics;
using NeoRadio.Dsp;
using Xunit;

namespace NeoRadio.Dsp.Tests;

public class ReceiverTests
{
    const double Fs = 96000;

    static double ToneAmplitude(float[] audio, double f, double fs, int skip)
    {
        Complex acc = Complex.Zero;
        int n = 0;
        for (int i = skip; i < audio.Length; i++, n++)
            acc += audio[i] * Complex.FromPolarCoordinates(1, -2 * Math.PI * f * i / fs);
        return 2 * acc.Magnitude / n;
    }

    static ReceiverConfig Usb(double tune, bool agc = false) => new(Sideband.Upper, Fs, 12000, tune, 200, 2800, agc);
    static ReceiverConfig Lsb(double tune) => new(Sideband.Lower, Fs, 12000, tune, 200, 2800, false);
    static ReceiverConfig Cw(double tune) => new(Sideband.Upper, Fs, 12000, tune, 350, 850, false);

    [Fact]
    public void Usb_station_appears_at_its_audio_frequency()
    {
        var audio = new Receiver(Usb(14000)).Process(FilterTests.Tone(14000 + 1500, Fs, 96000, 0.1));
        Assert.Equal(12000, audio.Length);
        double wanted = ToneAmplitude(audio, 1500, 12000, 2000);
        Assert.InRange(wanted, 0.09, 0.11);
        Assert.True(wanted > 100 * ToneAmplitude(audio, 1400, 12000, 2000));
    }

    [Fact]
    public void Usb_rejects_opposite_sideband_by_60_dB()
    {
        var wanted = new Receiver(Usb(14000)).Process(FilterTests.Tone(14000 + 1500, Fs, 96000, 0.1));
        var image = new Receiver(Usb(14000)).Process(FilterTests.Tone(14000 - 1500, Fs, 96000, 0.1));
        double w = ToneAmplitude(wanted, 1500, 12000, 2000);
        double rms = Math.Sqrt(image.Skip(2000).Average(v => (double)v * v));
        Assert.True(rms < w * 1e-3, $"image rms {rms} vs wanted {w}");
    }

    [Fact]
    public void Lsb_station_below_dial_appears_at_its_audio_frequency()
    {
        var audio = new Receiver(Lsb(-20000)).Process(FilterTests.Tone(-20000 - 1000, Fs, 96000, 0.1));
        Assert.InRange(ToneAmplitude(audio, 1000, 12000, 2000), 0.09, 0.11);
    }

    [Fact]
    public void Cw_filter_passes_pitch_and_rejects_neighbour()
    {
        var on = new Receiver(Cw(5000)).Process(FilterTests.Tone(5600, Fs, 96000, 0.1));
        var off = new Receiver(Cw(5000)).Process(FilterTests.Tone(6200, Fs, 96000, 0.1));
        double a = ToneAmplitude(on, 600, 12000, 2000);
        double rms = Math.Sqrt(off.Skip(2000).Average(v => (double)v * v));
        Assert.InRange(a, 0.09, 0.11);
        Assert.True(rms < a * 1e-3);
    }

    [Fact]
    public void Streaming_matches_one_shot()
    {
        var x = FilterTests.Tone(15500, Fs, 48000, 0.1);
        var once = new Receiver(Usb(14000)).Process(x);
        var rx = new Receiver(Usb(14000));
        var parts = rx.Process(x.AsSpan(0, 20003)).Concat(rx.Process(x.AsSpan(20003))).ToArray();
        Assert.Equal(once.Length, parts.Length);
        for (int i = 0; i < once.Length; i++) Assert.Equal(once[i], parts[i], 5);
    }

    [Fact]
    public void Agc_enabled_normalises_weak_signal()
    {
        var audio = new Receiver(Usb(14000, agc: true)).Process(FilterTests.Tone(15500, Fs, 96000, 1e-4));
        Assert.InRange(audio.Skip(6000).Max(v => Math.Abs(v)), 0.25f, 0.35f);
    }

    [Fact]
    public void Config_validation_rejects_tune_outside_window() =>
        Assert.Throws<ArgumentException>(() => new Receiver(Usb(42000)));

    [Fact]
    public void Config_validation_rejects_non_integer_rate_ratio() =>
        Assert.Throws<ArgumentException>(() => new Receiver(new ReceiverConfig(Sideband.Upper, Fs, 11025, 0, 200, 2800)));

    [Fact]
    public void Config_validation_rejects_inverted_passband() =>
        Assert.Throws<ArgumentException>(() => new Receiver(new ReceiverConfig(Sideband.Upper, Fs, 12000, 0, 2800, 200)));

    [Fact]
    public void Config_validation_rejects_passband_above_audio_nyquist() =>
        Assert.Throws<ArgumentException>(() => new Receiver(new ReceiverConfig(Sideband.Upper, Fs, 12000, 0, 200, 6000)));
}
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `dotnet test pi/NeoRadio.slnx -c Release`
Expected: FAIL — compile errors for `Agc`, `Receiver`, `ReceiverConfig`, `Sideband`.

- [ ] **Step 3: Implement**

`pi/src/NeoRadio.Dsp/Agc.cs`:

```csharp
namespace NeoRadio.Dsp;

/// <summary>
/// Automatic gain control: instant-attack peak follower with exponential decay. Because the envelope is
/// never below the current sample, the output magnitude never exceeds the target (no start-up overshoot).
/// </summary>
public sealed class Agc
{
    private readonly double _decay;
    private readonly double _target;
    private double _envelope;

    public Agc(double sampleRate, double decaySeconds = 0.5, double target = 0.3)
    {
        _decay = 1 - Math.Exp(-1 / (decaySeconds * sampleRate));
        _target = target;
    }

    public void Process(Span<float> audio)
    {
        for (int i = 0; i < audio.Length; i++)
        {
            double m = Math.Abs(audio[i]);
            if (m > _envelope) _envelope = m;
            else _envelope += _decay * (m - _envelope);
            audio[i] = _envelope > 1e-12 ? (float)(audio[i] * _target / _envelope) : 0f;
        }
    }
}
```

`pi/src/NeoRadio.Dsp/ReceiverConfig.cs`:

```csharp
namespace NeoRadio.Dsp;

public enum Sideband { Upper, Lower }

/// <summary>Receiver settings. TuneOffsetHz is dial frequency minus LO.</summary>
public sealed record ReceiverConfig(
    Sideband Sideband,
    double InputRate,
    double AudioRate,
    double TuneOffsetHz,
    double LowCutHz,
    double HighCutHz,
    bool AgcEnabled = true)
{
    public const double UsableFraction = 0.45;

    public int DecimationFactor => (int)Math.Round(InputRate / AudioRate);

    public void Validate()
    {
        if (InputRate <= 0 || AudioRate <= 0)
            throw new ArgumentException("sample rates must be positive");
        double ratio = InputRate / AudioRate;
        if (ratio < 1 || Math.Abs(ratio - Math.Round(ratio)) > 1e-9)
            throw new ArgumentException($"input rate {InputRate} must be an integer multiple of audio rate {AudioRate}");
        if (LowCutHz < 0 || LowCutHz >= HighCutHz)
            throw new ArgumentException($"audio passband [{LowCutHz}, {HighCutHz}] Hz is invalid");
        if (HighCutHz >= AudioRate / 2 - 100)
            throw new ArgumentException($"high cut {HighCutHz} Hz must be below {AudioRate / 2 - 100} Hz");
        double limit = UsableFraction * InputRate;
        if (Math.Abs(TuneOffsetHz) + HighCutHz > limit)
            throw new ArgumentException(
                $"tune offset {TuneOffsetHz} Hz with high cut {HighCutHz} Hz is outside the usable window ±{limit} Hz");
    }
}
```

`pi/src/NeoRadio.Dsp/Receiver.cs`:

```csharp
using System.Numerics;

namespace NeoRadio.Dsp;

/// <summary>
/// Single-sideband receiver: shift the wanted passband centre to 0 Hz, decimate, apply the channel
/// filter, shift back to the audio centre and take the real part.
/// </summary>
public sealed class Receiver
{
    private readonly Nco _shiftIn;
    private readonly Decimator _decimator;
    private readonly ComplexFir _channel;
    private readonly Nco _shiftOut;
    private readonly Agc? _agc;

    public Receiver(ReceiverConfig config)
    {
        config.Validate();
        double sign = config.Sideband == Sideband.Upper ? 1 : -1;
        double centre = (config.LowCutHz + config.HighCutHz) / 2;
        double half = (config.HighCutHz - config.LowCutHz) / 2;

        _shiftIn = new Nco(-(config.TuneOffsetHz + sign * centre), config.InputRate);
        _decimator = new Decimator(
            config.DecimationFactor,
            FirDesign.Lowpass(0.4 * config.AudioRate, 0.2 * config.AudioRate, config.InputRate));
        _channel = new ComplexFir(FirDesign.Lowpass(half + 75, 150, config.AudioRate));
        _shiftOut = new Nco(sign * centre, config.AudioRate);
        _agc = config.AgcEnabled ? new Agc(config.AudioRate) : null;
    }

    public float[] Process(ReadOnlySpan<Complex> iq)
    {
        var work = iq.ToArray();
        _shiftIn.Mix(work);
        var baseband = _decimator.Process(work);
        _channel.Process(baseband, baseband);
        _shiftOut.Mix(baseband);
        var audio = new float[baseband.Length];
        for (int i = 0; i < audio.Length; i++) audio[i] = (float)baseband[i].Real;
        _agc?.Process(audio);
        return audio;
    }
}
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `dotnet test pi/NeoRadio.slnx -c Release`
Expected: all tests pass (13 from Task 2 + 13 new = 26).

- [ ] **Step 5: Commit**

```bash
git add pi
git commit -m "feat(dsp): SSB/CW receiver pipeline with AGC and config validation"
```

---

### Task 4: `neoradio-dsp` CLI — SigMF in, WAV + metrics out

**Files:**
- Create: `pi/src/NeoRadio.Dsp.Cli/CliApp.cs`, `CliOptions.cs`, `SigMfReader.cs`, `WavWriter.cs`, `Metrics.cs`
- Modify: `pi/src/NeoRadio.Dsp.Cli/Program.cs` (replace stub)
- Test: `pi/tests/NeoRadio.Dsp.Cli.Tests/CliTests.cs`

**Interfaces:**
- Consumes: `Receiver`, `ReceiverConfig`, `Sideband` (Task 3).
- Produces: command line
  `neoradio-dsp --input X.sigmf-meta --output Y.wav --tune-offset HZ [--mode usb|lsb|cw] [--audio-rate 12000] [--low-cut 200] [--high-cut 2800] [--cw-pitch 600] [--cw-bandwidth 500] [--no-agc] [--metrics M.json]`
  exit codes 0/2/3; metrics JSON keys `input_samples`, `audio_samples`, `audio_rms`, `audio_peak`.
  C#: `static int CliApp.Run(string[] args, TextWriter stderr)`, `sealed record SigMfRecording(double SampleRate, double CenterFrequencyHz, Complex[] Samples)`, `static SigMfRecording SigMfReader.Read(string metaPath)`, `static void WavWriter.WriteMono16(string path, ReadOnlySpan<float> samples, int sampleRate)`.

- [ ] **Step 1: Write the failing tests**

`pi/tests/NeoRadio.Dsp.Cli.Tests/CliTests.cs`:

```csharp
using System.Numerics;
using System.Text;
using System.Text.Json;
using NeoRadio.Dsp.Cli;
using Xunit;

namespace NeoRadio.Dsp.Cli.Tests;

public sealed class CliTests : IDisposable
{
    private readonly string _dir = Directory.CreateTempSubdirectory("neoradio-cli-").FullName;

    public void Dispose() => Directory.Delete(_dir, recursive: true);

    static Complex[] Tone(double f, double fs, int n, double amp) =>
        Enumerable.Range(0, n).Select(i => Complex.FromPolarCoordinates(amp, 2 * Math.PI * f * i / fs)).ToArray();

    string WriteSigMf(string name, string datatype, double? rate, Complex[] samples, int dropBytes = 0)
    {
        var meta = Path.Combine(_dir, name + ".sigmf-meta");
        var global = new Dictionary<string, object> { ["core:datatype"] = datatype, ["core:version"] = "1.0.0" };
        if (rate is not null) global["core:sample_rate"] = rate.Value;
        var doc = new Dictionary<string, object>
        {
            ["global"] = global,
            ["captures"] = new[] { new Dictionary<string, object> { ["core:sample_start"] = 0, ["core:frequency"] = 14060000.0 } },
            ["annotations"] = Array.Empty<object>(),
        };
        File.WriteAllText(meta, JsonSerializer.Serialize(doc));
        using var ms = new MemoryStream();
        using (var w = new BinaryWriter(ms, Encoding.ASCII, leaveOpen: true))
            foreach (var s in samples) { w.Write((float)s.Real); w.Write((float)s.Imaginary); }
        var bytes = ms.ToArray();
        File.WriteAllBytes(Path.Combine(_dir, name + ".sigmf-data"), bytes[..^dropBytes]);
        return meta;
    }

    static (int Code, string Err) Run(params string[] args)
    {
        var err = new StringWriter();
        int code = CliApp.Run(args, err);
        return (code, err.ToString());
    }

    [Fact]
    public void Demodulates_to_wav_and_writes_metrics()
    {
        var meta = WriteSigMf("ok", "cf32_le", 96000, Tone(15500, 96000, 192000, 0.1));
        var wav = Path.Combine(_dir, "out.wav");
        var metrics = Path.Combine(_dir, "m.json");

        var (code, err) = Run("--input", meta, "--output", wav, "--mode", "usb", "--tune-offset", "14000", "--metrics", metrics);

        Assert.True(code == 0, err);
        var bytes = File.ReadAllBytes(wav);
        Assert.Equal("RIFF", Encoding.ASCII.GetString(bytes, 0, 4));
        Assert.Equal("WAVE", Encoding.ASCII.GetString(bytes, 8, 4));
        Assert.Equal(1, BitConverter.ToInt16(bytes, 22));       // channels
        Assert.Equal(12000, BitConverter.ToInt32(bytes, 24));   // sample rate
        Assert.Equal(16, BitConverter.ToInt16(bytes, 34));      // bits per sample
        Assert.Equal(24000 * 2, BitConverter.ToInt32(bytes, 40));
        using var m = JsonDocument.Parse(File.ReadAllText(metrics));
        Assert.Equal(192000, m.RootElement.GetProperty("input_samples").GetInt32());
        Assert.Equal(24000, m.RootElement.GetProperty("audio_samples").GetInt32());
        Assert.InRange(m.RootElement.GetProperty("audio_peak").GetDouble(), 0.2, 0.4);
    }

    [Fact]
    public void Unsupported_datatype_is_input_error()
    {
        var meta = WriteSigMf("ci16", "ci16_le", 96000, Tone(0, 96000, 10, 0.1));
        var (code, err) = Run("--input", meta, "--output", Path.Combine(_dir, "x.wav"), "--tune-offset", "0");
        Assert.Equal(3, code);
        Assert.Contains("cf32_le", err);
    }

    [Fact]
    public void Missing_sample_rate_is_input_error()
    {
        var meta = WriteSigMf("norate", "cf32_le", null, Tone(0, 96000, 10, 0.1));
        var (code, err) = Run("--input", meta, "--output", Path.Combine(_dir, "x.wav"), "--tune-offset", "0");
        Assert.Equal(3, code);
        Assert.Contains("core:sample_rate", err);
    }

    [Fact]
    public void Missing_data_file_is_input_error()
    {
        var meta = WriteSigMf("nodata", "cf32_le", 96000, Tone(0, 96000, 10, 0.1));
        File.Delete(Path.Combine(_dir, "nodata.sigmf-data"));
        var (code, err) = Run("--input", meta, "--output", Path.Combine(_dir, "x.wav"), "--tune-offset", "0");
        Assert.Equal(3, code);
        Assert.Contains("data file not found", err);
    }

    [Fact]
    public void Truncated_data_is_input_error()
    {
        var meta = WriteSigMf("trunc", "cf32_le", 96000, Tone(0, 96000, 10, 0.1), dropBytes: 3);
        var (code, err) = Run("--input", meta, "--output", Path.Combine(_dir, "x.wav"), "--tune-offset", "0");
        Assert.Equal(3, code);
        Assert.Contains("multiple of 8", err);
    }

    [Fact]
    public void Tune_offset_outside_window_is_usage_error()
    {
        var meta = WriteSigMf("far", "cf32_le", 96000, Tone(0, 96000, 96000, 0.1));
        var (code, err) = Run("--input", meta, "--output", Path.Combine(_dir, "x.wav"), "--tune-offset", "45000");
        Assert.Equal(2, code);
        Assert.Contains("outside the usable window", err);
    }

    [Fact]
    public void Unknown_option_is_usage_error()
    {
        var (code, err) = Run("--input", "a.sigmf-meta", "--output", "b.wav", "--tune-offset", "0", "--bogus", "1");
        Assert.Equal(2, code);
        Assert.Contains("unknown option --bogus", err);
    }

    [Fact]
    public void Missing_tune_offset_is_usage_error()
    {
        var (code, err) = Run("--input", "a.sigmf-meta", "--output", "b.wav");
        Assert.Equal(2, code);
        Assert.Contains("--tune-offset", err);
    }

    [Fact]
    public void Cw_mode_centres_passband_on_pitch()
    {
        var opts = CliOptions.Parse(new[] { "--input", "a.sigmf-meta", "--output", "b.wav", "--tune-offset", "0", "--mode", "cw" });
        var cfg = opts.ToReceiverConfig(96000);
        Assert.Equal(350, cfg.LowCutHz);
        Assert.Equal(850, cfg.HighCutHz);
        Assert.Equal(NeoRadio.Dsp.Sideband.Upper, cfg.Sideband);
    }
}
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `dotnet test pi/NeoRadio.slnx -c Release`
Expected: FAIL — `CliApp`, `CliOptions` not found.

- [ ] **Step 3: Implement**

`pi/src/NeoRadio.Dsp.Cli/CliOptions.cs`:

```csharp
using System.Globalization;
using NeoRadio.Dsp;

namespace NeoRadio.Dsp.Cli;

public sealed class CliUsageException(string message) : Exception(message);

public sealed record CliOptions(
    string InputPath,
    string OutputPath,
    string Mode,
    double TuneOffsetHz,
    double AudioRate,
    double LowCutHz,
    double HighCutHz,
    double CwPitchHz,
    double CwBandwidthHz,
    bool AgcEnabled,
    string? MetricsPath)
{
    public const string Usage =
        "usage: neoradio-dsp --input X.sigmf-meta --output Y.wav --tune-offset HZ [--mode usb|lsb|cw] " +
        "[--audio-rate 12000] [--low-cut 200] [--high-cut 2800] [--cw-pitch 600] [--cw-bandwidth 500] " +
        "[--no-agc] [--metrics M.json]";

    public static CliOptions Parse(string[] args)
    {
        var values = new Dictionary<string, string>();
        bool agc = true;
        for (int i = 0; i < args.Length; i++)
        {
            string key = args[i];
            if (key == "--no-agc") { agc = false; continue; }
            if (key is not ("--input" or "--output" or "--mode" or "--tune-offset" or "--audio-rate" or "--low-cut"
                or "--high-cut" or "--cw-pitch" or "--cw-bandwidth" or "--metrics"))
                throw new CliUsageException($"unknown option {key}");
            if (i + 1 >= args.Length) throw new CliUsageException($"{key} needs a value");
            values[key] = args[++i];
        }

        string Required(string key) =>
            values.TryGetValue(key, out var v) ? v : throw new CliUsageException($"missing required option {key}");
        double Number(string key, double fallback)
        {
            if (!values.TryGetValue(key, out var v)) return fallback;
            return double.TryParse(v, NumberStyles.Float, CultureInfo.InvariantCulture, out var d)
                ? d
                : throw new CliUsageException($"{key} expects a number, got '{v}'");
        }

        string mode = values.GetValueOrDefault("--mode", "usb");
        if (mode is not ("usb" or "lsb" or "cw")) throw new CliUsageException($"--mode must be usb, lsb or cw, got '{mode}'");

        return new CliOptions(
            Required("--input"),
            Required("--output"),
            mode,
            values.ContainsKey("--tune-offset") ? Number("--tune-offset", 0) : throw new CliUsageException("missing required option --tune-offset"),
            Number("--audio-rate", 12000),
            Number("--low-cut", 200),
            Number("--high-cut", 2800),
            Number("--cw-pitch", 600),
            Number("--cw-bandwidth", 500),
            agc,
            values.GetValueOrDefault("--metrics"));
    }

    public ReceiverConfig ToReceiverConfig(double inputRate) => Mode switch
    {
        "cw" => new ReceiverConfig(Sideband.Upper, inputRate, AudioRate, TuneOffsetHz,
            CwPitchHz - CwBandwidthHz / 2, CwPitchHz + CwBandwidthHz / 2, AgcEnabled),
        "lsb" => new ReceiverConfig(Sideband.Lower, inputRate, AudioRate, TuneOffsetHz, LowCutHz, HighCutHz, AgcEnabled),
        _ => new ReceiverConfig(Sideband.Upper, inputRate, AudioRate, TuneOffsetHz, LowCutHz, HighCutHz, AgcEnabled),
    };
}
```

`pi/src/NeoRadio.Dsp.Cli/SigMfReader.cs`:

```csharp
using System.Numerics;
using System.Runtime.InteropServices;
using System.Text.Json;

namespace NeoRadio.Dsp.Cli;

public sealed record SigMfRecording(double SampleRate, double CenterFrequencyHz, Complex[] Samples);

/// <summary>Reads SigMF recordings with core:datatype cf32_le.</summary>
public static class SigMfReader
{
    private const string MetaSuffix = ".sigmf-meta";

    public static SigMfRecording Read(string metaPath)
    {
        if (!metaPath.EndsWith(MetaSuffix, StringComparison.Ordinal))
            throw new InvalidDataException($"input must be a {MetaSuffix} file: {metaPath}");
        if (!File.Exists(metaPath))
            throw new FileNotFoundException($"metadata file not found: {metaPath}");

        using var doc = JsonDocument.Parse(File.ReadAllText(metaPath));
        if (!doc.RootElement.TryGetProperty("global", out var global))
            throw new InvalidDataException("metadata has no 'global' object");

        string datatype = global.TryGetProperty("core:datatype", out var dt) ? dt.GetString() ?? "" : "";
        if (datatype != "cf32_le")
            throw new InvalidDataException($"unsupported core:datatype '{datatype}', expected cf32_le");

        if (!global.TryGetProperty("core:sample_rate", out var sr) || sr.ValueKind != JsonValueKind.Number || sr.GetDouble() <= 0)
            throw new InvalidDataException("missing or invalid core:sample_rate");

        double centre = 0;
        if (doc.RootElement.TryGetProperty("captures", out var captures) && captures.ValueKind == JsonValueKind.Array
            && captures.GetArrayLength() > 0 && captures[0].TryGetProperty("core:frequency", out var f))
            centre = f.GetDouble();

        string dataPath = metaPath[..^MetaSuffix.Length] + ".sigmf-data";
        if (!File.Exists(dataPath)) throw new FileNotFoundException($"data file not found: {dataPath}");
        byte[] bytes = File.ReadAllBytes(dataPath);
        if (bytes.Length % 8 != 0)
            throw new InvalidDataException($"data file length {bytes.Length} is not a multiple of 8 bytes (cf32_le)");
        if (!BitConverter.IsLittleEndian)
            throw new PlatformNotSupportedException("big-endian hosts are not supported");

        var floats = MemoryMarshal.Cast<byte, float>(bytes);
        var samples = new Complex[floats.Length / 2];
        for (int i = 0; i < samples.Length; i++) samples[i] = new Complex(floats[2 * i], floats[2 * i + 1]);
        return new SigMfRecording(sr.GetDouble(), centre, samples);
    }
}
```

`pi/src/NeoRadio.Dsp.Cli/WavWriter.cs`:

```csharp
using System.Text;

namespace NeoRadio.Dsp.Cli;

public static class WavWriter
{
    /// <summary>Writes 16-bit PCM mono; samples are clipped to [-1, 1].</summary>
    public static void WriteMono16(string path, ReadOnlySpan<float> samples, int sampleRate)
    {
        using var w = new BinaryWriter(File.Create(path), Encoding.ASCII);
        int dataBytes = samples.Length * 2;
        w.Write(Encoding.ASCII.GetBytes("RIFF"));
        w.Write(36 + dataBytes);
        w.Write(Encoding.ASCII.GetBytes("WAVE"));
        w.Write(Encoding.ASCII.GetBytes("fmt "));
        w.Write(16);
        w.Write((short)1);              // PCM
        w.Write((short)1);              // mono
        w.Write(sampleRate);
        w.Write(sampleRate * 2);        // byte rate
        w.Write((short)2);              // block align
        w.Write((short)16);             // bits per sample
        w.Write(Encoding.ASCII.GetBytes("data"));
        w.Write(dataBytes);
        foreach (float s in samples)
            w.Write((short)Math.Round(Math.Clamp(s, -1f, 1f) * 32767));
    }
}
```

`pi/src/NeoRadio.Dsp.Cli/Metrics.cs`:

```csharp
using System.Text.Json.Serialization;

namespace NeoRadio.Dsp.Cli;

public sealed record Metrics(
    [property: JsonPropertyName("input_samples")] int InputSamples,
    [property: JsonPropertyName("audio_samples")] int AudioSamples,
    [property: JsonPropertyName("audio_rms")] double AudioRms,
    [property: JsonPropertyName("audio_peak")] double AudioPeak)
{
    public static Metrics From(int inputSamples, float[] audio)
    {
        double sumSq = 0, peak = 0;
        foreach (float s in audio)
        {
            sumSq += (double)s * s;
            peak = Math.Max(peak, Math.Abs(s));
        }
        return new Metrics(inputSamples, audio.Length, audio.Length == 0 ? 0 : Math.Sqrt(sumSq / audio.Length), peak);
    }
}
```

`pi/src/NeoRadio.Dsp.Cli/CliApp.cs`:

```csharp
using System.Text.Json;
using NeoRadio.Dsp;

namespace NeoRadio.Dsp.Cli;

public static class CliApp
{
    public const int Ok = 0;
    public const int UsageError = 2;
    public const int InputError = 3;

    public static int Run(string[] args, TextWriter stderr)
    {
        CliOptions options;
        try
        {
            options = CliOptions.Parse(args);
        }
        catch (CliUsageException e)
        {
            stderr.WriteLine($"usage error: {e.Message}");
            stderr.WriteLine(CliOptions.Usage);
            return UsageError;
        }

        SigMfRecording recording;
        try
        {
            recording = SigMfReader.Read(options.InputPath);
        }
        catch (Exception e) when (e is InvalidDataException or IOException or JsonException)
        {
            stderr.WriteLine($"input error: {e.Message}");
            return InputError;
        }

        Receiver receiver;
        try
        {
            receiver = new Receiver(options.ToReceiverConfig(recording.SampleRate));
        }
        catch (ArgumentException e)
        {
            stderr.WriteLine($"usage error: {e.Message}");
            return UsageError;
        }

        float[] audio = receiver.Process(recording.Samples);
        WavWriter.WriteMono16(options.OutputPath, audio, (int)options.AudioRate);
        if (options.MetricsPath is not null)
            File.WriteAllText(options.MetricsPath, JsonSerializer.Serialize(Metrics.From(recording.Samples.Length, audio)));
        return Ok;
    }
}
```

Note: `FileNotFoundException` derives from `IOException`, so it is caught by the input-error branch.

`pi/src/NeoRadio.Dsp.Cli/Program.cs`:

```csharp
using NeoRadio.Dsp.Cli;

return CliApp.Run(args, Console.Error);
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `dotnet test pi/NeoRadio.slnx -c Release`
Expected: all tests pass (26 DSP + 9 CLI = 35).

- [ ] **Step 5: Commit**

```bash
git add pi
git commit -m "feat(dsp-cli): neoradio-dsp SigMF to WAV with metrics and exit codes"
```

---

### Task 5: Scenario model and validation

**Files:**
- Create: `sim/neosim/morse.py`, `sim/neosim/scenario.py`
- Test: `sim/tests/test_morse.py`, `sim/tests/test_scenario.py`

**Interfaces:**
- Produces:
  - `neosim.morse.MORSE: dict[str, str]`, `neosim.morse.REVERSE: dict[str, str]`, `neosim.morse.keying_units(text: str) -> list[tuple[bool, int]]` (raises `ValueError` on unknown characters).
  - `neosim.scenario`: constants `FS_HZ = 96_000`, `USABLE_FRACTION = 0.45`, `WINDOW_HZ = 43_200.0`; `ScenarioError(ValueError)`; frozen dataclasses `Station(id, mode, rf_hz, snr_db, msg=None, text=None, wpm=None, audio_hz=None)` with property `bandwidth_hz`, `Receive(id, mode, dial_hz, decoder, expect)`, `Scenario(name, lo_hz, duration_s, seed, noise_dbfs, stations, receive)`; `parse_scenario(data: dict) -> Scenario`; `load_scenario(path) -> Scenario`.
  - FT8 stations: `rf_hz = dial_hz + audio_hz`.

- [ ] **Step 1: Write the failing tests**

`sim/tests/test_morse.py`:

```python
import pytest

from neosim import morse


def test_letters_and_gaps():
    assert morse.keying_units("ET") == [(True, 1), (False, 3), (True, 3)]


def test_word_gap():
    assert morse.keying_units("E T") == [(True, 1), (False, 7), (True, 3)]


def test_intra_character_gap():
    assert morse.keying_units("A") == [(True, 1), (False, 1), (True, 3)]


def test_unknown_character_raises():
    with pytest.raises(ValueError, match="no Morse code"):
        morse.keying_units("A#")


def test_reverse_table_round_trips():
    assert all(morse.REVERSE[code] == ch for ch, code in morse.MORSE.items())
```

`sim/tests/test_scenario.py`:

```python
import pytest

from neosim.scenario import ScenarioError, parse_scenario


def base():
    return {
        "name": "t",
        "lo_hz": 14_060_000,
        "stations": [
            {"id": "a", "mode": "ft8", "dial_hz": 14_074_000, "audio_hz": 1500, "snr_db": -10, "msg": "CQ EP2ABC LL35"},
            {"id": "b", "mode": "cw", "rf_hz": 14_045_600, "snr_db": 3, "text": "CQ TEST", "wpm": 20},
            {"id": "c", "mode": "carrier", "rf_hz": 15_400_000, "snr_db": 60},
        ],
        "receive": [
            {"id": "r1", "mode": "usb", "dial_hz": 14_074_000, "decoder": "ft8", "expect": {"contains": ["CQ EP2ABC LL35"]}},
            {"id": "r2", "mode": "cw", "dial_hz": 14_045_000, "decoder": "cw", "expect": {"text": "CQ TEST"}},
        ],
    }


def test_valid_scenario_with_defaults():
    s = parse_scenario(base())
    assert s.duration_s == 15.0 and s.seed == 0 and s.noise_dbfs == -100.0
    ft8 = s.stations[0]
    assert ft8.rf_hz == 14_075_500 and ft8.audio_hz == 1500
    assert s.stations[1].bandwidth_hz == 100.0
    assert s.receive[1].expect == {"text": "CQ TEST", "max_cer": 0.05}


def test_out_of_window_station_is_allowed():
    # stations outside the window are excluded at composition time, not rejected here
    assert parse_scenario(base()).stations[2].rf_hz == 15_400_000


@pytest.mark.parametrize(
    "mutate, message",
    [
        (lambda d: d["stations"].append(dict(d["stations"][0])), "duplicate station id"),
        (lambda d: d["stations"][0].pop("msg"), "needs 'msg'"),
        (lambda d: d["stations"][1].update(text="CQ #"), "no Morse code"),
        (lambda d: d["stations"][0].update(mode="psk31"), "unknown station mode"),
        (lambda d: d["receive"][0].update(dial_hz=14_200_000), "outside the receiver window"),
        (lambda d: d["receive"][0].update(decoder="rtty"), "unknown decoder"),
        (lambda d: d.update(noise={"type": "itu_p372_rural"}), "only awgn"),
        (lambda d: d.update(duration_s=0), "duration_s"),
        (lambda d: d["receive"][0].update(expect={}), "expect.contains"),
        (lambda d: d["stations"][0].update(audio_hz=5000), "audio_hz"),
    ],
)
def test_invalid_scenarios_raise(mutate, message):
    d = base()
    mutate(d)
    with pytest.raises(ScenarioError, match=message):
        parse_scenario(d)
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `cd sim && python -m pytest tests/test_morse.py tests/test_scenario.py -q`
Expected: FAIL — `ImportError`.

- [ ] **Step 3: Implement**

`sim/neosim/morse.py`:

```python
"""International Morse code table and keying in dit units."""
from __future__ import annotations

MORSE: dict[str, str] = {
    "A": ".-", "B": "-...", "C": "-.-.", "D": "-..", "E": ".", "F": "..-.", "G": "--.", "H": "....",
    "I": "..", "J": ".---", "K": "-.-", "L": ".-..", "M": "--", "N": "-.", "O": "---", "P": ".--.",
    "Q": "--.-", "R": ".-.", "S": "...", "T": "-", "U": "..-", "V": "...-", "W": ".--", "X": "-..-",
    "Y": "-.--", "Z": "--..",
    "0": "-----", "1": ".----", "2": "..---", "3": "...--", "4": "....-", "5": ".....", "6": "-....",
    "7": "--...", "8": "---..", "9": "----.",
    "/": "-..-.", "?": "..--..", ".": ".-.-.-", ",": "--..--", "=": "-...-",
}
REVERSE: dict[str, str] = {code: ch for ch, code in MORSE.items()}


def keying_units(text: str) -> list[tuple[bool, int]]:
    """Return (key_down, length_in_dits) segments for text; no leading/trailing gaps."""
    words = text.upper().split()
    segments: list[tuple[bool, int]] = []
    for w_index, word in enumerate(words):
        if w_index:
            segments.append((False, 7))
        for c_index, ch in enumerate(word):
            if ch not in MORSE:
                raise ValueError(f"no Morse code for {ch!r}")
            if c_index:
                segments.append((False, 3))
            for e_index, element in enumerate(MORSE[ch]):
                if e_index:
                    segments.append((False, 1))
                segments.append((True, 1 if element == "." else 3))
    return segments
```

`sim/neosim/scenario.py`:

```python
"""Scenario files: imaginary stations and receive jobs."""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import yaml

from neosim import morse

FS_HZ = 96_000
USABLE_FRACTION = 0.45
WINDOW_HZ = USABLE_FRACTION * FS_HZ  # 43 200 Hz either side of the LO
RECEIVE_MARGIN_HZ = 3_000.0

STATION_MODES = {"ft8", "cw", "carrier"}
RECEIVE_MODES = {"usb", "lsb", "cw"}
DECODERS = {"ft8", "cw"}
_BANDWIDTH_HZ = {"ft8": 50.0, "cw": 100.0, "carrier": 1.0}


class ScenarioError(ValueError):
    """The scenario file is invalid."""


@dataclass(frozen=True)
class Station:
    id: str
    mode: str
    rf_hz: float
    snr_db: float
    msg: str | None = None
    text: str | None = None
    wpm: float | None = None
    audio_hz: float | None = None

    @property
    def bandwidth_hz(self) -> float:
        return _BANDWIDTH_HZ[self.mode]


@dataclass(frozen=True)
class Receive:
    id: str
    mode: str
    dial_hz: float
    decoder: str
    expect: dict


@dataclass(frozen=True)
class Scenario:
    name: str
    lo_hz: float
    duration_s: float
    seed: int
    noise_dbfs: float
    stations: tuple[Station, ...]
    receive: tuple[Receive, ...]


def _require(d: dict, key: str, where: str):
    if key not in d:
        raise ScenarioError(f"{where} needs '{key}'")
    return d[key]


def _station(d: dict) -> Station:
    sid = _require(d, "id", "station")
    where = f"station {sid}"
    mode = _require(d, "mode", where)
    if mode not in STATION_MODES:
        raise ScenarioError(f"{where}: unknown station mode '{mode}'")
    snr = float(_require(d, "snr_db", where))
    if mode == "ft8":
        audio = float(_require(d, "audio_hz", where))
        if not 200 <= audio <= 3000:
            raise ScenarioError(f"{where}: audio_hz must be 200..3000, got {audio}")
        return Station(sid, mode, float(_require(d, "dial_hz", where)) + audio, snr,
                       msg=str(_require(d, "msg", where)), audio_hz=audio)
    rf = float(_require(d, "rf_hz", where))
    if mode == "cw":
        text = str(_require(d, "text", where))
        try:
            morse.keying_units(text)
        except ValueError as e:
            raise ScenarioError(f"{where}: {e}") from e
        wpm = float(_require(d, "wpm", where))
        if not 5 <= wpm <= 60:
            raise ScenarioError(f"{where}: wpm must be 5..60, got {wpm}")
        return Station(sid, mode, rf, snr, text=text, wpm=wpm)
    return Station(sid, mode, rf, snr)


def _receive(d: dict, lo_hz: float) -> Receive:
    rid = _require(d, "id", "receive")
    where = f"receive {rid}"
    mode = _require(d, "mode", where)
    if mode not in RECEIVE_MODES:
        raise ScenarioError(f"{where}: unknown receive mode '{mode}'")
    decoder = _require(d, "decoder", where)
    if decoder not in DECODERS:
        raise ScenarioError(f"{where}: unknown decoder '{decoder}'")
    dial = float(_require(d, "dial_hz", where))
    if abs(dial - lo_hz) + RECEIVE_MARGIN_HZ > WINDOW_HZ:
        raise ScenarioError(f"{where}: dial {dial} Hz is outside the receiver window LO±{WINDOW_HZ:.0f} Hz")
    expect = dict(d.get("expect") or {})
    if decoder == "ft8":
        if not expect.get("contains"):
            raise ScenarioError(f"{where}: ft8 receive needs expect.contains (list of messages)")
        expect["contains"] = [str(m) for m in expect["contains"]]
    else:
        if "text" not in expect:
            raise ScenarioError(f"{where}: cw receive needs expect.text")
        expect.setdefault("max_cer", 0.05)
    return Receive(rid, mode, dial, decoder, expect)


def parse_scenario(data: dict) -> Scenario:
    name = str(_require(data, "name", "scenario"))
    lo = float(_require(data, "lo_hz", "scenario"))
    duration = float(data.get("duration_s", 15.0))
    if not 1.0 <= duration <= 60.0:
        raise ScenarioError(f"duration_s must be 1..60, got {duration}")
    noise = data.get("noise", {"type": "awgn"})
    if noise.get("type") != "awgn":
        raise ScenarioError(f"noise type '{noise.get('type')}' not supported yet: only awgn")
    stations = tuple(_station(s) for s in data.get("stations", []))
    ids = [s.id for s in stations]
    dupes = {i for i in ids if ids.count(i) > 1}
    if dupes:
        raise ScenarioError(f"duplicate station id(s): {sorted(dupes)}")
    receive = tuple(_receive(r, lo) for r in _require(data, "receive", "scenario"))
    adc = data.get("adc", {})
    return Scenario(name, lo, duration, int(data.get("seed", 0)), float(adc.get("noise_dbfs", -100.0)),
                    stations, receive)


def load_scenario(path: str | Path) -> Scenario:
    with open(path, encoding="utf-8") as f:
        return parse_scenario(yaml.safe_load(f))
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `cd sim && python -m pytest tests/test_morse.py tests/test_scenario.py -q`
Expected: `17 passed`.

- [ ] **Step 5: Commit**

```bash
git add sim/neosim/morse.py sim/neosim/scenario.py sim/tests/test_morse.py sim/tests/test_scenario.py
git commit -m "feat(sim): scenario model with validation and Morse keying"
```

---

### Task 6: Signal sources and channel composition

**Files:**
- Create: `sim/neosim/sources.py`, `sim/neosim/channel.py`
- Test: `sim/tests/test_sources.py`, `sim/tests/test_channel.py`

**Interfaces:**
- Consumes: `Station`, `Scenario`, `FS_HZ`, `WINDOW_HZ` (Task 5); `morse.keying_units` (Task 5); `tools.tool_path("gen_ft8")` (Task 1).
- Produces:
  - `sources.ft8_audio(msg: str, audio_hz: float) -> np.ndarray` (float64, 12 kHz, 15 s).
  - `sources.ft8_baseband(msg, audio_hz, dial_offset_hz, n, fs=FS_HZ) -> np.ndarray[complex]`
  - `sources.cw_envelope(text, wpm, fs, start_s=0.5) -> np.ndarray[float]` (0..1)
  - `sources.cw_baseband(text, wpm, offset_hz, n, fs=FS_HZ) -> np.ndarray[complex]`
  - `sources.carrier_baseband(offset_hz, n, fs=FS_HZ) -> np.ndarray[complex]`
  - `sources.render_station(st: Station, lo_hz: float, n: int, fs=FS_HZ) -> np.ndarray[complex]` — **unit power while active**.
  - `channel.NOISE_POWER = FS_HZ / 2500` (complex noise power per sample ⇒ noise in 2500 Hz = 1).
  - `channel.Exclusion(station_id: str, reason: str)`; `channel.in_window(st, lo_hz) -> bool`; `channel.awgn(n, rng) -> np.ndarray`; `channel.compose(scenario, rng, render=sources.render_station) -> tuple[np.ndarray, list[Exclusion]]`.

- [ ] **Step 1: Write the failing tests**

`sim/tests/test_sources.py`:

```python
import numpy as np
import pytest

from neosim import sources
from neosim.scenario import FS_HZ
from neosim.tools import ToolNotFoundError, tool_path


def _peak_hz(x, fs):
    spec = np.abs(np.fft.fft(x))
    freqs = np.fft.fftfreq(len(x), 1 / fs)
    return freqs[np.argmax(spec)]


def test_cw_envelope_length_matches_keying():
    env = sources.cw_envelope("E", wpm=20, fs=12_000, start_s=0.5)
    dit = 1.2 / 20
    keyed = np.count_nonzero(env > 0.5) / 12_000
    assert keyed == pytest.approx(dit, abs=0.006)
    assert np.all(env[: int(0.49 * 12_000)] == 0)


def test_cw_baseband_has_unit_active_power_at_offset():
    x = sources.cw_baseband("TEST", 20, 2_000.0, FS_HZ * 3)
    active = np.abs(x) > 0.5
    assert np.mean(np.abs(x[active]) ** 2) == pytest.approx(1.0, rel=0.05)
    assert _peak_hz(x, FS_HZ) == pytest.approx(2_000.0, abs=2)


def test_carrier_is_unit_power_tone():
    x = sources.carrier_baseband(-7_000.0, FS_HZ)
    assert np.mean(np.abs(x) ** 2) == pytest.approx(1.0)
    assert _peak_hz(x, FS_HZ) == pytest.approx(-7_000.0, abs=1)


@pytest.mark.integration
def test_ft8_baseband_energy_sits_at_dial_offset_plus_audio():
    try:
        tool_path("gen_ft8")
    except ToolNotFoundError as e:
        pytest.skip(str(e))
    n = FS_HZ * 15
    x = sources.ft8_baseband("CQ EP2ABC LL35", 1_500.0, 14_000.0, n)
    assert len(x) == n
    spec = np.abs(np.fft.fft(x)) ** 2
    freqs = np.fft.fftfreq(n, 1 / FS_HZ)
    band = (freqs > 15_490) & (freqs < 15_560)
    assert spec[band].sum() / spec.sum() > 0.95
```

`sim/tests/test_channel.py`:

```python
import numpy as np
import pytest

from neosim import channel
from neosim.scenario import FS_HZ, Scenario, Station


def scenario(stations):
    return Scenario("t", 14_060_000.0, 2.0, 1, -100.0, tuple(stations), ())


def test_awgn_power_gives_unit_noise_in_2500_hz():
    x = channel.awgn(FS_HZ * 4, np.random.default_rng(0))
    assert np.mean(np.abs(x) ** 2) == pytest.approx(FS_HZ / 2500, rel=0.02)


def test_compose_scales_station_to_snr():
    st = Station("c", "carrier", 14_065_000.0, 20.0)
    iq, excluded = channel.compose(scenario([st]), np.random.default_rng(1))
    t = np.arange(len(iq)) / FS_HZ
    amp = np.abs(np.mean(iq * np.exp(-2j * np.pi * 5_000.0 * t)))
    assert excluded == []
    assert 20 * np.log10(amp) == pytest.approx(20.0, abs=0.3)


def test_compose_excludes_station_outside_window():
    far = Station("bcast", "carrier", 15_400_000.0, 60.0)
    iq, excluded = channel.compose(scenario([far]), np.random.default_rng(2))
    assert [e.station_id for e in excluded] == ["bcast"]
    assert "outside" in excluded[0].reason
    assert np.mean(np.abs(iq) ** 2) == pytest.approx(FS_HZ / 2500, rel=0.03)


def test_window_edge_respects_station_bandwidth():
    lo = 14_060_000.0
    inside = Station("a", "carrier", lo + 43_199.0, 0.0)
    outside = Station("b", "cw", lo + 43_199.0, 0.0, text="E", wpm=20)
    assert channel.in_window(inside, lo)
    assert not channel.in_window(outside, lo)
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `cd sim && python -m pytest tests/test_sources.py tests/test_channel.py -q`
Expected: FAIL — `ImportError`.

- [ ] **Step 3: Implement**

`sim/neosim/sources.py`:

```python
"""Imaginary station generators at complex baseband (unit power while active)."""
from __future__ import annotations

import subprocess
import tempfile
from pathlib import Path

import numpy as np
from scipy.io import wavfile
from scipy.signal import hilbert, resample_poly

from neosim import morse
from neosim.scenario import FS_HZ, Station
from neosim.tools import tool_path

FT8_AUDIO_RATE = 12_000
CW_EDGE_S = 0.005


def _fit(x: np.ndarray, n: int) -> np.ndarray:
    if len(x) >= n:
        return x[:n]
    return np.concatenate([x, np.zeros(n - len(x), dtype=x.dtype)])


def _unit_power(x: np.ndarray, active: np.ndarray) -> np.ndarray:
    if not np.any(active):
        raise ValueError("signal has no active samples")
    return x / np.sqrt(np.mean(np.abs(x[active]) ** 2))


def ft8_audio(msg: str, audio_hz: float) -> np.ndarray:
    """Real 12 kHz FT8 audio for one 15 s slot, generated by ft8_lib's gen_ft8."""
    with tempfile.TemporaryDirectory() as tmp:
        wav = Path(tmp) / "ft8.wav"
        result = subprocess.run([str(tool_path("gen_ft8")), msg, str(wav), f"{audio_hz:.1f}"],
                                capture_output=True, text=True)
        if result.returncode != 0 or not wav.exists():
            raise RuntimeError(f"gen_ft8 failed for {msg!r} (exit {result.returncode}): {result.stderr.strip()}")
        rate, data = wavfile.read(wav)
    if rate != FT8_AUDIO_RATE:
        raise RuntimeError(f"gen_ft8 produced {rate} Hz audio, expected {FT8_AUDIO_RATE}")
    return data.astype(np.float64) / 32768.0


def ft8_baseband(msg: str, audio_hz: float, dial_offset_hz: float, n: int, fs: int = FS_HZ) -> np.ndarray:
    analytic = hilbert(ft8_audio(msg, audio_hz))
    up = _fit(resample_poly(analytic, fs // FT8_AUDIO_RATE, 1), n)
    t = np.arange(n) / fs
    x = up * np.exp(2j * np.pi * dial_offset_hz * t)
    mag = np.abs(x)
    return _unit_power(x, mag > 0.1 * mag.max())


def cw_envelope(text: str, wpm: float, fs: int, start_s: float = 0.5) -> np.ndarray:
    dit = int(round(1.2 / wpm * fs))
    parts = [np.zeros(int(round(start_s * fs)))]
    for key_down, units in morse.keying_units(text):
        parts.append(np.full(units * dit, 1.0 if key_down else 0.0))
    parts.append(np.zeros(int(0.5 * fs)))
    env = np.concatenate(parts)
    k = max(1, int(CW_EDGE_S * fs))
    window = np.hanning(k + 2)[1:-1]
    smooth = np.convolve(env, window / window.sum(), mode="same")  # raised-cosine key edges
    return np.clip(smooth, 0.0, 1.0)


def cw_baseband(text: str, wpm: float, offset_hz: float, n: int, fs: int = FS_HZ) -> np.ndarray:
    env = _fit(cw_envelope(text, wpm, fs), n)
    t = np.arange(n) / fs
    return _unit_power(env * np.exp(2j * np.pi * offset_hz * t), env > 0.99)


def carrier_baseband(offset_hz: float, n: int, fs: int = FS_HZ) -> np.ndarray:
    t = np.arange(n) / fs
    return np.exp(2j * np.pi * offset_hz * t)


def render_station(st: Station, lo_hz: float, n: int, fs: int = FS_HZ) -> np.ndarray:
    offset = st.rf_hz - lo_hz
    if st.mode == "ft8":
        return ft8_baseband(st.msg, st.audio_hz, offset - st.audio_hz, n, fs)
    if st.mode == "cw":
        return cw_baseband(st.text, st.wpm, offset, n, fs)
    if st.mode == "carrier":
        return carrier_baseband(offset, n, fs)
    raise ValueError(f"unknown station mode {st.mode!r}")
```

`sim/neosim/channel.py`:

```python
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
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `cd sim && python -m pytest tests/test_sources.py tests/test_channel.py -q`
Expected: `8 passed` (the FT8 test runs because Task 1 built ft8_lib).

- [ ] **Step 5: Commit**

```bash
git add sim/neosim/sources.py sim/neosim/channel.py sim/tests/test_sources.py sim/tests/test_channel.py
git commit -m "feat(sim): FT8/CW/carrier sources and channel composition with window exclusion"
```

---

### Task 7: Ideal ADC and SigMF writer

**Files:**
- Create: `sim/neosim/adc.py`, `sim/neosim/sigmf.py`
- Test: `sim/tests/test_adc.py`, `sim/tests/test_sigmf.py`

**Interfaces:**
- Consumes: `channel.NOISE_POWER` (Task 6).
- Produces:
  - `adc.AdcStats(clipped_samples: int, peak_dbfs: float, scale: float)`
  - `adc.ideal_adc(iq: np.ndarray, noise_dbfs: float, noise_power: float, bits: int = 24) -> tuple[np.ndarray, AdcStats]` — scales so `noise_power` lands at `noise_dbfs` (dB relative to a full-scale complex sample of per-component amplitude 1), quantises each component to `bits`, saturates to `[-1, 1 - 2^-(bits-1)]`.
  - `sigmf.write_sigmf(base: Path, iq: np.ndarray, sample_rate: float, center_hz: float) -> Path` (returns the `.sigmf-meta` path; data written as cf32_le).

- [ ] **Step 1: Write the failing tests**

`sim/tests/test_adc.py`:

```python
import numpy as np
import pytest

from neosim.adc import ideal_adc


def test_noise_lands_at_requested_dbfs():
    rng = np.random.default_rng(0)
    x = (rng.normal(size=200_000) + 1j * rng.normal(size=200_000)) * np.sqrt(38.4 / 2)
    y, stats = ideal_adc(x, noise_dbfs=-100.0, noise_power=38.4)
    assert 10 * np.log10(np.mean(np.abs(y) ** 2)) == pytest.approx(-100.0, abs=0.2)
    assert stats.clipped_samples == 0


def test_output_is_on_24_bit_grid():
    y, _ = ideal_adc(np.array([0.123456789 + 0.5j]), noise_dbfs=0.0, noise_power=1.0)
    lsb = 2.0 ** -23
    assert y.real[0] / lsb == pytest.approx(round(y.real[0] / lsb), abs=1e-6)


def test_adc_clips_without_wrapping_and_counts():
    x = np.array([10 + 0j, -10 + 0j, 0.1 + 10j, 0.1 - 10j, 0.1 + 0.1j])
    y, stats = ideal_adc(x, noise_dbfs=0.0, noise_power=1.0)
    hi = 1.0 - 2.0 ** -23
    assert stats.clipped_samples == 4
    assert y[0].real == hi and y[1].real == -1.0
    assert y[2].imag == hi and y[3].imag == -1.0
    assert np.all(np.abs(y.real) <= 1.0) and np.all(np.abs(y.imag) <= 1.0)
    assert stats.peak_dbfs == pytest.approx(0.0, abs=1e-5)
```

`sim/tests/test_sigmf.py`:

```python
import json

import numpy as np

from neosim.sigmf import write_sigmf


def test_write_sigmf_round_trip(tmp_path):
    iq = np.array([1 + 2j, -0.5 + 0.25j], dtype=np.complex128)
    meta = write_sigmf(tmp_path / "cap", iq, 96_000, 14_060_000.0)
    assert meta.name == "cap.sigmf-meta"
    doc = json.loads(meta.read_text())
    assert doc["global"]["core:datatype"] == "cf32_le"
    assert doc["global"]["core:sample_rate"] == 96_000
    assert doc["captures"][0]["core:frequency"] == 14_060_000.0
    data = np.fromfile(tmp_path / "cap.sigmf-data", dtype="<c8")
    np.testing.assert_allclose(data, iq.astype(np.complex64))
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `cd sim && python -m pytest tests/test_adc.py tests/test_sigmf.py -q`
Expected: FAIL — `ImportError`.

- [ ] **Step 3: Implement**

`sim/neosim/adc.py`:

```python
"""Ideal ADC: gain, quantisation and saturation; no other impairments."""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class AdcStats:
    clipped_samples: int
    peak_dbfs: float
    scale: float


def ideal_adc(iq: np.ndarray, noise_dbfs: float, noise_power: float, bits: int = 24) -> tuple[np.ndarray, AdcStats]:
    scale = float(np.sqrt(10 ** (noise_dbfs / 10) / noise_power))
    x = iq * scale
    lsb = 2.0 ** -(bits - 1)
    hi = 1.0 - lsb
    clipped = (x.real > hi) | (x.real < -1.0) | (x.imag > hi) | (x.imag < -1.0)
    re = np.clip(np.round(x.real / lsb) * lsb, -1.0, hi)
    im = np.clip(np.round(x.imag / lsb) * lsb, -1.0, hi)
    y = re + 1j * im
    peak = float(max(np.max(np.abs(re)), np.max(np.abs(im)), 1e-30))
    return y, AdcStats(int(np.count_nonzero(clipped)), 20 * np.log10(peak), scale)
```

`sim/neosim/sigmf.py`:

```python
"""Minimal SigMF writer (cf32_le)."""
from __future__ import annotations

import json
from pathlib import Path

import numpy as np


def write_sigmf(base: Path, iq: np.ndarray, sample_rate: float, center_hz: float) -> Path:
    base = Path(base)
    base.parent.mkdir(parents=True, exist_ok=True)
    np.asarray(iq, dtype="<c8").tofile(base.with_suffix(".sigmf-data"))
    meta = base.with_suffix(".sigmf-meta")
    meta.write_text(json.dumps({
        "global": {"core:datatype": "cf32_le", "core:sample_rate": sample_rate, "core:version": "1.0.0",
                   "core:description": "NeoRadio digital twin capture"},
        "captures": [{"core:sample_start": 0, "core:frequency": center_hz}],
        "annotations": [],
    }, indent=2))
    return meta
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `cd sim && python -m pytest tests/test_adc.py tests/test_sigmf.py -q`
Expected: `4 passed`.

- [ ] **Step 5: Commit**

```bash
git add sim/neosim/adc.py sim/neosim/sigmf.py sim/tests/test_adc.py sim/tests/test_sigmf.py
git commit -m "feat(sim): ideal 24-bit ADC and SigMF writer"
```

---

### Task 8: Decoders — ft8_lib wrapper, CW oracle, CER

**Files:**
- Create: `sim/neosim/decoders.py`
- Test: `sim/tests/test_decoders.py`

**Interfaces:**
- Consumes: `tools.tool_path("decode_ft8")` (Task 1); `morse.REVERSE` (Task 5); `sources.cw_envelope`, `sources.ft8_audio` (Task 6, tests only).
- Produces:
  - `decoders.Ft8Decode(audio_hz: int, message: str)`
  - `decoders.parse_ft8_output(text: str) -> list[Ft8Decode]`
  - `decoders.decode_ft8(wav_path: Path) -> list[Ft8Decode]` (raises `RuntimeError` with stderr on non-zero exit)
  - `decoders.decode_cw(audio: np.ndarray, fs: int) -> str`
  - `decoders.cer(decoded: str, expected: str) -> float`

- [ ] **Step 1: Write the failing tests**

`sim/tests/test_decoders.py`:

```python
import numpy as np
import pytest
from scipy.io import wavfile

from neosim import decoders, sources
from neosim.tools import ToolNotFoundError, tool_path

REAL_OUTPUT = """Sample rate 12000 Hz, 180000 samples, 15.000 seconds
Block size = 1920
Subblock size = 960
N_FFT = 3840
#############################################################################################
Max magnitude: -6.0 dB
Decoded 2 messages, callsign hashtable size 1
000000 +16.5 +1.36 1500 ~  CQ EP2ABC LL35
000000 -12.0 +0.80 2100 ~  EP2ABC DL1XYZ JO62
"""


def test_parse_ft8_output_extracts_messages():
    assert decoders.parse_ft8_output(REAL_OUTPUT) == [
        decoders.Ft8Decode(1500, "CQ EP2ABC LL35"),
        decoders.Ft8Decode(2100, "EP2ABC DL1XYZ JO62"),
    ]


def test_parse_ft8_output_empty():
    assert decoders.parse_ft8_output("Decoded 0 messages\n") == []


def test_cer():
    assert decoders.cer("CQ TEST", "CQ TEST") == 0.0
    assert decoders.cer("CQ TEXT", "CQ TEST") == pytest.approx(1 / 7)
    assert decoders.cer(" cq   test ", "CQ TEST") == 0.0
    assert decoders.cer("", "CQ") == 1.0


def _cw_audio(text, wpm, snr_db_500hz, seed):
    fs = 12_000
    env = sources.cw_envelope(text, wpm, fs)
    t = np.arange(len(env)) / fs
    tone = env * np.sin(2 * np.pi * 600 * t)
    rng = np.random.default_rng(seed)
    # noise power in 500 Hz relative to key-down tone power (0.5)
    noise = rng.normal(size=len(env)) * np.sqrt(0.5 / 10 ** (snr_db_500hz / 10) * (fs / 2) / 500)
    return tone + noise, fs


def test_decode_cw_clean():
    audio, fs = _cw_audio("CQ TEST DE EP2ABC", 20, 60, 0)
    assert decoders.decode_cw(audio, fs) == "CQ TEST DE EP2ABC"


def test_decode_cw_noisy_has_low_cer():
    audio, fs = _cw_audio("CQ TEST DE EP2ABC", 22, 12, 1)
    assert decoders.cer(decoders.decode_cw(audio, fs), "CQ TEST DE EP2ABC") <= 0.05


def test_decode_cw_silence_is_empty():
    assert decoders.decode_cw(np.random.default_rng(2).normal(size=24_000) * 1e-3, 12_000) == ""


@pytest.mark.integration
def test_decode_ft8_on_generated_audio(tmp_path):
    try:
        tool_path("gen_ft8"), tool_path("decode_ft8")
    except ToolNotFoundError as e:
        pytest.skip(str(e))
    audio = sources.ft8_audio("CQ EP2ABC LL35", 1500)
    wav = tmp_path / "a.wav"
    wavfile.write(wav, 12_000, (audio * 32767).astype(np.int16))
    assert [d.message for d in decoders.decode_ft8(wav)] == ["CQ EP2ABC LL35"]
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `cd sim && python -m pytest tests/test_decoders.py -q`
Expected: FAIL — `ImportError`.

- [ ] **Step 3: Implement `sim/neosim/decoders.py`**

```python
"""Decoders used as test oracles: ft8_lib for FT8, a simple timing decoder for CW."""
from __future__ import annotations

import re
import subprocess
from dataclasses import dataclass
from pathlib import Path

import numpy as np
from scipy.signal import hilbert

from neosim import morse
from neosim.tools import tool_path

_FT8_LINE = re.compile(r"^\d{6}\s+[+-]?\d+(?:\.\d+)?\s+[+-]?\d+(?:\.\d+)?\s+(\d+)\s+~\s+(.+?)\s*$")


@dataclass(frozen=True)
class Ft8Decode:
    audio_hz: int
    message: str


def parse_ft8_output(text: str) -> list[Ft8Decode]:
    out = []
    for line in text.splitlines():
        m = _FT8_LINE.match(line.strip())
        if m:
            out.append(Ft8Decode(int(m.group(1)), m.group(2)))
    return out


def decode_ft8(wav_path: Path) -> list[Ft8Decode]:
    result = subprocess.run([str(tool_path("decode_ft8")), str(wav_path)], capture_output=True, text=True)
    if result.returncode != 0:
        raise RuntimeError(f"decode_ft8 failed on {wav_path} (exit {result.returncode}): {result.stderr.strip()}")
    return parse_ft8_output(result.stdout)


def _normalise(s: str) -> str:
    return " ".join(s.upper().split())


def cer(decoded: str, expected: str) -> float:
    a, b = _normalise(decoded), _normalise(expected)
    prev = list(range(len(b) + 1))
    for i, ca in enumerate(a, 1):
        cur = [i]
        for j, cb in enumerate(b, 1):
            cur.append(min(prev[j] + 1, cur[j - 1] + 1, prev[j - 1] + (ca != cb)))
        prev = cur
    return prev[-1] / max(1, len(b))


def decode_cw(audio: np.ndarray, fs: int) -> str:
    env = np.abs(hilbert(np.asarray(audio, dtype=float)))
    k = max(1, int(0.01 * fs))
    env = np.convolve(env, np.ones(k) / k, mode="same")
    low, high = np.percentile(env, [10, 99])
    if high < 4 * max(low, 1e-12):
        return ""
    keyed = env > (low + high) / 2
    edges = np.flatnonzero(np.diff(keyed.astype(np.int8))) + 1
    bounds = np.concatenate(([0], edges, [len(keyed)]))
    runs = [(bool(keyed[s]), int(e - s)) for s, e in zip(bounds[:-1], bounds[1:])]
    marks = np.array([length for on, length in runs if on], dtype=float)
    if marks.size == 0:
        return ""
    min_len = int(0.012 * fs)  # shorter marks are noise glitches (a dit at 60 wpm is 20 ms)
    marks = marks[marks >= min_len]
    if marks.size == 0:
        return ""
    rough = np.percentile(marks, 10)
    dit = float(np.median(marks[marks < 2 * rough]))

    text: list[str] = []
    symbol = ""

    def flush():
        nonlocal symbol
        if symbol:
            text.append(morse.REVERSE.get(symbol, "?"))
            symbol = ""

    for on, length in runs:
        units = length / dit
        if on:
            if length < min_len:
                continue
            symbol += "." if units < 2 else "-"
        elif units >= 5:
            flush()
            text.append(" ")
        elif units >= 2:
            flush()
    flush()
    return _normalise("".join(text))
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `cd sim && python -m pytest tests/test_decoders.py -q`
Expected: `7 passed`. If `test_decode_cw_noisy_has_low_cer` fails, print the decoded string, adjust the smoothing length `k` or the threshold (keep the test unchanged), and re-run.

- [ ] **Step 5: Commit**

```bash
git add sim/neosim/decoders.py sim/tests/test_decoders.py
git commit -m "feat(sim): ft8_lib wrapper, CW oracle decoder and CER"
```

---

### Task 9: Digital twin runner (end-to-end with shipping C# DSP)

**Files:**
- Create: `sim/neosim/dsp_runner.py`, `sim/neosim/twin.py`, `sim/scenarios/20m_ft8_cw_ideal.yaml`
- Test: `sim/tests/test_dsp_runner.py`, `sim/tests/test_twin.py`

**Interfaces:**
- Consumes: everything from Tasks 1, 5–8; the `neoradio-dsp` CLI from Task 4.
- Produces:
  - `dsp_runner.run_dsp(meta: Path, wav: Path, mode: str, tune_offset_hz: float) -> None` (raises `RuntimeError` including stderr and exit code).
  - `twin.ReceiveResult(id: str, decoder: str, output: str, passed: bool, detail: str)`
  - `twin.ScenarioResult(scenario: Scenario, excluded: list[Exclusion], adc: AdcStats, receives: list[ReceiveResult], psd_freqs_hz: np.ndarray, psd_db: np.ndarray)` with property `passed: bool`.
  - `twin.run_scenario(scenario: Scenario, workdir: Path) -> ScenarioResult`

- [ ] **Step 1: Write the scenario file**

`sim/scenarios/20m_ft8_cw_ideal.yaml`:

```yaml
name: 20 m FT8 + CW, ideal receiver
lo_hz: 14_060_000
duration_s: 15.0
seed: 1
noise: {type: awgn}
adc: {noise_dbfs: -100}
stations:
  - {id: ft8_weak,   mode: ft8, dial_hz: 14_074_000, audio_hz: 1500, snr_db: -15, msg: "CQ EP2ABC LL35"}
  - {id: ft8_strong, mode: ft8, dial_hz: 14_074_000, audio_hz: 2100, snr_db: -5,  msg: "EP2ABC DL1XYZ JO62"}
  - {id: cw1,        mode: cw,  rf_hz: 14_045_600,  snr_db: 3, text: "CQ TEST DE EP2ABC", wpm: 20}
  - {id: bcast,      mode: carrier, rf_hz: 15_400_000, snr_db: 60}   # outside the window: excluded
receive:
  - {id: rx_ft8, mode: usb, dial_hz: 14_074_000, decoder: ft8, expect: {contains: ["CQ EP2ABC LL35", "EP2ABC DL1XYZ JO62"]}}
  - {id: rx_cw,  mode: cw,  dial_hz: 14_045_000, decoder: cw,  expect: {text: "CQ TEST DE EP2ABC", max_cer: 0.05}}
```

- [ ] **Step 2: Write the failing tests**

`sim/tests/test_dsp_runner.py`:

```python
import stat

import pytest

from neosim.dsp_runner import run_dsp


def test_run_dsp_failure_includes_stderr(monkeypatch, tmp_path):
    fake = tmp_path / "neoradio-dsp"
    fake.write_text("#!/bin/sh\necho 'input error: boom' >&2\nexit 3\n")
    fake.chmod(fake.stat().st_mode | stat.S_IXUSR)
    monkeypatch.setenv("NEOSIM_DSP_CLI", str(fake))
    with pytest.raises(RuntimeError) as err:
        run_dsp(tmp_path / "x.sigmf-meta", tmp_path / "x.wav", "usb", 1000.0)
    assert "exit 3" in str(err.value) and "boom" in str(err.value)


def test_run_dsp_passes_arguments(monkeypatch, tmp_path):
    log = tmp_path / "args.txt"
    fake = tmp_path / "neoradio-dsp"
    fake.write_text(f'#!/bin/sh\necho "$@" > {log}\n')
    fake.chmod(fake.stat().st_mode | stat.S_IXUSR)
    monkeypatch.setenv("NEOSIM_DSP_CLI", str(fake))
    run_dsp(tmp_path / "x.sigmf-meta", tmp_path / "x.wav", "cw", -15000.0)
    args = log.read_text().split()
    assert args[args.index("--mode") + 1] == "cw"
    assert args[args.index("--tune-offset") + 1] == "-15000.000"
```

`sim/tests/test_twin.py`:

```python
from pathlib import Path

import pytest

from neosim.scenario import load_scenario
from neosim.tools import ToolNotFoundError, tool_path
from neosim.twin import run_scenario

SCENARIOS = Path(__file__).resolve().parents[1] / "scenarios"


@pytest.mark.integration
def test_20m_ideal_scenario_passes_end_to_end(tmp_path):
    try:
        for name in ("gen_ft8", "decode_ft8", "neoradio-dsp"):
            tool_path(name)
    except ToolNotFoundError as e:
        pytest.skip(str(e))
    result = run_scenario(load_scenario(SCENARIOS / "20m_ft8_cw_ideal.yaml"), tmp_path)
    details = {r.id: (r.passed, r.output, r.detail) for r in result.receives}
    assert result.passed, details
    assert [e.station_id for e in result.excluded] == ["bcast"]
    assert result.adc.clipped_samples == 0
    assert result.psd_freqs_hz.shape == result.psd_db.shape
```

- [ ] **Step 3: Run tests to verify they fail**

Run: `cd sim && python -m pytest tests/test_dsp_runner.py tests/test_twin.py -q`
Expected: FAIL — `ImportError`.

- [ ] **Step 4: Implement**

`sim/neosim/dsp_runner.py`:

```python
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
```

`sim/neosim/twin.py`:

```python
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
```

- [ ] **Step 5: Run tests to verify they pass**

Run: `make dsp && cd sim && python -m pytest tests/test_dsp_runner.py tests/test_twin.py -q`
Expected: `3 passed`. If the twin test fails, the assertion message shows each receive's decoded output and detail — debug from there (e.g. FT8 missing → inspect `rx_ft8.wav` with `third_party/ft8_lib/decode_ft8`).

- [ ] **Step 6: Commit**

```bash
git add sim/neosim/dsp_runner.py sim/neosim/twin.py sim/scenarios sim/tests/test_dsp_runner.py sim/tests/test_twin.py
git commit -m "feat(sim): digital twin runner using shipping C# DSP end to end"
```

---

### Task 10: FT8 sweep, HTML report, `python -m neosim`, `make sim`

**Files:**
- Create: `sim/neosim/sweep.py`, `sim/neosim/report.py`, `sim/neosim/__main__.py`
- Test: `sim/tests/test_sweep.py`, `sim/tests/test_report.py`, `sim/tests/test_main.py`

**Interfaces:**
- Consumes: `run_scenario`, `ScenarioResult`, `ReceiveResult` (Task 9); `sources.ft8_audio`, `decoders.decode_ft8` (Tasks 6, 8); `parse_scenario` (Task 5).
- Produces:
  - `sweep.SweepPoint(snr_db: float, reference_rate: float, twin_rate: float)`
  - `sweep.sweep_ft8(snrs: list[float], trials: int, workdir: Path, seed: int = 1) -> list[SweepPoint]`
  - `sweep.threshold_snr(snrs, rates, level=0.5) -> float | None` (lowest SNR where the interpolated decode rate reaches `level`)
  - `sweep.implementation_loss_db(points) -> float | None` (twin threshold − reference threshold; positive = twin needs more SNR)
  - `report.write_report(results: list[ScenarioResult], sweep_points: list[SweepPoint] | None, path: Path) -> Path`
  - `python -m neosim run SCENARIO... [--workdir DIR] [--report FILE] [--sweep-ft8 START STOP STEP] [--trials N]` → exit 0 if all scenarios pass, 1 otherwise; `neosim.__main__.main(argv: list[str]) -> int`.

- [ ] **Step 1: Write the failing tests**

`sim/tests/test_sweep.py`:

```python
import pytest

from neosim import sweep
from neosim.tools import ToolNotFoundError, tool_path


def test_threshold_snr_interpolates():
    assert sweep.threshold_snr([-10, -14, -18], [1.0, 1.0, 0.0]) == pytest.approx(-16.0)


def test_threshold_snr_none_when_never_reached():
    assert sweep.threshold_snr([-10, -14], [0.25, 0.0]) is None


def test_implementation_loss_positive_when_twin_worse():
    points = [sweep.SweepPoint(-10, 1.0, 1.0), sweep.SweepPoint(-14, 1.0, 0.0), sweep.SweepPoint(-18, 0.0, 0.0)]
    # reference crosses 0.5 at -16, twin at -12
    assert sweep.implementation_loss_db(points) == pytest.approx(4.0)


@pytest.mark.integration
def test_small_sweep_runs(tmp_path):
    try:
        for name in ("gen_ft8", "decode_ft8", "neoradio-dsp"):
            tool_path(name)
    except ToolNotFoundError as e:
        pytest.skip(str(e))
    points = sweep.sweep_ft8([-8.0, -30.0], trials=1, workdir=tmp_path)
    assert [(p.reference_rate, p.twin_rate) for p in points] == [(1.0, 1.0), (0.0, 0.0)]
```

`sim/tests/test_report.py`:

```python
import numpy as np

from neosim.adc import AdcStats
from neosim.channel import Exclusion
from neosim.report import write_report
from neosim.scenario import Scenario
from neosim.sweep import SweepPoint
from neosim.twin import ReceiveResult, ScenarioResult


def test_report_contains_results(tmp_path):
    scen = Scenario("demo scenario", 14_060_000.0, 15.0, 1, -100.0, (), ())
    res = ScenarioResult(
        scen, [Exclusion("bcast", "outside window")], AdcStats(0, -40.0, 1.0),
        [ReceiveResult("rx_ft8", "ft8", "CQ EP2ABC LL35", True, "ok"),
         ReceiveResult("rx_cw", "cw", "CQ TEXT", False, "CER 0.143")],
        np.linspace(-48_000, 48_000, 16), np.zeros(16))
    path = write_report([res], [SweepPoint(-10, 1.0, 1.0), SweepPoint(-20, 0.0, 0.0)], tmp_path / "r.html")
    html = path.read_text()
    assert "demo scenario" in html
    assert "FAIL" in html and "PASS" in html
    assert "bcast" in html and "outside window" in html
    assert "Implementation loss" in html
    assert html.count("data:image/png;base64,") == 2
```

`sim/tests/test_main.py`:

```python
from pathlib import Path

import pytest

from neosim.__main__ import main
from neosim.tools import ToolNotFoundError, tool_path

SCENARIOS = Path(__file__).resolve().parents[1] / "scenarios"


def test_bad_scenario_file_returns_error(tmp_path, capsys):
    bad = tmp_path / "bad.yaml"
    bad.write_text("name: x\n")
    assert main(["run", str(bad), "--workdir", str(tmp_path), "--report", str(tmp_path / "r.html")]) == 2
    assert "lo_hz" in capsys.readouterr().err


@pytest.mark.integration
def test_run_writes_report(tmp_path):
    try:
        for name in ("gen_ft8", "decode_ft8", "neoradio-dsp"):
            tool_path(name)
    except ToolNotFoundError as e:
        pytest.skip(str(e))
    report = tmp_path / "report.html"
    code = main(["run", str(SCENARIOS / "20m_ft8_cw_ideal.yaml"), "--workdir", str(tmp_path), "--report", str(report)])
    assert code == 0
    assert report.exists()
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `cd sim && python -m pytest tests/test_sweep.py tests/test_report.py tests/test_main.py -q`
Expected: FAIL — `ImportError`.

- [ ] **Step 3: Implement**

`sim/neosim/sweep.py`:

```python
"""FT8 decode rate vs SNR: audio-only reference vs full digital twin."""
from __future__ import annotations

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
    return run_scenario(scenario, workdir).passed


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
        points.append(SweepPoint(float(snr), ref_ok / trials, twin_ok / trials))
    return points


def threshold_snr(snrs: list[float], rates: list[float], level: float = 0.5) -> float | None:
    pairs = sorted(zip(snrs, rates), reverse=True)  # strongest first
    for (s_hi, r_hi), (s_lo, r_lo) in zip(pairs, pairs[1:]):
        if r_hi >= level > r_lo:
            return s_hi + (r_hi - level) / (r_hi - r_lo) * (s_lo - s_hi)
    if pairs and all(r >= level for _, r in pairs):
        return pairs[-1][0]
    return None


def implementation_loss_db(points: list[SweepPoint]) -> float | None:
    snrs = [p.snr_db for p in points]
    ref = threshold_snr(snrs, [p.reference_rate for p in points])
    twin = threshold_snr(snrs, [p.twin_rate for p in points])
    if ref is None or twin is None:
        return None
    return twin - ref
```

`sim/neosim/report.py`:

```python
"""Self-contained HTML report with embedded plots."""
from __future__ import annotations

import base64
import html
import io
from datetime import datetime
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

from neosim.sweep import SweepPoint, implementation_loss_db  # noqa: E402
from neosim.twin import ScenarioResult  # noqa: E402

_STYLE = """
body{font-family:system-ui,sans-serif;max-width:1000px;margin:2rem auto;padding:0 16px;color:#222}
table{border-collapse:collapse;width:100%;margin:.5rem 0 1.5rem}
td,th{border:1px solid #ccc;padding:.35rem .5rem;text-align:left;font-size:.9rem}
.pass{color:#0a7a2f;font-weight:600}.fail{color:#b00020;font-weight:600}
img{max-width:100%}
"""


def _png(fig) -> str:
    buf = io.BytesIO()
    fig.savefig(buf, format="png", dpi=110, bbox_inches="tight")
    plt.close(fig)
    return "data:image/png;base64," + base64.b64encode(buf.getvalue()).decode()


def _badge(ok: bool) -> str:
    return '<span class="pass">PASS</span>' if ok else '<span class="fail">FAIL</span>'


def _scenario_html(r: ScenarioResult) -> str:
    e = html.escape
    fig, ax = plt.subplots(figsize=(9, 3))
    ax.plot(r.psd_freqs_hz / 1e3, r.psd_db, lw=0.7)
    ax.set_xlabel(f"offset from LO {r.scenario.lo_hz / 1e6:.6f} MHz (kHz)")
    ax.set_ylabel("PSD (dBFS/Hz)")
    ax.grid(alpha=0.3)
    rows = "".join(
        f"<tr><td>{e(x.id)}</td><td>{e(x.decoder)}</td><td>{_badge(x.passed)}</td>"
        f"<td>{e(x.output)}</td><td>{e(x.detail)}</td></tr>" for x in r.receives)
    excl = "".join(f"<li>{e(x.station_id)}: {e(x.reason)}</li>" for x in r.excluded) or "<li>none</li>"
    return (f"<h2>{e(r.scenario.name)} {_badge(r.passed)}</h2>"
            f"<p>ADC: peak {r.adc.peak_dbfs:.1f} dBFS, clipped samples {r.adc.clipped_samples}</p>"
            f"<p>Excluded stations:</p><ul>{excl}</ul>"
            f"<table><tr><th>receive</th><th>decoder</th><th>result</th><th>output</th><th>detail</th></tr>{rows}</table>"
            f'<img alt="spectrum" src="{_png(fig)}">')


def _sweep_html(points: list[SweepPoint]) -> str:
    fig, ax = plt.subplots(figsize=(7, 3.5))
    snrs = [p.snr_db for p in points]
    ax.plot(snrs, [p.reference_rate for p in points], "o-", label="reference (audio only)")
    ax.plot(snrs, [p.twin_rate for p in points], "s-", label="digital twin")
    ax.set_xlabel("SNR in 2500 Hz (dB)")
    ax.set_ylabel("FT8 decode rate")
    ax.invert_xaxis()
    ax.grid(alpha=0.3)
    ax.legend()
    loss = implementation_loss_db(points)
    loss_txt = "not determined (a curve never crosses 50 %)" if loss is None else f"{loss:+.1f} dB"
    rows = "".join(f"<tr><td>{p.snr_db:+.1f}</td><td>{p.reference_rate:.2f}</td><td>{p.twin_rate:.2f}</td></tr>"
                   for p in points)
    return (f"<h2>FT8 sensitivity sweep</h2><p><b>Implementation loss:</b> {loss_txt}</p>"
            f"<table><tr><th>SNR (dB)</th><th>reference</th><th>twin</th></tr>{rows}</table>"
            f'<img alt="sweep" src="{_png(fig)}">')


def write_report(results: list[ScenarioResult], sweep_points: list[SweepPoint] | None, path: Path) -> Path:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    body = "".join(_scenario_html(r) for r in results)
    if sweep_points:
        body += _sweep_html(sweep_points)
    overall = all(r.passed for r in results)
    path.write_text(
        f"<!doctype html><html><head><meta charset='utf-8'><title>NeoRadio Sim Report</title>"
        f"<style>{_STYLE}</style></head><body><h1>NeoRadio simulation report {_badge(overall)}</h1>"
        f"<p>Generated {datetime.now():%Y-%m-%d %H:%M}</p>{body}</body></html>",
        encoding="utf-8")
    return path
```

`sim/neosim/__main__.py`:

```python
"""Command line: python -m neosim run SCENARIO... --report FILE"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

from neosim.report import write_report
from neosim.scenario import ScenarioError, load_scenario
from neosim.sweep import sweep_ft8
from neosim.tools import ToolNotFoundError
from neosim.twin import run_scenario


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(prog="neosim")
    sub = parser.add_subparsers(dest="command", required=True)
    run = sub.add_parser("run", help="run scenarios and write an HTML report")
    run.add_argument("scenarios", nargs="+", type=Path)
    run.add_argument("--workdir", type=Path, default=Path("build/sim"))
    run.add_argument("--report", type=Path, default=Path("build/sim/report.html"))
    run.add_argument("--sweep-ft8", nargs=3, type=float, metavar=("START", "STOP", "STEP"))
    run.add_argument("--trials", type=int, default=4)
    args = parser.parse_args(argv)

    try:
        scenarios = [load_scenario(p) for p in args.scenarios]
    except (ScenarioError, OSError) as e:
        print(f"scenario error: {e}", file=sys.stderr)
        return 2

    try:
        results = []
        for path, scenario in zip(args.scenarios, scenarios):
            result = run_scenario(scenario, args.workdir / path.stem)
            print(f"{'PASS' if result.passed else 'FAIL'}  {scenario.name}")
            results.append(result)
        points = None
        if args.sweep_ft8:
            start, stop, step = args.sweep_ft8
            snrs = list(np.arange(start, stop - 1e-9, -abs(step)))
            points = sweep_ft8(snrs, args.trials, args.workdir / "sweep")
    except (ToolNotFoundError, RuntimeError) as e:
        print(f"error: {e}", file=sys.stderr)
        return 2

    report = write_report(results, points, args.report)
    print(f"report: {report}")
    return 0 if all(r.passed for r in results) else 1


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `cd sim && python -m pytest tests/test_sweep.py tests/test_report.py tests/test_main.py -q`
Expected: `7 passed`.

- [ ] **Step 5: Run the full pipeline**

Run: `make test && make sim`
Expected: all .NET and Python tests pass; `make sim` prints `PASS  20 m FT8 + CW, ideal receiver` and `report: …/build/sim/report.html`. Open the report and check that the sweep shows both curves and an implementation loss close to 0 dB (an ideal receiver should be within about ±1 dB of the reference; a larger loss is a DSP bug to investigate before Phase 2).

- [ ] **Step 6: Commit**

```bash
git add sim/neosim/sweep.py sim/neosim/report.py sim/neosim/__main__.py sim/tests/test_sweep.py sim/tests/test_report.py sim/tests/test_main.py
git commit -m "feat(sim): FT8 sensitivity sweep, HTML report and neosim CLI"
```

---

## Self-review notes (for the executor)

- Spec §1 criterion 1 → Task 10 sweep + implementation loss. §8 C# solution layout → Tasks 1–4 (`NeoRadio.Daemon` is a later phase). §9 L1 → ideal models only in this phase (Tasks 6–7); L4 → Tasks 5–10. Hardware models (filters, mixer impairments, phase noise, ITU noise) are Phase 2+.
- Hardware-dependent items (KiCad, ngspice, Qucs-S, openEMS installation) are deliberately not in this phase; they arrive with the first circuit block.
