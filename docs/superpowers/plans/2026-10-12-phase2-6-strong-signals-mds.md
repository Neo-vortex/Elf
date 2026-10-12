# Phase 2.6: Strong-Signal Effects, I/Q Correction, MDS and the Hardware FT8 Sweep

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Complete the twin's hardware receiver (spec §7): IMD3 from station pairs, reciprocal mixing from the Si5351 phase noise, overload reporting, I/Q imbalance correction in the C# DSP, and the hardware checks of spec §1 (MDS, blocker, FT8 implementation loss with the hardware model).

**Architecture:** C# gains `NeoRadio.Dsp.IqCorrector` and `neoradio-dsp --iq-gain-db/--iq-phase-deg`. `neosim/hardware.py` reads a hand-entered `lo_phase_noise.json`, splits the front end at the preamp input, and adds reciprocal-mixing noise and IMD3 products to the antenna-referred envelope before the existing front-end → mixer → ADC chain; the twin passes the imbalance it applied to the DSP. A new `neosim/measure.py` measures MDS on the twin's I/Q and runs the blocker scenario with and without its blocker; the FT8 sweep gains a hardware curve. `neohw` is unchanged (no `make hw`).

**Tech Stack:** C# .NET 10 (xUnit), Python 3.13 (numpy, pytest).

**Spec:** `docs/superpowers/specs/2026-10-12-phase2-frontend-mixer-design.md` (§1 items 2–4, §6 `lo_phase_noise.json`, §7 and its plan 2.6 amendment); parent `docs/superpowers/specs/2026-10-12-neoradio-hf-receiver-design.md`.

All code here ran in a prototype worktree from `main` at 34db44a; Expected outputs are from those runs.

## Global Constraints

- The twin reads committed characterisation files and never runs ngspice; it refuses missing files, another schema_version or status `fail`.
- I/Q imbalance convention (neohw `iq_imbalance`, mixer.json, twin, C#): z = I − jQ; gain error = I over Q in dB; phase error = departure of Q from quadrature; the receiver delivers Q' = (Q·cos φ − I·sin φ)/g.
- `neoradio-dsp` stays a standard CLI: new options are optional, finite, gain within ±6 dB and phase within ±30°, out of range → usage error (exit 2).
- Phase 1 behaviour unchanged: ideal scenarios, SNR mode and the ideal FT8 sweep curve.
- MDS target: ≤ −130 dBm in 500 Hz and within ±1 dB of the noise budget (14.1 and 50.1 MHz, preamp on, thermal noise only).
- FT8 implementation loss with the hardware model ≤ 1 dB (1 dB steps, 20 trials, 95 % intervals shown).
- Host software must not depend on Pi-only features; tests run on x64 desktop Linux.
- NuGet is unreachable without the VPN: in a fresh worktree restore offline with `dotnet restore pi/NeoRadio.slnx --source ~/.nuget/packages`.
- Long runs (`make sim`, `make test`) run with a completion notification; no polling.

## Review Focus

- A station far outside the filter passband but inside the s2p range: its IMD3 and reciprocal mixing must be scaled down by the filter's S21 at its frequency, never applied at full level.
- Two in-window stations whose IMD3 product also lands in the window: the product is added once per ordered pair, and the decode of the wanted stations still works.
- The C# corrector receiving the twin's p5 imbalance (gain ≈ 0.16 dB, phase ≈ 0.43°): the corrected capture must decode at least as well as without correction (no sign error that doubles the imbalance).
- The MDS measurement on real characterisation: the DC offset and the tone's I/Q image must not be counted as noise.
- The blocker scenario at a level near overload (e.g. the carrier raised to −15 dBm): the twin must report the overload risk rather than silently decoding as if nothing happened.

---
### Task 1: I/Q imbalance correction in the C# DSP

**Files:**
- Create: `pi/src/NeoRadio.Dsp/IqCorrector.cs`, `pi/tests/NeoRadio.Dsp.Tests/IqCorrectorTests.cs`
- Modify: `pi/src/NeoRadio.Dsp.Cli/CliOptions.cs`, `pi/src/NeoRadio.Dsp.Cli/CliApp.cs`, `pi/tests/NeoRadio.Dsp.Cli.Tests/CliTests.cs`

**Interfaces:**
- Produces: `NeoRadio.Dsp.IqCorrector(double gainDb, double phaseDeg)` with `MaxGainDb = 6.0`, `MaxPhaseDeg = 30.0`, `bool IsIdentity`, `void Correct(Span<Complex> iq)` (throws `ArgumentException` for non-finite or out-of-range values); `CliOptions.IqGainDb`, `CliOptions.IqPhaseDeg` (default 0); CLI options `--iq-gain-db`, `--iq-phase-deg`, applied to the samples before `Receiver.Process`.

- [ ] **Step 1: Write the failing corrector tests**

Create `pi/tests/NeoRadio.Dsp.Tests/IqCorrectorTests.cs`:

```csharp
using System.Numerics;
using NeoRadio.Dsp;
using Xunit;

namespace NeoRadio.Dsp.Tests;

public class IqCorrectorTests
{
    const double Fs = 96000;

    /// <summary>The mixer's imbalance as neohw measures it and the twin applies it:
    /// q = −Im(z·e^{jφ}) / g with z = I − jQ (gain I over Q, phase of Q from quadrature).</summary>
    internal static Complex[] Imbalance(Complex[] z, double gainDb, double phaseDeg)
    {
        double g = Math.Pow(10, gainDb / 20);
        var rot = Complex.FromPolarCoordinates(1, phaseDeg * Math.PI / 180);
        return z.Select(s => new Complex(s.Real, (s * rot).Imaginary / g)).ToArray();
    }

    static double Bin(Complex[] z, double f)
    {
        Complex acc = Complex.Zero;
        for (int i = 0; i < z.Length; i++) acc += z[i] * Complex.FromPolarCoordinates(1, -2 * Math.PI * f * i / Fs);
        return acc.Magnitude / z.Length;
    }

    static double ImageRejectionDb(Complex[] z, double f) => 20 * Math.Log10(Bin(z, f) / Bin(z, -f));

    [Theory]
    [InlineData(0.16, 0.43)]     // mixer.json p5 run, direct path
    [InlineData(-0.5, 3.0)]
    [InlineData(1.0, -5.0)]
    public void Known_imbalance_is_corrected_to_at_least_60_dB_image_rejection(double gainDb, double phaseDeg)
    {
        var z = Imbalance(FilterTests.Tone(12000, Fs, 9600, 0.5), gainDb, phaseDeg);
        Assert.True(ImageRejectionDb(z, 12000) < 50);
        new IqCorrector(gainDb, phaseDeg).Correct(z);
        Assert.True(ImageRejectionDb(z, 12000) >= 60, $"{ImageRejectionDb(z, 12000):F1} dB");
        Assert.InRange(Bin(z, 12000), 0.49, 0.51);
    }

    [Fact]
    public void Zero_imbalance_leaves_samples_unchanged()
    {
        var z = FilterTests.Tone(-7000, Fs, 960, 0.3);
        var copy = z.ToArray();
        var c = new IqCorrector(0, 0);
        Assert.True(c.IsIdentity);
        c.Correct(z);
        Assert.Equal(copy, z);
    }

    [Theory]
    [InlineData(double.NaN, 0)]
    [InlineData(0, double.PositiveInfinity)]
    [InlineData(6.5, 0)]
    [InlineData(0, -31)]
    public void Out_of_range_or_non_finite_imbalance_is_rejected(double gainDb, double phaseDeg)
    {
        Assert.Throws<ArgumentException>(() => new IqCorrector(gainDb, phaseDeg));
    }
}
```

- [ ] **Step 2: Run them to verify they fail**

Run: `dotnet build pi/tests/NeoRadio.Dsp.Tests -c Release`
Expected: `error CS0246: The type or namespace name 'IqCorrector' could not be found`

- [ ] **Step 3: Implement the corrector**

Create `pi/src/NeoRadio.Dsp/IqCorrector.cs`:

```csharp
using System.Numerics;

namespace NeoRadio.Dsp;

/// <summary>
/// Removes a known I/Q gain and phase imbalance before demodulation. The imbalance follows the
/// convention of the hardware characterisation (neohw's mixer check, mixer.json): samples are
/// z = I − jQ, the gain error is I over Q in dB, and the phase error is the departure of Q from
/// quadrature, so the receiver delivers Q' = (Q·cos φ − I·sin φ) / g. Correction recovers
/// Q = (g·Q' + I·sin φ) / cos φ and leaves I unchanged.
/// </summary>
public sealed class IqCorrector
{
    public const double MaxGainDb = 6.0;
    public const double MaxPhaseDeg = 30.0;

    private readonly double _gain;
    private readonly double _sin;
    private readonly double _cos;

    public IqCorrector(double gainDb, double phaseDeg)
    {
        if (!double.IsFinite(gainDb) || Math.Abs(gainDb) > MaxGainDb)
            throw new ArgumentException($"I/Q gain error must be finite and within ±{MaxGainDb} dB, got {gainDb}");
        if (!double.IsFinite(phaseDeg) || Math.Abs(phaseDeg) > MaxPhaseDeg)
            throw new ArgumentException($"I/Q phase error must be finite and within ±{MaxPhaseDeg}°, got {phaseDeg}");
        _gain = Math.Pow(10, gainDb / 20);
        double phi = phaseDeg * Math.PI / 180;
        _sin = Math.Sin(phi);
        _cos = Math.Cos(phi);
        IsIdentity = gainDb == 0 && phaseDeg == 0;
    }

    public bool IsIdentity { get; }

    public void Correct(Span<Complex> iq)
    {
        if (IsIdentity) return;
        for (int k = 0; k < iq.Length; k++)
        {
            double i = iq[k].Real;
            double qMeasured = -iq[k].Imaginary;
            double q = (_gain * qMeasured + i * _sin) / _cos;
            iq[k] = new Complex(i, -q);
        }
    }
}
```

- [ ] **Step 4: Run the DSP tests**

Run: `dotnet test pi/tests/NeoRadio.Dsp.Tests -c Release`
Expected: `Passed: 34` (26 existing + 8 new).

- [ ] **Step 5: Write the failing CLI tests**

Apply from the repo root:

```bash
git apply <<'PATCH'
diff --git a/pi/tests/NeoRadio.Dsp.Cli.Tests/CliTests.cs b/pi/tests/NeoRadio.Dsp.Cli.Tests/CliTests.cs
index c535125..d77bfa1 100644
--- a/pi/tests/NeoRadio.Dsp.Cli.Tests/CliTests.cs
+++ b/pi/tests/NeoRadio.Dsp.Cli.Tests/CliTests.cs
@@ -188,3 +188,62 @@ public sealed class CliTests : IDisposable
         Assert.Contains("output error", err);
     }
+
+    static Complex[] Imbalanced(Complex[] z, double gainDb, double phaseDeg)
+    {
+        double g = Math.Pow(10, gainDb / 20);
+        var rot = Complex.FromPolarCoordinates(1, phaseDeg * Math.PI / 180);
+        return z.Select(s => new Complex(s.Real, (s * rot).Imaginary / g)).ToArray();
+    }
+
+    /// <summary>RMS of a 16-bit mono WAV after the filters have settled (skips the start-up transient).</summary>
+    static double SteadyRms(string wav, int skip = 2000)
+    {
+        var bytes = File.ReadAllBytes(wav);
+        var samples = Enumerable.Range(0, (bytes.Length - 44) / 2).Skip(skip)
+            .Select(i => BitConverter.ToInt16(bytes, 44 + 2 * i) / 32768.0).ToArray();
+        return Math.Sqrt(samples.Average(v => v * v));
+    }
+
+    [Fact]
+    public void Iq_correction_removes_the_image_of_a_station_on_the_other_side()
+    {
+        // a strong station at −15.5 kHz; its image (−20 dB from a 3 dB / 10° imbalance) lands on
+        // the USB channel at +15.5 kHz (tune 14 kHz, audio 1.5 kHz)
+        var input = Imbalanced(Tone(-15500, 96000, 96000, 0.5), 3.0, 10.0);
+        var meta = WriteSigMf("img", "cf32_le", 96000, input);
+        double Rms(params string[] extra)
+        {
+            var wav = Path.Combine(_dir, $"o{extra.Length}.wav");
+            var (code, err) = Run(new[] { "--input", meta, "--output", wav, "--tune-offset", "14000", "--no-agc" }
+                .Concat(extra).ToArray());
+            Assert.True(code == 0, err);
+            return SteadyRms(wav);
+        }
+        double raw = Rms();
+        double corrected = Rms("--iq-gain-db", "3", "--iq-phase-deg", "10");
+        Assert.True(raw > 0.01, $"image before correction {raw}");
+        Assert.True(corrected < raw / 1000, $"image after correction {corrected} vs {raw}");   // > 60 dB
+    }
+
+    [Theory]
+    [InlineData("--iq-gain-db", "7")]
+    [InlineData("--iq-phase-deg", "-45")]
+    [InlineData("--iq-gain-db", "NaN")]
+    public void Iq_imbalance_out_of_range_is_a_usage_error(string option, string value)
+    {
+        var (code, err) = Run("--input", "a.sigmf-meta", "--output", "b.wav", "--tune-offset", "0", option, value);
+        Assert.Equal(2, code);
+        Assert.Contains(option, err);
+    }
+
+    [Fact]
+    public void Iq_options_default_to_no_correction()
+    {
+        var o = CliOptions.Parse(new[] { "--input", "a", "--output", "b", "--tune-offset", "0" });
+        Assert.Equal(0.0, o.IqGainDb);
+        Assert.Equal(0.0, o.IqPhaseDeg);
+        var p = CliOptions.Parse(new[] { "--input", "a", "--output", "b", "--tune-offset", "0", "--iq-gain-db", "0.16", "--iq-phase-deg", "-0.43" });
+        Assert.Equal(0.16, p.IqGainDb);
+        Assert.Equal(-0.43, p.IqPhaseDeg);
+    }
 }
PATCH
```

Run: `dotnet build pi/tests/NeoRadio.Dsp.Cli.Tests -c Release`
Expected: `error CS1061: 'CliOptions' does not contain a definition for 'IqGainDb'`

The end-to-end test measures the steady-state RMS of the WAV (after 2000 samples): the WAV's peak includes the strong station's filter start-up transient and hides the correction.

- [ ] **Step 6: Implement the options and apply the corrector**

Apply from the repo root:

```bash
git apply <<'PATCH'
diff --git a/pi/src/NeoRadio.Dsp.Cli/CliOptions.cs b/pi/src/NeoRadio.Dsp.Cli/CliOptions.cs
index 341b27c..9f24f33 100644
--- a/pi/src/NeoRadio.Dsp.Cli/CliOptions.cs
+++ b/pi/src/NeoRadio.Dsp.Cli/CliOptions.cs
@@ -17,10 +17,12 @@ public sealed record CliOptions(
     double CwBandwidthHz,
     bool AgcEnabled,
-    string? MetricsPath)
+    string? MetricsPath,
+    double IqGainDb = 0,
+    double IqPhaseDeg = 0)
 {
     public const string Usage =
         "usage: neoradio-dsp --input X.sigmf-meta --output Y.wav --tune-offset HZ [--mode usb|lsb|cw] " +
         "[--audio-rate 12000] [--low-cut 200] [--high-cut 2800] [--cw-pitch 600] [--cw-bandwidth 500] " +
-        "[--no-agc] [--metrics M.json]";
+        "[--no-agc] [--metrics M.json] [--iq-gain-db 0] [--iq-phase-deg 0]";
 
     public static CliOptions Parse(string[] args)
@@ -33,5 +35,5 @@ public sealed record CliOptions(
             if (key == "--no-agc") { agc = false; continue; }
             if (key is not ("--input" or "--output" or "--mode" or "--tune-offset" or "--audio-rate" or "--low-cut"
-                or "--high-cut" or "--cw-pitch" or "--cw-bandwidth" or "--metrics"))
+                or "--high-cut" or "--cw-pitch" or "--cw-bandwidth" or "--metrics" or "--iq-gain-db" or "--iq-phase-deg"))
                 throw new CliUsageException($"unknown option {key}");
             if (i + 1 >= args.Length) throw new CliUsageException($"{key} needs a value");
@@ -52,4 +54,10 @@ public sealed record CliOptions(
         if (mode is not ("usb" or "lsb" or "cw")) throw new CliUsageException($"--mode must be usb, lsb or cw, got '{mode}'");
 
+        double iqGain = Number("--iq-gain-db", 0), iqPhase = Number("--iq-phase-deg", 0);
+        if (Math.Abs(iqGain) > IqCorrector.MaxGainDb)
+            throw new CliUsageException($"--iq-gain-db must be within ±{IqCorrector.MaxGainDb} dB, got {iqGain}");
+        if (Math.Abs(iqPhase) > IqCorrector.MaxPhaseDeg)
+            throw new CliUsageException($"--iq-phase-deg must be within ±{IqCorrector.MaxPhaseDeg}°, got {iqPhase}");
+
         return new CliOptions(
             Required("--input"),
@@ -63,5 +71,7 @@ public sealed record CliOptions(
             Number("--cw-bandwidth", 500),
             agc,
-            values.GetValueOrDefault("--metrics"));
+            values.GetValueOrDefault("--metrics"),
+            iqGain,
+            iqPhase);
     }
 
PATCH
```

Apply from the repo root:

```bash
git apply <<'PATCH'
diff --git a/pi/src/NeoRadio.Dsp.Cli/CliApp.cs b/pi/src/NeoRadio.Dsp.Cli/CliApp.cs
index 6f154fb..f990fc5 100644
--- a/pi/src/NeoRadio.Dsp.Cli/CliApp.cs
+++ b/pi/src/NeoRadio.Dsp.Cli/CliApp.cs
@@ -46,4 +46,5 @@ public static class CliApp
         }
 
+        new IqCorrector(options.IqGainDb, options.IqPhaseDeg).Correct(recording.Samples);
         float[] audio = receiver.Process(recording.Samples);
         try
PATCH
```

- [ ] **Step 7: Run all C# tests**

Run: `dotnet test pi/NeoRadio.slnx -c Release`
Expected: `NeoRadio.Dsp.Tests` 34 passed, `NeoRadio.Dsp.Cli.Tests` 22 passed.

- [ ] **Step 8: Commit**

```bash
git add pi/src/NeoRadio.Dsp/IqCorrector.cs pi/tests/NeoRadio.Dsp.Tests/IqCorrectorTests.cs pi/src/NeoRadio.Dsp.Cli/CliOptions.cs pi/src/NeoRadio.Dsp.Cli/CliApp.cs pi/tests/NeoRadio.Dsp.Cli.Tests/CliTests.cs
git commit -m "feat(dsp): I/Q imbalance correction and --iq-gain-db/--iq-phase-deg"
```
### Task 2: The LO phase-noise file

**Files:**
- Create: `hardware/characterized/lo_phase_noise.json`
- Modify: `hardware/tests/test_characterized_contract.py`

**Interfaces:**
- Produces: `lo_phase_noise.json` {schema_version 1, block "lo_phase_noise", status "pass", generated_by "hand-entered (plan 2.6)", reference_hz 9e6, offset_hz [100 … 1e6], dbc_per_hz [−103, −128, −136, −138, −144], scaling, source}. `make hw` does not write or delete it.

- [ ] **Step 1: Write the failing contract test**

Apply from the repo root:

```bash
git apply <<'PATCH'
diff --git a/hardware/tests/test_characterized_contract.py b/hardware/tests/test_characterized_contract.py
index a9eadc3..45b1546 100644
--- a/hardware/tests/test_characterized_contract.py
+++ b/hardware/tests/test_characterized_contract.py
@@ -26,2 +26,12 @@ def test_preamp_grid_reaches_5x_the_top_band():
     assert p["freq_hz"][0] <= 0.1e6 * 1.001 and p["freq_hz"][-1] >= 270e6
     assert len(p["freq_hz"]) == len(p["gain_db"]) == len(p["nf_db"])
+
+
+def test_lo_phase_noise_is_hand_entered_with_its_source():
+    """Not generated by make hw: the Si5351 datasheet gives only jitter, so the curve is a cited
+    measurement (spec §6, amended in plan 2.6)."""
+    pn = load("lo_phase_noise.json")
+    assert pn["schema_version"] == 1 and pn["block"] == "lo_phase_noise" and pn["status"] == "pass"
+    assert "rfzero.net" in pn["source"] and pn["reference_hz"] == 9e6
+    assert pn["offset_hz"] == sorted(pn["offset_hz"]) and len(pn["offset_hz"]) == len(pn["dbc_per_hz"])
+    assert all(-160 < v < -90 for v in pn["dbc_per_hz"])
PATCH
```

Run: `cd hardware && python3 -m pytest -q tests/test_characterized_contract.py`
Expected: `FileNotFoundError` for `lo_phase_noise.json`.

- [ ] **Step 2: Add the file**

Create `hardware/characterized/lo_phase_noise.json`:

```json
{
 "schema_version": 1,
 "block": "lo_phase_noise",
 "status": "pass",
 "generated_by": "hand-entered (plan 2.6)",
 "reference_hz": 9000000.0,
 "offset_hz": [
  100.0,
  1000.0,
  10000.0,
  100000.0,
  1000000.0
 ],
 "dbc_per_hz": [
  -103.0,
  -128.0,
  -136.0,
  -138.0,
  -144.0
 ],
 "scaling": "L(f_lo) = L(reference) + 20*log10(f_lo / reference_hz); held flat below 100 Hz and above 1 MHz offset. The \u00f74 path (LO < 3.2 MHz) divides a 4*f_lo Si5351 output by 4, which gives the same scaling.",
 "source": "Si5351A measured at 9 MHz by RFzero (https://rfzero.net/documentation/rf/) on a Rohde & Schwarz FSIQ 26 with the low-phase-noise option B4. Cross-check: ARRL QST product review of the QRP Labs QCX (Aug 2019), Si5351A-driven transmitter output better than -135 dBc/Hz at 10 and 50 kHz (https://qrp-labs.com/qcx/phasenoise). The Si5351 datasheet (Rev. 1.3) gives only period and cycle-to-cycle jitter (70 ps pk-pk typical), no phase-noise curve.",
 "checks": []
}
```

- [ ] **Step 3: Run the contract test**

Run: `cd hardware && python3 -m pytest -q tests/test_characterized_contract.py`
Expected: 3 passed.

- [ ] **Step 4: Commit**

```bash
git add hardware/characterized/lo_phase_noise.json hardware/tests/test_characterized_contract.py
git commit -m "feat(hw): Si5351 LO phase-noise curve (RFzero measurement, cited)"
```
### Task 3: Reciprocal mixing, IMD3 and overload in the hardware model

**Files:**
- Create: `sim/tests/charfixture.py` (the synthetic characterisation writer, moved out of `test_hardware.py` so other tests can use it)
- Modify: `sim/neosim/hardware.py`, `sim/tests/test_hardware.py`

**Interfaces:**
- Consumes: `lo_phase_noise.json` (Task 2); mixer.json `iip3_dbm`, `adc_full_scale_dbm`; preamp.json `oip3_dbm`.
- Produces: `Hardware.lo_phase_noise`; `Hardware.front_parts(f_rf, f_lo, preamp, attenuator_db) -> (attenuator × filter complex, preamp amplitude)`; `Hardware.phase_noise_dbc_hz(offset_hz, f_lo)`; `HardwareStats.imd3: tuple[(id_i, id_j, f_hz, dbm)]`, `.overload: tuple[str]`, `.reciprocal_dbm_per_hz: float`, `.imbalance: Imbalance`; `REQUIRED` gains `iip3_dbm`, `adc_full_scale_dbm` and the `lo_phase_noise` fields; out-of-window exclusion reason ends "it contributes only reciprocal mixing and IMD3"; `charfixture.write_char(..., iip3_dbm=30.0, oip3_dbm=24.0, pn=None)`.

- [ ] **Step 1: Write the failing tests** (moves `write_char` into `charfixture.py`, adds the phase-noise file to it, and adds the new tests)

Create `sim/tests/charfixture.py`:

```python
"""Synthetic characterisation files for twin tests."""
import json
import math

import numpy as np


def _doc(block, **payload):
    return {"schema_version": 1, "block": block, "status": "pass", "generated_by": "test", **payload}


def write_char(d, *, il_db=1.0, pre_gain_db=14.0, pre_nf_db=4.0, mixer_nf_db=7.0, conv_gain_db=36.0,
               harmonics=None, imbalance=(0.0, 0.0), adc_dr_db=108.0, dc=0.0, bypass=1.0,
               tp_gain_db=None, tp_harmonics=None, iip3_dbm=30.0, oip3_dbm=24.0, pn=None):
    """`tp_gain_db` / `tp_harmonics`: (value at the 1 MHz test point, value at 60 MHz); default
    both equal to the top-level (nominal) values."""
    """Flat synthetic characterisation: two filters (1–10 MHz, 10–60 MHz) with S21 = −il_db in
    band and −80 dB outside, flat preamp and attenuator, flat mixer response."""
    d.mkdir(parents=True, exist_ok=True)
    filters = [{"index": 1, "lo_hz": 1e6, "hi_hz": 10e6}, {"index": 2, "lo_hz": 10e6, "hi_hz": 60e6}]
    (d / "bpf_bank.json").write_text(json.dumps(_doc("bpf_bank", filters=filters)))
    for flt in filters:
        lines = ["# Hz S RI R 50"]
        for f in np.geomspace(0.05e6, 400e6, 400):
            inside = flt["lo_hz"] * 0.9 <= f <= flt["hi_hz"] * 1.1
            s21 = 10 ** ((-il_db if inside else -80.0) / 20)
            lines.append(f"{f:.6g} 0 0 {s21:.9g} 0 {s21:.9g} 0 0 0")
        (d / f"bpf_{flt['index']}.s2p").write_text("\n".join(lines) + "\n")
    grid = list(np.geomspace(0.1e6, 300e6, 50))
    (d / "preamp.json").write_text(json.dumps(_doc(
        "preamp", freq_hz=grid, gain_db=[pre_gain_db] * 50, nf_db=[pre_nf_db] * 50,
        bypass_loss_db=0.0, oip3_dbm=oip3_dbm)))
    curve = pn or {"offset_hz": [100.0, 1e3, 1e4, 1e5, 1e6], "dbc_per_hz": [-200.0] * 5}
    (d / "lo_phase_noise.json").write_text(json.dumps(_doc(
        "lo_phase_noise", reference_hz=9e6, source="test", **curve)))
    (d / "attenuator.json").write_text(json.dumps(_doc(
        "attenuator", freq_hz=[0.1e6, 54e6], pad_db=[-20.0, -20.0], bypass_db=[0.0, 0.0])))
    offs = [-60e3, -3e3, 3e3, 60e3]
    harm = {"1": {"db": 0.0, "phase_deg": 0.0, "sideband": "upper"}}
    harm.update(harmonics or {})
    ir = {"p5_gain_err_db": imbalance[0], "p5_phase_err_deg": imbalance[1]}
    (d / "mixer.json").write_text(json.dumps(_doc(
        "mixer", conv_gain_db=conv_gain_db,
        test_points=[{"f_lo_hz": f, "conv_gain_db": g, "harmonics": h} for f, g, h in zip(
            (1e6, 60e6), tp_gain_db or (conv_gain_db, conv_gain_db), tp_harmonics or (harmonics or {}, harmonics or {}))],
        baseband_response={"offset_hz": offs, "re": [1.0] * 4, "im": [0.0] * 4},
        harmonic_responses=harm, image_rejection_db={"direct": ir, "div4": ir},
        nf_db=mixer_nf_db, adc_fs_peak_v=2.825, clip_peak_v=2.0, iip3_dbm=iip3_dbm, adc_full_scale_dbm=-20.0,
        adc_noise={"dynamic_range_db": adc_dr_db, "bandwidth_hz": 40e3},
        dc_offset_v={"typical": dc, "worst": dc},
        bypass_correction={str(i): {"offset_hz": offs, "re": [bypass] * 4, "im": [0.0] * 4,
                                    "mean_db": 20 * math.log10(bypass)} for i in (1, 2)})))
    return d
```

Apply from the repo root:

```bash
git apply <<'PATCH'
diff --git a/sim/tests/test_hardware.py b/sim/tests/test_hardware.py
index 5dccd60..57faa25 100644
--- a/sim/tests/test_hardware.py
+++ b/sim/tests/test_hardware.py
@@ -8,53 +8,9 @@ from neosim import hardware, sources
 from neosim.hardware import Hardware, HardwareModelError, Imbalance
 from neosim.scenario import FS_HZ, WINDOW_HZ, parse_scenario
+from tests.charfixture import write_char
 
 K_T0 = hardware.K_B * hardware.T0
 
 
-def _doc(block, **payload):
-    return {"schema_version": 1, "block": block, "status": "pass", "generated_by": "test", **payload}
-
-
-def write_char(d, *, il_db=1.0, pre_gain_db=14.0, pre_nf_db=4.0, mixer_nf_db=7.0, conv_gain_db=36.0,
-               harmonics=None, imbalance=(0.0, 0.0), adc_dr_db=108.0, dc=0.0, bypass=1.0,
-               tp_gain_db=None, tp_harmonics=None):
-    """`tp_gain_db` / `tp_harmonics`: (value at the 1 MHz test point, value at 60 MHz); default
-    both equal to the top-level (nominal) values."""
-    """Flat synthetic characterisation: two filters (1–10 MHz, 10–60 MHz) with S21 = −il_db in
-    band and −80 dB outside, flat preamp and attenuator, flat mixer response."""
-    d.mkdir(parents=True, exist_ok=True)
-    filters = [{"index": 1, "lo_hz": 1e6, "hi_hz": 10e6}, {"index": 2, "lo_hz": 10e6, "hi_hz": 60e6}]
-    (d / "bpf_bank.json").write_text(json.dumps(_doc("bpf_bank", filters=filters)))
-    for flt in filters:
-        lines = ["# Hz S RI R 50"]
-        for f in np.geomspace(0.05e6, 400e6, 400):
-            inside = flt["lo_hz"] * 0.9 <= f <= flt["hi_hz"] * 1.1
-            s21 = 10 ** ((-il_db if inside else -80.0) / 20)
-            lines.append(f"{f:.6g} 0 0 {s21:.9g} 0 {s21:.9g} 0 0 0")
-        (d / f"bpf_{flt['index']}.s2p").write_text("\n".join(lines) + "\n")
-    grid = list(np.geomspace(0.1e6, 300e6, 50))
-    (d / "preamp.json").write_text(json.dumps(_doc(
-        "preamp", freq_hz=grid, gain_db=[pre_gain_db] * 50, nf_db=[pre_nf_db] * 50,
-        bypass_loss_db=0.0, oip3_dbm=24.0)))
-    (d / "attenuator.json").write_text(json.dumps(_doc(
-        "attenuator", freq_hz=[0.1e6, 54e6], pad_db=[-20.0, -20.0], bypass_db=[0.0, 0.0])))
-    offs = [-60e3, -3e3, 3e3, 60e3]
-    harm = {"1": {"db": 0.0, "phase_deg": 0.0, "sideband": "upper"}}
-    harm.update(harmonics or {})
-    ir = {"p5_gain_err_db": imbalance[0], "p5_phase_err_deg": imbalance[1]}
-    (d / "mixer.json").write_text(json.dumps(_doc(
-        "mixer", conv_gain_db=conv_gain_db,
-        test_points=[{"f_lo_hz": f, "conv_gain_db": g, "harmonics": h} for f, g, h in zip(
-            (1e6, 60e6), tp_gain_db or (conv_gain_db, conv_gain_db), tp_harmonics or (harmonics or {}, harmonics or {}))],
-        baseband_response={"offset_hz": offs, "re": [1.0] * 4, "im": [0.0] * 4},
-        harmonic_responses=harm, image_rejection_db={"direct": ir, "div4": ir},
-        nf_db=mixer_nf_db, adc_fs_peak_v=2.825, clip_peak_v=2.0,
-        adc_noise={"dynamic_range_db": adc_dr_db, "bandwidth_hz": 40e3},
-        dc_offset_v={"typical": dc, "worst": dc},
-        bypass_correction={str(i): {"offset_hz": offs, "re": [bypass] * 4, "im": [0.0] * 4,
-                                    "mean_db": 20 * math.log10(bypass)} for i in (1, 2)})))
-    return d
-
-
 def scenario(lo, stations=(), preamp=False, att=0, noise=-174.0, duration=1.0):
     return parse_scenario({
@@ -160,5 +116,5 @@ def test_out_of_window_station_is_excluded_with_a_reason(tmp_path):
     st = {"id": "far", "mode": "carrier", "rf_hz": 15.4e6, "level_dbm": -30.0}
     _, excluded, _ = run(scenario(14e6, [st]), hw)
-    assert excluded[0][0] == "far" and "plan 2.6" in excluded[0][1]
+    assert excluded[0][0] == "far" and "reciprocal mixing and IMD3" in excluded[0][1]
 
 
@@ -263,2 +219,102 @@ def test_budget_total_includes_the_adc_noise_and_matches_the_twin_floor(tmp_path
     measured = np.mean(np.abs(iq) ** 2) / FS_HZ
     assert 10 * math.log10(measured / density) == pytest.approx(0.0, abs=0.1)
+
+
+def density_dbm_hz(iq, f_lo_bb, f_hi_bb, hw, lo, preamp=False):
+    """Average output density between two baseband offsets, referred to the antenna (dBm/Hz)."""
+    spec = np.abs(np.fft.fft(iq)) ** 2 / (len(iq) * FS_HZ)
+    f = np.fft.fftfreq(len(iq), 1 / FS_HZ)
+    band = (f >= f_lo_bb) & (f <= f_hi_bb)
+    v2 = np.mean(spec[band]) * 2.825 ** 2
+    g2 = abs(hw.front_gain(lo + (f_lo_bb + f_hi_bb) / 2, lo, preamp, 0)) ** 2 * hw.conv_gain(lo) ** 2
+    return 10 * math.log10(v2 / g2 / (2 * 50) / 1e-3)
+
+
+def test_phase_noise_curve_scales_with_the_lo(tmp_path):
+    pn = {"offset_hz": [100.0, 1e3, 1e4, 1e5, 1e6], "dbc_per_hz": [-103.0, -128.0, -136.0, -138.0, -144.0]}
+    hw = Hardware.load(write_char(tmp_path, pn=pn))
+    assert hw.phase_noise_dbc_hz(1e4, 9e6) == pytest.approx(-136.0)
+    assert hw.phase_noise_dbc_hz(1e4, 18e6) == pytest.approx(-136.0 + 20 * math.log10(2))
+    assert hw.phase_noise_dbc_hz(10.0, 9e6) == pytest.approx(-103.0)        # clamped below 100 Hz
+    assert hw.phase_noise_dbc_hz(5e6, 9e6) == pytest.approx(-144.0)         # floor beyond 1 MHz
+    assert hw.phase_noise_dbc_hz(math.sqrt(1e3 * 1e4), 9e6) == pytest.approx(-132.0)   # log-offset interpolation
+
+
+def test_reciprocal_mixing_raises_the_floor_by_station_power_times_phase_noise(tmp_path):
+    pn = {"offset_hz": [100.0, 1e6], "dbc_per_hz": [-110.0, -110.0]}     # flat −110 dBc/Hz
+    hw = Hardware.load(write_char(tmp_path, pn=pn, adc_dr_db=200.0))
+    lo = 9e6        # the curve's reference frequency: no 20·log(f/9 MHz) scaling
+    blocker = {"id": "b", "mode": "carrier", "rf_hz": lo + 300e3, "level_dbm": -30.0}
+    iq, excluded, _ = run(scenario(lo, [blocker], noise=-200.0, duration=2.0), hw)
+    # −30 dBm (−1 dB filter: −31 dBm at the mixer) × −110 dBc/Hz = −141 dBm/Hz at the mixer, −140 at the antenna
+    assert density_dbm_hz(iq, 5e3, 20e3, hw, lo) == pytest.approx(-140.0, abs=0.3)
+    assert excluded[0][0] == "b" and "reciprocal mixing" in excluded[0][1]
+
+
+def test_imd3_of_an_out_of_window_pair_lands_in_the_window(tmp_path):
+    hw = Hardware.load(write_char(tmp_path, adc_dr_db=200.0, iip3_dbm=30.0))
+    lo = 14e6
+    st = [{"id": "a", "mode": "carrier", "rf_hz": lo + 100e3, "level_dbm": -20.0},
+          {"id": "c", "mode": "carrier", "rf_hz": lo + 170e3, "level_dbm": -20.0}]
+    iq, _, stats = run(scenario(lo, st, noise=-200.0), hw)
+    # at the mixer: −21 dBm each, product 2·(−21) + (−21) − 2·30 = −123 dBm at lo + 30 kHz
+    expected = 20 * math.log10(math.sqrt(2 * 50 * 1e-3 * 10 ** (-123 / 10)) * 10 ** (36 / 20) / 2.825)
+    assert tone_dbfs(iq, 30e3) == pytest.approx(expected, abs=0.1)
+    assert ("a", "c") in [tuple(x[:2]) for x in stats.imd3]
+
+
+def test_imd3_with_the_preamp_adds_its_own_product(tmp_path):
+    hw = Hardware.load(write_char(tmp_path, adc_dr_db=200.0, iip3_dbm=30.0, oip3_dbm=24.0))
+    lo = 14e6
+    st = [{"id": "a", "mode": "carrier", "rf_hz": lo + 100e3, "level_dbm": -20.0},
+          {"id": "c", "mode": "carrier", "rf_hz": lo + 170e3, "level_dbm": -20.0}]
+    iq, _, _ = run(scenario(lo, st, preamp=True, noise=-200.0), hw)
+    # preamp: IIP3 = 24 − 14 = 10 dBm; inputs −21 dBm → −83 dBm at its input, −69 dBm at the mixer
+    # mixer: inputs −7 dBm → −21 − 60 = −81 dBm; same phase (flat front end): amplitudes add
+    amp = lambda p: math.sqrt(2 * 50 * 1e-3 * 10 ** (p / 10))
+    expected = 20 * math.log10((amp(-69) + amp(-81)) * 10 ** (36 / 20) / 2.825)
+    assert tone_dbfs(iq, 30e3) == pytest.approx(expected, abs=0.1)
+
+
+def test_a_station_above_full_scale_at_the_mixer_is_reported(tmp_path):
+    hw = Hardware.load(write_char(tmp_path))
+    st = {"id": "loud", "mode": "carrier", "rf_hz": 14.5e6, "level_dbm": -10.0}    # −11 dBm at the mixer > −20
+    _, _, stats = run(scenario(14e6, [st]), hw)
+    assert stats.overload == ("loud",)
+
+
+def test_missing_phase_noise_file_is_refused(tmp_path):
+    d = write_char(tmp_path)
+    (d / "lo_phase_noise.json").unlink()
+    with pytest.raises(HardwareModelError, match="lo_phase_noise.json is missing"):
+        Hardware.load(d)
+
+
+def test_imd3_and_reciprocal_mixing_of_a_station_outside_the_passband_are_filtered(tmp_path):
+    """A station the filter rejects by 80 dB contributes its IMD3 and phase-noise products 80 dB
+    down (per its own S21), never at full level."""
+    pn = {"offset_hz": [100.0, 1e6], "dbc_per_hz": [-110.0, -110.0]}
+    hw = Hardware.load(write_char(tmp_path, pn=pn, adc_dr_db=200.0))
+    lo = 14e6
+    st = [{"id": "near", "mode": "carrier", "rf_hz": 11.015e6, "level_dbm": -20.0},     # in filter 2's passband
+          {"id": "far", "mode": "carrier", "rf_hz": 8.0e6, "level_dbm": -20.0}]         # below it: −80 dB
+    _, _, stats = run(scenario(lo, st, noise=-200.0), hw)
+    product = {(i, j): p for i, j, _, p in stats.imd3}
+    # 2·(−21) + (−100) − 2·30 = −202 dBm at the mixer, −201 at the antenna
+    assert product[("near", "far")] == pytest.approx(-201.0, abs=0.2)
+    # reciprocal mixing: only "near" counts (−21 − 110 + 20·log(14/9)), "far" is 79 dB lower
+    assert stats.reciprocal_dbm_per_hz == pytest.approx(-21 - 110 + 20 * math.log10(14 / 9) + 1, abs=0.3)
+
+
+def test_an_in_window_pair_adds_one_product_per_ordered_pair(tmp_path):
+    # IIP3 0 dBm puts the product (−123 dBm) 40 dB above the fixture's receiver noise in a 1 Hz bin
+    hw = Hardware.load(write_char(tmp_path, adc_dr_db=200.0, iip3_dbm=0.0))
+    lo = 14e6
+    st = [{"id": "a", "mode": "carrier", "rf_hz": lo + 10e3, "level_dbm": -40.0},
+          {"id": "b", "mode": "carrier", "rf_hz": lo + 20e3, "level_dbm": -40.0}]
+    iq, _, stats = run(scenario(lo, st, noise=-200.0), hw)
+    assert sorted((i, j) for i, j, _, _ in stats.imd3) == [("a", "b"), ("b", "a")]
+    # 2·20 − 10 = 30 kHz: 3·(−41) − 2·0 = −123 dBm at the mixer
+    expected = 20 * math.log10(math.sqrt(2 * 50 * 1e-3 * 10 ** (-123 / 10)) * 10 ** (36 / 20) / 2.825)
+    assert tone_dbfs(iq, 30e3) == pytest.approx(expected, abs=0.1)
+    assert tone_dbfs(iq, 10e3) == pytest.approx(20 * math.log10(math.sqrt(2 * 50 * 1e-3 * 10 ** (-40 / 10)) * 10 ** (35 / 20) / 2.825), abs=0.05)
PATCH
```

- [ ] **Step 2: Run them to verify they fail**

Run: `cd sim && python3 -m pytest -q tests/test_hardware.py`
Expected: 6 failed (phase-noise curve, reciprocal mixing, two IMD3 tests, overload, missing phase-noise file); the out-of-window reason test also fails on its new text.

- [ ] **Step 3: Implement**

Apply from the repo root:

```bash
git apply <<'PATCH'
diff --git a/sim/neosim/hardware.py b/sim/neosim/hardware.py
index e700f9c..b0ab1d4 100644
--- a/sim/neosim/hardware.py
+++ b/sim/neosim/hardware.py
@@ -30,5 +30,7 @@ class HardwareModelError(RuntimeError):
 
 # fields the twin reads beyond the plan 2.4 contract (spec §6, plan 2.5)
-REQUIRED = {"mixer": ("nf_db", "adc_fs_peak_v", "clip_peak_v", "adc_noise", "bypass_correction"),
+REQUIRED = {"mixer": ("nf_db", "adc_fs_peak_v", "clip_peak_v", "adc_noise", "bypass_correction", "iip3_dbm",
+                       "adc_full_scale_dbm"),
+            "lo_phase_noise": ("reference_hz", "offset_hz", "dbc_per_hz", "source"),
             "preamp": ("freq_hz", "gain_db", "nf_db", "bypass_loss_db")}
 
@@ -101,4 +103,6 @@ class Hardware:
     mixer: dict
 
+    lo_phase_noise: dict
+
     @classmethod
     def load(cls, char_dir: Path = DEFAULT_DIR) -> "Hardware":
@@ -107,5 +111,5 @@ class Hardware:
         s21 = {f["index"]: _read_s21(d / f"bpf_{f['index']}.s2p") for f in bank["filters"]}
         return cls(bank, s21, _read(d / "preamp.json", "preamp"), _read(d / "attenuator.json", "attenuator"),
-                   _read(d / "mixer.json", "mixer"))
+                   _read(d / "mixer.json", "mixer"), _read(d / "lo_phase_noise.json", "lo_phase_noise"))
 
     # --- front end -------------------------------------------------------------------------
@@ -128,4 +132,10 @@ class Hardware:
         """Complex transducer gain from the antenna to the mixer input at RF frequencies f_rf,
         with the filter selected for f_lo."""
+        g_in, g_pre = self.front_parts(f_rf, f_lo, preamp, attenuator_db)
+        return g_in * g_pre
+
+    def front_parts(self, f_rf, f_lo: float, preamp: bool, attenuator_db: int):
+        """(attenuator × filter, complex; preamp or bypass amplitude gain): the front end split at
+        the preamp input, where its IMD is referred."""
         f_rf = np.asarray(f_rf, dtype=float)
         a = self.attenuator
@@ -134,6 +144,15 @@ class Hardware:
         flt = _interp_complex(f_rf, fs, s21)
         p = self.preamp
-        pre_db = np.interp(f_rf, p["freq_hz"], p["gain_db"]) if preamp else -p["bypass_loss_db"]
-        return 10 ** ((att_db + pre_db) / 20) * flt
+        pre_db = np.interp(f_rf, p["freq_hz"], p["gain_db"]) if preamp else -p["bypass_loss_db"] + 0 * f_rf
+        return 10 ** (att_db / 20) * flt, 10 ** (pre_db / 20)
+
+    def phase_noise_dbc_hz(self, offset_hz, f_lo: float):
+        """LO single-sideband phase noise L(offset) at f_lo: the measured curve (lo_phase_noise.json,
+        at reference_hz) interpolated over log offset, held flat outside its range, scaled by
+        20·log10(f_lo / reference_hz)."""
+        c = self.lo_phase_noise
+        o = np.log10(np.maximum(np.abs(np.asarray(offset_hz, dtype=float)), 1e-3))
+        db = np.interp(o, np.log10(c["offset_hz"]), c["dbc_per_hz"])
+        return db + 20 * math.log10(f_lo / c["reference_hz"])
 
     # --- mixer -----------------------------------------------------------------------------
@@ -284,4 +303,46 @@ class HardwareStats:
     clipped_samples: int
     peak_dbfs: float
+    imd3: tuple = ()                 # (id_i, id_j, product Hz, antenna-referred dBm) for 2·f_i − f_j in the window
+    overload: tuple[str, ...] = ()   # stations above the full-scale level at the mixer input
+    reciprocal_dbm_per_hz: float = float("-inf")   # mean reciprocal-mixing density in the window, antenna-referred
+    imbalance: Imbalance | None = None
+
+
+def _amp(dbm) -> float:
+    return math.sqrt(2 * R0 * 1e-3 * 10 ** (dbm / 10))
+
+
+def _imd3(scenario, hw: "Hardware", usable, lo: float, n: int, fs: int, window_hz: float, envelope):
+    """Third-order products 2·f_i − f_j of every ordered station pair that land in the window,
+    from the mixer IIP3 (at its input) and, with the preamp on, the preamp IIP3 (OIP3 − gain, at
+    its input). Each product is c·u_i²·conj(u_j) at f_im; the two stages' products add with the
+    phases of the front-end gains (in phase for a flat front end: the worst case). Returned
+    antenna-referred, ready to add to the in-window envelope."""
+    rx = scenario.receiver
+    a_mix = _amp(hw.mixer["iip3_dbm"])
+    t = np.arange(n) / fs
+    x = np.zeros(n, dtype=complex)
+    found = []
+    for si in usable:
+        for sj in usable:
+            if si is sj:
+                continue
+            f_im = 2 * si.rf_hz - sj.rf_hz
+            if abs(f_im - lo) + (2 * si.bandwidth_hz + sj.bandwidth_hz) / 2 > window_hz:
+                continue
+            ai, aj = _amp(si.level_dbm), _amp(sj.level_dbm)
+            gi_in, gi_pre = hw.front_parts(si.rf_hz, lo, rx.preamp, rx.attenuator_db)
+            gj_in, gj_pre = hw.front_parts(sj.rf_hz, lo, rx.preamp, rx.attenuator_db)
+            fi, fj = gi_in * gi_pre, gj_in * gj_pre
+            c = fi ** 2 * np.conj(fj) * ai ** 2 * aj / a_mix ** 2
+            if rx.preamp:
+                g_im = float(np.interp(f_im, hw.preamp["freq_hz"], hw.preamp["gain_db"]))
+                a_pre = _amp(hw.preamp["oip3_dbm"] - g_im)
+                c = c + gi_in ** 2 * np.conj(gj_in) * ai ** 2 * aj / a_pre ** 2 * 10 ** (g_im / 20)
+            c_ant = complex(c / hw.front_gain(f_im, lo, rx.preamp, rx.attenuator_db))
+            u = envelope(si) ** 2 * np.conj(envelope(sj))
+            x += c_ant * u * np.exp(2j * np.pi * (f_im - lo) * t)
+            found.append((si.id, sj.id, f_im, 10 * math.log10(abs(c_ant) ** 2 / (2 * R0) / 1e-3)))
+    return x, tuple(found)
 
 
@@ -305,4 +366,27 @@ def receive(scenario, rng: np.random.Generator, hw: Hardware, render, fs: int, w
     excluded: list[tuple[str, str]] = []
     folded = []
+    # stations the front-end data covers (a harmonic station beyond the filter data is excluded below)
+    f_lo_s2p, f_hi_s2p = hw.s2p_range(band)
+    usable = [st for st in scenario.stations if f_lo_s2p <= st.rf_hz <= f_hi_s2p]
+    envelopes: dict[str, np.ndarray] = {}
+
+    def envelope(st):   # unit-power envelope about the station's own frequency
+        if st.id not in envelopes:
+            envelopes[st.id] = render(st, st.rf_hz, n)
+        return envelopes[st.id]
+
+    # reciprocal mixing: each station's power at the mixer × L(offset), offset = f_s − LO − f_bb
+    rm = np.zeros(n)
+    overload = []
+    for st in usable:
+        p_mix = 1e-3 * 10 ** (st.level_dbm / 10) * float(abs(hw.front_gain(st.rf_hz, lo, rx.preamp, rx.attenuator_db)) ** 2)
+        rm += p_mix * 10 ** (hw.phase_noise_dbc_hz(st.rf_hz - lo - f_bins, lo) / 10)
+        if 10 * math.log10(p_mix / 1e-3) > hw.mixer["adc_full_scale_dbm"]:
+            overload.append(st.id)
+    rm_ant = rm / np.abs(hw.front_gain(lo + f_bins, lo, rx.preamp, rx.attenuator_db)) ** 2
+    w = rng.normal(size=n) + 1j * rng.normal(size=n)
+    x += np.fft.ifft(np.fft.fft(w) * np.sqrt(2 * R0 * rm_ant * fs / 2))
+    x_imd, imd3 = _imd3(scenario, hw, usable, lo, n, fs, window_hz, envelope)
+    x += x_imd
     spec_h: dict[tuple[int, str], np.ndarray] = {}
     harmonics = hw.harmonics(lo)
@@ -314,5 +398,4 @@ def receive(scenario, rng: np.random.Generator, hw: Hardware, render, fs: int, w
         for k, (_, side) in harmonics.items():
             if abs(st.rf_hz - k * lo) + st.bandwidth_hz / 2 <= window_hz:
-                f_lo_s2p, f_hi_s2p = hw.s2p_range(band)
                 if not f_lo_s2p <= st.rf_hz <= f_hi_s2p:
                     excluded.append((st.id, f"{st.rf_hz / 1e6:.6f} MHz near {k}·LO is beyond the filter "
@@ -325,6 +408,6 @@ def receive(scenario, rng: np.random.Generator, hw: Hardware, render, fs: int, w
         else:
             excluded.append((st.id, f"{st.rf_hz / 1e6:.6f} MHz is outside the receiver window "
-                                    f"{lo / 1e6:.6f} MHz ±{window_hz / 1e3:.1f} kHz and its LO harmonics "
-                                    "(out-of-window effects: plan 2.6)"))
+                                    f"{lo / 1e6:.6f} MHz ±{window_hz / 1e3:.1f} kHz and its LO harmonics: "
+                                    "it contributes only reciprocal mixing and IMD3"))
     y = np.fft.fft(x) * hw.front_gain(lo + f_bins, lo, rx.preamp, rx.attenuator_db)
     if not rx.preamp:
@@ -352,5 +435,7 @@ def receive(scenario, rng: np.random.Generator, hw: Hardware, render, fs: int, w
     dq = np.clip(np.round((q + rng.normal(scale=sigma, size=n)) / fsv / lsb) * lsb, -1.0, 1.0 - lsb)
     peak = float(max(np.max(np.abs(di)), np.max(np.abs(dq)), 1e-30))
+    band_rm = np.abs(f_bins) <= window_hz
+    rm_db = 10 * math.log10(max(float(np.mean(rm_ant[band_rm])), 1e-300) / 1e-3)
     stats = HardwareStats(band, budget, 20 * math.log10(g), image_rejection_db(imb), tuple(folded), clipped,
-                          20 * math.log10(peak))
+                          20 * math.log10(peak), imd3, tuple(overload), rm_db, imb)
     return di - 1j * dq, excluded, stats
PATCH
```

- [ ] **Step 4: Run the tests**

Run: `cd sim && python3 -m pytest -q tests/test_hardware.py`
Expected: 27 passed. (Reciprocal mixing is checked at a 9 MHz LO: at 14 MHz the curve is 20·log(14/9) = 3.84 dB higher.)

- [ ] **Step 5: Commit**

```bash
git add sim/neosim/hardware.py sim/tests/test_hardware.py sim/tests/charfixture.py
git commit -m "feat(sim): reciprocal mixing, IMD3 and overload in the hardware model"
```
### Task 4: The twin passes the known imbalance to the DSP

**Files:**
- Modify: `sim/neosim/dsp_runner.py`, `sim/neosim/twin.py`, `sim/tests/test_dsp_runner.py`

**Interfaces:**
- Consumes: `HardwareStats.imbalance` (Task 3); CLI options of Task 1.
- Produces: `run_dsp(meta, wav, mode, tune_offset_hz, iq: tuple[float, float] | None = None)`.

- [ ] **Step 1: Write the failing test**

Apply from the repo root:

```bash
git apply <<'PATCH'
diff --git a/sim/tests/test_dsp_runner.py b/sim/tests/test_dsp_runner.py
index 5685117..348beab 100644
--- a/sim/tests/test_dsp_runner.py
+++ b/sim/tests/test_dsp_runner.py
@@ -26,2 +26,15 @@ def test_run_dsp_passes_arguments(monkeypatch, tmp_path):
     assert args[args.index("--mode") + 1] == "cw"
     assert args[args.index("--tune-offset") + 1] == "-15000.000"
+
+
+def test_run_dsp_passes_the_iq_correction_only_when_given(monkeypatch, tmp_path):
+    log = tmp_path / "args.txt"
+    fake = tmp_path / "neoradio-dsp"
+    fake.write_text(f'#!/bin/sh\necho "$@" > {log}\n')
+    fake.chmod(fake.stat().st_mode | stat.S_IXUSR)
+    monkeypatch.setenv("NEOSIM_DSP_CLI", str(fake))
+    run_dsp(tmp_path / "x.sigmf-meta", tmp_path / "x.wav", "usb", 0.0)
+    assert "--iq-gain-db" not in log.read_text()
+    run_dsp(tmp_path / "x.sigmf-meta", tmp_path / "x.wav", "usb", 0.0, iq=(0.1608, -0.4273))
+    args = log.read_text().split()
+    assert args[args.index("--iq-gain-db") + 1] == "0.1608" and args[args.index("--iq-phase-deg") + 1] == "-0.4273"
PATCH
```

Run: `cd sim && python3 -m pytest -q tests/test_dsp_runner.py`
Expected: 1 failed (`run_dsp() got an unexpected keyword argument 'iq'`).

- [ ] **Step 2: Implement**

Apply from the repo root:

```bash
git apply <<'PATCH'
diff --git a/sim/neosim/dsp_runner.py b/sim/neosim/dsp_runner.py
index 06605c7..a7fd78e 100644
--- a/sim/neosim/dsp_runner.py
+++ b/sim/neosim/dsp_runner.py
@@ -8,7 +8,11 @@ from neosim.tools import tool_path
 
 
-def run_dsp(meta: Path, wav: Path, mode: str, tune_offset_hz: float) -> None:
+def run_dsp(meta: Path, wav: Path, mode: str, tune_offset_hz: float,
+            iq: tuple[float, float] | None = None) -> None:
+    """`iq` = (gain dB, phase °): the known I/Q imbalance for neoradio-dsp to correct."""
     cmd = [str(tool_path("neoradio-dsp")), "--input", str(meta), "--output", str(wav),
            "--mode", mode, "--tune-offset", f"{tune_offset_hz:.3f}"]
+    if iq is not None:
+        cmd += ["--iq-gain-db", f"{iq[0]:.4f}", "--iq-phase-deg", f"{iq[1]:.4f}"]
     result = subprocess.run(cmd, capture_output=True, text=True)
     if result.returncode != 0:
PATCH
```

Apply from the repo root:

```bash
git apply <<'PATCH'
diff --git a/sim/neosim/twin.py b/sim/neosim/twin.py
index d48a778..e530c0b 100644
--- a/sim/neosim/twin.py
+++ b/sim/neosim/twin.py
@@ -73,5 +73,7 @@ def run_scenario(scenario: Scenario, workdir: Path, hw: hardware.Hardware | None
     for rx in scenario.receive:
         wav = workdir / f"{rx.id}.wav"
-        run_dsp(meta, wav, rx.mode, rx.dial_hz - scenario.lo_hz)
+        # the hardware path passes the known imbalance (automatic estimation is a firmware task)
+        iq = None if hw_stats is None else (hw_stats.imbalance.gain_db, hw_stats.imbalance.phase_deg)
+        run_dsp(meta, wav, rx.mode, rx.dial_hz - scenario.lo_hz, iq)
         receives.append(_evaluate(rx, wav))
 
PATCH
```

- [ ] **Step 3: Run the tests**

Run: `cd sim && python3 -m pytest -q tests/test_dsp_runner.py`
Expected: 3 passed.

- [ ] **Step 4: Commit**

```bash
git add sim/neosim/dsp_runner.py sim/neosim/twin.py sim/tests/test_dsp_runner.py
git commit -m "feat(sim): the twin passes the applied I/Q imbalance to neoradio-dsp"
```
### Task 5: MDS, the blocker check and the hardware FT8 sweep

**Files:**
- Create: `sim/neosim/measure.py`, `sim/tests/test_measure.py`, `sim/scenarios/20m_blocker_hardware.yaml`
- Modify: `sim/neosim/sweep.py`, `sim/neosim/__main__.py`, `sim/neosim/report.py`, `Makefile`

**Interfaces:**
- Consumes: `hardware.receive`, `HardwareStats` (Task 3), `run_scenario` (Task 4).
- Produces: `measure.measure_mds(hw, f_lo, preamp=True, seed=1, duration_s=4.0) -> MdsResult(f_lo_hz, preamp, measured_dbm, predicted_dbm, nf_total_db, passed)`; `measure.blocker_check(path, workdir, hw=None) -> BlockerResult(with_blocker, without_blocker, blocker_ids, degradation_db, passed)`; `HardwareChecks(mds, blocker, passed)`; constants `MDS_BW_HZ`, `MDS_TARGET_DBM = −130`, `MDS_AGREEMENT_DB = 1`, `MDS_LO_HZ = (14.1e6, 50.1e6)`, `BLOCKER_SCENARIO`; `sweep.sweep_scenario(snr_db, seed, hardware=False)`, `sweep.BAND_NOISE_DBM_HZ = −155`, `SweepPoint.hardware_rate`, `sweep_ft8(..., hardware=False)`, `implementation_loss_db(points, curve="twin"|"hardware")`; CLI `--sweep-hardware`, `--hardware-checks`; `write_report(results, points, path, checks=None)`.

- [ ] **Step 1: Write the failing tests**

Create `sim/tests/test_measure.py`:

```python
import math

import pytest

from neosim import measure, sweep
from neosim.hardware import Hardware
from neosim.scenario import SNR_BANDWIDTH_HZ
from tests.charfixture import write_char


def test_mds_measured_in_the_twin_matches_the_noise_budget(tmp_path):
    hw = Hardware.load(write_char(tmp_path, adc_dr_db=108.0))
    r = measure.measure_mds(hw, 14.1e6, preamp=True)
    assert r.measured_dbm == pytest.approx(r.predicted_dbm, abs=0.5)
    # flat synthetic chain: thermal −174 dBm/Hz × F_total in 500 Hz
    k_t0 = 1.380649e-23 * 290
    expected = 10 * math.log10((10 ** (-17.4) * 1e-3 + k_t0 * (10 ** (r.nf_total_db / 10) - 1)) * 500 / 1e-3)
    assert r.predicted_dbm == pytest.approx(expected, abs=1e-9)


def test_mds_without_the_preamp_is_worse(tmp_path):
    hw = Hardware.load(write_char(tmp_path))
    assert measure.measure_mds(hw, 14.1e6, preamp=False).predicted_dbm > measure.measure_mds(hw, 14.1e6, preamp=True).predicted_dbm


def test_sweep_scenario_for_the_hardware_uses_absolute_levels():
    sc = sweep.sweep_scenario(-15.0, seed=3, hardware=True)
    assert sc.receiver.model == "hardware" and not sc.receiver.preamp
    assert sc.noise_dbm_per_hz == sweep.BAND_NOISE_DBM_HZ
    assert sc.stations[0].level_dbm == pytest.approx(-15.0 + sweep.BAND_NOISE_DBM_HZ + 10 * math.log10(SNR_BANDWIDTH_HZ))
    assert sweep.sweep_scenario(-15.0, seed=3, hardware=False).receiver.model == "ideal"


def test_implementation_loss_of_the_hardware_curve():
    points = [sweep.SweepPoint(-10, 1.0, 1.0, hardware_rate=1.0), sweep.SweepPoint(-14, 1.0, 1.0, hardware_rate=0.0),
              sweep.SweepPoint(-18, 0.0, 0.0, hardware_rate=0.0)]
    assert sweep.implementation_loss_db(points, "hardware") == pytest.approx(4.0)
    assert sweep.implementation_loss_db(points) == pytest.approx(0.0)      # ideal twin = reference here


def test_mds_ignores_the_dc_offset_and_the_tone_image(tmp_path):
    """The real mixer has millivolts of DC offset at 0 Hz and a −40 dB image of the tone at
    −10 kHz; both lie within 20 kHz of the tone and must not count as noise."""
    hw = Hardware.load(write_char(tmp_path, dc=2.7e-3, imbalance=(0.16, 0.43)))
    r = measure.measure_mds(hw, 14.1e6, preamp=True)
    assert r.measured_dbm == pytest.approx(r.predicted_dbm, abs=0.5)
```

Run: `cd sim && python3 -m pytest -q tests/test_measure.py`
Expected: `ImportError: cannot import name 'measure' from 'neosim'`

- [ ] **Step 2: Implement the measurements and the sweep curve**

Create `sim/neosim/measure.py`:

```python
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
    def passed(self) -> bool:
        return self.with_blocker.passed


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
```

Apply from the repo root:

```bash
git apply <<'PATCH'
diff --git a/sim/neosim/sweep.py b/sim/neosim/sweep.py
index 9ada704..3f9a5a1 100644
--- a/sim/neosim/sweep.py
+++ b/sim/neosim/sweep.py
@@ -2,4 +2,5 @@
 from __future__ import annotations
 
+import math
 import shutil
 from dataclasses import dataclass
@@ -10,5 +11,5 @@ from scipy.io import wavfile
 
 from neosim import decoders, sources
-from neosim.scenario import parse_scenario
+from neosim.scenario import SNR_BANDWIDTH_HZ, Scenario, parse_scenario
 from neosim.twin import run_scenario
 
@@ -17,4 +18,5 @@ AUDIO_HZ = 1500
 LO_HZ = 14_060_000
 DIAL_HZ = 14_074_000
+BAND_NOISE_DBM_HZ = -155.0      # hardware curve: quiet 20 m band noise at the antenna (preamp off on 20 m)
 
 
@@ -25,4 +27,5 @@ class SweepPoint:
     twin_rate: float
     trials: int = 1
+    hardware_rate: float | None = None
 
 
@@ -49,19 +52,29 @@ def _reference_trial(audio: np.ndarray, snr_db: float, rng: np.random.Generator,
 
 
-def _twin_trial(snr_db: float, seed: int, workdir: Path) -> bool:
-    scenario = parse_scenario({
-        "name": f"sweep {snr_db:+.1f} dB", "lo_hz": LO_HZ, "seed": seed,
-        "stations": [{"id": "s", "mode": "ft8", "dial_hz": DIAL_HZ, "audio_hz": AUDIO_HZ,
-                      "snr_db": snr_db, "msg": MESSAGE}],
-        "receive": [{"id": "rx", "mode": "usb", "dial_hz": DIAL_HZ, "decoder": "ft8",
-                     "expect": {"contains": [MESSAGE]}}],
-    })
+def sweep_scenario(snr_db: float, seed: int, hardware: bool = False) -> Scenario:
+    """One sweep trial: the ideal receiver in SNR mode, or the hardware model with the same SNR
+    given as absolute levels over BAND_NOISE_DBM_HZ."""
+    station = {"id": "s", "mode": "ft8", "dial_hz": DIAL_HZ, "audio_hz": AUDIO_HZ, "msg": MESSAGE}
+    data = {"name": f"sweep {snr_db:+.1f} dB", "lo_hz": LO_HZ, "seed": seed, "stations": [station],
+            "receive": [{"id": "rx", "mode": "usb", "dial_hz": DIAL_HZ, "decoder": "ft8",
+                         "expect": {"contains": [MESSAGE]}}]}
+    if hardware:
+        data["receiver"] = {"model": "hardware", "preamp": "off"}
+        data["noise"] = {"type": "awgn", "dbm_per_hz": BAND_NOISE_DBM_HZ}
+        station["level_dbm"] = snr_db + BAND_NOISE_DBM_HZ + 10 * math.log10(SNR_BANDWIDTH_HZ)
+    else:
+        station["snr_db"] = snr_db
+    return parse_scenario(data)
+
+
+def _twin_trial(snr_db: float, seed: int, workdir: Path, hardware: bool = False) -> bool:
     try:
-        return run_scenario(scenario, workdir).passed
+        return run_scenario(sweep_scenario(snr_db, seed, hardware), workdir).passed
     finally:
         shutil.rmtree(workdir, ignore_errors=True)  # an 11.5 MB capture per trial adds up fast
 
 
-def sweep_ft8(snrs: list[float], trials: int, workdir: Path, seed: int = 1) -> list[SweepPoint]:
+def sweep_ft8(snrs: list[float], trials: int, workdir: Path, seed: int = 1, hardware: bool = False) -> list[SweepPoint]:
+    """`hardware` adds a third curve: the same trials through the hardware receiver model."""
     workdir = Path(workdir)
     workdir.mkdir(parents=True, exist_ok=True)
@@ -70,9 +83,12 @@ def sweep_ft8(snrs: list[float], trials: int, workdir: Path, seed: int = 1) -> l
     points = []
     for snr in snrs:
-        ref_ok = twin_ok = 0
+        ref_ok = twin_ok = hw_ok = 0
         for trial in range(trials):
             ref_ok += _reference_trial(audio, snr, rng, workdir / "reference.wav")
             twin_ok += _twin_trial(snr, seed * 1000 + trial, workdir / f"twin_{snr:+.1f}_{trial}")
-        points.append(SweepPoint(float(snr), ref_ok / trials, twin_ok / trials, trials))
+            if hardware:
+                hw_ok += _twin_trial(snr, seed * 1000 + trial, workdir / f"hw_{snr:+.1f}_{trial}", hardware=True)
+        points.append(SweepPoint(float(snr), ref_ok / trials, twin_ok / trials, trials,
+                                 hw_ok / trials if hardware else None))
     return points
 
@@ -86,8 +102,11 @@ def threshold_snr(snrs: list[float], rates: list[float], level: float = 0.5) ->
 
 
-def implementation_loss_db(points: list[SweepPoint]) -> float | None:
+def implementation_loss_db(points: list[SweepPoint], curve: str = "twin") -> float | None:
+    """Loss of the ideal twin ("twin") or the hardware model ("hardware") against the audio-only reference."""
     snrs = [p.snr_db for p in points]
     ref = threshold_snr(snrs, [p.reference_rate for p in points])
-    twin = threshold_snr(snrs, [p.twin_rate for p in points])
+    if curve == "hardware" and any(p.hardware_rate is None for p in points):
+        return None
+    twin = threshold_snr(snrs, [p.twin_rate if curve == "twin" else p.hardware_rate for p in points])
     if ref is None or twin is None:
         return None
PATCH
```

- [ ] **Step 3: Run the tests**

Run: `cd sim && python3 -m pytest -q tests/test_measure.py tests/test_sweep.py`
Expected: all pass. `test_mds_ignores_the_dc_offset_and_the_tone_image` pins the guard bands: without them the prototype measured −105.9 dBm against a −141.7 dBm budget.

- [ ] **Step 4: The blocker scenario, CLI flags, report and `make sim`**

Create `sim/scenarios/20m_blocker_hardware.yaml`:

```yaml
name: 20 m FT8 at -15 dB with a +60 dB carrier at 15.4 MHz (blocker, hardware receiver)
lo_hz: 14_060_000
duration_s: 15.0
seed: 4
receiver: {model: hardware, preamp: off, attenuator: 0}
noise: {type: awgn, dbm_per_hz: -155}       # -121 dBm in 2500 Hz
stations:
  - {id: ft8_weak, mode: ft8, dial_hz: 14_074_000, audio_hz: 1500, level_dbm: -136, msg: "CQ EP2ABC LL35"}   # -15 dB SNR
  - {id: bcast, mode: carrier, rf_hz: 15_400_000, level_dbm: -61}   # +60 dB over the noise in 2500 Hz, inside filter 7 (12-16 MHz)
receive:
  - {id: rx_ft8, mode: usb, dial_hz: 14_074_000, decoder: ft8, expect: {contains: ["CQ EP2ABC LL35"]}}
```

Apply from the repo root:

```bash
git apply <<'PATCH'
diff --git a/sim/neosim/__main__.py b/sim/neosim/__main__.py
index 9ba35bd..1a870d3 100644
--- a/sim/neosim/__main__.py
+++ b/sim/neosim/__main__.py
@@ -8,4 +8,6 @@ from pathlib import Path
 import numpy as np
 
+from neosim.hardware import Hardware
+from neosim.measure import BLOCKER_SCENARIO, MDS_LO_HZ, HardwareChecks, blocker_check, measure_mds
 from neosim.report import write_report
 from neosim.scenario import ScenarioError, load_scenario
@@ -24,4 +26,6 @@ def main(argv: list[str]) -> int:
     run.add_argument("--sweep-ft8", nargs=3, type=float, metavar=("START", "STOP", "STEP"))
     run.add_argument("--trials", type=int, default=4)
+    run.add_argument("--sweep-hardware", action="store_true", help="add the hardware receiver to the FT8 sweep")
+    run.add_argument("--hardware-checks", action="store_true", help="MDS at 14.1 and 50.1 MHz and the blocker scenario")
     args = parser.parse_args(argv)
 
@@ -42,12 +46,22 @@ def main(argv: list[str]) -> int:
             start, stop, step = args.sweep_ft8
             snrs = list(np.arange(start, stop - 1e-9, -abs(step)))
-            points = sweep_ft8(snrs, args.trials, args.workdir / "sweep")
+            points = sweep_ft8(snrs, args.trials, args.workdir / "sweep", hardware=args.sweep_hardware)
+        checks = None
+        if args.hardware_checks:
+            hw = Hardware.load()
+            checks = HardwareChecks([measure_mds(hw, f, preamp=True) for f in MDS_LO_HZ],
+                                    blocker_check(BLOCKER_SCENARIO, args.workdir / "blocker", hw))
+            for m in checks.mds:
+                print(f"{'PASS' if m.passed else 'FAIL'}  MDS {m.f_lo_hz / 1e6:.1f} MHz: {m.measured_dbm:.1f} dBm "
+                      f"(budget {m.predicted_dbm:.1f})")
+            print(f"{'PASS' if checks.blocker.passed else 'FAIL'}  blocker: floor {checks.blocker.degradation_db:+.2f} dB")
     except (ToolNotFoundError, RuntimeError) as e:
         print(f"error: {e}", file=sys.stderr)
         return 2
 
-    report = write_report(results, points, args.report)
+    report = write_report(results, points, args.report, checks)
     print(f"report: {report}")
-    return 0 if all(r.passed for r in results) else 1
+    ok = all(r.passed for r in results) and (checks is None or checks.passed)
+    return 0 if ok else 1
 
 
PATCH
```

Apply from the repo root:

```bash
git apply <<'PATCH'
diff --git a/sim/neosim/report.py b/sim/neosim/report.py
index b5c7407..6710cad 100644
--- a/sim/neosim/report.py
+++ b/sim/neosim/report.py
@@ -13,5 +13,6 @@ matplotlib.use("Agg")
 import matplotlib.pyplot as plt  # noqa: E402
 
-from neosim.sweep import SweepPoint, implementation_loss_db, wilson_interval  # noqa: E402
+from neosim.measure import MDS_AGREEMENT_DB, MDS_BW_HZ, MDS_TARGET_DBM, HardwareChecks  # noqa: E402
+from neosim.sweep import BAND_NOISE_DBM_HZ, SweepPoint, implementation_loss_db, wilson_interval  # noqa: E402
 from neosim import hardware  # noqa: E402
 from neosim.twin import ScenarioResult  # noqa: E402
@@ -71,6 +72,8 @@ def _receiver_html(r: ScenarioResult) -> str:
             f"{h.noise['nf_db']:.2f} dB, preamp noise folded in from LO harmonics "
             f"{h.noise['harmonic_noise_w_per_hz'] / (hardware.K_B * hardware.T0):.3f} kT0, ADC "
-            f"{h.noise['adc_added_factor']:.3f}). See the hardware report "
-            f"(<code>build/hw/report.html</code>) for the block checks.</p>"
+            f"{h.noise['adc_added_factor']:.3f}). Block checks: <a href=\"../hw/report.html\">hardware report</a>.</p>"
+            f"<p>IMD3 products in the window: {html.escape(', '.join(f'{i}×{j} at {f / 1e6:.4f} MHz ({p:.1f} dBm)' for i, j, f, p in h.imd3)) or 'none'}; "
+            f"reciprocal mixing {h.reciprocal_dbm_per_hz:.1f} dBm/Hz; above full scale at the mixer: "
+            f"{html.escape(', '.join(h.overload)) or 'none'}</p>"
             f"<table><tr><th>stage</th><th>gain (dB)</th><th>NF (dB)</th><th>added noise factor (Friis)</th></tr>"
             f"{rows}</table>")
@@ -81,5 +84,8 @@ def _sweep_html(points: list[SweepPoint]) -> str:
     snrs = [p.snr_db for p in points]
     ax.plot(snrs, [p.reference_rate for p in points], "o-", label="reference (audio only)")
-    ax.plot(snrs, [p.twin_rate for p in points], "s-", label="digital twin")
+    ax.plot(snrs, [p.twin_rate for p in points], "s-", label="digital twin, ideal receiver")
+    hw = all(p.hardware_rate is not None for p in points)
+    if hw:
+        ax.plot(snrs, [p.hardware_rate for p in points], "^-", label="digital twin, hardware model")
     ax.set_xlabel("SNR in 2500 Hz (dB)")
     ax.set_ylabel("FT8 decode rate")
@@ -87,6 +93,7 @@ def _sweep_html(points: list[SweepPoint]) -> str:
     ax.grid(alpha=0.3)
     ax.legend()
-    loss = implementation_loss_db(points)
-    loss_txt = ("not determined (a curve does not cross 50 % inside the swept range)" if loss is None
+    def loss_txt(curve):
+        loss = implementation_loss_db(points, curve)
+        return ("not determined (a curve does not cross 50 % inside the swept range)" if loss is None
                 else f"{loss:+.1f} dB (50 % decode points, linearly interpolated)")
 
@@ -97,13 +104,35 @@ def _sweep_html(points: list[SweepPoint]) -> str:
     trials = points[0].trials
     rows = "".join(f"<tr><td>{p.snr_db:+.1f}</td><td>{cell(p.reference_rate, p.trials)}</td>"
-                   f"<td>{cell(p.twin_rate, p.trials)}</td></tr>" for p in points)
-    return (f"<h2>FT8 sensitivity sweep</h2><p><b>Implementation loss:</b> {loss_txt}</p>"
+                   f"<td>{cell(p.twin_rate, p.trials)}</td>"
+                   + (f"<td>{cell(p.hardware_rate, p.trials)}</td>" if hw else "") + "</tr>" for p in points)
+    hw_loss = (f"<p><b>Implementation loss, hardware model</b> (20 m, band noise {BAND_NOISE_DBM_HZ:g} dBm/Hz, "
+               f"preamp off; target ≤ 1 dB): {loss_txt('hardware')}</p>" if hw else "")
+    return (f"<h2>FT8 sensitivity sweep</h2><p><b>Implementation loss, ideal receiver:</b> {loss_txt('twin')}</p>{hw_loss}"
             f"<p>{trials} trials per point; brackets show the 95 % Wilson confidence interval of each decode rate. "
             f"Treat loss differences smaller than the SNR step as noise.</p>"
-            f"<table><tr><th>SNR (dB)</th><th>reference</th><th>twin</th></tr>{rows}</table>"
+            f"<table><tr><th>SNR (dB)</th><th>reference</th><th>twin</th>{'<th>hardware</th>' if hw else ''}</tr>"
+            f"{rows}</table>"
             f'<img alt="sweep" src="{_png(fig)}">')
 
 
-def write_report(results: list[ScenarioResult], sweep_points: list[SweepPoint] | None, path: Path) -> Path:
+def _checks_html(c: HardwareChecks) -> str:
+    rows = "".join(f"<tr><td>{m.f_lo_hz / 1e6:.1f} MHz</td><td>{'on' if m.preamp else 'off'}</td>"
+                   f"<td>{m.nf_total_db:.2f}</td><td>{m.predicted_dbm:.1f}</td><td>{m.measured_dbm:.1f}</td>"
+                   f"<td>{_badge(m.passed)}</td></tr>" for m in c.mds)
+    b = c.blocker
+    decoded = " | ".join(x.output for x in b.with_blocker.receives) or "nothing"
+    return (f"<h2>Hardware checks {_badge(c.passed)}</h2>"
+            f"<p><b>MDS</b> (CW level for 3 dB (S+N)/N in {MDS_BW_HZ:g} Hz, thermal noise only; target ≤ "
+            f"{MDS_TARGET_DBM:g} dBm and within ±{MDS_AGREEMENT_DB:g} dB of the noise budget):</p>"
+            f"<table><tr><th>LO</th><th>preamp</th><th>receiver NF (dB)</th><th>budget (dBm)</th>"
+            f"<th>measured (dBm)</th><th>result</th></tr>{rows}</table>"
+            f"<p><b>Blocker</b> ({html.escape(b.with_blocker.scenario.name)}): {_badge(b.passed)} decoded "
+            f"<code>{html.escape(decoded)}</code>; noise floor in the window {b.degradation_db:+.2f} dB with "
+            f"{', '.join(map(html.escape, b.blocker_ids))} against without; reciprocal mixing "
+            f"{b.with_blocker.hardware.reciprocal_dbm_per_hz:.1f} dBm/Hz at the antenna.</p>")
+
+
+def write_report(results: list[ScenarioResult], sweep_points: list[SweepPoint] | None, path: Path,
+                 checks: HardwareChecks | None = None) -> Path:
     path = Path(path)
     path.parent.mkdir(parents=True, exist_ok=True)
@@ -111,5 +140,7 @@ def write_report(results: list[ScenarioResult], sweep_points: list[SweepPoint] |
     if sweep_points:
         body += _sweep_html(sweep_points)
-    overall = all(r.passed for r in results)
+    if checks is not None:
+        body += _checks_html(checks)
+    overall = all(r.passed for r in results) and (checks is None or checks.passed)
     path.write_text(
         f"<!doctype html><html><head><meta charset='utf-8'><title>NeoRadio Sim Report</title>"
PATCH
```

Apply from the repo root:

```bash
git apply <<'PATCH'
diff --git a/Makefile b/Makefile
index aaf31f9..cc2ebe6 100644
--- a/Makefile
+++ b/Makefile
@@ -22,5 +22,5 @@ test: tools
 
 sim: tools
-	cd sim && $(PYTHON) -m neosim run scenarios/*.yaml --workdir ../build/sim --report ../build/sim/report.html --sweep-ft8 -14 -22 1 --trials 20
+	cd sim && $(PYTHON) -m neosim run scenarios/*.yaml --workdir ../build/sim --report ../build/sim/report.html --sweep-ft8 -14 -22 1 --trials 20 --sweep-hardware --hardware-checks
 
 hw:
PATCH
```

- [ ] **Step 5: Run the sim suite and the hardware checks**

Run: `make tools && cd sim && python3 -m pytest -q tests && python3 -m neosim run scenarios/*.yaml --workdir ../build/sim --report ../build/sim/report.html --hardware-checks`
Expected: 107 tests passed; `PASS` for the four scenarios (blocker, 20 m hardware, 20 m ideal, 6 m hardware); `PASS  MDS 14.1 MHz: -140.5 dBm (budget -140.5)`, `PASS  MDS 50.1 MHz: -140.6 dBm (budget -140.5)`, `PASS  blocker: floor +0.00 dB` (sign may print as -0.00).

- [ ] **Step 6: Commit**

```bash
git add sim/neosim/measure.py sim/tests/test_measure.py sim/scenarios/20m_blocker_hardware.yaml sim/neosim/sweep.py sim/neosim/__main__.py sim/neosim/report.py Makefile
git commit -m "feat(sim): MDS and blocker checks, hardware FT8 sweep"
```
### Task 6: Full runs and the README

- [ ] **Step 1: `make sim`** (background; the sweep alone took 5.6 minutes in the prototype)

Run: `make sim`
Expected: all scenarios PASS, MDS and blocker PASS; the report shows implementation loss −0.1 dB (ideal receiver) and +0.5 dB (hardware model), each ±1 dB run to run (20 trials).

- [ ] **Step 2: `make test`**

Run: `make test`
Expected: C# 34 + 22 passed; sim 107 passed; hardware 202 passed.

- [ ] **Step 3: README roadmap** — tick `Intermodulation and reciprocal-mixing models; blocker and MDS scenarios` and `I/Q imbalance correction in the DSP`. Commit with `docs: README roadmap for plan 2.6`.
