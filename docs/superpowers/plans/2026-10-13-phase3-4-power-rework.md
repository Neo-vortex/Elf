# Phase 3.4: Power Rework (9–30 V Jack Only) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the USB/5 V power design with a 9–30 V barrel jack as the only supply (LM2576HV buck to a 5.5 V main rail, NTC inrush limiting, reverse and surge protection), with USB carrying data only, and make the controller checks and the firmware follow.

**Architecture:** `blocks/power.py` is rebuilt around an averaged LM2576HV model; `checks/power.py` simulates the rails from 9 to 30 V, a reversed plug, the TVS clamp, hot-plug transients (cold NTC, fuse I²t, buck input overshoot), junction temperatures and the buck's conducted ripple. The controller check's relay-coil margin becomes a gate; the firmware reads the monitor divider ratios from `power.json` and replaces the USB-C current gate on the bias-T with a 5V_MAIN-in-regulation rule.

**Tech Stack:** Python 3, SKiDL 2.3, ngspice 45.2, C11 firmware (CMake), pytest.

**Spec:** `docs/superpowers/specs/2026-10-12-phase3-board-circuits-design.md` (§3.2 and its "plan 3.4 power rework" amendment); `docs/superpowers/specs/2026-10-13-phase4-firmware-virtual-board-design.md` (supervisor).

## Global Constraints

- Input 9–30 V on the jack only; USB VBUS powers nothing (≤ 1 mA beyond its sense divider).
- Every IC and semiconductor confirmed on Torob (survey `2026-10-13-parts-survey-power-torob.json`); LM2576HV from listings not marked "copy".
- Rails in regulation at 9 and 30 V and maximum load; 5V_MAIN 5.3–5.7 V; junctions ≤ 125 °C at 50 °C ambient.
- The buck input stays ≤ 60 V (TVS clamp, hot-plug overshoot); plug-in I²t ≤ the time-lag fuse's melting I²t / 5.
- Conducted spurs ≤ MDS − 20 dB = −160.5 dBm on every amateur band.
- Relay coils pull in at every supply corner at 50 °C.

## Review Focus

- The averaged LM2576HV model (current into the switch node until FB = 1.23 V, saturation at VIN − 1.4 V): rails must not depend on its artefacts (test: `test_rails_hold_from_9_to_30_volts`).
- Cold start at 9 V through the cold NTC, and the hot-plug I²t at 30 V (test: `test_hot_plug_starts_the_board_within_the_fuse_and_buck_ratings`).
- The ripple deck holds 5V_MAIN at its DC operating point (1 H feed) so the LDO models are not in dropout (the plan 3.2 review's M2).
- The external sense divider at 30 V stays inside the ESP32-S3 ADC range (test: `test_ext_sense_divider_keeps_30_volts_inside_the_adc_range`).
- Firmware: bias-T refused and dropped while 5V_MAIN is out of regulation; divider ratios come from `power.json` (tests: `bias_needs_5v_main_in_regulation`, `bias_drops_when_main_falls`, `monitors_read_through_the_dividers`).

---

### Task 1: Power parts, block and check

**Files:**
- Modify: `hardware/neohw/parts.py` (remove ao3401, ss34, smbj5, mt3608, tlv74033; add ntc_inrush, ss36, smbj30a, lm2576hv, ams1117_33, fuse_i2t_min; LED Torob keys; button footprint), `hardware/neohw/availability.py`, `hardware/neohw/blocks/power.py`, `hardware/neohw/checks/power.py`, `hardware/neohw/blocks/bias_tee.py` (docstring), `hardware/characterized/power.json` (regenerated)
- Test: `hardware/tests/test_power.py`, `hardware/tests/test_pin_names.py`

**Interfaces:**
- Produces: `parts.lm2576hv()`, `ss36()`, `smbj30a()`, `ntc_inrush()`, `ams1117_33()`, `fuse_i2t_min(amps)`, `LM2576_VREF`, `LM2576_EFFICIENCY`, `NTC_COLD_OHM`; `power.build(ckt)` ports `vbus, ext_tip, ext_p, buck_in, sw, main, ldo_a_in, ldo_rf_in, v5a, v3rf, v3d, bias5, sense_*`; `power.SENSE_DIVIDERS`, `FUSE_A`, `BUCK_L`; `checks.power.dc_scenarios`, `surge_clamp`, `plug_in(blk, volts)`, `junction_temps`, `ripple_spurs`, `MAIN_V`; `power.json` keys `scenarios` (names `jack 9 V, maximum load` …) and `sense_dividers`.

- [ ] **Step 1: Write the failing tests**

```python
import pytest

from neohw import parts
from neohw.blocks import power
from neohw.checks import power as check
from neohw.checks.clock import net_check
from neohw.tools import ToolNotFoundError, tool_path


def by_value(blk, value):
    return [p for p in blk.circuit.parts if p.value == value]


@pytest.fixture
def ngspice():
    try:
        tool_path("ngspice")
    except ToolNotFoundError as e:
        pytest.skip(str(e))


@pytest.fixture(scope="module")
def dc():
    try:
        tool_path("ngspice")
    except ToolNotFoundError as e:
        pytest.skip(str(e))
    return check.dc_scenarios(power.build())


def test_usb_powers_nothing():
    blk = power.build()
    vbus = blk.ports["vbus"]
    on_vbus = {p.value for p in blk.circuit.parts for pin in p.pins if pin.net is not None and pin.net.name == vbus.name}
    assert on_vbus == {"100kΩ"}                      # only the sense divider (the USB-C pads are in the controller)


def test_input_chain_is_fuse_ntc_diode_then_tvs():
    blk = power.build()
    fuse = by_value(blk, f"{power.FUSE_A:g}A fuse")[0]
    ntc = next(p for p in blk.circuit.parts if p.value.startswith("NTC"))
    d = next(p for p in by_value(blk, "SS36") if p["K"].net.name == "ext_p")
    tvs = by_value(blk, "SMBJ30A")[0]
    assert fuse["1"].net.name == "ext_tip" and fuse["2"].net is ntc["1"].net
    assert ntc["2"].net is d["A"].net and tvs["K"].net.name == "ext_p"


def test_buck_feedback_sets_5v5_and_every_net_is_connected():
    blk = power.build()
    top, bottom = (float(v.rstrip("k")) for v in (power.BUCK_FB_TOP, power.BUCK_FB_BOTTOM))
    assert parts.LM2576_VREF * (1 + top / bottom) == pytest.approx(5.5, abs=0.02)
    assert {p.value for p in blk.circuit.parts} >= {"LM2576HV-ADJ", "MIC5205-5.0", "MIC5205-3.3", "AMS1117-3.3", "SS36", "SMBJ30A"}
    assert not {p.value for p in blk.circuit.parts} & {"MT3608", "AO3401", "TLV74033", "SS34"}
    net_check(blk)


def test_psrr_assumption_is_bounded():
    assert check.psrr_db(100.0) == 75.0 and check.psrr_db(1e6) == 15.0 and check.psrr_db(30e6) == 15.0


def test_junction_temperature_uses_each_regulators_own_input_at_its_worst_maximum_load_case():
    base = {"main": 5.46, "ldo_a_in": 5.43, "ldo_rf_in": 5.45, "v5a": 5.0, "v3rf": 3.3, "v3d": 3.3}
    hot = {**base, "main": 5.6, "ldo_rf_in": 5.6}
    dc = {"jack 9 V, maximum load": base, "jack 30 V, maximum load": hot,
          "jack 12 V, typical load": {**base, "main": 9.0, "ldo_rf_in": 9.0}}   # a typical case never sets the worst
    tj = check.junction_temps(dc)
    assert tj["AMS1117-3.3"] == pytest.approx(50 + 90 * (5.6 - 3.3) * check.LOADS_MAX["v3d"])
    assert tj["MIC5205-3.3"] == pytest.approx(50 + 220 * (5.6 - 3.3) * check.LOADS_MAX["v3rf"])
    p_out = 5.6 * sum(check.LOADS_MAX.values())
    assert tj["LM2576HV"] == pytest.approx(50 + 50 * p_out * (1 / parts.LM2576_EFFICIENCY - 1))


def test_ext_sense_divider_keeps_30_volts_inside_the_adc_range():
    assert 30.0 / power.SENSE_DIVIDERS["ext"] <= 3.1


@pytest.mark.integration
def test_rails_hold_from_9_to_30_volts(dc):
    for name, s in dc.items():
        if "reversed" in name:
            continue
        assert check.MAIN_V[0] <= s["main"] <= check.MAIN_V[1], name
        assert s["v5a"] >= 4.95 and s["v3rf"] >= 3.25 and s["v3d"] >= 3.25, name
        assert abs(s["i_usb"]) < 1e-4, name                    # only the 200 kΩ divider: 25 µA


@pytest.mark.integration
def test_a_reversed_plug_draws_nothing(dc):
    s = dc["jack reversed (−12 V)"]
    assert abs(s["i_ext"]) < 1e-3 and s["main"] > -0.3


@pytest.mark.integration
def test_surge_clamp_stays_under_the_buck_rating(ngspice):
    assert check.surge_clamp(power.build()) <= check.BUCK_VIN_MAX_V


@pytest.mark.integration
@pytest.mark.parametrize("volts", [9.0, 30.0])
def test_hot_plug_starts_the_board_within_the_fuse_and_buck_ratings(ngspice, volts):
    r = check.plug_in(power.build(), volts)
    assert r["rails_up_s"] < 20e-3                                       # 9 V: the cold NTC still lets it start
    assert r["buck_in_peak_v"] <= check.BUCK_VIN_MAX_V
    assert r["i2t_a2s"] <= parts.fuse_i2t_min(power.FUSE_A) / check.FUSE_PULSE_MARGIN


@pytest.mark.integration
def test_conducted_spurs_are_far_below_mds(ngspice):
    spurs = check.ripple_spurs(power.build())
    assert set(spurs) == set(check.BANDS)
    assert max(s["total_dbm"] for s in spurs.values()) <= check.MDS_DBM - check.SPUR_MARGIN_DB
```

```diff
diff --git a/hardware/tests/test_pin_names.py b/hardware/tests/test_pin_names.py
index a3b04af..a1cc83a 100644
--- a/hardware/tests/test_pin_names.py
+++ b/hardware/tests/test_pin_names.py
@@ -6,8 +6,8 @@ from neohw import parts
 
 CONSTRUCTORS = [
     lambda: parts.resistor("1k"), lambda: parts.capacitor("1n"), lambda: parts.inductor(1e-6, 30, 1e6),
-    parts.si5351c, parts.crystal_25mhz, parts.lvc74, parts.lvc157, parts.ao3401, parts.ss34, parts.smbj5,
-    lambda: parts.fuse_1206(1.0), parts.barrel_jack, parts.mt3608, lambda: parts.mic5205(5.0), parts.tlv74033,
+    parts.si5351c, parts.crystal_25mhz, parts.lvc74, parts.lvc157, parts.ss36, parts.smbj30a, parts.lm2576hv, parts.ntc_inrush,
+    lambda: parts.fuse_1206(1.0), parts.barrel_jack, lambda: parts.mic5205(5.0), parts.ams1117_33,
     parts.esp32s3_wroom1, parts.usb_c_16p, parts.usblc6, parts.lm75a, parts.hc595, parts.uln2803,
     lambda: parts.sy6280(17.4e3), parts.oscillator_24m576, parts.led_0805, parts.button, parts.fst3253,
     parts.lm4562_dual, parts.cs5361, parts.pnp_bc857b, parts.npn_bfr92a, parts.diode_pair_bav99, parts.gdt,
```

- [ ] **Step 2: Run them to verify they fail**

Run: `cd hardware && python3 -m pytest -q tests/test_power.py tests/test_pin_names.py`
Expected: FAIL — `AttributeError: module 'neohw.parts' has no attribute 'ss36'` (collection error in test_pin_names) and `AttributeError: ... 'SENSE_DIVIDERS'`.

- [ ] **Step 3: Implement**

```diff
diff --git a/hardware/neohw/availability.py b/hardware/neohw/availability.py
index 757a023..fe29e60 100644
--- a/hardware/neohw/availability.py
+++ b/hardware/neohw/availability.py
@@ -10,7 +10,8 @@ from neohw.tools import REPO_ROOT
 SPECS = REPO_ROOT / "docs" / "superpowers" / "specs"
 DEFAULT_PATH = SPECS / "2026-10-12-parts-availability-torob.json"
 SURVEYS = (DEFAULT_PATH, SPECS / "2026-10-12-parts-survey-passives-torob.json",
-           SPECS / "2026-10-12-parts-survey-phase3-torob.json")
+           SPECS / "2026-10-12-parts-survey-phase3-torob.json",
+           SPECS / "2026-10-13-parts-survey-power-torob.json")
 
 
 def load(path: Path | None = None) -> dict:
diff --git a/hardware/neohw/blocks/bias_tee.py b/hardware/neohw/blocks/bias_tee.py
index 2d11c04..12542ef 100644
--- a/hardware/neohw/blocks/bias_tee.py
+++ b/hardware/neohw/blocks/bias_tee.py
@@ -1,6 +1,6 @@
 """Antenna input: GDT + 1 MΩ bleed at the connector, then the 5 V bias-T (spec §4).
 
-The bias feed (switched by an AO3401 + PTC in phase 3) is decoupled to ground here. The
+The bias feed (switched and current-limited by the SY6280 in the controller block) is decoupled to ground here. The
 choke chain runs smallest-first from the RF line: 4.7 µH, 47 µH (damped by 2.2 kΩ: without
 it the 47 µH and the capacitive 1 mH form a series resonance near 38 MHz) and 1 mH.
 """
diff --git a/hardware/neohw/parts.py b/hardware/neohw/parts.py
index 8301ae6..55858fc 100644
--- a/hardware/neohw/parts.py
+++ b/hardware/neohw/parts.py
@@ -347,35 +347,13 @@ def lvc157(nc=()) -> Part:
 
 # --- power (phase 3 spec §3.2) -----------------------------------------------------------
 
-def ao3401() -> Part:
-    """Alpha & Omega AO3401A P-MOSFET, SOT-23 (G 1, S 2, D 3): −30 V, Rds(on) ≤ 60 mΩ at
-    Vgs −4.5 V, Vgs(th) −0.5 to −1.3 V (typ −0.9). Level-1 fit: vto −0.9 V, kp from 55 mΩ at
-    |Vgs| 4.65 V; body diode drain→source... (anode at drain, cathode at source)."""
-    p = _ic("AO3401", {"G": "1", "S": "2", "D": "3"}, "Package_TO_SOT_SMD:SOT-23", "AO3401A", "AO3401")
-    p.spice_models = (".model AO3401_M pmos(level=1 vto=-0.9 kp=4.85 lambda=0.01)\n"
-                      ".model AO3401_BD d(is=1e-12 n=1.0 rs=0.05)",)
-    p.spice_lines = lambda n, k=1.0: [f"M{p.ref} {n['D']} {n['G']} {n['S']} {n['S']} AO3401_M w=1 l=1",
-                                      f"D{p.ref}_bd {n['D']} {n['S']} AO3401_BD"]
-    return p
-
-
-def ss34() -> Part:
-    """3 A 40 V Schottky, SMA (K 1, A 2). Fit: 0.35 V at 0.3 A, 0.5 V at 3 A (datasheet typical)."""
-    p = _ic("SS34", {"K": "1", "A": "2"}, "Diode_SMD:D_SMA", "SS34", "SS34")
-    p.spice_models = (".model SS34_D d(is=1.4e-6 n=1.0 rs=0.028 cjo=200p bv=40)",)
-    p.spice_lines = lambda n, k=1.0: [f"D{p.ref} {n['A']} {n['K']} SS34_D"]
-    return p
-
-
-def smbj5() -> Part:
-    """SMBJ5.0A 600 W TVS, SMB (K 1, A 2): stand-off 5.0 V, breakdown 6.4 V min."""
-    p = _ic("SMBJ5.0A", {"K": "1", "A": "2"}, "Diode_SMD:D_SMB", "SMBJ5.0A", "SMBJ5.0A")
-    p.spice_models = (".model SMBJ5_D d(is=1e-12 n=1.5 bv=6.6 ibv=1m rs=0.1 cjo=1n)",)
-    p.spice_lines = lambda n, k=1.0: [f"D{p.ref} {n['A']} {n['K']} SMBJ5_D"]
-    return p
+FUSE_1206_SLOW_I2T_MIN = 0.05     # A²s, melting I²t of a 1 A time-lag 1206 fuse (lower end of the series)
 
 
-FUSE_1206_SLOW_I2T_MIN = 0.05     # A²s, melting I²t of a 1 A time-lag 1206 fuse (lower end of the series)
+def fuse_i2t_min(amps: float) -> float:
+    """Melting I²t of a time-lag 1206 fuse, scaled from the 1 A figure with the rating squared
+    (I²t of a fuse family grows roughly with the rated current squared)."""
+    return FUSE_1206_SLOW_I2T_MIN * amps ** 2
 
 
 def fuse_1206(amps: float) -> Part:
@@ -395,24 +373,70 @@ def barrel_jack() -> Part:
     return p
 
 
-MT3608_EFFICIENCY = 0.85
+NTC_COLD_OHM, NTC_HOT_OHM = 2.5, 0.5
 
 
-def mt3608() -> Part:
-    """Aerosemi MT3608 1.2 MHz boost, SOT23-6 (SW 1, GND 2, FB 3, EN 4, IN 5, NC/OC 6); FB 0.6 V
-    (0.588–0.612), 1.2 MHz typical, PFM at light load (MT3608L datasheet Rev 1.1).
-    DC model (averaged): a current pushed into SW until FB reaches 0.6 V (the external diode
-    and FB divider then set VOUT), and IN drawing that power over the efficiency, η = 0.85. The
-    boost inductor is open in the DC deck (the averaged model replaces its switching); the
-    ripple is analysed separately."""
-    p = _ic("MT3608", {"SW": "1", "GND": "2", "FB": "3", "EN": "4", "IN": "5", "NC": "6"},
-            "Package_TO_SOT_SMD:SOT-23-6", "MT3608", "MT3608", nc=("NC",))
-    reg = "max(0, min(3, 20 * (0.6 - v({fb}))))"
-    p.spice_lines = lambda n, k=1.0: [
-        f"B{p.ref}_reg {n['GND']} {n['SW']} I = " + reg.format(fb=n["FB"]),
-        f"B{p.ref}_in {n['IN']} {n['GND']} I = " + reg.format(fb=n["FB"])
-        + f" * v({n['SW']}) / ({MT3608_EFFICIENCY:g} * max(v({n['IN']}), 0.5))",
-        f"R{p.ref}_en {n['EN']} {n['GND']} 1e9"]
+def ntc_inrush() -> Part:
+    """2.5 Ω inrush-limiting NTC thermistor, 7 mm disc, 5 mm pitch (2.5D-7 class, ≥ 2 A): 2.5 Ω cold
+    limits the plug-in charge of the buck's input capacitor yet still lets the board start cold
+    at 9 V (5 Ω would not: its drop at 0.44 A starves the buck); warm it falls to ≈ 0.3 Ω (the DC
+    model uses 0.5 Ω, conservative; the plug-in check uses 2.5 Ω cold, and the cold-start case
+    too). A passive: the manufacturer sources it."""
+    p = _tagged(_template("NTC", "TH", ["1", "2"], "Varistor:RV_Disc_D7mm_W3.4mm_P5mm")(value="NTC 2.5D-7"))
+    p.mpn, p.torob, p.tol, p.bom_note = "NTC 2.5D-7", None, 0.2, "inrush limiter, 2.5 Ω cold, ≥ 2 A"
+    p.spice_lines = lambda n, k=1.0: [f"R{p.ref} {n['1']} {n['2']} {NTC_HOT_OHM:g}"]
+    return p
+
+
+def ss36() -> Part:
+    """3 A 60 V Schottky, SMC/SMB-class DO-214AB (K 1, A 2): the input's reverse-polarity diode and the
+    buck's catch diode. Fit: 0.45 V at 0.3 A, 0.7 V at 3 A (datasheet typical)."""
+    p = _ic("SS36", {"K": "1", "A": "2"}, "Diode_SMD:D_SMC", "SS36", "SS36")
+    p.spice_models = (".model SS36_D d(is=5e-7 n=1.05 rs=0.06 cjo=150p bv=60)",)
+    p.spice_lines = lambda n, k=1.0: [f"D{p.ref} {n['A']} {n['K']} SS36_D"]
+    return p
+
+
+def smbj30a() -> Part:
+    """SMBJ30A 600 W TVS, SMB (K 1, A 2): stand-off 30 V, breakdown 33.3 V min, clamp 48.4 V at
+    12.4 A (8/20-class surge): the buck's 60 V rating sits above the clamp."""
+    p = _ic("SMBJ30A", {"K": "1", "A": "2"}, "Diode_SMD:D_SMB", "SMBJ30A", "SMBJ30A")
+    p.spice_models = (".model SMBJ30_D d(is=1e-12 n=1.5 bv=35 ibv=1m rs=1.1 cjo=600p)",)
+    p.spice_lines = lambda n, k=1.0: [f"D{p.ref} {n['A']} {n['K']} SMBJ30_D"]
+    return p
+
+
+LM2576_VREF, LM2576_EFFICIENCY, LM2576_VSAT = 1.23, 0.77, 1.4
+
+
+def lm2576hv() -> Part:
+    """TI/ON LM2576HVS-ADJ 3 A buck, 52 kHz, 60 V input (63 V abs.), TO-263-5 (1 VIN, 2 OUTPUT, 3 GND
+    and tab, 4 FEEDBACK, 5 /ON-OFF: low = on). Feedback 1.23 V. Torob: buy listings not marked
+    "copy" (a clone's input rating cannot be trusted).
+    DC model (averaged): the regulator pushes current into the switch node until FEEDBACK reaches
+    1.23 V; the inductor carries it (its DC resistance included); VIN supplies that power over the
+    efficiency η = 0.77 (datasheet curve, 5 V out, 12–30 V in, 0.5 A); the switch cannot raise the
+    output above VIN − 1.4 V (saturation)."""
+    p = _ic("LM2576HV-ADJ", {"VIN": "1", "OUT": "2", "GND": "3", "FB": "4", "ONOFF": "5"},
+            "Package_TO_SOT_SMD:TO-263-5_TabPin3", "LM2576HVS-ADJ", "LM2576HV")
+    def lines(n, k=1.0):
+        reg = (f"max(0, min(3, 50 * ({LM2576_VREF:g} - v({n['FB']})))) "
+               f"* 0.5 * (1 + tanh(10 * (v({n['VIN']}) - v({n['OUT']}) - {LM2576_VSAT:g})))")
+        return [f"B{p.ref}_reg {n['GND']} {n['OUT']} I = {reg}",
+                f"B{p.ref}_in {n['VIN']} {n['GND']} I = {reg} * max(v({n['OUT']}), 0) / "
+                f"({LM2576_EFFICIENCY:g} * max(v({n['VIN']}), 1))",
+                f"R{p.ref}_on {n['ONOFF']} {n['GND']} 1e9"]
+    p.spice_lines = lines
+    return p
+
+
+def ams1117_33() -> Part:
+    """AMS1117-3.3, SOT-223 (GND 1, OUT 2 and tab, IN 3). From 5.5 V its dropout (1.3 V max) is
+    not a limit; the SOT-223 tab carries its 0.33 W (≈ 90 °C/W)."""
+    p = _ic("AMS1117-3.3", {"GND": "1", "OUT": "2", "IN": "3"}, "Package_TO_SOT_SMD:SOT-223-3_TabPin2",
+            "AMS1117-3.3", "AMS1117-3.3")
+    p.v_out, p.dropout_v = 3.3, 1.3
+    p.spice_lines = lambda n, k=1.0: _ldo_lines(p.ref, n, 3.3, 1.3)
     return p
 
 
@@ -428,18 +452,6 @@ def mic5205(v_out: float) -> Part:
     return p
 
 
-def tlv74033() -> Part:
-    """TI TLV74033PDBVR 300 mA LDO, SOT-23-5 (IN 1, GND 2, EN 3, NC 4, OUT 5), stable with a 1 µF
-    ceramic. Dropout ≈ 0.23 V typical at 300 mA (datasheet SBVS235); the model uses 0.4 V, above
-    the maximum at our ≤ 150 mA. On Torob under the LDL1117S33 search (TLV74033PDBVR listing)."""
-    p = _ic("TLV74033", {"IN": "1", "GND": "2", "EN": "3", "NC": "4", "OUT": "5"},
-            "Package_TO_SOT_SMD:SOT-23-5", "TLV74033PDBVR", "LDL1117S33", nc=("NC",))
-    p.torob_match = lambda name: "TLV74033" in name.upper()
-    p.v_out, p.dropout_v = 3.3, 0.4
-    p.spice_lines = lambda n, k=1.0: _ldo_lines(p.ref, n, 3.3, 0.4) + [f"R{p.ref}_en {n['EN']} {n['GND']} 1e7"]
-    return p
-
-
 def _ldo_lines(ref: str, n: dict, v_out: float, dropout: float, byp: str | None = None) -> list[str]:
     out = [f"B{ref}_reg {ref}_r {n['GND']} V = min({v_out:g}, v({n['IN']}) - {dropout:g})",
            f"V{ref}_sense {ref}_r {ref}_s dc 0",
@@ -534,14 +546,20 @@ def oscillator_24m576() -> Part:
     confirm (pending, user away 2026-10-13)."""
     p = _ic("24.576MHz", {"EN": "1", "GND": "2", "OUT": "3", "VDD": "4"},
             "Oscillator:Oscillator_SMD_Abracon_ASE-4Pin_3.2x2.5mm", "24.576MHz 3.3V CMOS XO", "اسیلاتور 24.576")
-    p.bom_note = "Torob availability not yet confirmed"
+    p.bom_note = "on Torob (2026-10-13) as a 2 × 3 mm part: match the footprint to the bought package"
     return p
 
 
+LED_TOROB = {"green": "LED SMD 0805 سبز", "yellow": "LED SMD 0805 زرد", "red": "LED SMD 0805 قرمز"}
+
+
 def led_0805(colour: str = "green") -> Part:
-    p = _ic(f"LED {colour}", {"K": "1", "A": "2"}, "LED_SMD:LED_0805_2012Metric", f"LED 0805 {colour}", None)
+    p = _ic(f"LED {colour}", {"K": "1", "A": "2"}, "LED_SMD:LED_0805_2012Metric", f"LED 0805 {colour}",
+            LED_TOROB.get(colour))
     return p
 
 
 def button() -> Part:
-    return _ic("button", {"1": "1", "2": "2"}, "Button_Switch_SMD:SW_SPST_SKQG_WithoutStem", "SKQG tactile", None)
+    """6 × 3.5 mm two-pin SMD tactile switch (CK PTS636-class; on Torob as a 3 × 6 mm SMD switch)."""
+    return _ic("button", {"1": "1", "2": "2"}, "Button_Switch_SMD:SW_Tactile_SPST_NO_Straight_CK_PTS636Sx25SMTRLFS",
+               "6x3.5 mm SMD tactile", "تک سوئیچ SMD")
```

`hardware/neohw/blocks/power.py`:

```python
"""Power (phase 3 spec §3.2, plan 3.4 amendment): the 9–30 V barrel jack is the only supply.

Input:
- Jack 9–30 V → 1.5 A time-lag fuse → 2.5 Ω NTC inrush limiter (cold; ≈ 0.3 Ω warm) → SS36 Schottky (a reversed plug conducts nothing; at the
  board's ≤ 0.35 A input current its 0.45 V drop costs little) → SMBJ30A TVS (clamps a surge at
  ≈ 48 V, under the buck's 60 V rating) → input LC (10 µH + 100 µF / 50 V electrolytic ‖ 1 µF),
  which also keeps the buck's input pulses off the adapter cable.
- LM2576HVS-ADJ buck (52 kHz) → 100 µH → 5V_MAIN = 5.5 V (FB 3.48 k / 1 k), 470 µF / 16 V
  low-ESR electrolytic ‖ 22 µF ‖ 100 nF; SS36 catch diode.

Rails from 5V_MAIN:
- relay coils (ULN2803 COM and coils): ≈ 4.6 V on a coil, above the G6K-2F's must-operate
  voltage even at 50 °C;
- 5V_A (CS5361 VA, LM4562, FST3253, preamp, mixer bias): LC (10 µH + 22 µF ‖ 100 nF) → MIC5205-5.0;
- 3V3_RF (Si5351C, 74LVC74/157, LM75A): LC → MIC5205-3.3;
- 3V3_D (ESP32-S3, CS5361 VL/VD, logic): AMS1117-3.3 (SOT-223: from 5.5 V it dissipates
  0.33 W, too much for a SOT-23-5);
- 5V_BIAS (the bias-T switch): LC from 5V_MAIN.

USB carries data only: VBUS reaches the USBLC6 and a sense divider (host present) and powers
nothing.

Monitoring: VBUS 100 k / 100 k, the jack (after the SS36) 100 k / 10 k (30 V → 2.7 V),
5V_MAIN 100 k / 100 k, each to an ADC pin.
"""
from __future__ import annotations

from skidl import Circuit, Net

from neohw import parts
from neohw.blocks import Block

BUCK_FB_TOP, BUCK_FB_BOTTOM = "3.48k", "1k"     # 1.23 V × (1 + 3.48) = 5.51 V
FUSE_A = 1.5                    # time-lag: rides the cold-NTC plug-in pulse at 30 V; 3 × the 0.44 A maximum input
CERAMIC_ESL = 1.0e-9           # 1206 / 0805 MLCC, mounted
CERAMIC_ESR = 5e-3
ELECTROLYTIC_ESL = 5e-9
LC_L = 10e-6                   # filter inductors: 1210 wirewound, SRF ≈ 30 MHz (2.5 pF), DCR ≈ 0.3 Ω
BUCK_L = 100e-6
SENSE_DIVIDERS = {"vbus": 2.0, "ext": 11.0, "main": 2.0}   # node voltage / ADC voltage
BIAS_FILTER_NOTE = "bias-T feed filter, 1210 wirewound, rated ≥ 0.4 A"   # carries the SY6280's 0.39 A limit


def _r(value, a, b):
    r = parts.resistor(value)
    r[1] += a
    r[2] += b
    return r


def _c(value, a, b, note="X7R decoupling", footprint="Capacitor_SMD:C_0805_2012Metric"):
    c = parts.capacitor(value, note=note, footprint=footprint, esr=CERAMIC_ESR, esl=CERAMIC_ESL)
    c[1] += a
    c[2] += b
    return c


def _elec(value, a, b, esr, note):
    c = parts.capacitor(value, note=note, footprint="Capacitor_SMD:CP_Elec_8x10", esr=esr, esl=ELECTROLYTIC_ESL,
                        tol=0.2)
    c[1] += a
    c[2] += b
    return c


def _bulk(node, gnd, value="22u", note="X5R 16 V bulk"):
    _c(value, node, gnd, note, "Capacitor_SMD:C_1206_3216Metric")
    _c("100n", node, gnd)


def _filter_l(a, b, note="filter, 1210 wirewound, rated ≥ 300 mA"):
    l = parts.inductor(LC_L, 20.0, 1e6, c_par=2.5e-12, dcr=0.3, tol=0.2,
                       footprint="Inductor_SMD:L_1210_3225Metric", note=note)
    l[1] += a
    l[2] += b
    return l


def build(ckt: Circuit | None = None) -> Block:
    ckt = ckt or Circuit()   # a shared circuit when the board netlist builds every block
    with ckt:
        gnd = Net("GND")
        tip, fused, ext_p, vin, sw, fb = Net("ext_tip"), Net("ext_fused"), Net("ext_p"), Net("buck_in"), Net("buck_sw"), Net("buck_fb")
        main, vbus = Net("+5V_MAIN"), Net("VBUS")
        ldo_a_in, rf_in, v5a, v3rf, v3d, bias5 = Net("ldo_a_in"), Net("ldo_rf_in"), Net("+5V_A"), Net("+3V3_RF"), Net("+3V3_D"), Net("+5V_BIAS")
        s_vbus, s_ext, s_main = Net("sense_vbus"), Net("sense_ext"), Net("sense_main")
        byp_a, byp_rf = Net("ldo_a_byp"), Net("ldo_rf_byp")
        # input protection
        j = parts.barrel_jack()
        j["TIP"] += tip
        j["SLEEVE"] += gnd
        f = parts.fuse_1206(FUSE_A)
        f["1"] += tip
        f["2"] += fused
        ntc, limited = parts.ntc_inrush(), Net("ext_ntc")
        ntc["1"] += fused
        ntc["2"] += limited
        d_rev = parts.ss36()
        d_rev["A"] += limited
        d_rev["K"] += ext_p
        tvs = parts.smbj30a()
        tvs["K"] += ext_p
        tvs["A"] += gnd
        _filter_l(ext_p, vin, "buck input filter, 1210 wirewound, rated ≥ 0.6 A")
        _elec("100u", vin, gnd, 0.3, "aluminium electrolytic 50 V, low ESR")
        _c("1u", vin, gnd, "X7R 50 V", "Capacitor_SMD:C_1206_3216Metric")
        # buck to 5.5 V
        b = parts.lm2576hv()
        b["VIN"] += vin
        b["GND"] += gnd
        b["ONOFF"] += gnd                 # low = on
        b["OUT"] += sw
        b["FB"] += fb
        d_catch = parts.ss36()
        d_catch["K"] += sw
        d_catch["A"] += gnd
        lb = parts.inductor(BUCK_L, 20.0, 50e3, c_par=10e-12, dcr=0.1, tol=0.2,
                            footprint="Inductor_SMD:L_Bourns_SRP7028A_7.3x6.6mm",
                            note="buck inductor, 7.3×6.6 mm power, Isat ≥ 3 A (Torob: 100 µH 4.6 A SMD0630)")
        lb[1] += sw
        lb[2] += main
        _r(BUCK_FB_TOP, main, fb)
        _r(BUCK_FB_BOTTOM, fb, gnd)
        _elec("470u", main, gnd, 0.1, "aluminium electrolytic 16 V, low ESR (LM2576 output stability)")
        _bulk(main, gnd)
        # 5V_A: LC and the low-noise LDO
        _filter_l(main, ldo_a_in)
        _bulk(ldo_a_in, gnd)
        la = parts.mic5205(5.0)
        la["IN"] += ldo_a_in
        la["EN"] += ldo_a_in
        la["GND"] += gnd
        la["BYP"] += byp_a
        la["OUT"] += v5a
        _c("470p", byp_a, gnd, "C0G noise bypass")
        _c("10u", v5a, gnd, "X5R output", "Capacitor_SMD:C_1206_3216Metric")
        _c("100n", v5a, gnd)
        # 3V3_RF
        _filter_l(main, rf_in)
        _bulk(rf_in, gnd)
        lr = parts.mic5205(3.3)
        lr["IN"] += rf_in
        lr["EN"] += rf_in
        lr["GND"] += gnd
        lr["BYP"] += byp_rf
        lr["OUT"] += v3rf
        _c("470p", byp_rf, gnd, "C0G noise bypass")
        _c("10u", v3rf, gnd, "X5R output", "Capacitor_SMD:C_1206_3216Metric")
        _c("100n", v3rf, gnd)
        # 3V3_D
        ld = parts.ams1117_33()
        ld["IN"] += main
        ld["GND"] += gnd
        ld["OUT"] += v3d
        c_ta = parts.capacitor("22u", footprint="Capacitor_Tantalum_SMD:CP_EIA-3528-21_Kemet-B", esr=0.5,
                               note="tantalum 10 V: AMS1117 stability needs its ESR")
        c_ta[1] += v3d
        c_ta[2] += gnd
        _c("100n", v3d, gnd)
        # bias-T supply
        _filter_l(main, bias5, BIAS_FILTER_NOTE)
        _bulk(bias5, gnd)
        # monitoring
        _r("100k", vbus, s_vbus)
        _r("100k", s_vbus, gnd)
        _c("100n", s_vbus, gnd, "X7R ADC filter")
        _r("100k", ext_p, s_ext)
        _r("10k", s_ext, gnd)
        _c("100n", s_ext, gnd, "X7R ADC filter")
        _r("100k", main, s_main)
        _r("100k", s_main, gnd)
        _c("100n", s_main, gnd, "X7R ADC filter")
    return Block("power", ckt, {"vbus": vbus, "ext_tip": tip, "ext_p": ext_p, "buck_in": vin, "sw": sw, "main": main,
                                "ldo_a_in": ldo_a_in, "ldo_rf_in": rf_in, "v5a": v5a, "v3rf": v3rf, "v3d": v3d,
                                "bias5": bias5, "sense_vbus": s_vbus, "sense_ext": s_ext, "sense_main": s_main})
```

`hardware/neohw/checks/power.py`:

```python
"""Power check (phase 3 spec §3.2, plan 3.4 amendment): the 9–30 V jack as the only supply.
Rails at 9 and 30 V input and maximum load, a reversed plug, the TVS surge clamp against the
buck's rating, regulator junction temperatures, the buck's conducted ripple per amateur band,
the hot-plug transient (buck input overshoot, fuse I²t, rails up) and the input current.
Writes power.json."""
from __future__ import annotations

import json
import math
from pathlib import Path

import numpy as np

from neohw import availability, charfile, export, parts
from neohw.blocks import bias_tee, mixer, power, preamp
from neohw.checks import BlockResult, CheckResult
from neohw.ngspice import run_decks
from neohw.spice import Ac, Load, Op, Tran, node_name, to_deck
from neohw.touchstone import read_s2p

# rail loads (A), from datasheet currents: 5V_A = CS5361 VA 50 mA + 2 × LM4562 20 mA + preamp 22 mA;
# 3V3_RF = Si5351C 35 mA + logic; 3V3_D continuous maximum = ESP32-S3 240 MHz dual core, radio off
# (≈ 107 mA) + CS5361 VL/VD 17 mA + 24.576 MHz oscillator 10 mA + 74HC595/ULN2803 inputs and LED
# 8 mA ≈ 0.15 A; 5V_MAIN = G6K-2F coils at ≈ 4.6 V (178 Ω): 26 mA each (2 filter relays + preamp
# typ, + attenuator max); BIAS = active antenna (0 typ, 100 mA max)
LOADS_TYP = {"v5a": 0.092, "v3rf": 0.040, "v3d": 0.130, "main": 0.078, "bias5": 0.0}
LOADS_MAX = {"v5a": 0.110, "v3rf": 0.050, "v3d": 0.150, "main": 0.104, "bias5": 0.100}
RAIL_MIN_V = {"v5a": 4.95, "v3rf": 3.25, "v3d": 3.25}     # every rail in regulation
MAIN_V = (5.3, 5.7)            # 5V_MAIN window: coils, LDO headroom, AMS1117 dissipation
BUCK_VIN_MAX_V = 60.0          # LM2576HV recommended maximum input (63 V absolute)
SURGE_A = 12.4                 # SMBJ30A rated peak pulse current (its clamp voltage is specified there)
FUSE_PULSE_MARGIN = 5.0        # inrush I²t at most 1/5 of the fuse's melting I²t (pulse derating)
LEAK_MAX_A = 1e-3
# thermal: θJA (MIC5205 SOT-23-5 220 °C/W; AMS1117 SOT-223 ≈ 90 °C/W; LM2576 TO-263 on ≈ 1 in²
# copper ≈ 50 °C/W, all of the converter's loss charged to it: conservative); 50 °C ambient
AMBIENT_C, TJ_MAX_C = 50.0, 125.0
THETA_JA = {"MIC5205-5.0": 220.0, "MIC5205-3.3": 220.0, "AMS1117-3.3": 90.0, "LM2576HV": 50.0}
# linear regulator: (input node, output node, load key)
REGULATORS = {"MIC5205-5.0": ("ldo_a_in", "v5a", "v5a"), "MIC5205-3.3": ("ldo_rf_in", "v3rf", "v3rf"),
              "AMS1117-3.3": ("main", "v3d", "v3d")}
F_SW = 52e3                    # LM2576 (fixed frequency, 47–58 kHz)
SPUR_MARGIN_DB = 20.0          # conducted spurs at least this far below MDS
MDS_DBM = -140.5
BANDS = {"160m": 1.9e6, "80m": 3.65e6, "60m": 5.35e6, "40m": 7.1e6, "30m": 10.12e6, "20m": 14.1e6, "17m": 18.1e6,
         "15m": 21.2e6, "12m": 24.94e6, "10m": 28.5e6, "6m": 50.1e6}
# name: (jack volts, loads)
SCENARIOS = {
    "jack 9 V, maximum load": (9.0, LOADS_MAX),
    "jack 12 V, typical load": (12.0, LOADS_TYP),
    "jack 12 V, maximum load": (12.0, LOADS_MAX),
    "jack 30 V, maximum load": (30.0, LOADS_MAX),
    "jack 30 V, typical load": (30.0, LOADS_TYP),
    "jack reversed (−12 V)": (-12.0, {}),        # unpowered: the loads draw nothing
}


def psrr_db(f):
    """MIC5205 ripple rejection: 75 dB at 100 Hz (datasheet); assumed −20 dB/decade above 1 kHz to a
    15 dB floor (no curve in the datasheet text)."""
    return np.clip(75 - 20 * np.log10(np.maximum(f, 1e3) / 1e3), 15, 75)


def _ports(blk):
    return [Load(blk.ports["vbus"], "vbus_hz", 1e9), Load(blk.ports["ext_tip"], "tip_hz", 1e9)]


def dc_scenarios(blk) -> dict:
    P = {k: node_name(v) for k, v in blk.ports.items()}
    decks = []
    for name, (ve, loads) in SCENARIOS.items():
        lines = [f"VEXT {P['ext_tip']} 0 dc {ve}", f"VUSB {P['vbus']} 0 dc 5"]
        lines += [f"ILD_{k} {P[k]} 0 dc {i}" for k, i in loads.items()] + [".options gmin=1e-12 itl1=500"]
        decks.append(to_deck(blk.circuit, f"power {name}", _ports(blk), [Op()], extra_lines=lines)[0])
    out = {}
    for name, r in zip(SCENARIOS, run_decks(decks, ["a0.raw"])):
        d = r["a0.raw"][0].data
        out[name] = {k: float(d[f"v({P[k]})"][0].real)
                     for k in ("ext_p", "buck_in", "main", "ldo_a_in", "ldo_rf_in", "v5a", "v3rf", "v3d", "bias5",
                               "sense_ext", "sense_main", "sense_vbus")}
        out[name]["i_ext"] = float(-d["i(vext)"][0].real)
        out[name]["i_usb"] = float(-d["i(vusb)"][0].real)
    return out


def surge_clamp(blk) -> float:
    """Voltage at the buck side of the TVS with the TVS's rated peak current forced into it."""
    P = {k: node_name(v) for k, v in blk.ports.items()}
    deck = to_deck(blk.circuit, "power surge clamp", _ports(blk), [Op()],
                   extra_lines=[f"REXT {P['ext_tip']} 0 1e6", f"ISURGE 0 {P['ext_p']} dc {SURGE_A}",
                                f"VUSB {P['vbus']} 0 dc 0", ".options gmin=1e-12 itl1=500"])[0]
    d = run_decks([deck], ["a0.raw"])[0]["a0.raw"][0].data
    return float(d[f"v({P['ext_p']})"][0].real)


def junction_temps(dc: dict) -> dict:
    """Linear regulators at their own input over every maximum-load case; the buck at its loss
    (output power × (1/η − 1)) charged entirely to the TO-263."""
    worst = [s for name, s in dc.items() if "maximum load" in name]
    tj = {reg: AMBIENT_C + THETA_JA[reg] * max(s[i] - s[o] for s in worst) * LOADS_MAX[load]
          for reg, (i, o, load) in REGULATORS.items()}
    p_out = max(s["main"] * sum(LOADS_MAX.values()) for s in worst)
    tj["LM2576HV"] = AMBIENT_C + THETA_JA["LM2576HV"] * p_out * (1 / parts.LM2576_EFFICIENCY - 1)
    return tj


def plug_in(blk, volts: float) -> dict:
    """The jack plugged into a live adapter: a step to `volts` through 1 µH / 0.2 Ω of cable, rails
    loaded as resistors at maximum load. Peak buck input (LC overshoot), peak and I²t of the
    input current, and the time every rail is up."""
    P = {k: node_name(v) for k, v in blk.ports.items()}
    nominal = {"v5a": 5.0, "v3rf": 3.3, "v3d": 3.3, "main": 5.5, "bias5": 5.5}
    lines = [f"VSRC {P['ext_tip']}_src 0 pwl(0 0 0.1m 0 0.101m {volts})", f"RCAB {P['ext_tip']}_src {P['ext_tip']}_l 0.2",
             f"LCAB {P['ext_tip']}_l {P['ext_tip']} 1u", f"VUSB {P['vbus']} 0 dc 0"]
    lines += [f"RLD_{k} {P[k]} 0 {nominal[k] / i:g}" for k, i in LOADS_MAX.items()]
    lines += [".options method=gear"]        # the input diode blocking the LC's reverse swing is stiff
    ntc = next(p for p in blk.circuit.parts if p.value.startswith("NTC"))
    deck = to_deck(blk.circuit, f"power plug-in {volts:g} V", _ports(blk), [Tran(2e-6, 20e-3, 0.0, 1e-6)],
                   extra_lines=lines,
                   overrides={ntc.ref: lambda n: [f"R{ntc.ref} {n['1']} {n['2']} {parts.NTC_COLD_OHM:g}"]})[0]
    d = run_decks([deck], ["a0.raw"])[0]["a0.raw"][0].data
    t, i_in = d["time"].real, -d["i(vsrc)"].real
    i_steady = float(np.mean(i_in[t > 18e-3]))
    up = []
    for k in ("v5a", "v3rf", "v3d"):
        v = d[f"v({P[k]})"].real
        ok = v >= RAIL_MIN_V[k]
        up.append(float(t[np.argmax(ok)]) if ok.any() else math.inf)
    return {"buck_in_peak_v": float(d[f"v({P['buck_in']})"].real.max()), "peak_a": float(i_in.max()),
            "i2t_a2s": float(np.trapezoid(np.clip(i_in - i_steady, 0, None) ** 2, t)), "rails_up_s": max(up) - 0.1e-3}


def ripple_spurs(blk) -> dict:
    """Conducted buck ripple → RF path, per amateur band (the harmonic of F_SW nearest the band):
    the inductor's triangular current (worst at 30 V in) is driven into the simulated output
    network; 5V_A ripple reaches the RF path through the preamp's and the mixer bias network's
    supply rejection after its LC and LDO, and 5V_MAIN ripple through the bias-T feed. Spurs are
    referred to the antenna (equivalent available power)."""
    P = {k: node_name(v) for k, v in blk.ports.items()}
    b = next(p for p in blk.circuit.parts if p.value == "LM2576HV-ADJ")
    ov = {b.ref: lambda n: [f"R{b.ref}_o {n['OUT']} {n['GND']} 1e9"]}   # the source is the stimulus
    ports = _ports(blk)
    ac = Ac(20e3, 70e6, 200)
    # 5V_MAIN held at its DC operating point through 1 H (≥ 125 kΩ above 20 kHz), so the LDO
    # models work at their real headroom instead of in dropout
    extra = [f"VEXT {P['ext_tip']} 0 dc 12", f"VUSB {P['vbus']} 0 dc 0",
             f"VMDC {P['main']}_dc 0 dc 5.5", f"LMDC {P['main']}_dc {P['main']} 1"]
    decks = [to_deck(blk.circuit, "ripple main", ports, [ac], extra_lines=extra + [f"IRIP 0 {P['sw']} dc 0 ac 1"],
                     overrides=ov)[0]]
    pre, bt, mx = preamp.build("amp"), bias_tee.build(), mixer.build()
    sw = next(p for p in mx.circuit.parts if p.name == "FST3253")
    decks += [to_deck(pre.circuit, "preamp supply rejection", [Load(pre.ports["rf_in"], "term", 50.0), Load(pre.ports["rf_out"])],
                      [ac], extra_lines=[f"VSUP {node_name(pre.ports['vcc'])} 0 dc 5 ac 1"])[0],
              to_deck(bt.circuit, "bias feed", [Load(bt.ports["rf_in"], "ant", 50.0), Load(bt.ports["rf_out"])],
                      [ac], extra_lines=[f"VFEED {node_name(bt.ports['bias_feed'])} 0 dc 5 ac 1"])[0],
              to_deck(mx.circuit, "mixer supply rejection",
                      [Load(mx.ports["rf_in"], "rf", 50.0), Load(mx.ports["s0"], "s0", 1e6), Load(mx.ports["s1"], "s1", 1e6)],
                      [ac], extra_lines=[f"VSUP {node_name(mx.ports['vcc'])} 0 dc 5 ac 1"],
                      overrides={sw.ref: lambda n: [f"R{sw.ref}_a {n['1A']} 0 1e9"]
                                 + [f"R{sw.ref}_b{k} {n[f'1B{k}']} 0 1e9" for k in range(4)]})[0]]
    res = run_decks(decks, ["a0.raw"])

    def tr(r, node):
        d = r["a0.raw"][0].data
        return d["frequency"].real, np.abs(d[f"v({node})"])
    z_a = tr(res[0], P["ldo_a_in"])
    z_bias = tr(res[0], P["bias5"])
    t_pre = tr(res[1], node_name(pre.ports["rf_out"]))
    t_bias = tr(res[2], node_name(bt.ports["rf_out"]))
    t_mix = tr(res[3], node_name(mx.ports["a"]))
    v_in, v_out = 30.0, 5.5
    duty = v_out / v_in
    di = (v_in - v_out) * duty / (power.BUCK_L * F_SW)
    char = Path(__file__).resolve().parents[2] / "characterized"
    pj = json.loads((char / "preamp.json").read_text())
    bank = json.loads((char / "bpf_bank.json").read_text())

    def front(f):
        idx = next((x["index"] for x in bank["filters"] if x["lo_hz"] <= f < x["hi_hz"]), bank["filters"][-1]["index"])
        ff, s = read_s2p(char / f"bpf_{idx}.s2p")
        return float(np.interp(f, ff, np.abs(s["21"]))) * 10 ** (float(np.interp(f, pj["freq_hz"], pj["gain_db"])) / 20)

    def at(t, f):
        return float(np.interp(f, *t))

    def dbm(v):      # node voltage amplitude → equivalent antenna source available power (50 Ω both ways)
        return 10 * math.log10(max(v, 1e-30) ** 2 / 100 / 1e-3)

    out = {}
    for band, f in BANDS.items():
        n = max(1, round(f / F_SW))
        fn = n * F_SW
        c = di * abs(math.sin(math.pi * n * duty)) / (math.pi ** 2 * n ** 2 * duty * (1 - duty))
        v5a = c * at(z_a, fn) * 10 ** (-float(psrr_db(fn)) / 20)
        paths = {"preamp": dbm(v5a * at(t_pre, fn) / front(fn)), "mixer": dbm(v5a * at(t_mix, fn) / front(fn)),
                 "bias_tee": dbm(c * at(z_bias, fn) * at(t_bias, fn))}
        out[band] = {"harmonic": n, "f_hz": fn, **paths,
                     "total_dbm": 10 * math.log10(sum(10 ** (p / 10) for p in paths.values()))}
    return out


def run(out_dir: Path, char_dir: Path) -> BlockResult:
    out_dir, char_dir = Path(out_dir), Path(char_dir)
    blk = power.build()
    dc = dc_scenarios(blk)
    tj = junction_temps(dc)
    spurs = ripple_spurs(blk)
    clamp = surge_clamp(blk)
    hot = {v: plug_in(blk, v) for v in (9.0, 12.0, 30.0)}
    powered = [s for name, s in dc.items() if "reversed" not in name]
    rail_min = {k: min(s[k] for s in powered) for k in RAIL_MIN_V}
    main_lo, main_hi = min(s["main"] for s in powered), max(s["main"] for s in powered)
    rev = dc["jack reversed (−12 V)"]
    worst_spur = max(s["total_dbm"] for s in spurs.values())
    fuse_limit = parts.fuse_i2t_min(power.FUSE_A) / FUSE_PULSE_MARGIN
    usb_draw = max(abs(s["i_usb"]) - 5.0 / 200e3 for s in powered)        # beyond the sense divider
    checks = [CheckResult(f"{k}_min_v", v, f"≥ {RAIL_MIN_V[k]:g}", v >= RAIL_MIN_V[k], "V") for k, v in rail_min.items()]
    checks += [
        CheckResult("main_min_v", main_lo, f"≥ {MAIN_V[0]:g}", main_lo >= MAIN_V[0], "V"),
        CheckResult("main_max_v", main_hi, f"≤ {MAIN_V[1]:g}", main_hi <= MAIN_V[1], "V"),
        CheckResult("input_current_12v_typ_ma", dc["jack 12 V, typical load"]["i_ext"] * 1e3, "report", True, "mA"),
        CheckResult("input_current_9v_max_ma", dc["jack 9 V, maximum load"]["i_ext"] * 1e3, "report (adapter rating)", True, "mA"),
        CheckResult("usb_vbus_draw_ma", usb_draw * 1e3, f"≤ {LEAK_MAX_A * 1e3:g} (USB is data only)", usb_draw <= LEAK_MAX_A, "mA"),
        CheckResult("reversed_jack_current_ma", abs(rev["i_ext"]) * 1e3, f"≤ {LEAK_MAX_A * 1e3:g}", abs(rev["i_ext"]) <= LEAK_MAX_A, "mA"),
        CheckResult("reversed_jack_main_v", rev["main"], "≥ −0.3 (nothing reverse-biased)", rev["main"] >= -0.3, "V"),
        CheckResult("surge_clamp_v", clamp, f"≤ {BUCK_VIN_MAX_V:g} (LM2576HV input)", clamp <= BUCK_VIN_MAX_V, "V"),
        CheckResult("sense_ext_at_30v_v", dc["jack 30 V, maximum load"]["sense_ext"], "≤ 3.1 (ESP32-S3 ADC)",
                    dc["jack 30 V, maximum load"]["sense_ext"] <= 3.1, "V"),
    ]
    for v, h in hot.items():
        tag = f"{v:g}v"
        checks += [
            CheckResult(f"plug_in_{tag}_buck_in_peak_v", h["buck_in_peak_v"], f"≤ {BUCK_VIN_MAX_V:g}", h["buck_in_peak_v"] <= BUCK_VIN_MAX_V, "V"),
            CheckResult(f"plug_in_{tag}_peak_a", h["peak_a"], "report", True, "A"),
            CheckResult(f"plug_in_{tag}_i2t_a2s", h["i2t_a2s"], f"≤ {fuse_limit:g} (time-lag fuse / {FUSE_PULSE_MARGIN:g})",
                        h["i2t_a2s"] <= fuse_limit, "A²s"),
            CheckResult(f"plug_in_{tag}_rails_up_ms", h["rails_up_s"] * 1e3, "≤ 20", h["rails_up_s"] <= 20e-3, "ms"),
        ]
    checks += [CheckResult(f"tj_{k}", t, f"≤ {TJ_MAX_C:g}", t <= TJ_MAX_C, "°C") for k, t in tj.items()]
    checks.append(CheckResult("conducted_spur_worst_dbm", worst_spur, f"≤ MDS − {SPUR_MARGIN_DB:g} = {MDS_DBM - SPUR_MARGIN_DB:g}",
                              worst_spur <= MDS_DBM - SPUR_MARGIN_DB, "dBm"))
    result = BlockResult("power", checks)
    data = availability.load()
    result.artefacts = {
        "kicad_netlist": export.kicad_netlist(blk.circuit, out_dir / "power.net"),
        "bom": export.write_bom(blk.circuit, out_dir / "power_bom.csv", data),
        "svg": export.svg_diagram(blk.circuit, out_dir / "power.svg"),
        "characterisation": charfile.write(char_dir / "power.json", "power", "pass" if result.passed else "fail", {
            "input_v": [9.0, 30.0], "loads_typ_a": LOADS_TYP, "loads_max_a": LOADS_MAX, "scenarios": dc,
            "sense_dividers": power.SENSE_DIVIDERS, "junction_c": tj, "surge_clamp_v": clamp,
            "plug_in": {f"{v:g}": h for v, h in hot.items()}, "spurs_dbm": spurs,
            "psrr_assumption": "MIC5205 75 dB at 100 Hz (datasheet), assumed -20 dB/dec above 1 kHz to 15 dB",
            "spur_note": "conducted only (inductor ripple, triangle model); switch-node edges and radiated/layout coupling are checked in the layout phase",
            "checks": [c.__dict__ for c in checks]}),
    }
    return result
```

Then regenerate the power characterisation (Tasks 2 and 3 read it):

```bash
cd hardware && python3 -c "from pathlib import Path; from neohw.checks import power as c; c.run(Path('../build/hw'), Path('characterized'))"
```

- [ ] **Step 4: Run them to verify they pass**

Run: `cd hardware && python3 -m pytest -q tests/test_power.py tests/test_pin_names.py`
Expected: `13 passed` (with ngspice). The check reports: 5V_MAIN 5.464–5.480 V; v5a ≥ 4.994, 3V3_RF ≥ 3.297, 3V3_D ≥ 3.292 V; input 214 mA at 12 V typical, 450 mA at 9 V maximum; surge clamp 49.0 V; hot-plug 9/12/30 V: rails up 4.4/1.6/1.0 ms, I²t 0.0018/0.0054/0.0144 A²s (limit 0.0225), buck input ≤ 29.3 V; Tj MIC5205-5.0 60.6, MIC5205-3.3 73.7, AMS1117 79.3, LM2576HV 91.9 °C; worst conducted spur −224.6 dBm.

- [ ] **Step 5: Commit**

```bash
git add hardware/neohw hardware/tests hardware/characterized/power.json
git commit -m "feat(hw): 9-30 V jack as the only supply: LM2576HV buck to 5.5 V, NTC inrush limit, SS36 and SMBJ30A protection"
```

### Task 2: Controller and board follow the new rails

**Files:**
- Modify: `hardware/neohw/checks/controller.py` (relay-coil margin over the new scenarios, as a gate), `hardware/neohw/checks/board.py` (AMS1117 on 3V3_D), `hardware/neohw/blocks/controller.py` (CC comment)
- Test: `hardware/tests/test_controller.py`

**Interfaces:**
- Consumes: Task 1 `power.json` scenarios with `maximum load` in their names.
- Produces: check `relay_coil_margin_v` (gate ≥ 0); `_supply_corners()` = every maximum-load scenario's 5V_MAIN.

- [ ] **Step 1: Write the failing test**

```diff
diff --git a/hardware/tests/test_controller.py b/hardware/tests/test_controller.py
index a05d804..a536c91 100644
--- a/hardware/tests/test_controller.py
+++ b/hardware/tests/test_controller.py
@@ -27,7 +27,11 @@ def test_bias_sense_is_monotonic_stable_and_within_the_adc_range():
 
 
 def test_relay_coil_voltage_is_reported_for_each_supply_corner():
-    r = ctl_check.relay_coil_margin({"usb 4.75 V": 4.62, "ext 4.75 V": 4.30})
-    assert r["ext 4.75 V"]["coil_v"] == pytest.approx(4.30 - ctl_check.ULN2803_VCE_SAT_V)
-    assert r["ext 4.75 V"]["need_v"] == pytest.approx(ctl_check.RELAY_MUST_OPERATE_V * ctl_check.coil_heat_factor(50.0))
-    assert r["usb 4.75 V"]["margin_v"] > r["ext 4.75 V"]["margin_v"]
+    r = ctl_check.relay_coil_margin({"low": 4.30, "high": 5.46})
+    assert r["low"]["coil_v"] == pytest.approx(4.30 - ctl_check.ULN2803_VCE_SAT_V)
+    assert r["low"]["need_v"] == pytest.approx(ctl_check.RELAY_MUST_OPERATE_V * ctl_check.coil_heat_factor(50.0))
+    assert r["low"]["margin_v"] < 0 < r["high"]["margin_v"]
+
+
+def test_relays_pull_in_at_every_supply_corner_with_the_5v5_rail():
+    assert min(m["margin_v"] for m in ctl_check.relay_coil_margin(ctl_check._supply_corners()).values()) > 0
```

- [ ] **Step 2: Run it to verify it fails**

Run: `cd hardware && python3 -m pytest -q tests/test_controller.py`
Expected: FAIL — `KeyError: 'usb 4.75 V, maximum load'` (the old scenario names are gone from power.json).

- [ ] **Step 3: Implement**

```diff
diff --git a/hardware/neohw/blocks/controller.py b/hardware/neohw/blocks/controller.py
index 1fa5187..37236b7 100644
--- a/hardware/neohw/blocks/controller.py
+++ b/hardware/neohw/blocks/controller.py
@@ -71,7 +71,7 @@ def build(ckt: Circuit | None = None) -> Block:
         j["B5"] += cc2
         _r("5.1k", cc1, gnd)
         _r("5.1k", cc2, gnd)
-        _r("10k", cc1, g["cc1"])           # CC voltage (0.41 / 0.92 / 1.68 V = default / 1.5 A / 3 A) to an ADC
+        _r("10k", cc1, g["cc1"])           # CC voltage to an ADC (telemetry only: the jack powers the board)
         _r("10k", cc2, g["cc2"])
         esd = parts.usblc6()
         esd["IO1A"] += dp
diff --git a/hardware/neohw/checks/board.py b/hardware/neohw/checks/board.py
index e10c51e..6875285 100644
--- a/hardware/neohw/checks/board.py
+++ b/hardware/neohw/checks/board.py
@@ -19,7 +19,7 @@ SUPPLY_RAILS = {
     ("LM4562", "V+"): "+5V_A", ("CS5361", "VA"): "+5V_A", ("CS5361", "VD"): "+3V3_D", ("CS5361", "VL"): "+3V3_D",
     ("ESP32-S3-WROOM-1", "3V3"): "+3V3_D", ("74HC595", "VCC"): "+3V3_D", ("ULN2803", "COM"): "+5V_MAIN",
     ("G6K-2F", "COIL+"): "+5V_MAIN", ("MIC5205-5.0", "OUT"): "+5V_A", ("MIC5205-3.3", "OUT"): "+3V3_RF",
-    ("TLV74033", "OUT"): "+3V3_D", ("USBLC6-2SC6", "VBUS"): "VBUS",
+    ("AMS1117-3.3", "OUT"): "+3V3_D", ("USBLC6-2SC6", "VBUS"): "VBUS",
 }
 GROUND_PINS = ("GND", "GND2", "EPAD", "PAD", "V-", "REFGND")
 
diff --git a/hardware/neohw/checks/controller.py b/hardware/neohw/checks/controller.py
index f482f48..3029132 100644
--- a/hardware/neohw/checks/controller.py
+++ b/hardware/neohw/checks/controller.py
@@ -62,8 +62,7 @@ def relay_coil_margin(main_v_by_corner: dict[str, float], t_c: float = RELAY_HOT
 def _supply_corners() -> dict[str, float]:
     import json
     sc = json.loads((Path(__file__).resolve().parents[2] / "characterized" / "power.json").read_text())["scenarios"]
-    return {"usb 4.75 V, maximum load": sc["usb 4.75 V, maximum load"]["main"],
-            "ext 4.75 V, maximum load": sc["ext 4.75 V only, maximum load"]["main"]}
+    return {name: s["main"] for name, s in sc.items() if "maximum load" in name}
 
 
 def gpio_findings() -> list[str]:
@@ -95,6 +94,7 @@ def run(out_dir: Path, char_dir: Path) -> BlockResult:
     v_max = max(v["sense_v"] for v in bs["short"].values())
     i_short = bs["short"][27.0]["i_a"]
     bad = gpio_findings()
+    coil = relay_coil_margin(_supply_corners())
     addrs = list(I2C_ADDRESSES.values())
     coil_total = RELAY_COIL_A * RELAYS_ON_MAX
     checks = [
@@ -105,9 +105,9 @@ def run(out_dir: Path, char_dir: Path) -> BlockResult:
         CheckResult("bias_sense_max_v", v_max, f"≤ {ADC_MAX_V:g}", v_max <= ADC_MAX_V, "V"),
         CheckResult("gpio_rule_violations", len(bad), "= 0", not bad, ""),
         CheckResult("i2c_addresses_unique", float(len(set(addrs)) == len(addrs)), "= 1", len(set(addrs)) == len(addrs), ""),
-        *[CheckResult(f"relay_coil_margin_{'usb' if k.startswith('usb') else 'ext'}_v", m["margin_v"],
-                      "report: decision pending (spec §3.3 relay supply)", True, "V")
-          for k, m in relay_coil_margin(_supply_corners()).items()],
+        CheckResult("relay_coil_margin_v", min(m["margin_v"] for m in coil.values()),
+                    f"≥ 0 (G6K-2F must-operate at {RELAY_HOT_C:g} °C, every supply corner)",
+                    min(m["margin_v"] for m in coil.values()) >= 0, "V"),
         CheckResult("relay_coil_current_a", coil_total, f"per channel ≤ {ULN2803_CHANNEL_A:g}", RELAY_COIL_A <= ULN2803_CHANNEL_A, "A"),
     ]
     checks = [CheckResult(c.name, float(c.value), c.limit, bool(c.passed), c.unit) for c in checks]   # plain JSON types
```

- [ ] **Step 4: Run to verify it passes**

Run: `cd hardware && python3 -m pytest -q tests/test_controller.py tests/test_board.py`
Expected: all pass; the controller check reports `relay_coil_margin_v` = +0.416 V; the board check reports 461 parts, 0 errors and no unconfirmed semiconductors.

- [ ] **Step 5: Commit**

```bash
git add hardware/neohw hardware/tests
git commit -m "feat(hw): controller and board follow the 5.5 V main rail; relay pull-in is now a gate"
```

### Task 3: Firmware follows: divider ratios from power.json, bias-T gated by 5V_MAIN

**Files:**
- Modify: `firmware/tools/gen_tables.py`, `firmware/CMakeLists.txt`, `firmware/core/supervisor.{c,h}`, `firmware/core/cat.h`
- Test: `firmware/tests/test_supervisor.c`, `firmware/tests/test_cat.c`

**Interfaces:**
- Consumes: Task 1 `power.json` `sense_dividers`.
- Produces: `TABLES_SENSE_DIV_{EXT,MAIN,VBUS}_X1000`; `SUP_FLAG_HOST`, `SUP_FLAG_INPUT_LOW`, `SUP_FLAG_MAIN_LOW`; `supervisor_bias_enable` refuses while 5V_MAIN < 5.3 V; `sup_status_t` without `cc`.

- [ ] **Step 1: Write the failing tests**

```c
#include "check.h"
#include "supervisor.h"
#include "tables_gen.h"
#include "test_hal.h"

static uint32_t sense_mv_for(uint32_t ua)
{
    /* inverse of the table, by search: the smallest mV that reads at least ua */
    uint32_t mv = 0;
    while (supervisor_bias_ua(mv) < ua && mv < 3300) mv++;
    return mv;
}

/* the board's monitors as the dividers present them (ratios from power.json) */
static void supplies(uint32_t jack_mv, uint32_t main_mv, uint32_t vbus_mv)
{
    test_vhw.adc_mv[HAL_ADC_SENSE_EXT] = (uint32_t)((uint64_t)jack_mv * 1000 / TABLES_SENSE_DIV_EXT_X1000);
    test_vhw.adc_mv[HAL_ADC_SENSE_MAIN] = (uint32_t)((uint64_t)main_mv * 1000 / TABLES_SENSE_DIV_MAIN_X1000);
    test_vhw.adc_mv[HAL_ADC_SENSE_VBUS] = (uint32_t)((uint64_t)vbus_mv * 1000 / TABLES_SENSE_DIV_VBUS_X1000);
}

static void sense_table_reads_back_its_own_points(void)
{
    for (int k = 0; k < TABLES_BIAS_SENSE_POINTS; k++)
        CHECK(supervisor_bias_ua(TABLES_BIAS_SENSE_MV[k]) == TABLES_BIAS_SENSE_UA[k]);
    CHECK(supervisor_bias_ua(0) == 0 && supervisor_bias_ua(500) == 0);
    uint32_t prev = 0;
    for (uint32_t mv = 600; mv <= 2000; mv += 10) {
        uint32_t ua = supervisor_bias_ua(mv);
        CHECK(ua >= prev);
        prev = ua;
    }
}

static void monitors_read_through_the_dividers(void)
{
    supervisor_init();
    supplies(12000, 5480, 5000);
    const sup_status_t *s = supervisor_poll();
    CHECK(s->ext_mv >= 11980 && s->ext_mv <= 12020);              /* 100 k / 10 k: 30 V fits the ADC */
    CHECK(s->main_mv >= 5470 && s->main_mv <= 5490);
    CHECK((s->flags & SUP_FLAG_EXT) && (s->flags & SUP_FLAG_HOST) && !(s->flags & SUP_FLAG_INPUT_LOW));
    supplies(7500, 5480, 0);                                       /* input sagging, no host */
    s = supervisor_poll();
    CHECK((s->flags & SUP_FLAG_INPUT_LOW) && !(s->flags & SUP_FLAG_HOST));
}

static void bias_needs_5v_main_in_regulation(void)
{
    supervisor_init();
    supplies(12000, 4800, 5000);                                   /* buck not up yet */
    CHECK(!supervisor_bias_enable(true));
    CHECK(test_vhw.gpio[HAL_GPIO_BIAS_EN] == false);
    supplies(12000, 5480, 0);                                      /* no USB host needed: the jack powers it */
    CHECK(supervisor_bias_enable(true));
    CHECK(test_vhw.gpio[HAL_GPIO_BIAS_EN] == true);
}

static void bias_drops_when_main_falls(void)
{
    supervisor_init();
    supplies(12000, 5480, 5000);
    CHECK(supervisor_bias_enable(true));
    supplies(6000, 4600, 5000);                                    /* the jack pulled: main collapsing */
    const sup_status_t *s = supervisor_poll();
    CHECK(!(s->flags & SUP_FLAG_BIAS_ON) && test_vhw.gpio[HAL_GPIO_BIAS_EN] == false);
}

static void over_current_cuts_the_feed(void)
{
    supervisor_init();
    supplies(12000, 5480, 5000);
    CHECK(supervisor_bias_enable(true));
    test_vhw.adc_mv[HAL_ADC_BIAS_ISENSE] = sense_mv_for(140000);
    CHECK(supervisor_poll()->flags & SUP_FLAG_BIAS_ON);
    test_vhw.adc_mv[HAL_ADC_BIAS_ISENSE] = sense_mv_for(160000);
    const sup_status_t *s = supervisor_poll();
    CHECK(!(s->flags & SUP_FLAG_BIAS_ON) && (s->flags & SUP_FLAG_BIAS_TRIPPED));
    CHECK(test_vhw.gpio[HAL_GPIO_BIAS_EN] == false);
    test_vhw.adc_mv[HAL_ADC_BIAS_ISENSE] = sense_mv_for(50000);   /* fault cleared */
    CHECK(supervisor_bias_enable(true));                 /* re-enabling clears the trip */
    s = supervisor_poll();
    CHECK((s->flags & SUP_FLAG_BIAS_ON) && !(s->flags & SUP_FLAG_BIAS_TRIPPED));
}

static void sense_zero_is_measured_with_the_feed_off(void)
{
    /* a board whose mirror reads 0.70 V at zero current (the table's zero is 0.628 V) */
    supervisor_init();
    supplies(12000, 5480, 5000);
    test_vhw.adc_mv[HAL_ADC_BIAS_ISENSE] = 700;       /* feed off: the zero reading */
    CHECK(supervisor_bias_enable(true));
    CHECK(supervisor_poll()->bias_ua < 1000);           /* still no current: reads ≈ 0, not ≈ 14 mA */
    test_vhw.adc_mv[HAL_ADC_BIAS_ISENSE] = sense_mv_for(100000) + (700 - TABLES_BIAS_SENSE_MV[0]);
    uint32_t ua = supervisor_poll()->bias_ua;
    CHECK(ua > 98000 && ua < 102000);
}

static void temperature_is_read_and_a_missing_sensor_flagged(void)
{
    supervisor_init();
    supplies(12000, 5480, 5000);
    vhw_set_temp_mc(&test_vhw, 41500);
    CHECK(supervisor_poll()->temp_mc == 41500);
    test_vhw.lm75_present = false;
    CHECK(supervisor_poll()->flags & SUP_FLAG_TEMP_FAIL);
}

int main(void)
{
    RUN(sense_table_reads_back_its_own_points);
    RUN(monitors_read_through_the_dividers);
    RUN(bias_needs_5v_main_in_regulation);
    RUN(bias_drops_when_main_falls);
    RUN(over_current_cuts_the_feed);
    RUN(sense_zero_is_measured_with_the_feed_off);
    RUN(temperature_is_read_and_a_missing_sensor_flagged);
    return CHECK_DONE();
}
```

```diff
diff --git a/firmware/tests/test_cat.c b/firmware/tests/test_cat.c
index 502e7c2..d126b54 100644
--- a/firmware/tests/test_cat.c
+++ b/firmware/tests/test_cat.c
@@ -30,8 +30,9 @@ static void start(void)
     radio_init();
     supervisor_init();
     cat_init(collect);
-    test_vhw.adc_mv[HAL_ADC_SENSE_VBUS] = 2500;
-    test_vhw.adc_mv[HAL_ADC_CC1] = 410;
+    test_vhw.adc_mv[HAL_ADC_SENSE_VBUS] = 2500;               /* host present */
+    test_vhw.adc_mv[HAL_ADC_SENSE_EXT] = 1091;               /* 12 V jack through 100 k / 10 k */
+    test_vhw.adc_mv[HAL_ADC_SENSE_MAIN] = 2000;              /* 4.0 V: the buck not up yet */
 }
 
 static void fa_tunes_and_reads_back(void)
@@ -93,11 +94,11 @@ static void preamp_and_attenuator_switch_relays(void)
 static void bias_t_needs_power_and_reports_telemetry(void)
 {
     start();
-    CHECK(strcmp(ask("ZB1;"), "?;") == 0);                 /* USB default current only */
-    test_vhw.adc_mv[HAL_ADC_CC1] = 920;
+    CHECK(strcmp(ask("ZB1;"), "?;") == 0);                 /* 5V_MAIN not in regulation */
+    test_vhw.adc_mv[HAL_ADC_SENSE_MAIN] = 2740;              /* 5.48 V */
     CHECK(strcmp(ask("ZB1;ZB;"), "ZB1;") == 0);
     const char *t = ask("ZT;");
-    CHECK(strncmp(t, "ZT5000,0,", 9) == 0 && t[strlen(t) - 1] == ';');
+    CHECK(strncmp(t, "ZT5000,12001,5480,", 18) == 0 && t[strlen(t) - 1] == ';');
 }
 
 static void calibration_tone_on_and_off(void)
```

- [ ] **Step 2: Run them to verify they fail**

Run: `make fw-test`
Expected: build FAIL — `'TABLES_SENSE_DIV_EXT_X1000' undeclared`, `'SUP_FLAG_HOST' undeclared`.

- [ ] **Step 3: Implement**

`firmware/core/supervisor.h`:

```c
/* Power and protection supervision (phase 3 spec §3.2, plan 3.4: the 9–30 V jack is the only
 * supply, USB is data only): input and 5V_MAIN monitoring, host presence on VBUS, the bias-T
 * over-current cut-off (hardware limit 0.39 A, firmware 150 mA) and board temperature. */
#ifndef ELF_SUPERVISOR_H
#define ELF_SUPERVISOR_H

#include <stdbool.h>
#include <stdint.h>

#define SUP_EXT_PRESENT_MV 3000u       /* any input on the jack side of the board */
#define SUP_INPUT_LOW_MV 8000u         /* after the fuse, NTC and SS36: a 9 V adapter sagging below ≈ 8.5 V */
#define SUP_MAIN_OK_MV 5300u           /* 5V_MAIN in regulation (5.5 V nominal) */
#define SUP_HOST_MV 4000u              /* VBUS from a USB host */

enum {
    SUP_FLAG_EXT = 1u << 0,            /* the jack supplies the board */
    SUP_FLAG_INPUT_LOW = 1u << 1,      /* input below the 9 V specification */
    SUP_FLAG_BIAS_ON = 1u << 2,
    SUP_FLAG_BIAS_TRIPPED = 1u << 3,   /* cut off for over-current; cleared by the next enable */
    SUP_FLAG_TEMP_FAIL = 1u << 4,      /* LM75A not answering */
    SUP_FLAG_HOST = 1u << 5,           /* a USB host is connected (VBUS present) */
    SUP_FLAG_MAIN_LOW = 1u << 6,       /* 5V_MAIN out of regulation */
};

typedef struct {
    uint32_t vbus_mv, ext_mv, main_mv;
    uint32_t bias_ua;
    int32_t temp_mc;                   /* milli-°C */
    uint32_t flags;
} sup_status_t;

void supervisor_init(void);
/* Reads every monitor and applies the bias-T cut-offs (over-current, 5V_MAIN out of regulation);
 * call every few ms. */
const sup_status_t *supervisor_poll(void);
/* Bias-T on only with 5V_MAIN in regulation; returns false when refused. */
bool supervisor_bias_enable(bool on);
/* Current from the sense voltage (piecewise-linear table of the simulated mirror). Readings are
 * first referred to the table: this board's zero, measured with the feed off before each enable,
 * is subtracted and the table's zero added. */
uint32_t supervisor_bias_ua(uint32_t sense_mv);

#endif
```

```diff
diff --git a/firmware/CMakeLists.txt b/firmware/CMakeLists.txt
index f38b983..b62627d 100644
--- a/firmware/CMakeLists.txt
+++ b/firmware/CMakeLists.txt
@@ -14,7 +14,7 @@ find_package(Python3 REQUIRED COMPONENTS Interpreter)
 add_custom_command(
     OUTPUT ${GEN_DIR}/tables_gen.h ${GEN_DIR}/tables_gen.c
     COMMAND Python3::Interpreter ${CMAKE_CURRENT_SOURCE_DIR}/tools/gen_tables.py ${CHAR_DIR} ${GEN_DIR}
-    DEPENDS tools/gen_tables.py ${CHAR_DIR}/bpf_bank.json ${CHAR_DIR}/controller.json
+    DEPENDS tools/gen_tables.py ${CHAR_DIR}/bpf_bank.json ${CHAR_DIR}/controller.json ${CHAR_DIR}/power.json
     COMMENT "Generating firmware tables from the characterisation files")
 
 add_library(elf_core STATIC
diff --git a/firmware/core/cat.h b/firmware/core/cat.h
index d884dd4..e9ef2af 100644
--- a/firmware/core/cat.h
+++ b/firmware/core/cat.h
@@ -8,7 +8,7 @@
  *   PA        preamp: PA0 / PA1 (answer "PAx0;")
  *   RA        attenuator: RA00 / RA01 (answer "RAxx00;")
  *   AI, PS, RX  accepted (AI0, PS1, receive)
- *   ZB        Elf: bias-T, ZB0 / ZB1 (refused with "?;" without ≥ 1.5 A or the jack)
+ *   ZB        Elf: bias-T, ZB0 / ZB1 (refused with "?;" while 5V_MAIN is out of regulation)
  *   ZT        Elf: telemetry "ZT<vbus mV>,<ext mV>,<main mV>,<bias µA>,<temp m°C>,<flags>;"
  *   ZC        Elf: calibration tone, ZC0 / ZC1<offset Hz>, answer "ZC<0|1><offset>;" */
 #ifndef ELF_CAT_H
diff --git a/firmware/core/supervisor.c b/firmware/core/supervisor.c
index 40dd292..07c69c6 100644
--- a/firmware/core/supervisor.c
+++ b/firmware/core/supervisor.c
@@ -27,14 +27,6 @@ uint32_t supervisor_bias_ua(uint32_t mv)
     return v < 0 ? 0 : (uint32_t)v;
 }
 
-sup_cc_t supervisor_cc_from_mv(uint32_t mv)
-{
-    if (mv < 200) return SUP_CC_NONE;
-    if (mv < SUP_CC_1A5_MIN_MV) return SUP_CC_DEFAULT;
-    if (mv < SUP_CC_3A_MIN_MV) return SUP_CC_1A5;
-    return SUP_CC_3A;
-}
-
 void supervisor_init(void)
 {
     st = (sup_status_t){0};
@@ -54,21 +46,30 @@ static void read_temp(void)
     st.temp_mc = raw * 125;
 }
 
+static uint32_t divided(hal_adc_t ch, uint32_t ratio_x1000)
+{
+    return (uint32_t)((uint64_t)hal_adc_mv(ch) * ratio_x1000 / 1000u);
+}
+
+static void bias_off(uint32_t why)
+{
+    hal_gpio_set(HAL_GPIO_BIAS_EN, false);
+    st.flags = (st.flags & ~SUP_FLAG_BIAS_ON) | why;
+}
+
 const sup_status_t *supervisor_poll(void)
 {
-    st.vbus_mv = hal_adc_mv(HAL_ADC_SENSE_VBUS) * SUP_DIVIDER;
-    st.ext_mv = hal_adc_mv(HAL_ADC_SENSE_EXT) * SUP_DIVIDER;
-    st.main_mv = hal_adc_mv(HAL_ADC_SENSE_MAIN) * SUP_DIVIDER;
-    uint32_t cc1 = hal_adc_mv(HAL_ADC_CC1), cc2 = hal_adc_mv(HAL_ADC_CC2);
-    st.cc = supervisor_cc_from_mv(cc1 > cc2 ? cc1 : cc2);
-    st.flags = (st.flags & (SUP_FLAG_BIAS_ON | SUP_FLAG_BIAS_TRIPPED | SUP_FLAG_TEMP_FAIL));
+    st.vbus_mv = divided(HAL_ADC_SENSE_VBUS, TABLES_SENSE_DIV_VBUS_X1000);
+    st.ext_mv = divided(HAL_ADC_SENSE_EXT, TABLES_SENSE_DIV_EXT_X1000);
+    st.main_mv = divided(HAL_ADC_SENSE_MAIN, TABLES_SENSE_DIV_MAIN_X1000);
+    st.flags &= SUP_FLAG_BIAS_ON | SUP_FLAG_BIAS_TRIPPED | SUP_FLAG_TEMP_FAIL;
     if (st.ext_mv >= SUP_EXT_PRESENT_MV) st.flags |= SUP_FLAG_EXT;
-    else if (st.vbus_mv < SUP_VBUS_LOW_MV) st.flags |= SUP_FLAG_VBUS_LOW;
+    if (st.ext_mv < SUP_INPUT_LOW_MV) st.flags |= SUP_FLAG_INPUT_LOW;
+    if (st.vbus_mv >= SUP_HOST_MV) st.flags |= SUP_FLAG_HOST;
+    if (st.main_mv < SUP_MAIN_OK_MV) st.flags |= SUP_FLAG_MAIN_LOW;
     st.bias_ua = (st.flags & SUP_FLAG_BIAS_ON) ? supervisor_bias_ua(referred_mv(hal_adc_mv(HAL_ADC_BIAS_ISENSE))) : 0;
-    if ((st.flags & SUP_FLAG_BIAS_ON) && st.bias_ua > TABLES_BIAS_CUTOFF_UA) {
-        hal_gpio_set(HAL_GPIO_BIAS_EN, false);
-        st.flags = (st.flags & ~SUP_FLAG_BIAS_ON) | SUP_FLAG_BIAS_TRIPPED;
-    }
+    if ((st.flags & SUP_FLAG_BIAS_ON) && st.bias_ua > TABLES_BIAS_CUTOFF_UA) bias_off(SUP_FLAG_BIAS_TRIPPED);
+    if ((st.flags & SUP_FLAG_BIAS_ON) && (st.flags & SUP_FLAG_MAIN_LOW)) bias_off(0);
     read_temp();
     return &st;
 }
@@ -81,7 +82,7 @@ bool supervisor_bias_enable(bool on)
         return true;
     }
     supervisor_poll();
-    if (!(st.flags & SUP_FLAG_EXT) && st.cc != SUP_CC_1A5 && st.cc != SUP_CC_3A) return false;
+    if (st.flags & SUP_FLAG_MAIN_LOW) return false;
     zero_mv = hal_adc_mv(HAL_ADC_BIAS_ISENSE);   /* the feed is still off: this board's zero */
     st.flags &= ~SUP_FLAG_BIAS_TRIPPED;
     st.flags |= SUP_FLAG_BIAS_ON;
diff --git a/firmware/tools/gen_tables.py b/firmware/tools/gen_tables.py
index b7b34eb..e51f36f 100644
--- a/firmware/tools/gen_tables.py
+++ b/firmware/tools/gen_tables.py
@@ -1,5 +1,5 @@
 """Generates firmware/core/tables_gen.{h,c} from the committed characterisation files, so the
-firmware's filter edges, relay bit order, GPIO pads and bias-sense table are the hardware
+firmware's filter edges, relay bit order, GPIO pads, bias-sense table and monitor dividers are the hardware
 design's own numbers (phase 4 spec §2). Files are rewritten only when their text changes."""
 from __future__ import annotations
 
@@ -17,6 +17,7 @@ def _array(name: str, values) -> str:
 def render(char_dir: Path) -> tuple[str, str]:
     bank = json.loads((char_dir / "bpf_bank.json").read_text())
     ctl = json.loads((char_dir / "controller.json").read_text())
+    pw = json.loads((char_dir / "power.json").read_text())
     filters = sorted(bank["filters"], key=lambda f: f["index"])
     order = ctl["relay_chain_order"]
     table = ctl["bias_sense_table_27c"]
@@ -34,7 +35,10 @@ def render(char_dir: Path) -> tuple[str, str]:
           "extern const uint32_t TABLES_BIAS_SENSE_MV[];", "extern const uint32_t TABLES_BIAS_SENSE_UA[];",
           f"#define TABLES_BIAS_CUTOFF_UA {round(ctl['bias_cutoff_a'] * 1e6)}u",
           f"#define TABLES_I2C_SI5351 0x{ctl['i2c']['Si5351C']:02x}",
-          f"#define TABLES_I2C_LM75A 0x{ctl['i2c']['LM75A']:02x}", "", "#endif", ""]
+          f"#define TABLES_I2C_LM75A 0x{ctl['i2c']['LM75A']:02x}", "",
+          "/* monitor dividers: node voltage = ADC voltage × ratio / 1000 */"]
+    h += [f"#define TABLES_SENSE_DIV_{k.upper()}_X1000 {round(v * 1000)}u" for k, v in sorted(pw["sense_dividers"].items())]
+    h += ["", "#endif", ""]
     c = [HEADER, '#include "tables_gen.h"', "",
          _array("TABLES_FILTER_LO_HZ", (int(f["lo_hz"]) for f in filters)),
          _array("TABLES_FILTER_HI_HZ", (int(f["hi_hz"]) for f in filters)),
```

- [ ] **Step 4: Run them to verify they pass**

Run: `make fw-test`
Expected: ctest `100% tests passed, 0 tests failed out of 7`; cross-checks `3 passed`.

- [ ] **Step 5: Commit**

```bash
git add firmware
git commit -m "feat(fw): supervisor for the jack-only supply: divider ratios from power.json, bias-T gated by 5V_MAIN"
```

### Task 4: Regenerate, verify, README

- [ ] **Step 1:** Check RAM and swap, then `make hw-fresh`. Expected: every block PASS.
- [ ] **Step 2:** Commit `hardware/characterized`; cached `make hw`. Expected: identical (0 changed files).
- [ ] **Step 3:** `make test` and `make sim`. Expected: all pass.
- [ ] **Step 4:** README: the power feature line describes the 9–30 V jack, USB data only, buck, NTC and protection; commit.
