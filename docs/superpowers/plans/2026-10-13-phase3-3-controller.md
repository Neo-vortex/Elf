# Phase 3.3: Controller, Relay Drive, Bias-T Switch and Board Netlist Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add the ESP32-S3 controller (USB-C, CS5361 digital interface with a 24.576 MHz master clock, 74HC595/ULN2803 relay drive, LM75A, status LED), the SY6280 bias-T switch with a simulated current-mirror sense, and a board-level netlist that joins every block, with net checks.

**Architecture:** Every block builder takes an optional shared SKiDL `Circuit`; `blocks/board.py` builds all of them into one circuit, wires the interfaces through block ports, and merges only the shared rail and bus names. `checks/controller.py` simulates the bias sense in ngspice and checks the GPIO map; `checks/board.py` checks the whole netlist (single-pin nets, open pins, supply rails, footprints) and writes the board netlist and BOM.

**Tech Stack:** Python 3, SKiDL 2.3, ngspice 45.2, KiCad 9 footprint libraries, pytest.

**Spec:** `docs/superpowers/specs/2026-10-12-phase3-board-circuits-design.md` (§3.3 and its "Amended 2026-10-13 (plan 3.3 prototype)" block).

## Global Constraints

- Chips and semiconductors must be confirmed on Torob; pending ones (24.576 MHz oscillator, LED, button, SS34) are reported by the board check, not failed.
- No variable components; calibration is in firmware.
- Characterisation files keep `schema_version` 1 with `generated_by` and `status`.
- `make hw` CHECKS order ends `…, "clock", "mixer", "power", "controller", "board"`.
- ESP32-S3: never GPIO33–37 (octal PSRAM), no strapping pins (0, 3, 45, 46), USB on IO19/IO20, every ADC function on ADC1 (IO1–IO10).
- At power-up every relay is released (/OE pulled up), the bias feed is off (EN pulled down), the CS5361 is in reset.

## Review Focus

- Power-up state: /OE, bias EN and CS5361 RST pulls hold the safe state before the firmware runs (test_board.py: `test_power_up_holds_every_relay_released_and_the_bias_feed_off`).
- USB D+/D− reach IO20/IO19 unswapped on both receptacle rows (test_board.py: `test_usb_data_lines_reach_the_esp32_usb_pads_unswapped`).
- I2C pull-ups on 3V3_RF so the bus never back-powers the Si5351C (test_board.py: `test_i2c_pullups_go_to_the_si5351_core_rail`).
- CS5361 mode straps: M1/M0 = 0/1, MDIV = 1, MCLK from the oscillator (test_board.py: `test_cs5361_mode_pins_select_double_speed_master_clock_divide_by_two`).
- Relay chain bit order equals `controller.RELAYS`, the table the firmware uses (test_board.py: `test_relay_chain_bit_order_matches_the_firmware_table`).

---

### Task 1: New parts and the crystal pin fix

**Files:**
- Modify: `hardware/neohw/parts.py`, `hardware/neohw/availability.py`, `hardware/neohw/blocks/clock.py`
- Test: `hardware/tests/test_pin_names.py` (new), `hardware/tests/test_clock.py`

**Interfaces:**
- Produces: `parts.esp32s3_wroom1(nc)`, `ESP32S3_PADS`, `ESP32S3_PSRAM_PINS`, `ESP32S3_STRAPPING_PINS`, `usb_c_16p`, `usblc6`, `lm75a`, `hc595` (serial out pin `QHS`), `uln2803`, `sy6280(r_set)`, `oscillator_24m576`, `led_0805`, `button`; `crystal_25mhz` pins `X1`, `X2`, `GND1`, `GND2`.

- [ ] **Step 1: Write the failing tests**

```python
"""SKiDL resolves part["2"] as pad number 2 before pin name "2", so a pin whose name is a digit must
sit on the pad with that number (the bug that shorted the crystal's XB to ground)."""
from skidl import Circuit

from neohw import parts

CONSTRUCTORS = [
    lambda: parts.resistor("1k"), lambda: parts.capacitor("1n"), lambda: parts.inductor(1e-6, 30, 1e6),
    parts.si5351c, parts.crystal_25mhz, parts.lvc74, parts.lvc157, parts.ao3401, parts.ss34, parts.smbj5,
    lambda: parts.fuse_1206(1.0), parts.barrel_jack, parts.mt3608, lambda: parts.mic5205(5.0), parts.tlv74033,
    parts.esp32s3_wroom1, parts.usb_c_16p, parts.usblc6, parts.lm75a, parts.hc595, parts.uln2803,
    lambda: parts.sy6280(17.4e3), parts.oscillator_24m576, parts.led_0805, parts.button, parts.fst3253,
    parts.lm4562_dual, parts.cs5361, parts.pnp_bc857b, parts.npn_bfr92a, parts.diode_pair_bav99, parts.gdt,
    lambda: parts.relay_dpdt("released"),
]


def test_digit_pin_names_sit_on_the_pad_with_that_number():
    with Circuit():
        for make in CONSTRUCTORS:
            p = make()
            for pin in p.pins:
                if pin.name.isdigit():
                    assert pin.num == pin.name, (p.value, pin.name, pin.num)
```

```diff

```

- [ ] **Step 2: Run them to verify they fail**

Run: `cd hardware && python3 -m pytest -q tests/test_pin_names.py tests/test_clock.py`
Expected: collection ERROR in test_pin_names.py: `AttributeError: module 'neohw.parts' has no attribute 'esp32s3_wroom1'`.

- [ ] **Step 3: Implement**

```diff
diff --git a/hardware/neohw/parts.py b/hardware/neohw/parts.py
index f0f705c..8301ae6 100644
--- a/hardware/neohw/parts.py
+++ b/hardware/neohw/parts.py
@@ -193,7 +193,7 @@ def npn_bfr92a(mpn: str = "BFR92A", torob: str | None = "BFR92", bf: float | Non
     return p
 
 
-def pnp_bc857b(mpn: str = "BC857B", torob: str | None = None) -> Part:
+def pnp_bc857b(mpn: str = "BC857B", torob: str | None = "BC857B") -> Part:
     """Nexperia BC857B general-purpose PNP, SOT-23 (pin 1 base, 2 emitter, 3 collector)."""
     p = _tagged(_template("BC857B", "Q", {"B": "1", "E": "2", "C": "3"},
                           "Package_TO_SOT_SMD:SOT-23")(value="BC857B"))
@@ -203,7 +203,7 @@ def pnp_bc857b(mpn: str = "BC857B", torob: str | None = None) -> Part:
     return p
 
 
-def diode_pair_bav99(mpn: str = "BAV99", torob: str | None = None) -> Part:
+def diode_pair_bav99(mpn: str = "BAV99", torob: str | None = "BAV99") -> Part:
     """BAV99, SOT-23: pin 1 anode of D1, pin 3 = D1 cathode + D2 anode, pin 2 cathode of D2."""
     p = _tagged(_template("BAV99", "D", {"A": "1", "K": "2", "COM": "3"},
                           "Package_TO_SOT_SMD:SOT-23")(value="BAV99"))
@@ -319,7 +319,8 @@ def crystal_25mhz() -> Part:
     the load capacitance internally (6/8/10 pF, register 183), so the crystal's CL must be one of
     those: buy CL ≤ 10 pF (generic 3225 crystals are often 12–20 pF, which pulls the frequency by
     tens of ppm)."""
-    p = _ic("25MHz", {"1": "1", "GND1": "2", "2": "3", "GND2": "4"}, "Crystal:Crystal_SMD_3225-4Pin_3.2x2.5mm",
+    # pin names must not be digits: SKiDL resolves x["2"] as pad number 2 (a case-ground pad)
+    p = _ic("25MHz", {"X1": "1", "GND1": "2", "X2": "3", "GND2": "4"}, "Crystal:Crystal_SMD_3225-4Pin_3.2x2.5mm",
             "25MHz 3225 CL=8pF", "کریستال 25MHZ 3225")
     p.torob_match = crystal_25mhz_hit
     p.bom_note = "CL ≤ 10 pF (Si5351 internal load 6/8/10 pF); confirm CL with the seller"
@@ -447,3 +448,100 @@ def _ldo_lines(ref: str, n: dict, v_out: float, dropout: float, byp: str | None
     if byp:
         out += [f"R{ref}_byp {n[byp]} {n['GND']} 1e9", f"R{ref}_en {n['EN']} {n['GND']} 1e7"]
     return out
+
+
+# --- controller, USB, ADC interface, relay drive (phase 3 spec §3.3) -----------------------
+
+# ESP32-S3-WROOM-1 pads, from the KiCad 9 RF_Module symbol (IO19/IO20 are USB D−/D+;
+# IO35–IO37 carry the octal PSRAM on the N16R8 and are not brought out for use)
+ESP32S3_PADS = {"GND": "1", "3V3": "2", "EN": "3", "IO4": "4", "IO5": "5", "IO6": "6", "IO7": "7", "IO15": "8",
+                "IO16": "9", "IO17": "10", "IO18": "11", "IO8": "12", "IO19": "13", "IO20": "14", "IO3": "15",
+                "IO46": "16", "IO9": "17", "IO10": "18", "IO11": "19", "IO12": "20", "IO13": "21", "IO14": "22",
+                "IO21": "23", "IO47": "24", "IO48": "25", "IO45": "26", "IO0": "27", "IO35": "28", "IO36": "29",
+                "IO37": "30", "IO38": "31", "IO39": "32", "IO40": "33", "IO41": "34", "IO42": "35", "RXD0": "36",
+                "TXD0": "37", "IO2": "38", "IO1": "39", "GND2": "40", "EPAD": "41"}
+ESP32S3_PSRAM_PINS = ("IO35", "IO36", "IO37")
+ESP32S3_STRAPPING_PINS = ("IO0", "IO3", "IO45", "IO46")
+
+
+def esp32s3_wroom1(nc=()) -> Part:
+    """Espressif ESP32-S3-WROOM-1-N16R8 module (16 MB flash, 8 MB octal PSRAM)."""
+    return _ic("ESP32-S3-WROOM-1", ESP32S3_PADS, "RF_Module:ESP32-S3-WROOM-1", "ESP32-S3-WROOM-1-N16R8",
+               "ESP32-S3-WROOM-1 N16R8", nc)
+
+
+USB_C_PADS = {p: p for p in ("A1", "A4", "A5", "A6", "A7", "A8", "A9", "A12", "B1", "B4", "B5", "B6", "B7", "B8",
+                             "B9", "B12", "S1")}
+
+
+def usb_c_16p() -> Part:
+    """USB Type-C receptacle, 16 pins (USB 2.0), HRO TYPE-C-31-M-12 footprint: A1/A12/B1/B12 GND,
+    A4/A9/B4/B9 VBUS, A5 CC1, B5 CC2, A6/B6 D+, A7/B7 D−, A8/B8 SBU (unused), S1 shell."""
+    return _ic("USB-C 16P", USB_C_PADS, "Connector_USB:USB_C_Receptacle_HRO_TYPE-C-31-M-12", "TYPE-C-31-M-12",
+               "USB TYPE-C 16PIN", nc=("A8", "B8"))
+
+
+def usblc6() -> Part:
+    """ST USBLC6-2SC6 ESD array, SOT-23-6: 1 I/O1, 2 GND, 3 I/O2, 4 I/O2, 5 VBUS, 6 I/O1 (flow-through)."""
+    return _ic("USBLC6-2SC6", {"IO1A": "1", "GND": "2", "IO2A": "3", "IO2B": "4", "VBUS": "5", "IO1B": "6"},
+               "Package_TO_SOT_SMD:SOT-23-6", "USBLC6-2SC6", "USBLC6-2SC6")
+
+
+def lm75a() -> Part:
+    """NXP LM75A temperature sensor, SO-8: 1 SDA, 2 SCL, 3 OS, 4 GND, 5 A2, 6 A1, 7 A0, 8 VCC."""
+    return _ic("LM75A", {"SDA": "1", "SCL": "2", "OS": "3", "GND": "4", "A2": "5", "A1": "6", "A0": "7", "VCC": "8"},
+               "Package_SO:SOIC-8_3.9x4.9mm_P1.27mm", "LM75AD", "LM75AD", nc=("OS",))
+
+
+HC595_PINS = {"QB": "1", "QC": "2", "QD": "3", "QE": "4", "QF": "5", "QG": "6", "QH": "7", "GND": "8", "QHS": "9",
+              "/SRCLR": "10", "SRCLK": "11", "RCLK": "12", "/OE": "13", "SER": "14", "QA": "15", "VCC": "16"}
+
+
+def hc595(nc=()) -> Part:
+    """74HC595 8-bit shift register with output latch, SO-16 (TI SN74HC595 pinout)."""
+    return _ic("74HC595", HC595_PINS, "Package_SO:SOIC-16_3.9x9.9mm_P1.27mm", "74HC595D", "74HC595 SMD", nc)
+
+
+ULN2803_PINS = {**{f"I{k}": str(k) for k in range(1, 9)}, "GND": "9", "COM": "10",
+                **{f"O{k}": str(19 - k) for k in range(1, 9)}}
+
+
+def uln2803(nc=()) -> Part:
+    """ULN2803A Darlington array, SO-18W: inputs 1–8, GND 9, COM 10 (flyback diodes' common, to
+    the coil supply), outputs 18–11 (O1 = 18 … O8 = 11)."""
+    return _ic("ULN2803", ULN2803_PINS, "Package_SO:SOIC-18W_7.5x11.6mm_P1.27mm", "ULN2803ADWR", "ULN2803", nc)
+
+
+def sy6280(r_set: float) -> Part:
+    """Silergy SY6280AAC current-limited load switch, SOT-23-5: 1 OUT, 2 GND, 3 ISET, 4 EN, 5 IN;
+    I_LIM = 6800 / R_SET (0.4 A minimum), R_on 80 mΩ, EN high ≥ 2.4 V (VIN 4.2–5.5 V), no body
+    diode (reverse blocking), thermal auto-retry (datasheet). Behavioural model (smooth, for Newton):
+    IN→OUT current I_LIM·tanh(((V_IN − V_OUT)/80 mΩ) / I_LIM) gated by a logistic step at EN = 1.6 V;
+    nothing flows backwards."""
+    p = _ic("SY6280", {"OUT": "1", "GND": "2", "ISET": "3", "EN": "4", "IN": "5"},
+            "Package_TO_SOT_SMD:SOT-23-5", "SY6280AAC", "SY6280")
+    i_lim = 6800.0 / r_set
+    p.spice_lines = lambda n, k=1.0: [
+        f"B{p.ref}_sw {n['IN']} {n['OUT']} I = 1 / (1 + exp(-20 * (v({n['EN']}) - 1.6))) * "
+        f"{i_lim:.4g} * tanh(max(0, v({n['IN']}) - v({n['OUT']})) / 0.08 / {i_lim:.4g})",
+        f"R{p.ref}_en {n['EN']} {n['GND']} 1e9", f"R{p.ref}_off {n['OUT']} {n['GND']} 1e6"]
+    return p
+
+
+def oscillator_24m576() -> Part:
+    """24.576 MHz 3.3 V CMOS crystal oscillator, 3.2 × 2.5 mm 4-pad (1 EN/tri-state, 2 GND, 3 OUT, 4 VDD):
+    the CS5361's master clock (master mode, MDIV = 1 → 96 kHz double speed). Torob availability to
+    confirm (pending, user away 2026-10-13)."""
+    p = _ic("24.576MHz", {"EN": "1", "GND": "2", "OUT": "3", "VDD": "4"},
+            "Oscillator:Oscillator_SMD_Abracon_ASE-4Pin_3.2x2.5mm", "24.576MHz 3.3V CMOS XO", "اسیلاتور 24.576")
+    p.bom_note = "Torob availability not yet confirmed"
+    return p
+
+
+def led_0805(colour: str = "green") -> Part:
+    p = _ic(f"LED {colour}", {"K": "1", "A": "2"}, "LED_SMD:LED_0805_2012Metric", f"LED 0805 {colour}", None)
+    return p
+
+
+def button() -> Part:
+    return _ic("button", {"1": "1", "2": "2"}, "Button_Switch_SMD:SW_SPST_SKQG_WithoutStem", "SKQG tactile", None)
```

- [ ] **Step 4: Run to verify they pass**

Run: `cd hardware && python3 -m pytest -q tests/test_pin_names.py tests/test_clock.py tests/test_parts.py`
Expected: `31 passed`.

- [ ] **Step 5: Commit**

```bash
git add hardware/neohw/parts.py hardware/neohw/availability.py hardware/neohw/blocks/clock.py hardware/tests/test_pin_names.py hardware/tests/test_clock.py
git commit -m "feat(hw): controller parts; fix the crystal pins wired to a ground pad"
```

### Task 2: Shared circuits, bias-T switch, controller block and check

**Files:**
- Modify: every block in `hardware/neohw/blocks/` (optional `ckt` parameter), `hardware/tests/test_footprints.py`
- Create: `hardware/neohw/blocks/bias_switch.py`, `hardware/neohw/blocks/controller.py`, `hardware/neohw/checks/controller.py`
- Test: `hardware/tests/test_controller.py`

**Interfaces:**
- Consumes: Task 1 parts.
- Produces: `<block>.build(..., ckt=None)`; `bias_switch.build(ckt)` ports `bias5, en, feed, sense, raw`; `controller.GPIO`, `controller.RELAYS`, `controller.ADC_FUNCTIONS`, `controller.ADC1_PINS`, `controller.build(ckt)` ports `vbus, main, bias5, v3d, v3rf, bias_feed, coil{name}, adc{mclk,m0,m1,mdiv,hpf}, gpio{name}`; `checks.controller.bias_sense()`, `gpio_findings()`, `run(out_dir, char_dir)`.

- [ ] **Step 1: Write the failing tests**

```python
import pytest

from neohw.blocks import controller
from neohw.checks import controller as ctl_check
from neohw.tools import ToolNotFoundError, tool_path


def test_gpio_map_follows_the_esp32_s3_rules():
    assert ctl_check.gpio_findings() == []


def test_gpio_rules_flag_a_psram_pin(monkeypatch):
    monkeypatch.setitem(controller.GPIO, "status_led", "IO35")
    assert any("octal PSRAM" in x for x in ctl_check.gpio_findings())


@pytest.mark.integration
def test_bias_sense_is_monotonic_stable_and_within_the_adc_range():
    try:
        tool_path("ngspice")
    except ToolNotFoundError as e:
        pytest.skip(str(e))
    bs = ctl_check.bias_sense()
    for tab in bs["table"].values():
        assert all(b[1] > a[1] for a, b in zip(tab, tab[1:]))
    assert all(0.3 <= s["i_a"] <= 0.5 and s["sense_v"] <= 3.1 for s in bs["short"].values())
```

```diff

```

- [ ] **Step 2: Run to verify they fail**

Run: `cd hardware && python3 -m pytest -q tests/test_footprints.py`
Expected: FAIL / ERROR: `cannot import name 'controller' from 'neohw.blocks'`.

- [ ] **Step 3: Implement**

Shared-circuit parameter in every existing block:

```diff

```

`hardware/neohw/blocks/bias_switch.py`:

```python
"""Bias-T switch and current sense (phase 3 spec §3.3): 5V_BIAS → 1 Ω shunt → SY6280 (R_SET 17.4 kΩ:
I_LIM ≈ 0.39 A, the part's minimum) → bias feed.

A BC857B pair mirrors the shunt drop. The diode-connected reference sits on the shunt's load side
through 100 Ω and the output transistor on its supply side through 100 Ω, so the shunt drop raises
the output current by about I_load · R_shunt / (R_E + r_e). 680 Ω turns it into a voltage for an
ESP32-S3 ADC pin, on top of the reference bias (≈ 0.6 V; the firmware reads it as zero with the
feed switched off). The transfer is monotonic and slightly non-linear (r_e falls as the current
rises), so the check exports it as a table for the firmware. The 680 Ω keeps the reading below
3.1 V even at the SY6280's current limit. The firmware cuts the feed at about 150 mA; the SY6280
limit is the hardware backstop."""
from __future__ import annotations

from skidl import Circuit, Net

from neohw import parts
from neohw.blocks import Block

SHUNT, RE, RL, RBIAS, RSET = "1", "100", "680", "4.7k", 17.4e3


def _r(value, a, b):
    r = parts.resistor(value)
    r[1] += a
    r[2] += b


def build(ckt: Circuit | None = None) -> Block:
    ckt = ckt or Circuit()
    with ckt:
        gnd, bias5, en, feed, sense = Net("GND"), Net("+5V_BIAS"), Net("bias_en"), Net("bias_feed"), Net("bias_isense")
        lo, raw, b, e1, e2, iset = (Net(f"bias_{x}") for x in ("shunt_lo", "isense_raw", "mirror_b", "mirror_e1",
                                                                "mirror_e2", "iset"))
        _r(SHUNT, bias5, lo)
        sw = parts.sy6280(RSET)
        sw["IN"] += lo
        sw["OUT"] += feed
        sw["GND"] += gnd
        sw["EN"] += en
        sw["ISET"] += iset
        _r("17.4k", iset, gnd)
        _r("100k", en, gnd)
        c = parts.capacitor("1u", note="X7R switch input")
        c[1] += lo
        c[2] += gnd
        q1, q2 = parts.pnp_bc857b(), parts.pnp_bc857b()
        _r(RE, lo, e1)          # reference (diode-connected): load side of the shunt
        _r(RE, bias5, e2)       # output: supply side
        q1["E"] += e1
        q1["B"] += b
        q1["C"] += b
        _r(RBIAS, b, gnd)
        q2["E"] += e2
        q2["B"] += b
        q2["C"] += raw
        _r(RL, raw, gnd)
        _r("10k", raw, sense)
        cf = parts.capacitor("100n", note="X7R ADC filter")
        cf[1] += sense
        cf[2] += gnd
    return Block("bias_switch", ckt, {"bias5": bias5, "en": en, "feed": feed, "sense": sense, "raw": raw})
```

`hardware/neohw/blocks/controller.py`:

```python
"""Controller, USB, ADC interface, relay drive and bias-T switch (phase 3 spec §3.3).

- ESP32-S3-WROOM-1-N16R8 on 3V3_D: native USB on IO19/IO20 to a 16-pin USB-C receptacle (5.1 kΩ
  CC pull-downs, USBLC6-2SC6), EN reset RC and button, IO0 boot button.
- CS5361 digital side: a 24.576 MHz oscillator is the master clock and the CS5361 is clock master
  (MDIV = 1, M1/M0 = 0/1: 96 kHz double speed), so the ESP32's fractional-divider I2S clock never
  times the ADC; the ESP32 receives SCLK/LRCK/SDOUT as I2S slave. M/S and I2S/LJ are driven by
  GPIOs (their pin polarity is not in the datasheet text: the firmware selects master/I2S and
  checks that LRCK and SCLK toggle before enabling I2S). HPF high (enabled).
- I2C (Si5351C 0x60, LM75A 0x48): 4.7 kΩ pull-ups to 3V3_RF, the Si5351's core rail (datasheet:
  pull up to VDD core), so the bus never back-powers it.
- Relay drive: 3 × 74HC595 (chained, /OE held high by 10 kΩ until the firmware enables the
  outputs: every relay released, the bank muted) → 3 × ULN2803 → 24 coils on 5V_MAIN
  (COM to 5V_MAIN for the flyback diodes).
- Bias-T: the bias_switch block (1 Ω shunt, SY6280 limited to ≈ 0.39 A, BC857B current mirror into
  an ADC pin; the firmware cuts the feed at ≈ 150 mA). EN pulled low.
"""
from __future__ import annotations

from skidl import Circuit, Net

from neohw import parts
from neohw.blocks import Block, bias_switch

GPIO = {
    "sense_vbus": "IO1", "sense_ext": "IO2", "sense_main": "IO4", "bias_isense": "IO5", "cc1": "IO6", "cc2": "IO7",
    "i2c_sda": "IO8", "i2c_scl": "IO9",
    "i2s_bclk": "IO10", "i2s_ws": "IO11", "i2s_din": "IO12",
    "adc_rst": "IO13", "adc_ovfl": "IO14", "adc_ms": "IO41", "adc_fmt": "IO42",
    "lo_path": "IO15",
    "sr_ser": "IO16", "sr_clk": "IO17", "sr_latch": "IO18", "sr_oe": "IO21",
    "bias_en": "IO38", "status_led": "IO40",
}
ADC_FUNCTIONS = ("sense_vbus", "sense_ext", "sense_main", "bias_isense", "cc1", "cc2")
ADC1_PINS = tuple(f"IO{k}" for k in range(1, 11))
# relay order on the shift-register chain (bit 0 = first 74HC595 QA)
RELAYS = tuple([f"bank{k}_{end}" for k in range(1, 12) for end in ("in", "out")] + ["atten", "preamp"])


def _r(value, a, b):
    r = parts.resistor(value)
    r[1] += a
    r[2] += b
    return r


def _c(value, a, b, note="X7R decoupling"):
    c = parts.capacitor(value, note=note)
    c[1] += a
    c[2] += b


def build(ckt: Circuit | None = None) -> Block:
    ckt = ckt or Circuit()
    with ckt:
        gnd, v3d, v3rf, main, bias5 = Net("GND"), Net("+3V3_D"), Net("+3V3_RF"), Net("+5V_MAIN"), Net("+5V_BIAS")
        vbus, dp, dm, cc1, cc2 = Net("VBUS"), Net("usb_dp"), Net("usb_dm"), Net("usb_cc1"), Net("usb_cc2")
        en, io0 = Net("esp_en"), Net("esp_io0")
        g = {name: Net(name) for name in GPIO}
        # USB-C and ESD
        j = parts.usb_c_16p()
        for pad in ("A1", "A12", "B1", "B12", "S1"):
            j[pad] += gnd
        for pad in ("A4", "A9", "B4", "B9"):
            j[pad] += vbus
        j["A6"] += dp
        j["B6"] += dp
        j["A7"] += dm
        j["B7"] += dm
        j["A5"] += cc1
        j["B5"] += cc2
        _r("5.1k", cc1, gnd)
        _r("5.1k", cc2, gnd)
        _r("10k", cc1, g["cc1"])           # CC voltage (0.41 / 0.92 / 1.68 V = default / 1.5 A / 3 A) to an ADC
        _r("10k", cc2, g["cc2"])
        esd = parts.usblc6()
        esd["IO1A"] += dp
        esd["IO1B"] += dp
        esd["IO2A"] += dm
        esd["IO2B"] += dm
        esd["VBUS"] += vbus
        esd["GND"] += gnd
        # ESP32-S3
        used = set(GPIO.values()) | {"IO19", "IO20", "IO0"}
        free = [pad for pad in parts.ESP32S3_PADS if pad.startswith("IO") and pad not in used] + ["RXD0", "TXD0"]
        u = parts.esp32s3_wroom1(nc=tuple(free))
        u["3V3"] += v3d
        for pad in ("GND", "GND2", "EPAD"):
            u[pad] += gnd
        u["EN"] += en
        u["IO0"] += io0
        u["IO19"] += dm
        u["IO20"] += dp
        for name, pad in GPIO.items():
            u[pad] += g[name]
        _r("10k", en, v3d)
        _c("1u", en, gnd, "X7R EN delay")
        for node in (en, io0):
            b = parts.button()
            b["1"] += node
            b["2"] += gnd
        _r("10k", io0, v3d)
        _c("22u", v3d, gnd, "X5R bulk")
        _c("100n", v3d, gnd)
        # I2C pull-ups to the Si5351's core rail; LM75A on the same rail
        _r("4.7k", g["i2c_sda"], v3rf)
        _r("4.7k", g["i2c_scl"], v3rf)
        t = parts.lm75a()
        t["SDA"] += g["i2c_sda"]
        t["SCL"] += g["i2c_scl"]
        for pad in ("A0", "A1", "A2", "GND"):
            t[pad] += gnd
        t["VCC"] += v3rf
        _c("100n", v3rf, gnd)
        # CS5361 digital interface (the CS5361 part is in the mixer block; the board wires these)
        osc_vdd, mclk_raw = Net("osc_vdd"), Net("osc_out")
        adc = {k: Net(f"adc_{k}") for k in ("mclk", "m0", "m1", "mdiv", "hpf")}
        x = parts.oscillator_24m576()
        x["VDD"] += osc_vdd
        x["EN"] += osc_vdd
        x["GND"] += gnd
        x["OUT"] += mclk_raw
        _r("10", v3d, osc_vdd)
        _c("10u", osc_vdd, gnd, "X5R oscillator supply")
        _c("100n", osc_vdd, gnd)
        _r("33", mclk_raw, adc["mclk"])
        _r("10k", adc["m0"], v3d)          # M1/M0 = 0/1: double speed (Table 1)
        _r("10k", adc["m1"], gnd)
        _r("10k", adc["mdiv"], v3d)        # MCLK ÷ 2: 24.576 → 12.288 MHz = 128 × 96 kHz
        _r("10k", adc["hpf"], v3d)
        _r("10k", g["adc_ms"], gnd)
        _r("10k", g["adc_fmt"], v3d)
        _r("10k", g["adc_rst"], gnd)       # held in reset until the firmware releases it
        _r("10k", g["adc_ovfl"], v3d)      # open-drain overflow flag
        # relay drive
        sr = [parts.hc595() for _ in range(3)]
        drv = [parts.uln2803() for _ in range(3)]
        coil = {name: Net(f"coil_{name}") for name in RELAYS}
        chain = g["sr_ser"]
        outs = ["QA", "QB", "QC", "QD", "QE", "QF", "QG", "QH"]
        for k, (s, d) in enumerate(zip(sr, drv)):
            s["VCC"] += v3d
            s["GND"] += gnd
            s["SER"] += chain
            s["SRCLK"] += g["sr_clk"]
            s["RCLK"] += g["sr_latch"]
            s["/OE"] += g["sr_oe"]
            s["/SRCLR"] += v3d
            nxt = Net(f"sr_chain{k + 1}")
            s["QHS"] += nxt              # serial out (QH' on the datasheet)
            chain = nxt
            d["GND"] += gnd
            d["COM"] += main
            for bit, q in enumerate(outs):
                idx = 8 * k + bit
                link = Net(f"sr_q{idx}")
                s[q] += link
                d[f"I{bit + 1}"] += link
                if idx < len(RELAYS):
                    d[f"O{bit + 1}"] += coil[RELAYS[idx]]
                else:
                    d[f"O{bit + 1}"] += Net(f"drv_spare{idx}")
            _c("100n", v3d, gnd)
        _r("10k", g["sr_oe"], v3d)
        _r("10k", chain, gnd)               # last QH' unused: tied off through 10 kΩ
        # bias-T switch and current sense (its own block, also simulated on its own)
        bsw = bias_switch.build(ckt)
        bsw.ports["bias5"] += bias5
        bsw.ports["en"] += g["bias_en"]
        bsw.ports["sense"] += g["bias_isense"]
        feed = bsw.ports["feed"]
        # status LED
        led, led_a = parts.led_0805(), Net("led_a")
        _r("330", g["status_led"], led_a)
        led["A"] += led_a
        led["K"] += gnd
    return Block("controller", ckt, {"vbus": vbus, "main": main, "bias5": bias5, "v3d": v3d, "v3rf": v3rf,
                                     "bias_feed": feed, "coil": coil, "adc": adc, "gpio": g})
```

`hardware/neohw/checks/controller.py`:

```python
"""Controller check (phase 3 spec §3.3): bias-T switch and current sense in SPICE (transfer table for
the firmware, temperature drift, current limit, ADC range), ESP32-S3 GPIO rules, I2C address map
and relay-driver budget. Writes controller.json (the GPIO map and sense table the firmware uses)."""
from __future__ import annotations

from pathlib import Path

from neohw import availability, charfile, export, parts
from neohw.blocks import bias_switch, controller
from neohw.checks import BlockResult, CheckResult
from neohw.ngspice import run_decks
from neohw.spice import Load, Op, node_name, to_deck

LOAD_OHMS = (1e7, 250, 100, 50, 33, 25)        # 0 … ~190 mA from 5 V
TEMPS_C = (0.0, 27.0, 50.0)
ADC_MAX_V = 3.1                                  # ESP32-S3 ADC, 11 dB attenuation (≈ 0–3.1 V)
I_LIM_A = (0.30, 0.50)                           # SY6280 at R_SET 17.4 kΩ: 6800/17400 = 0.39 A nominal
DRIFT_MAX = 0.03                                 # reading drift 0–50 °C at ≈ 150 mA, relative
I2C_ADDRESSES = {"Si5351C": 0x60, "LM75A": 0x48}
RELAY_COIL_A = 0.028                             # G6K-2F 5 VDC coil (178 Ω)
RELAYS_ON_MAX = 4                                # selected filter's 2 + attenuator + preamp
ULN2803_CHANNEL_A = 0.5


def bias_sense() -> dict:
    blk = bias_switch.build()
    P = {k: node_name(v) for k, v in blk.ports.items()}
    cases, decks = [], []
    for t in TEMPS_C:
        for r in (*LOAD_OHMS, 1.0):
            decks.append(to_deck(blk.circuit, "bias sense", [Load(blk.ports["sense"], "adc", 1e7),
                                                              Load(blk.ports["feed"], "ant", 1e7)], [Op()],
                                 extra_lines=[f"VB {P['bias5']} 0 dc 5.0", f"VEN {P['en']} 0 dc 3.3",
                                              f"RLD {P['feed']} 0 {r:g}", f".options temp={t:g}"])[0])
            cases.append((t, r))
    out: dict = {"table": {}, "short": {}}
    for (t, r), res in zip(cases, run_decks(decks, ["a0.raw"])):
        d = res["a0.raw"][0].data
        v_feed, v_sense = d[f"v({P['feed']})"][0].real, d[f"v({P['sense']})"][0].real
        if r == 1.0:
            out["short"][t] = {"i_a": v_feed / r, "sense_v": v_sense}
        else:
            out["table"].setdefault(t, []).append((v_feed / r, v_sense))
    return out


def gpio_findings() -> list[str]:
    g = controller.GPIO
    pads = list(g.values())
    bad = []
    if len(set(pads)) != len(pads):
        bad.append("a GPIO is assigned twice")
    for name, pad in g.items():
        if pad in parts.ESP32S3_PSRAM_PINS:
            bad.append(f"{name} on {pad}: octal PSRAM pin")
        if pad in parts.ESP32S3_STRAPPING_PINS:
            bad.append(f"{name} on {pad}: strapping pin")
        if pad in ("IO19", "IO20"):
            bad.append(f"{name} on {pad}: USB pin")
        if name in controller.ADC_FUNCTIONS and pad not in controller.ADC1_PINS:
            bad.append(f"{name} on {pad}: not an ADC1 pin")
    return bad


def run(out_dir: Path, char_dir: Path) -> BlockResult:
    out_dir, char_dir = Path(out_dir), Path(char_dir)
    bs = bias_sense()
    room = bs["table"][27.0]
    zero = room[0][1]
    near150 = {t: min(tab, key=lambda iv: abs(iv[0] - 0.15)) for t, tab in bs["table"].items()}
    drift = max(abs((near150[t][1] - bs["table"][t][0][1]) / (near150[27.0][1] - zero) - 1) for t in TEMPS_C)
    monotonic = all(all(b[1] > a[1] for a, b in zip(tab, tab[1:])) for tab in bs["table"].values())
    v_max = max(v["sense_v"] for v in bs["short"].values())
    i_short = bs["short"][27.0]["i_a"]
    bad = gpio_findings()
    addrs = list(I2C_ADDRESSES.values())
    coil_total = RELAY_COIL_A * RELAYS_ON_MAX
    checks = [
        CheckResult("bias_sense_monotonic", float(monotonic), "= 1", monotonic, ""),
        CheckResult("bias_sense_zero_v", zero, "report (subtracted by the firmware, feed off)", True, "V"),
        CheckResult("bias_sense_drift_0_50c", drift * 100, f"≤ {DRIFT_MAX * 100:g}", drift <= DRIFT_MAX, "%"),
        CheckResult("bias_short_current_a", i_short, f"{I_LIM_A[0]:g}–{I_LIM_A[1]:g}", I_LIM_A[0] <= i_short <= I_LIM_A[1], "A"),
        CheckResult("bias_sense_max_v", v_max, f"≤ {ADC_MAX_V:g}", v_max <= ADC_MAX_V, "V"),
        CheckResult("gpio_rule_violations", len(bad), "= 0", not bad, ""),
        CheckResult("i2c_addresses_unique", float(len(set(addrs)) == len(addrs)), "= 1", len(set(addrs)) == len(addrs), ""),
        CheckResult("relay_coil_current_a", coil_total, f"per channel ≤ {ULN2803_CHANNEL_A:g}", RELAY_COIL_A <= ULN2803_CHANNEL_A, "A"),
    ]
    checks = [CheckResult(c.name, float(c.value), c.limit, bool(c.passed), c.unit) for c in checks]   # plain JSON types
    result = BlockResult("controller", checks, plots={
        f"bias sense {t:g} °C (V vs A)": ([i for i, _ in tab], [v for _, v in tab]) for t, tab in bs["table"].items()})
    blk = controller.build()
    data = availability.load()
    result.artefacts = {
        "kicad_netlist": export.kicad_netlist(blk.circuit, out_dir / "controller.net"),
        "bom": export.write_bom(blk.circuit, out_dir / "controller_bom.csv", data),
        "svg": export.svg_diagram(blk.circuit, out_dir / "controller.svg"),
        "characterisation": charfile.write(char_dir / "controller.json", "controller", "pass" if result.passed else "fail", {
            "gpio": controller.GPIO, "relay_chain_order": list(controller.RELAYS), "i2c": I2C_ADDRESSES,
            "bias_sense_table_27c": [{"i_a": float(i), "sense_v": float(v)} for i, v in room],
            "bias_cutoff_a": 0.15, "gpio_findings": bad,
            "checks": [c.__dict__ for c in checks]}),
    }
    return result
```

- [ ] **Step 4: Run to verify they pass**

Run: `cd hardware && python3 -m pytest -q tests/test_controller.py tests/test_footprints.py`
Expected: `4 passed` (with ngspice installed; the bias-sense test simulates 21 decks).

Run: `cd hardware && python3 -c "from pathlib import Path; from neohw.checks import controller as c; r=c.run(Path('build/plan33/out'),Path('build/plan33/char')); print([(x.name, round(x.value,3), x.passed) for x in r.checks])"`
Expected: monotonic 1.0; zero ≈ 0.628 V; drift ≈ 1.30 %; short ≈ 0.391 A; max sense ≈ 2.877 V; 0 GPIO violations; I2C unique 1.0; coil 0.112 A; all True.

- [ ] **Step 5: Commit**

```bash
git add hardware/neohw/blocks hardware/neohw/checks/controller.py hardware/tests/test_controller.py hardware/tests/test_footprints.py
git commit -m "feat(hw): controller, relay drive and bias-T switch with simulated current sense"
```

### Task 3: Board netlist and check

**Files:**
- Create: `hardware/neohw/blocks/board.py`, `hardware/neohw/checks/board.py`
- Modify: `hardware/neohw/__main__.py`, `hardware/tests/test_cli.py`
- Test: `hardware/tests/test_board.py` (including the five Review Focus tests)

**Interfaces:**
- Consumes: Task 2 block builders with `ckt`, controller ports.
- Produces: `board.build()` → Block with ports per block and `parts_by_block`; `board.SHARED_NETS`; `checks.board.findings(ckt)` → `{single_pin_nets, open_pins, rail_errors, missing_footprints}`; `checks.board._net(pin)`; `run(out_dir, char_dir)` writing `board.net`, `board_bom.csv`, `board.json`.

- [ ] **Step 1: Write the failing tests**

```python
from skidl import Circuit, Net

from neohw import parts
from neohw.blocks import board, controller
from neohw.checks import board as check


def test_the_whole_board_has_no_open_pins_and_every_supply_on_its_rail():
    f = check.findings(board.build().circuit)
    assert f == {"single_pin_nets": [], "open_pins": [], "rail_errors": [], "missing_footprints": []}


def test_the_rail_check_catches_a_chip_on_the_wrong_rail():
    ckt = Circuit()
    with ckt:
        s = parts.hc595()
        rf, gnd = Net("+3V3_RF"), Net("GND")
        s["VCC"] += rf               # should be +3V3_D
        s["GND"] += gnd
    errors = check.findings(ckt)["rail_errors"]
    assert any("74HC595.VCC on +3V3_RF, expected +3V3_D" in e for e in errors)


def test_the_rail_check_catches_a_ground_pin_off_ground():
    ckt = Circuit()
    with ckt:
        u = parts.lm75a()
        u["GND"] += Net("+3V3_RF")
    assert any("LM75A.GND on +3V3_RF, expected GND" in e for e in check.findings(ckt)["rail_errors"])


def test_relay_chain_drives_every_relay_once():
    b = board.build()
    coils = {}
    for p in b.circuit.parts:
        if p.value == "G6K-2F":
            coils.setdefault(check._net(p["COIL-"]), []).append(p.ref)
    assert len(coils) == 24 and all(len(v) == 1 for v in coils.values())
    assert set(controller.RELAYS) == {n.removeprefix("coil_") for n in coils}


def _part(ckt, value):
    found = [p for p in ckt.parts if p.value == value]
    assert len(found) == 1, (value, len(found))
    return found[0]


def _net(pin):
    return check._net(pin)


def _other_end(ckt, net, value_prefix="R"):
    """Nets reached from `net` through one resistor."""
    ends = set()
    for p in ckt.parts:
        if p.ref.startswith(value_prefix) and p.ref[1:].isdigit():
            a, b = _net(p[1]), _net(p[2])
            if a == net:
                ends.add(b)
            elif b == net:
                ends.add(a)
    return ends


def test_power_up_holds_every_relay_released_and_the_bias_feed_off():
    ckt = board.build().circuit
    assert "+3V3_D" in _other_end(ckt, "sr_oe")       # 74HC595 outputs disabled until the firmware drives /OE low
    assert "GND" in _other_end(ckt, "bias_en")        # SY6280 off
    assert "GND" in _other_end(ckt, "adc_rst")        # CS5361 held in reset


def test_usb_data_lines_reach_the_esp32_usb_pads_unswapped():
    ckt = board.build().circuit
    u, j = _part(ckt, "ESP32-S3-WROOM-1"), _part(ckt, "USB-C 16P")
    assert _net(u["IO20"]) == _net(j["A6"]) == _net(j["B6"])      # D+
    assert _net(u["IO19"]) == _net(j["A7"]) == _net(j["B7"])      # D−


def test_i2c_pullups_go_to_the_si5351_core_rail():
    ckt = board.build().circuit
    for line in ("i2c_sda", "i2c_scl"):
        assert _other_end(ckt, line) == {"+3V3_RF"}


def test_cs5361_mode_pins_select_double_speed_master_clock_divide_by_two():
    ckt = board.build().circuit
    adc = _part(ckt, "CS5361")
    assert _other_end(ckt, _net(adc["M0"])) == {"+3V3_D"}
    assert _other_end(ckt, _net(adc["M1"])) == {"GND"}
    assert _other_end(ckt, _net(adc["MDIV"])) == {"+3V3_D"}
    osc = _part(ckt, "24.576MHz")
    assert _net(adc["MCLK"]) in _other_end(ckt, _net(osc["OUT"]))


def test_relay_chain_bit_order_matches_the_firmware_table():
    """Bit k of the chain (74HC595 #k//8, output Q(k%8)) drives the coil RELAYS[k]."""
    ckt = board.build().circuit
    regs = {_net(p["SER"]): p for p in ckt.parts if p.value == "74HC595"}
    drivers = [p for p in ckt.parts if p.value == "ULN2803"]
    ser, k = "sr_ser", 0
    while ser in regs:
        reg = regs[ser]
        for q in "ABCDEFGH":
            link = _net(reg[f"Q{q}"])
            drv, ch = next((d, c) for d in drivers for c in range(1, 9) if _net(d[f"I{c}"]) == link)
            if k < len(controller.RELAYS):
                assert _net(drv[f"O{ch}"]) == f"coil_{controller.RELAYS[k]}", k
            k += 1
        ser = _net(reg["QHS"])
    assert k == 24
```

```diff

```

- [ ] **Step 2: Run to verify they fail**

Run: `cd hardware && python3 -m pytest -q tests/test_board.py`
Expected: collection ERROR: `cannot import name 'board' from 'neohw.blocks'`.

- [ ] **Step 3: Implement**

`hardware/neohw/blocks/board.py`:

```python
"""The whole receiver board (phase 3 spec §1 item 2): every block built into one circuit.

Blocks keep their own net names. Two nets with the same name in one SKiDL circuit become e.g.
"GND" and "GND1"; the board merges exactly the shared names in SHARED_NETS (rails, ground and
the buses that several blocks name the same way), and wires every other interface through the
blocks' ports. Relay coils go to the controller's driver outputs, the CS5361's digital pins to the
controller's ADC interface.
"""
from __future__ import annotations

import re

from skidl import Circuit, Net

from neohw import parts
from neohw.blocks import (Block, attenuator, bias_tee, bpf_bank, cal_tone, clock, controller, mixer, power,
                          preamp, protection)

SHARED_NETS = ("GND", "+5V_A", "+3V3_RF", "+3V3_D", "+5V_MAIN", "+5V_BIAS", "VBUS", "i2c_sda", "i2c_scl",
               "lo_path", "lo_s0", "lo_s1", "cal_clk", "bias_feed", "sense_vbus", "sense_ext", "sense_main")
# the board's default relay states: everything released (muted bank, attenuator out, preamp bypassed)
CS5361_DIGITAL = {"MCLK": ("adc", "mclk"), "SCLK": ("gpio", "i2s_bclk"), "LRCK": ("gpio", "i2s_ws"),
                  "SDOUT": ("gpio", "i2s_din"), "RST": ("gpio", "adc_rst"), "M/S": ("gpio", "adc_ms"),
                  "I2S/LJ": ("gpio", "adc_fmt"), "M0": ("adc", "m0"), "M1": ("adc", "m1"), "MDIV": ("adc", "mdiv"),
                  "HPF": ("adc", "hpf"), "OVFL": ("gpio", "adc_ovfl")}


def _merge_shared(ckt) -> None:
    """Join each SHARED_NETS name with the copies SKiDL made of it: GND1, GND2 … or, for names that
    end in a digit, lo_s0_1, lo_s0_2 …"""
    groups: dict[str, list] = {}
    for net in ckt.nets:
        for base in SHARED_NETS:
            rest = net.name[len(base):] if net.name.startswith(base) else None
            if rest is not None and (rest == "" or rest.isdigit() or (rest.startswith("_") and rest[1:].isdigit())):
                groups.setdefault(base, []).append(net)
    with ckt:
        for nets in groups.values():
            first = nets[0]
            for other in nets[1:]:
                first += other


def _relay(own_parts, predicate):
    found = [p for p in own_parts if p.value == "G6K-2F" and predicate(p)]
    if len(found) != 1:
        raise RuntimeError(f"expected one relay, found {len(found)}")
    return found[0]


def build() -> Block:
    ckt = Circuit()
    own: dict[str, list] = {}

    def built(name, make):
        before = set(map(id, ckt.parts))
        blk = make()
        own[name] = [p for p in ckt.parts if id(p) not in before]
        return blk
    pw = built("power", lambda: power.build(ckt))
    ctl = built("controller", lambda: controller.build(ckt))
    bt = built("bias_tee", lambda: bias_tee.build(ckt))
    att = built("attenuator", lambda: attenuator.build("released", ckt))
    prot = built("protection", lambda: protection.build(ckt))
    bank = built("bank", lambda: bpf_bank.build(None, ckt=ckt))
    pre = built("preamp", lambda: preamp.build("bypass", ckt=ckt))
    mx = built("mixer", lambda: mixer.build(ckt))
    clk = built("clock", lambda: clock.build(ckt))
    cal = built("cal_tone", lambda: cal_tone.build(ckt))
    with ckt:
        # RF chain: antenna → bias-T → attenuator → limiter → (cal tone) → filter bank → preamp → mixer
        bt.ports["rf_out"] += att.ports["rf_in"]
        att.ports["rf_out"] += prot.ports["rf_in"]
        prot.ports["rf_out"] += bank.ports["rf_in"]
        cal.ports["rf"] += bank.ports["rf_in"]
        bank.ports["rf_out"] += pre.ports["rf_in"]
        pre.ports["rf_out"] += mx.ports["rf_in"]
        # relay coils: COIL+ to 5V_MAIN, COIL− to the controller's driver output
        coils = ctl.ports["coil"]
        for k in range(1, 12):
            for end in ("in", "out"):
                rly = _relay(own["bank"], lambda p, k=k, end=end: p["COM1"].net.name == f"f{k}_{end}")
                rly["COIL+"] += pw.ports["main"]
                rly["COIL-"] += coils[f"bank{k}_{end}"]
        for name in ("atten", "preamp"):
            rly = _relay(own["attenuator" if name == "atten" else "preamp"], lambda p: True)
            rly["COIL+"] += pw.ports["main"]
            rly["COIL-"] += coils[name]
        # CS5361 digital side (the part is in the mixer block)
        adc = next(p for p in own["mixer"] if p.value == "CS5361")
        for pin, (group, name) in CS5361_DIGITAL.items():
            adc[pin] += ctl.ports[group][name]
        adc["VD"] += pw.ports["v3d"]
        adc["VL"] += pw.ports["v3d"]
        adc["VA"] += pw.ports["v5a"]
        for pin in ("GND2", "REFGND"):
            adc[pin] += adc["GND"].net
        filt = Net("adc_filt")
        adc["FILT+"] += filt
        for value, note in (("47u", "X5R FILT+ reserve (typical connection)"), ("10n", "C0G FILT+")):
            c = parts.capacitor(value, note=note)
            c[1] += filt
            c[2] += adc["GND"].net
        for value in ("10u", "100n"):
            for rail in (pw.ports["v3d"], pw.ports["v5a"]):
                c = parts.capacitor(value, note="CS5361 supply decoupling")
                c[1] += rail
                c[2] += adc["GND"].net
        # bias-T feed and the USB supply
        bt.ports["bias_feed"] += ctl.ports["bias_feed"]
        pw.ports["vbus"] += ctl.ports["vbus"]
    _merge_shared(ckt)
    return Block("board", ckt, {"parts_by_block": own, "power": pw, "controller": ctl, "bias_tee": bt, "attenuator": att, "protection": prot,
                                "bank": bank, "preamp": pre, "mixer": mx, "clock": clk, "cal_tone": cal})
```

`hardware/neohw/checks/board.py`:

```python
"""Board check (phase 3 spec §1 item 2): the whole receiver as one netlist. No net with a single
pin, no required pin left open, every IC supply pin on its rail and every ground pin on GND, every
footprint in the KiCad 9 libraries. Writes the board KiCad netlist and BOM, and board.json."""
from __future__ import annotations

from pathlib import Path

from neohw import availability, charfile, export
from neohw.blocks import board
from neohw.checks import BlockResult, CheckResult
from neohw.spice import _canonical

KICAD_FOOTPRINTS = Path("/usr/share/kicad/footprints")
# (part value, pin) -> rail the pin must be on
SUPPLY_RAILS = {
    ("Si5351C", "VDD"): "+3V3_RF", ("Si5351C", "VDDOA"): "+3V3_RF", ("Si5351C", "VDDOB"): "+3V3_RF",
    ("Si5351C", "VDDOC"): "+3V3_RF", ("Si5351C", "VDDOD"): "+3V3_RF", ("74LVC74", "VCC"): "+3V3_RF",
    ("74LVC157", "VCC"): "+3V3_RF", ("LM75A", "VCC"): "+3V3_RF", ("FST3253", "VCC"): "+5V_A",
    ("LM4562", "V+"): "+5V_A", ("CS5361", "VA"): "+5V_A", ("CS5361", "VD"): "+3V3_D", ("CS5361", "VL"): "+3V3_D",
    ("ESP32-S3-WROOM-1", "3V3"): "+3V3_D", ("74HC595", "VCC"): "+3V3_D", ("ULN2803", "COM"): "+5V_MAIN",
    ("G6K-2F", "COIL+"): "+5V_MAIN", ("MIC5205-5.0", "OUT"): "+5V_A", ("MIC5205-3.3", "OUT"): "+3V3_RF",
    ("TLV74033", "OUT"): "+3V3_D", ("USBLC6-2SC6", "VBUS"): "VBUS",
}
GROUND_PINS = ("GND", "GND2", "EPAD", "PAD", "V-", "REFGND")


def _net(pin):
    return _canonical(pin.net).name if pin.net is not None else None


def findings(ckt) -> dict:
    pins_on: dict[int, list] = {}
    for p in ckt.parts:
        for pin in p.pins:
            if pin.net is not None and pin.is_connected():
                n = _canonical(pin.net)
                pins_on.setdefault(id(n), [n, 0])[1] += 1
    single = sorted(n.name for n, c in pins_on.values() if c < 2)
    open_pins = sorted(f"{p.ref}.{pin.name}" for p in ckt.parts for pin in p.pins
                       if (pin.net is None or not pin.is_connected()) and pin.name not in getattr(p, "nc_pins", ()))
    rails = []
    for p in ckt.parts:
        for pin in p.pins:
            want = SUPPLY_RAILS.get((p.value, pin.name))
            if want and _net(pin) != want:
                rails.append(f"{p.ref} {p.value}.{pin.name} on {_net(pin)}, expected {want}")
            if pin.name in GROUND_PINS and p.value not in ("LM4562",) and _net(pin) not in ("GND", None):
                rails.append(f"{p.ref} {p.value}.{pin.name} on {_net(pin)}, expected GND")
            if p.value == "LM4562" and pin.name == "V-" and _net(pin) != "GND":
                rails.append(f"{p.ref} LM4562.V- on {_net(pin)}, expected GND (single supply)")
    missing = sorted({p.footprint for p in ckt.parts if not p.footprint.startswith("NeoRadio:")
                      and not (KICAD_FOOTPRINTS / f"{p.footprint.split(':')[0]}.pretty" / f"{p.footprint.split(':')[1]}.kicad_mod").exists()})
    return {"single_pin_nets": single, "open_pins": open_pins, "rail_errors": rails, "missing_footprints": missing}


def run(out_dir: Path, char_dir: Path) -> BlockResult:
    out_dir, char_dir = Path(out_dir), Path(char_dir)
    b = board.build()
    f = findings(b.circuit)
    data = availability.load()
    rows = export.bom_rows(b.circuit, data)
    semis = [r for r in rows if r["refs"].split()[0].rstrip("0123456789") in ("U", "Q", "D", "K", "X", "GDT")]
    unconfirmed = sorted(r["value"] for r in semis if r["availability"] != "available")
    checks = [
        CheckResult("parts", len(b.circuit.parts), "report", True, ""),
        CheckResult("single_pin_nets", len(f["single_pin_nets"]), "= 0", not f["single_pin_nets"], ""),
        CheckResult("open_required_pins", len(f["open_pins"]), "= 0", not f["open_pins"], ""),
        CheckResult("supply_rail_errors", len(f["rail_errors"]), "= 0", not f["rail_errors"], ""),
        CheckResult("missing_footprints", len(f["missing_footprints"]), "= 0", not f["missing_footprints"], ""),
        CheckResult("semiconductors_unconfirmed_on_torob", len(unconfirmed), "report (survey pending)", True, ""),
    ]
    result = BlockResult("board", checks)
    result.artefacts = {
        "kicad_netlist": export.kicad_netlist(b.circuit, out_dir / "board.net"),
        "bom": export.write_bom(b.circuit, out_dir / "board_bom.csv", data),
        "characterisation": charfile.write(char_dir / "board.json", "board", "pass" if result.passed else "fail", {
            "parts_by_block": {k: len(v) for k, v in b.ports["parts_by_block"].items()},
            "bom_lines": len(rows), "unconfirmed_semiconductors": unconfirmed, **f,
            "checks": [c.__dict__ for c in checks]}),
    }
    return result
```

```diff

```

- [ ] **Step 4: Run to verify they pass**

Run: `cd hardware && python3 -m pytest -q tests/test_board.py tests/test_cli.py`
Expected: `14 passed`.

Run: `cd hardware && python3 -c "from pathlib import Path; from neohw.checks import board as b; r=b.run(Path('build/plan33/out'),Path('build/plan33/char')); print([(x.name, x.value) for x in r.checks])"`
Expected: parts 471, single_pin_nets 0, open_required_pins 0, supply_rail_errors 0, missing_footprints 0, semiconductors_unconfirmed_on_torob 4 (24.576MHz, LED green, SS34, button).

- [ ] **Step 5: Commit**

```bash
git add hardware/neohw/blocks/board.py hardware/neohw/checks/board.py hardware/neohw/__main__.py hardware/tests/test_board.py hardware/tests/test_cli.py
git commit -m "feat(hw): board-level netlist with rail, pin and footprint checks"
```

### Task 4: Regenerate, verify, README

**Files:**
- Modify: `hardware/characterized/*` (controller.json, board.json new; generated_by stamps), `README.md`

- [ ] **Step 1:** Check RAM and swap (`free -g`), then `make hw-fresh`. Expected: every block PASS including `controller` and `board`.
- [ ] **Step 2:** Commit `hardware/characterized`; run cached `make hw`. Expected: PASS and `git status --short hardware/characterized` empty.
- [ ] **Step 3:** `make test` and `make sim`. Expected: all pass.
- [ ] **Step 4:** README: tick "ADC digital interface (CS5361, I2S)", "Controller and USB: ESP32-S3, USB-C, ESD protection", "Relay drivers, bias-T switching, current and temperature monitoring"; add feature lines for the board-level netlist checks and the bias-T current limit. Commit.
