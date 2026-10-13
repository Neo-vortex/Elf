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
PLOT_X_LABEL, PLOT_X_SCALE = "bias-T current (mA)", 1e-3


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
        f"bias sense {t:g} °C (V)": ([i for i, _ in tab], [v for _, v in tab]) for t, tab in bs["table"].items()},
        x_label=PLOT_X_LABEL, x_scale=PLOT_X_SCALE)
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
