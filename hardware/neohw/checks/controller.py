"""Controller check (phase 3 spec §3.3): bias-T switch and current sense in SPICE, once per population
(INA180A1 for China, BC857B mirror for Iran: transfer table for the firmware, temperature drift,
current limit, ADC range, and the zero-reading split the firmware tells them apart by), ESP32-S3 GPIO
rules, I2C address map and relay-driver budget. Writes controller.json (the GPIO map and the two
sense tables the firmware uses)."""
from __future__ import annotations

from pathlib import Path

from neohw import availability, charfile, export, parts, population
from neohw.blocks import bias_switch, controller
from neohw.checks import BlockResult, CheckResult
from neohw.ngspice import run_decks
from neohw.spice import Load, Op, node_name, to_deck

LOAD_OHMS = (1e7, 250, 100, 50, 40, 33, 31, 29, 25)   # 0 … ~190 mA from 5 V; finer near the 150 mA cut-off, where the INA180 turns into its limit
TEMPS_C = (0.0, 27.0, 50.0)
ADC_MAX_V = 3.3                                  # STM32F411 ADC range, 0–VDDA (replaces the ESP32-S3; plan 5.2 changes the controller)
ZERO_SPLIT_V = 0.3                               # zero reading below: INA180A1 (≈ 0 V); above: BC857B mirror (≈ 0.63 V)
SATURATED_STEP_V = 1e-3                          # a table point that rises less than this over the last is on the clamp
V3D_V = 3.3                                      # INA180 supply (+3V3_D)
SENSE_TABLE_KEYS = {"china": "ina180", "iran": "mirror"}
I_LIM_A = (0.30, 0.50)                           # SY6280 at R_SET 17.4 kΩ: 6800/17400 = 0.39 A nominal
DRIFT_MAX = 0.03                                 # reading drift 0–50 °C at ≈ 150 mA, relative
I2C_ADDRESSES = {"Si5351C": 0x60, "LM75A": 0x48}
RELAY_COIL_A = 0.028                             # G6K-2F 5 VDC coil (178 Ω)
RELAYS_ON_MAX = 4                                # selected filter's 2 + attenuator + preamp
ULN2803_CHANNEL_A = 0.5
# relay pull-in: G6K-2F-5VDC must operate at 75 % of rated (3.75 V) at 23 °C; coil copper +0.393 %/K;
# ULN2803 Darlington saturation ≈ 0.9 V (datasheet typical at 100 mA; conservative at 28 mA)
RELAY_MUST_OPERATE_V, ULN2803_VCE_SAT_V, COIL_TC, RELAY_HOT_C = 3.75, 0.9, 0.00393, 50.0
PLOT_X_LABEL, PLOT_X_SCALE = "bias-T current (mA)", 1e-3


def bias_sense(pop: str = population.DEFAULT) -> dict:
    """The bias-T sense of the circuit fitted in population `pop`, at each temperature."""
    blk = bias_switch.build()
    P = {k: node_name(v) for k, v in blk.ports.items()}
    cases, decks = [], []
    for t in TEMPS_C:
        for r in (*LOAD_OHMS, 1.0):
            decks.append(to_deck(blk.circuit, "bias sense", [Load(blk.ports["sense"], "adc", 1e7),
                                                              Load(blk.ports["feed"], "ant", 1e7)], [Op()],
                                 extra_lines=[f"VB {P['bias5']} 0 dc 5.0", f"VEN {P['en']} 0 dc 3.3",
                                              *([f"VV3 {P['v3d']} 0 dc {V3D_V:g}"] if pop == "china" else []),
                                              f"RLD {P['feed']} 0 {r:g}", f".options temp={t:g}"],
                                 population=pop)[0])
            cases.append((t, r))
    out: dict = {"table": {}, "short": {}}
    for (t, r), res in zip(cases, run_decks(decks, ["a0.raw"])):
        d = res["a0.raw"][0].data
        v_feed, v_sense = d[f"v({P['feed']})"][0].real, d[f"v({P['sense']})"][0].real
        if r == 1.0:
            out["short"][t] = {"i_a": v_feed / r, "sense_v": v_sense}
        else:
            out["table"].setdefault(t, []).append((v_feed / r, v_sense))
    for tab in out["table"].values():       # a saturated output is not invertible: keep the first point on the clamp
        while len(tab) > 1 and tab[-1][1] <= tab[-2][1] + SATURATED_STEP_V:
            tab.pop()
    return out


def coil_heat_factor(t_c: float) -> float:
    return 1 + COIL_TC * (t_c - 23.0)


def relay_coil_margin(main_v_by_corner: dict[str, float], t_c: float = RELAY_HOT_C) -> dict:
    """Coil voltage at each supply corner against the must-operate voltage at t_c."""
    need = RELAY_MUST_OPERATE_V * coil_heat_factor(t_c)
    return {k: {"main_v": v, "coil_v": v - ULN2803_VCE_SAT_V, "need_v": need, "margin_v": v - ULN2803_VCE_SAT_V - need}
            for k, v in main_v_by_corner.items()}


def _supply_corners() -> dict[str, float]:
    import json
    sc = json.loads((Path(__file__).resolve().parents[2] / "characterized" / "power.json").read_text())["scenarios"]
    return {name: s["main"] for name, s in sc.items() if "maximum load" in name}


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


def _sense_checks(pop: str, bs: dict) -> list[CheckResult]:
    """China keeps the original check names; the Iran mirror's carry `iran`."""
    tag = "" if pop == "china" else "iran_"
    room = bs["table"][27.0]
    zero = room[0][1]
    near150 = {t: min(tab, key=lambda iv: abs(iv[0] - 0.15)) for t, tab in bs["table"].items()}
    drift = max(abs((near150[t][1] - bs["table"][t][0][1]) / (near150[27.0][1] - zero) - 1) for t in TEMPS_C)
    monotonic = all(all(b[1] > a[1] for a, b in zip(tab, tab[1:])) for tab in bs["table"].values())
    v_max = max([v["sense_v"] for v in bs["short"].values()] + [v for tab in bs["table"].values() for _, v in tab])
    i_short = bs["short"][27.0]["i_a"]
    return [
        CheckResult(f"bias_sense_{tag}monotonic", float(monotonic), "= 1", monotonic, ""),
        CheckResult(f"bias_sense_{tag}zero_v", zero, "report (subtracted by the firmware, feed off)", True, "V"),
        CheckResult(f"bias_sense_{tag}drift_0_50c", drift * 100, f"≤ {DRIFT_MAX * 100:g}", drift <= DRIFT_MAX, "%"),
        CheckResult(f"bias_short_current_{tag}a", i_short, f"{I_LIM_A[0]:g}–{I_LIM_A[1]:g}", I_LIM_A[0] <= i_short <= I_LIM_A[1], "A"),
        CheckResult(f"bias_sense_{tag}max_v", v_max, f"≤ {ADC_MAX_V:g}", v_max <= ADC_MAX_V, "V"),
    ]


def run(out_dir: Path, char_dir: Path) -> BlockResult:
    out_dir, char_dir = Path(out_dir), Path(char_dir)
    sense = {pop: bias_sense(pop) for pop in population.POPULATIONS}
    zero_v = {pop: sense[pop]["table"][27.0][0][1] for pop in population.POPULATIONS}
    bad = gpio_findings()
    coil = relay_coil_margin(_supply_corners())
    addrs = list(I2C_ADDRESSES.values())
    coil_total = RELAY_COIL_A * RELAYS_ON_MAX
    checks = [
        *[c for pop in population.POPULATIONS for c in _sense_checks(pop, sense[pop])],
        CheckResult("bias_sense_zero_split", max(zero_v["china"], 0.0), f"INA180 zero < {ZERO_SPLIT_V:g} V < mirror zero",
                    zero_v["china"] < ZERO_SPLIT_V < zero_v["iran"], "V"),
        CheckResult("gpio_rule_violations", len(bad), "= 0", not bad, ""),
        CheckResult("i2c_addresses_unique", float(len(set(addrs)) == len(addrs)), "= 1", len(set(addrs)) == len(addrs), ""),
        CheckResult("relay_coil_margin_v", min(m["margin_v"] for m in coil.values()),
                    f"≥ 0 (G6K-2F must-operate at {RELAY_HOT_C:g} °C, every supply corner)",
                    min(m["margin_v"] for m in coil.values()) >= 0, "V"),
        CheckResult("relay_coil_current_a", coil_total, f"per channel ≤ {ULN2803_CHANNEL_A:g}", RELAY_COIL_A <= ULN2803_CHANNEL_A, "A"),
    ]
    checks = [CheckResult(c.name, float(c.value), c.limit, bool(c.passed), c.unit) for c in checks]   # plain JSON types
    result = BlockResult("controller", checks, plots={
        f"bias sense {pop} {t:g} °C (V)": ([i for i, _ in tab], [v for _, v in tab])
        for pop in population.POPULATIONS for t, tab in sense[pop]["table"].items()},
        x_label=PLOT_X_LABEL, x_scale=PLOT_X_SCALE)
    blk = controller.build()
    data = availability.load()
    result.artefacts = {
        "kicad_netlist": export.kicad_netlist(blk.circuit, out_dir / "controller.net"),
        "bom": export.write_bom(blk.circuit, out_dir / "controller_bom.csv", data),
        "svg": export.svg_diagram(blk.circuit, out_dir / "controller.svg"),
        "characterisation": charfile.write(char_dir / "controller.json", "controller", "pass" if result.passed else "fail", {
            "gpio": controller.GPIO, "relay_chain_order": list(controller.RELAYS), "i2c": I2C_ADDRESSES,
            "bias_sense_tables_27c": {SENSE_TABLE_KEYS[pop]: [{"i_a": float(i), "sense_v": float(v)}
                                                              for i, v in sense[pop]["table"][27.0]]
                                      for pop in population.POPULATIONS},
            "bias_sense_zero_split_v": ZERO_SPLIT_V,
            "bias_cutoff_a": 0.15, "gpio_findings": bad,
            "checks": [c.__dict__ for c in checks]}),
    }
    return result
