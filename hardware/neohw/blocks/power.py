"""Power (phase 3 spec §3.2, plan 3.4 amendment, plan 5.1): the 9–20 V barrel jack is the only supply.

Input:
- Jack 9–20 V → resettable PTC fuse (hold 0.75 A, 33 V: it ignores the plug-in pulse, which
  the SS36's 80 A surge rating carries) → SS36 Schottky (a reversed plug conducts nothing; at the
  board's ≤ 0.35 A input current its 0.45 V drop costs little) → SMBJ20A TVS (stand-off 20 V, clamps
  a surge at 32.4 V at 18.6 A, under the buck's 60 V rating and the PTC's 33 V; the clamp is the
  lowest of the SMBJ parts whose stand-off reaches the 20 V input) → input LC (10 µH + 100 µF / 50 V
  electrolytic ‖ 1 µF / 50 V), which also keeps the buck's input pulses off the adapter cable.
  The PTC's rated 33 V is above the 20 V maximum input it must interrupt; during a surge it is in
  series ahead of the TVS.
- LM2576HVS-ADJ buck (52 kHz) → 100 µH (12 × 12 mm, Isat ≥ 2 A) → 5V_MAIN = 5.5 V (FB 3.48 k /
  1 k), 1000 µF / 16 V low-ESR electrolytic ‖ 22 µF ‖ 100 nF; SS36 catch diode.

Rails from 5V_MAIN:
- relay coils (ULN2803 COM and coils): ≈ 4.6 V on a coil, above the G6K-2F's must-operate
  voltage even at 50 °C;
- 5V_A (CS5361 VA, LM4562, FST3253, preamp, mixer bias): LC (10 µH + 22 µF ‖ 100 nF) → TPS7A2050
  (China) or MIC5205-5.0 (Iran) on the same SOT-23-5 pads;
- 3V3_RF (Si5351C, 74LVC74/157, LM75A): LC → TPS7A2033 (China) or MIC5205-3.3 (Iran), same pads;
  pin 4 is NC on the TPS7A20 and the MIC5205's BYP, whose 470 pF is fitted in Iran only;
- 3V3_D (STM32F411, CS5361 VL/VD, logic): AMS1117-3.3 (SOT-223, both populations; from 5.5 V it
  dissipates 0.33 W at 0.15 A, which the SOT-223 tab carries at ≈ 90 °C/W) with a 22 µF tantalum
  whose ESR it needs for stability;
- 5V_BIAS (the bias-T switch): LC from 5V_MAIN.

USB carries data only: VBUS reaches the USBLC6 and a sense divider (host present) and powers
nothing.

Monitoring: VBUS 1 M / 220 k (a live host never pushes more than ≈ 5 µA into an unpowered
MCU pin), the jack (after the SS36) 100 k / 10 k (20 V → 1.8 V, under the STM32F411's 3.3 V ADC),
5V_MAIN 100 k / 100 k, each to an ADC pin.
"""
from __future__ import annotations

from skidl import Circuit, Net

from neohw import parts
from neohw.blocks import Block
from neohw.population import only

BUCK_FB_TOP, BUCK_FB_BOTTOM = "3.48k", "1k"     # 1.23 V × (1 + 3.48) = 5.51 V
BUCK_C_OUT_UF = 1000           # LM2576 adjustable stability: ≥ 13 300 · 20 V / (5.5 V · 100 µH) = 484 µF
BUCK_L_DCR = 0.15              # 12 × 12 mm 100 µH power inductor (Isat ≥ 2 A)
VBUS_DIVIDER = (1e6, 220e3)    # high impedance: an unpowered board's MCU pin sees ≤ ≈ 5 µA from a live VBUS
CERAMIC_ESL = 1.0e-9           # 1206 / 0805 MLCC, mounted
CERAMIC_ESR = 5e-3
ELECTROLYTIC_ESL = 5e-9
LC_L = 10e-6                   # filter inductors: 1210 wirewound, SRF ≈ 30 MHz (2.5 pF), DCR ≈ 0.3 Ω
BUCK_L = 100e-6
SENSE_DIVIDERS = {"vbus": (VBUS_DIVIDER[0] + VBUS_DIVIDER[1]) / VBUS_DIVIDER[1], "ext": 11.0, "main": 2.0}   # node / ADC
BIAS_FILTER_NOTE = "bias-T feed filter, 1210 wirewound, rated ≥ 0.4 A"   # carries the SY6280's 0.39 A limit


def _r(value, a, b):
    r = parts.resistor(value)
    r[1] += a
    r[2] += b
    return r


def _c(value, a, b, note="X7R decoupling", footprint="Capacitor_SMD:C_0805_2012Metric", voltage="50 V"):
    c = parts.capacitor(value, note=note, footprint=footprint, esr=CERAMIC_ESR, esl=CERAMIC_ESL, voltage=voltage)
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
    _c(value, node, gnd, note, "Capacitor_SMD:C_1206_3216Metric", "16 V")
    _c("100n", node, gnd, voltage="50 V")


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
        f = parts.ptc_resettable()
        f["1"] += tip
        f["2"] += fused
        d_rev = parts.ss36()
        d_rev["A"] += fused
        d_rev["K"] += ext_p
        tvs = parts.smbj20a()
        tvs["K"] += ext_p
        tvs["A"] += gnd
        _filter_l(ext_p, vin, "buck input filter, 1210 wirewound, rated ≥ 0.6 A")
        _elec("100u", vin, gnd, 0.3, "aluminium electrolytic 50 V, low ESR")
        _c("1u", vin, gnd, "X7R 50 V", "Capacitor_SMD:C_1206_3216Metric", "50 V")
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
        lb = parts.inductor(BUCK_L, 20.0, 50e3, c_par=10e-12, dcr=BUCK_L_DCR, tol=0.2,
                            footprint="Inductor_SMD:L_12x12mm_H8mm",
                            note="buck inductor, 12×12 mm shielded power, Isat ≥ 2 A, DCR ≤ 0.15 Ω (check the datasheet: "
                                 "listing titles overstate small parts)")
        lb[1] += sw
        lb[2] += main
        _r(BUCK_FB_TOP, main, fb)
        _r(BUCK_FB_BOTTOM, fb, gnd)
        c_out = parts.capacitor(f"{BUCK_C_OUT_UF}u", note="aluminium electrolytic 16 V, low ESR 0.05–0.2 Ω (LM2576 stability)",
                                footprint="Capacitor_SMD:CP_Elec_10x10.5", esr=0.1, esl=ELECTROLYTIC_ESL, tol=0.2)
        c_out[1] += main
        c_out[2] += gnd
        _bulk(main, gnd)
        # 5V_A: LC and the low-noise LDO
        _filter_l(main, ldo_a_in)
        _bulk(ldo_a_in, gnd)
        la = parts.rf_ldo(5.0)
        la["IN"] += ldo_a_in
        la["EN"] += ldo_a_in
        la["GND"] += gnd
        la["NC"] += byp_a
        la["OUT"] += v5a
        only(_c("470p", byp_a, gnd, "C0G noise bypass (MIC5205 only; pin 4 is NC on the TPS7A20)", voltage="50 V"), "iran")
        _c("10u", v5a, gnd, "X5R output", "Capacitor_SMD:C_1206_3216Metric", "16 V")
        _c("100n", v5a, gnd, voltage="50 V")
        # 3V3_RF
        _filter_l(main, rf_in)
        _bulk(rf_in, gnd)
        lr = parts.rf_ldo(3.3)
        lr["IN"] += rf_in
        lr["EN"] += rf_in
        lr["GND"] += gnd
        lr["NC"] += byp_rf
        lr["OUT"] += v3rf
        only(_c("470p", byp_rf, gnd, "C0G noise bypass (MIC5205 only; pin 4 is NC on the TPS7A20)", voltage="50 V"), "iran")
        _c("10u", v3rf, gnd, "X5R output", "Capacitor_SMD:C_1206_3216Metric", "16 V")
        _c("100n", v3rf, gnd, voltage="50 V")
        # 3V3_D: AMS1117 + tantalum, both populations
        ld = parts.ams1117_33()
        ld["IN"] += main
        ld["GND"] += gnd
        ld["OUT"] += v3d
        c_ta = parts.capacitor("22u", footprint="Capacitor_Tantalum_SMD:CP_EIA-3528-21_Kemet-B", esr=0.5,
                               note="tantalum 10 V: AMS1117 stability needs its ESR")
        c_ta[1] += v3d
        c_ta[2] += gnd
        _c("100n", v3d, gnd, voltage="50 V")
        # bias-T supply
        _filter_l(main, bias5, BIAS_FILTER_NOTE)
        _bulk(bias5, gnd)
        # monitoring
        _r("1M", vbus, s_vbus)
        _r("220k", s_vbus, gnd)
        _c("100n", s_vbus, gnd, "X7R ADC filter", voltage="50 V")
        _r("100k", ext_p, s_ext)
        _r("10k", s_ext, gnd)
        _c("100n", s_ext, gnd, "X7R ADC filter", voltage="50 V")
        _r("100k", main, s_main)
        _r("100k", s_main, gnd)
        _c("100n", s_main, gnd, "X7R ADC filter", voltage="50 V")
    return Block("power", ckt, {"vbus": vbus, "ext_tip": tip, "ext_p": ext_p, "buck_in": vin, "sw": sw, "main": main,
                                "ldo_a_in": ldo_a_in, "ldo_rf_in": rf_in, "v5a": v5a, "v3rf": v3rf, "v3d": v3d,
                                "bias5": bias5, "sense_vbus": s_vbus, "sense_ext": s_ext, "sense_main": s_main})
