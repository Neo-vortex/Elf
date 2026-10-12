"""Power (phase 3 spec §3.2).

Inputs:
- USB VBUS → 1 A fuse → Q1a/Q1b (two AO3401 back to back, common source) → 5V_MAIN. Their
  gates are the external-supply node: with external power present both are off and the
  opposing body diodes block current both ways (no USB draw, no back-feed into the host).
  The switch is hard, so pulling the jack hands over to USB without a dip; the price is a USB
  plug-in inrush into the board's ≈ 220 µF (reported by the check; a soft start on this shared
  gate would slow the hand-over too, so a dedicated USB load switch is a Rev A decision).
- Barrel jack 5 V → 1 A fuse → Q2 (AO3401, gate to ground: a reversed plug leaves it off, so
  nothing conducts) → EXT_P (SMBJ5.0A clamp) → SS34 Schottky → 5V_MAIN. The Schottky stops
  MAIN back-feeding the jack (and a PSU shared with the host).
- EXT_P is pulled down by 1 kΩ (25 mW with the jack in use) so Q1 conducts when no jack is
  plugged in, even with the SS34's hot reverse leakage from 5V_MAIN (mA at 50 °C) flowing into it.

Rails:
- 5V_A (CS5361 VA, LM4562, FST3253, preamp, mixer bias): MT3608 boost to 7.0 V (FB 107 k / 10 k)
  → two LC stages (10 µH + 22 µF ‖ 100 nF each) → MIC5205-5.0.
- 3V3_RF (Si5351C, 74LVC74/157): LC (10 µH + 22 µF ‖ 100 nF) → MIC5205-3.3.
- 3V3_D (ESP32-S3, CS5361 VL/VD, logic): TLV74033 low-dropout regulator (0.4 V worst case at
  our load: an AMS1117's 1.3 V maximum would leave 3V3_D below the CS5361 VD minimum with the
  external supply at 4.75 V).
- BIAS_5V (to the bias-T switch, plan 3.3): LC from 5V_MAIN (10 µH + 22 µF), keeping the boost's
  input ripple off the antenna feed.

Monitoring: 100 k / 100 k dividers from VBUS, EXT_P and 5V_MAIN to ADC ports.
"""
from __future__ import annotations

from skidl import Circuit, Net

from neohw import parts
from neohw.blocks import Block

BOOST_FB_TOP, BOOST_FB_BOTTOM = "107k", "10k"     # 0.6 V × (1 + 107/10) = 7.02 V
FUSE_A = 1.0
CERAMIC_ESL = 1.0e-9           # 1206 / 0805 MLCC, mounted
CERAMIC_ESR = 5e-3
LC_L = 10e-6                   # filter inductors: 1210 wirewound, SRF ≈ 30 MHz (2.5 pF), DCR ≈ 0.3 Ω


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


def _bulk(node, gnd, value="22u", note="X5R 16 V bulk"):
    _c(value, node, gnd, note, "Capacitor_SMD:C_1206_3216Metric")
    _c("100n", node, gnd)


def _filter_l(a, b, note="filter, 1210 wirewound, rated ≥ 300 mA"):
    l = parts.inductor(LC_L, 20.0, 1e6, c_par=2.5e-12, dcr=0.3, tol=0.2,
                       footprint="Inductor_SMD:L_1210_3225Metric", note=note)
    l[1] += a
    l[2] += b
    return l


def _sense(node, out, gnd):
    _r("100k", node, out)
    _r("100k", out, gnd)
    _c("100n", out, gnd, "X7R ADC filter")


def build(ckt: Circuit | None = None) -> Block:
    ckt = ckt or Circuit()   # a shared circuit when the board netlist builds every block
    with ckt:
        gnd = Net("GND")
        vbus, vbus_f, main = Net("VBUS"), Net("vbus_f"), Net("+5V_MAIN")
        tip, ext_raw, ext_p = Net("ext_tip"), Net("ext_raw"), Net("ext_p")
        boost_in, sw, v7, n7a, ldo_a_in = Net("boost_in"), Net("boost_sw"), Net("+7V"), Net("v7_f1"), Net("ldo_a_in")
        rf_in, v5a, v3rf, v3d, fb, bias5 = Net("ldo_rf_in"), Net("+5V_A"), Net("+3V3_RF"), Net("+3V3_D"), Net("boost_fb"), Net("+5V_BIAS")
        s_vbus, s_ext, s_main = Net("sense_vbus"), Net("sense_ext"), Net("sense_main")
        byp_a, byp_rf = Net("ldo_a_byp"), Net("ldo_rf_byp")
        # USB input
        f1 = parts.fuse_1206(FUSE_A)
        f1["1"] += vbus
        f1["2"] += vbus_f
        q1a, q1b, mid = parts.ao3401(), parts.ao3401(), Net("usb_mid")
        q1a["D"] += vbus_f
        q1a["S"] += mid
        q1a["G"] += ext_p
        q1b["D"] += main
        q1b["S"] += mid
        q1b["G"] += ext_p
        _r("1M", mid, ext_p)              # holds Vgs at 0 (off) when no source drives the gates
        # external input
        j = parts.barrel_jack()
        j["TIP"] += tip
        j["SLEEVE"] += gnd
        f2 = parts.fuse_1206(FUSE_A)
        f2["1"] += tip
        f2["2"] += ext_raw
        q2 = parts.ao3401()
        q2["D"] += ext_raw
        q2["S"] += ext_p
        q2["G"] += gnd
        tvs = parts.smbj5()
        tvs["K"] += ext_p
        tvs["A"] += gnd
        _r("1k", ext_p, gnd)
        d1 = parts.ss34()
        d1["A"] += ext_p
        d1["K"] += main
        _bulk(main, gnd)
        # boost to 7 V
        _filter_l(main, boost_in, "boost input filter, 1210 wirewound, rated ≥ 500 mA")
        _bulk(boost_in, gnd)
        b = parts.mt3608()
        b["IN"] += boost_in
        b["EN"] += boost_in
        b["GND"] += gnd
        b["FB"] += fb
        lb = parts.inductor(10e-6, 20.0, 1.2e6, c_par=5e-12, dcr=0.05, tol=0.2,
                            footprint="Inductor_SMD:L_7.3x7.3_H3.5", note="boost inductor, 7×7 mm power, Isat ≥ 1.5 A")
        lb.dc_open = True                  # averaged DC model: the MT3608 model replaces the switching
        lb[1] += boost_in
        lb[2] += sw
        b["SW"] += sw
        d2 = parts.ss34()
        d2["A"] += sw
        d2["K"] += v7
        _r(BOOST_FB_TOP, v7, fb)
        _r(BOOST_FB_BOTTOM, fb, gnd)
        _c("22u", v7, gnd, "X5R 16 V boost output", "Capacitor_SMD:C_1206_3216Metric")
        _bulk(v7, gnd)
        # 5V_A: two LC stages and the low-noise LDO
        _filter_l(v7, n7a)
        _bulk(n7a, gnd)
        _filter_l(n7a, ldo_a_in)
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
        ld = parts.tlv74033()
        ld["IN"] += main
        ld["EN"] += main
        ld["GND"] += gnd
        ld["OUT"] += v3d
        _c("10u", v3d, gnd, "X5R output", "Capacitor_SMD:C_1206_3216Metric")
        _c("100n", v3d, gnd)
        # bias-T supply
        _filter_l(main, bias5, "bias-T feed filter, 1210 wirewound, rated ≥ 200 mA")
        _bulk(bias5, gnd)
        # monitoring
        _sense(vbus_f, s_vbus, gnd)
        _sense(ext_p, s_ext, gnd)
        _sense(main, s_main, gnd)
    return Block("power", ckt, {"vbus": vbus, "ext_tip": tip, "main": main, "ext_p": ext_p, "v7": v7,
                                "ldo_a_in": ldo_a_in, "ldo_rf_in": rf_in, "v5a": v5a, "v3rf": v3rf, "v3d": v3d,
                                "bias5": bias5, "boost_in": boost_in, "sw": sw,
                                "sense_vbus": s_vbus, "sense_ext": s_ext, "sense_main": s_main})
