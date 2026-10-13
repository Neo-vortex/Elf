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
    "sense_vbus": "IO1", "sense_ext": "IO2", "sense_main": "IO4", "bias_isense": "IO5",
    "i2c_sda": "IO8", "i2c_scl": "IO9",
    "i2s_bclk": "IO10", "i2s_ws": "IO11", "i2s_din": "IO12",
    "adc_rst": "IO13", "adc_ovfl": "IO14", "adc_ms": "IO41", "adc_fmt": "IO42",
    "lo_path": "IO15",
    "sr_ser": "IO16", "sr_clk": "IO17", "sr_latch": "IO18", "sr_oe": "IO21",
    "bias_en": "IO38", "status_led": "IO40",
}
ADC_FUNCTIONS = ("sense_vbus", "sense_ext", "sense_main", "bias_isense")
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
        # (no CC taps to the ESP32: the jack powers the board, and a live host must not drive an
        # unpowered ESP32 pin)
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
