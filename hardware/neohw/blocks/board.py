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
        adc.nc_pins = ()                  # the mixer block left these to the board: every pin is checked now
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
