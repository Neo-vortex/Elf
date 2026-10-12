import numpy as np
import pytest
from skidl import Circuit, Net

from neohw import parts
from neohw.blocks import Block, attenuator
from neohw.spice import Noise, Op, Port, Source, Sp, SpiceError, Stage, cascade_to_deck, node_name, to_deck
from neohw.tools import ToolNotFoundError, tool_path


def divider():
    ckt = Circuit()
    with ckt:
        a, mid, gnd = Net("a"), Net("mid"), Net("GND")
        r1 = parts.resistor("1k"); r1[1] += a; r1[2] += mid
        r2 = parts.resistor("2k"); r2[1] += mid; r2[2] += gnd
    return Block("div", ckt, {"a": a, "mid": mid})


def test_override_replaces_one_parts_model():
    blk = divider()
    r1, r2 = sorted(blk.circuit.parts, key=lambda p: p.ref)
    deck, _ = to_deck(blk.circuit, "x", [Port(blk.ports["a"], 1), Port(blk.ports["mid"], 2)], [Op()],
                      overrides={r1.ref: lambda n: [f"RSTAND {n['1']} {n['2']} 5 noisy=0"]})
    assert "RSTAND a mid 5 noisy=0" in deck
    assert f"R{r1.ref} " not in deck and f"R{r2.ref} mid 0 2000" in deck


def test_noise_can_take_a_differential_output():
    blk = divider()
    src = Source(blk.ports["a"], "in")
    cmd = Noise(blk.ports["a"], src, 1e3, 4e4, 10, ref_net=blk.ports["mid"]).command()
    assert cmd.startswith("noise v(a, mid) VS_in0 dec 10 1000 40000")
    assert Noise(blk.ports["a"], src, 1e3, 4e4).command().startswith("noise v(a) VS_in0")


def test_a_string_is_already_a_node_name():
    assert node_name("mix_lo_s0") == "mix_lo_s0"


def test_cascade_prefixes_private_nodes_renames_refs_and_restores_them():
    a, b = divider(), divider()
    refs = sorted(p.ref for p in a.circuit.parts)
    deck, _ = cascade_to_deck([Stage(a, "s1", {"a": "in", "mid": "link"}), Stage(b, "s2", {"a": "link"})],
                              "x", [Port("in", 1), Port("s2_mid", 2)], [Op()])
    assert f"R{refs[0]}_s1 in link 1000" in deck and f"R{refs[1]}_s1 link 0 2000" in deck
    assert f"R{refs[0]}_s2 link s2_mid 1000" in deck
    assert sorted(p.ref for p in a.circuit.parts) == refs


def test_cascade_rejects_repeated_tags():
    a, b = divider(), divider()
    with pytest.raises(SpiceError, match="unique"):
        cascade_to_deck([Stage(a, "s", {}), Stage(b, "s", {})], "x", [], [Op()])


def test_cascade_still_rejects_a_dangling_net_no_port_drives():
    a, b = divider(), divider()
    # stage 2's "mid" is neither joined nor driven by a port: it only has... two pins, but "a" has one
    with pytest.raises(SpiceError, match="single connection"):
        cascade_to_deck([Stage(a, "s1", {"a": "in", "mid": "link"}), Stage(b, "s2", {})], "x",
                        [Port("in", 1)], [Op()])


@pytest.mark.integration
def test_two_attenuators_in_cascade_give_40_db():
    try:
        tool_path("ngspice")
    except ToolNotFoundError as e:
        pytest.skip(str(e))
    from neohw.ngspice import run_deck
    p1, p2 = attenuator.build("energised"), attenuator.build("energised")
    deck, raws = cascade_to_deck([Stage(p1, "a1", {"rf_in": "in", "rf_out": "mid"}),
                                  Stage(p2, "a2", {"rf_in": "mid", "rf_out": "out"})],
                                 "pads", [Port("in", 1), Port("out", 2)], [Sp(1e6, 10e6, 3)])
    s21 = run_deck(deck, raws)[raws[0]][0].data["s_2_1"]
    assert 20 * np.log10(np.abs(s21)) == pytest.approx([-39.92] * 3, abs=0.05)
