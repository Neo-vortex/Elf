from skidl import Circuit

from neohw import parts


def test_fst3253_pins_follow_the_datasheet_and_section_2_may_stay_open():
    ckt = Circuit()
    with ckt:
        u = parts.fst3253()
    assert {p.name: p.num for p in u.pins}["1A"] == "7" and {p.name: p.num for p in u.pins}["VCC"] == "16"
    assert set(u.nc_pins) == {"2A", "2B0", "2B1", "2B2", "2B3"}
    assert "VCC must be 4.0-5.5 V" in u.spice_models[0]


def test_lm4562_dual_is_two_macromodel_instances():
    ckt = Circuit()
    with ckt:
        u = parts.lm4562_dual()
    n = {p.name: p.name.lower() for p in u.pins}
    assert u.spice_lines(n) == [f"X{u.ref}a +ina -ina outa v+ v- LM4562", f"X{u.ref}b +inb -inb outb v+ v- LM4562"]


def test_cs5361_input_is_7k5_differential_with_vq():
    ckt = Circuit()
    with ckt:
        u = parts.cs5361()
    n = {p.name: p.name.lower() for p in u.pins}
    lines = u.spice_lines(n)
    assert f"R{u.ref}_l ainl+ ainl- 7500" in lines and f"V{u.ref}_vq {u.ref}_vqi gnd dc 2.5" in lines
    assert {p.name: p.num for p in u.pins}["AINL+"] == "16"


def test_fst3253_ron_matches_the_datasheet_points(tmp_path):
    import pytest
    from skidl import Net
    from neohw.ngspice import run_decks
    from neohw.spice import Load, Supply, Op, to_deck
    from neohw.tools import ToolNotFoundError, tool_path
    try:
        tool_path("ngspice")
    except ToolNotFoundError as e:
        pytest.skip(str(e))

    def ron(vcc, vin, amps):
        ckt = Circuit()
        with ckt:
            u = parts.fst3253()
            nets = {name: Net(name.replace("/", "n")) for name in ("VCC", "S0", "S1", "/1OE", "/2OE", "1A", "1B0", "1B1", "1B2", "1B3")}
            gnd = Net("GND")
            for name, net in nets.items():
                u[name] += net
            u["GND"] += gnd
        terms = [Supply(nets["VCC"], vcc, "vcc"), Supply(nets["S0"], 0.0, "s0"), Supply(nets["S1"], 0.0, "s1"),
                 Supply(nets["/1OE"], 0.0, "oe1"), Supply(nets["/2OE"], vcc, "oe2"), Supply(nets["1A"], vin, "a")]
        terms += [Load(nets[f"1B{k}"], f"b{k}", 1e9) for k in (1, 2, 3)] + [Load(nets["1B0"], "b0", 1e9)]
        deck, raws = to_deck(ckt, "ron", terms, [Op()], extra_lines=[f"ISINK 1b0 0 dc {amps}"])
        (res,) = run_decks([deck], raws)
        d = res[raws[0]][0].data
        return (vin - d["v(1b0)"][0].real) / amps

    assert ron(4.5, 0.0, 64e-3) == pytest.approx(4.0, rel=0.25)    # datasheet 4 typ / 7 max
    assert 6.0 <= ron(4.5, 2.4, 15e-3) <= 15.0                       # 8 typ / 15 max
    assert 8.0 <= ron(4.0, 2.4, 15e-3) <= 20.0                       # 11 typ / 20 max
