import json
import math
from pathlib import Path

import numpy as np
import pytest

from neohw.blocks import mixer
from neohw.checks import mixer as check
from neohw.rfmath import K_B, T0, T_SIM
from neohw.tools import ToolNotFoundError, tool_path

CHAR = Path(__file__).resolve().parents[1] / "characterized"


@pytest.fixture
def ngspice():
    try:
        tool_path("ngspice")
    except ToolNotFoundError as e:
        pytest.skip(str(e))


def test_noise_figure_definition():
    """N_I + N_Q = 4·R·G²·k·T0 doubles the noise factor: 3.01 dB."""
    g = 64.0
    n = 4 * 50 * g ** 2 * K_B * T0
    assert check.noise_figure_db(n / 2, n / 2, g) == pytest.approx(10 * math.log10(2), abs=1e-9)
    assert check.noise_figure_db(0.0, 0.0, g) == 0.0


def test_standin_source_ratio_is_pi2_over_8_for_an_ideal_sampler():
    g = 64.0
    src = math.pi ** 2 / 8 * 4 * 50 * g ** 2 * K_B * T_SIM
    noise = {"added": {"i": 1e-15, "q": 1e-15}, "with_source": {"i": 1e-15 + src / 2, "q": 1e-15 + src / 2}}
    assert check.standin_source_ratio(noise, g) == pytest.approx(math.pi ** 2 / 8)


def test_standin_branches_are_4rs_noiseless_and_4ron_noisy():
    nodes = {"1A": "mix_a", **{f"1B{k}": f"mix_b{k}" for k in range(4)}}
    lines = check.noise_standin("U1", 7.5)(nodes)
    assert lines[0] == "VU1_bias mix_a 0 dc 2.5"
    assert "RU1_s0 mix_a U1_x0 200 noisy=0" in lines and "RU1_on3 U1_x3 mix_b3 30" in lines
    assert all("noisy" not in x for x in check.noise_standin("U1", 7.5, source_noisy=True)(nodes))


def test_bypass_correction_is_the_ratio_to_the_product_model():
    f = np.array([13.9e6, 14.1e6])
    s21 = np.array([0.8 + 0.1j, 0.8 + 0.1j])
    alone = {o: 60.0 * np.exp(1j * o * 1e-5) for o in check.OFFSETS_HZ}
    chain = {o: 0.5 * s21[0] * v for o, v in alone.items()}
    c = check.bypass_correction(chain, alone, f, s21, 14e6)
    assert c["offset_hz"] == sorted(check.OFFSETS_HZ)
    assert c["re"] == pytest.approx([0.5] * len(c["re"])) and c["im"] == pytest.approx([0.0] * len(c["im"]), abs=1e-12)
    assert c["mean_db"] == pytest.approx(20 * math.log10(0.5))


def test_chain_deck_drives_the_bank_input_and_the_mixer_selects(ngspice):
    deck, raws = check.chain_deck(mixer.build(), 7, 13.856e6)
    assert raws == ["a0.raw"]
    assert any(x.startswith("RS_rf ") and x.endswith(" chain_in 50") for x in deck.splitlines())
    assert any(x.startswith("VLO_s0 mix_lo_s0 ") for x in deck.splitlines())
    assert ".ic v(mix_mix_a)=2.5" in deck and "v(mix_ain_q_n)" in deck


@pytest.mark.integration
def test_fst3253_ron_at_the_bias(ngspice):
    assert check.fst3253_ron() == pytest.approx(7.64, abs=0.05)


@pytest.mark.integration
def test_noise_standin_gives_the_prototype_noise_figure(ngspice):
    noise = check.noise_runs(mixer.build(), check.fst3253_ron())
    assert noise["added"]["i"] == pytest.approx(noise["added"]["q"], rel=1e-6)
    gain = 10 ** (json.loads((CHAR / "mixer.json").read_text())["conv_gain_db"] / 20)
    assert check.noise_figure_db(noise["added"]["i"], noise["added"]["q"], gain) == pytest.approx(7.24, abs=0.1)
    ratio_db = 10 * math.log10(check.standin_source_ratio(noise, gain) / (math.pi ** 2 / 8))
    assert abs(ratio_db) <= check.STANDIN_TOL_DB
