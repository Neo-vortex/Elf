import shutil
from pathlib import Path

import numpy as np
import pytest

from neohw.ngspice import NgspiceError, run_deck
from neohw.tools import ToolNotFoundError, tool_path


def _need_ngspice():
    try:
        tool_path("ngspice")
    except ToolNotFoundError as e:
        pytest.skip(str(e))


RC = """rc
V1 in 0 dc 0 ac 1
R1 in out 1k
C1 out 0 159.155n
.control
set filetype=ascii
ac dec 10 100 10k
write ac.raw all
.endc
.end
"""


@pytest.mark.integration
def test_rc_lowpass_is_3db_down_at_corner():
    _need_ngspice()
    plots = run_deck(RC, ["ac.raw"])
    (ac,) = plots["ac.raw"]
    f = ac.data["frequency"].real
    gain = np.abs(ac.data["v(out)"])
    k = np.argmin(np.abs(f - 1000))
    assert 20 * np.log10(gain[k]) == pytest.approx(-3.01, abs=0.05)


@pytest.mark.integration
def test_failed_simulation_reports_log_tail(tmp_path):
    _need_ngspice()
    bad = RC.replace("R1 in out 1k", "R1 in out")
    with pytest.raises(NgspiceError) as err:
        run_deck(bad, ["ac.raw"], keep_dir=tmp_path)
    assert "deck.cir" in str(err.value)
    assert "log" in str(err.value).lower()


@pytest.mark.integration
def test_missing_rawfile_is_error(tmp_path):
    _need_ngspice()
    with pytest.raises(NgspiceError, match="nothere.raw"):
        run_deck(RC, ["nothere.raw"], keep_dir=tmp_path)


@pytest.mark.integration
def test_failed_run_keeps_deck_for_inspection():
    _need_ngspice()
    bad = RC.replace("R1 in out 1k", "R1 in out")
    with pytest.raises(NgspiceError) as err:
        run_deck(bad, ["ac.raw"])
    deck = Path(str(err.value).split("(deck: ")[1].split(")")[0])
    try:
        assert deck.exists() and (deck.parent / "ngspice.log").exists()
    finally:
        shutil.rmtree(deck.parent, ignore_errors=True)


@pytest.mark.integration
def test_node_reached_only_through_a_capacitor_is_error(tmp_path):
    _need_ngspice()
    floating = RC.replace("C1 out 0 159.155n", "C1 out 0 159.155n\nC2 out x 1n\nC3 x 0 1n")
    with pytest.raises(NgspiceError, match="singular matrix"):
        run_deck(floating, ["ac.raw"], keep_dir=tmp_path)


@pytest.mark.integration
def test_unknown_value_model_is_error(tmp_path):
    _need_ngspice()
    with pytest.raises(NgspiceError, match="find model"):
        run_deck(RC.replace("159.155n", "1x"), ["ac.raw"], keep_dir=tmp_path)


@pytest.mark.integration
def test_run_decks_returns_results_in_input_order():
    _need_ngspice()
    from neohw.ngspice import run_decks
    decks = [RC.replace("R1 in out 1k", f"R1 in out {r}") for r in ("1k", "2k", "4k")]
    results = run_decks(decks, ["ac.raw"], workers=3)
    corners = []
    for res in results:
        ac = res["ac.raw"][0]
        f, g = ac.data["frequency"].real, np.abs(ac.data["v(out)"])
        corners.append(f[np.argmin(np.abs(g - 2 ** -0.5))])
    assert corners[0] > corners[1] > corners[2]
