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
def test_missing_rawfile_is_error():
    _need_ngspice()
    with pytest.raises(NgspiceError, match="nothere.raw"):
        run_deck(RC, ["nothere.raw"])
