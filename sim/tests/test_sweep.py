import pytest

from neosim import sweep
from neosim.tools import ToolNotFoundError, tool_path


def test_threshold_snr_interpolates():
    assert sweep.threshold_snr([-10, -14, -18], [1.0, 1.0, 0.0]) == pytest.approx(-16.0)


def test_threshold_snr_none_when_never_reached():
    assert sweep.threshold_snr([-10, -14], [0.25, 0.0]) is None


def test_implementation_loss_positive_when_twin_worse():
    points = [sweep.SweepPoint(-10, 1.0, 1.0), sweep.SweepPoint(-14, 1.0, 0.0), sweep.SweepPoint(-18, 0.0, 0.0)]
    # reference crosses 0.5 at -16, twin at -12
    assert sweep.implementation_loss_db(points) == pytest.approx(4.0)


@pytest.mark.integration
def test_small_sweep_runs(tmp_path):
    try:
        for name in ("gen_ft8", "decode_ft8", "neoradio-dsp"):
            tool_path(name)
    except ToolNotFoundError as e:
        pytest.skip(str(e))
    points = sweep.sweep_ft8([-8.0, -30.0], trials=1, workdir=tmp_path)
    assert [(p.reference_rate, p.twin_rate) for p in points] == [(1.0, 1.0), (0.0, 0.0)]
