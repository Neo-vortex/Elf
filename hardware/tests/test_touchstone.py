import numpy as np
import pytest

from neohw.touchstone import read_s2p, write_s2p


def _s(n):
    k = np.arange(n)
    return {"11": 0.1 + 0.01j * k, "21": 0.9 - 0.02j * k, "12": 0.9 - 0.02j * k, "22": 0.2 + 0j * k}


def test_round_trip(tmp_path):
    f = np.array([1e6, 2e6, 3e6])
    p = write_s2p(tmp_path / "a.s2p", f, _s(3), comments=("hello",))
    text = p.read_text().splitlines()
    assert text[0] == "! hello" and text[1] == "# HZ S RI R 50"
    f2, s2 = read_s2p(p)
    np.testing.assert_allclose(f2, f)
    for k in ("11", "21", "12", "22"):
        np.testing.assert_allclose(s2[k], _s(3)[k], atol=1e-9)


def test_missing_parameter_rejected(tmp_path):
    s = _s(1)
    del s["22"]
    with pytest.raises(ValueError, match="22"):
        write_s2p(tmp_path / "a.s2p", np.array([1e6]), s)


def test_unsupported_option_line_rejected(tmp_path):
    p = tmp_path / "a.s2p"
    p.write_text("# MHZ S MA R 50\n1 0 0 0 0 0 0 0 0\n")
    with pytest.raises(ValueError, match="option line"):
        read_s2p(p)
