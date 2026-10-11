import numpy as np
import pytest

from neohw.rawfile import RawfileError, parse_ascii

COMPLEX = """Title: rc test
Date: Mon Oct 12 01:26:11  2026
Command: ngspice-45.2, Build Fri Sep 12 11:58:13 UTC 2025
Plotname: AC Analysis
Flags: complex
No. Variables: 3
No. Points: 2
Variables:
\t0\tfrequency\tfrequency grid=3
\t1\tv(in)\tvoltage
\t2\tv(out)\tvoltage
Values:
 0\t1.000000000000000e+02,0.000000000000000e+00
\t1.000000000000000e+00,0.000000000000000e+00
\t9.900996188197071e-01,-9.900688679469646e-02

 1\t1.000000000000000e+03,0.000000000000000e+00
\t1.000000000000000e+00,0.000000000000000e+00
\t5.000155294223400e-01,-4.999999997588371e-01

"""

REAL = """Title: rc test
Date: Mon Oct 12 01:26:11  2026
Plotname: Operating Point
Flags: real
No. Variables: 2
No. Points: 1
Variables:
\t0\tv(in)\tvoltage
\t1\tv(out)\tvoltage
Values:
 0\t0.000000000000000e+00
\t2.500000000000000e+00

"""


def test_parse_complex_plot():
    (plot,) = parse_ascii(COMPLEX)
    assert plot.name == "AC Analysis"
    assert plot.variables == ["frequency", "v(in)", "v(out)"]
    np.testing.assert_allclose(plot.data["frequency"].real, [100.0, 1000.0])
    assert plot.data["v(out)"][1] == pytest.approx(0.5000155294223400 - 0.4999999997588371j)


def test_parse_real_plot():
    (plot,) = parse_ascii(REAL)
    assert plot.data["v(out)"][0] == pytest.approx(2.5)
    assert not np.iscomplexobj(plot.data["v(out)"])


def test_parse_two_plots_in_one_file():
    plots = parse_ascii(COMPLEX + REAL)
    assert [p.name for p in plots] == ["AC Analysis", "Operating Point"]


def test_truncated_values_raise():
    with pytest.raises(RawfileError, match="expected 2 points"):
        parse_ascii(COMPLEX.split(" 1\t")[0])


def test_missing_header_raises():
    with pytest.raises(RawfileError):
        parse_ascii("garbage\n")
