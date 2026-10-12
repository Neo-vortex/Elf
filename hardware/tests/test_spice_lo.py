from skidl import Net

from neohw.spice import QuadratureLO, Tran


def test_quadrature_lo_puts_s1_a_quarter_period_after_s0():
    lo = QuadratureLO(Net("s0"), Net("s1"), 10e6, skew_s=1e-10)
    s0, s1 = lo.lines()
    assert s0.startswith("VLO_s0 s0 0 dc 0 pulse(0 3.3 0 1e-09 1e-09 4.9e-08 1e-07)")
    assert " 2.51e-08 " in s1


def test_tran_max_step_is_separate_from_output_step():
    assert Tran(0.25e-6, 1e-3, 1e-5, 2.5e-10).command() == "tran 2.5e-07 0.001 1e-05 2.5e-10\nlinearize"
