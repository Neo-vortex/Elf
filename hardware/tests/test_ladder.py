import math

import numpy as np
import pytest

from neohw.synth import ladder
from neohw.synth.filters import lowpass


def _lossless():
    return ladder.Losses(inductor_z=lambda h, f: 1j * 2 * math.pi * f * h, cap_q=1e12)


def test_lossless_chebyshev_ripple_and_cutoff():
    els = lowpass(10e6, 5, 0.1)
    br = [ladder.Branch("shunt", "C", c=e.value) if e.kind == "C_shunt" else ladder.Branch("series", "L", l=e.value)
          for e in els]
    f = np.linspace(0.1e6, 10e6, 400)
    s21, _ = ladder.s21_s11(br, f, _lossless())
    assert -ladder.db(s21).min() == pytest.approx(0.1, abs=0.01)     # equiripple 0.1 dB
    s21c, _ = ladder.s21_s11(br, np.array([10e6]), _lossless())
    assert -ladder.db(s21c)[0] == pytest.approx(0.1, abs=0.02)      # ripple bandwidth edge


def test_lossless_network_conserves_power():
    br = [ladder.Branch("series", "LC_par", l=1e-6, c=100e-12), ladder.Branch("shunt", "LC_ser", l=2e-6, c=50e-12),
          ladder.Branch("shunt", "C", c=200e-12)]
    f = np.linspace(1e6, 50e6, 200)
    s21, s11 = ladder.s21_s11(br, f, _lossless())
    assert np.allclose(np.abs(s21) ** 2 + np.abs(s11) ** 2, 1.0, atol=1e-9)


def test_lossless_network_with_bus_conserves_power():
    br = [ladder.Branch("series", "L", l=1e-6), ladder.Branch("shunt", "C", c=200e-12)]
    f = np.linspace(1e6, 50e6, 200)
    s21, s11 = ladder.s21_s11(br, f, _lossless(), ladder.Bus(shunt_c=5e-12))
    assert np.allclose(np.abs(s21) ** 2 + np.abs(s11) ** 2, 1.0, atol=1e-9)


def test_branch_rejects_missing_values():
    with pytest.raises(ValueError):
        ladder.Branch("series", "LC_par", l=1e-6)
    with pytest.raises(ValueError):
        ladder.Branch("diagonal", "L", l=1e-6)


def test_bank_bus_is_read_from_the_relay_model():
    import inspect
    from neohw import parts
    sig = inspect.signature(parts.relay_dpdt).parameters
    bus = ladder.bank_bus()
    assert bus.series_r == sig["contact_r"].default
    open_c, ipc = sig["open_c"].default, sig["inter_pole_c"].default
    # 10 released relays: open NO1-COM1 + NO1-NO2 each; selected relay: NO1-NO2 (bus side),
    # open COM1-NC1 + COM1-COM2 (filter side)  ->  11 open contacts, 12 inter-pole C
    assert bus.shunt_c == pytest.approx(11 * open_c + 12 * ipc)


def test_metrics_keys_and_none_rules():
    from neohw.blocks import bpf
    d = bpf.design(8)
    m = ladder.metrics(ladder.from_positions(d.positions), d.lo_hz, d.hi_hz, ladder.robust_losses(d.centre_hz),
                       ladder.bank_bus())
    assert set(m) == {"il_db", "rl_db", "rej3f_db", "att_half_lo_db", "att_2hi_db"}
    assert m["rej3f_db"] > 40 and m["il_db"] < 3 and m["rl_db"] > 10
    d1 = bpf.design(1)
    m1 = ladder.metrics(ladder.from_positions(d1.positions), d1.lo_hz, d1.hi_hz,
                        ladder.robust_losses(d1.centre_hz))
    assert m1["att_half_lo_db"] is not None and m1["rej3f_db"] is None   # lo/2 = 50 kHz (inclusive); 3·lo = 0.3 MHz ≤ hi = 0.5 MHz
    m80 = ladder.metrics(ladder.from_positions(d1.positions), 80e3, d1.hi_hz, ladder.robust_losses(d1.centre_hz))
    assert m80["att_half_lo_db"] is None                                  # lo/2 = 40 kHz < 50 kHz


def _lossy(r, q, f0):
    return ladder.Losses(lambda h, f: r + 1j * 2 * math.pi * f * h, cap_q=q, f_cap_q_hz=f0)


def test_resonant_branches_lossless():
    l, c = 1e-6, 100e-12
    f0 = 1 / (2 * math.pi * math.sqrt(l * c))
    w0l = 2 * math.pi * f0 * l
    zs = ladder.branch_z(ladder.Branch("series", "LC_ser", l=l, c=c), np.array([f0]), _lossless())
    zp = ladder.branch_z(ladder.Branch("series", "LC_par", l=l, c=c), np.array([f0]), _lossless())
    assert abs(zs[0]) < 1e-6 * w0l
    assert abs(zp[0]) > 1e6 * w0l


def test_resonant_branches_lossy_match_exact_expressions():
    l, c, r, q = 1e-6, 100e-12, 0.7, 300.0
    f0 = 1 / (2 * math.pi * math.sqrt(l * c))
    losses = _lossy(r, q, f0)
    esr = 1 / (2 * math.pi * f0 * c * q)
    for f in (f0, 0.7 * f0):
        w = 2 * math.pi * f
        zl, zc = r + 1j * w * l, esr + 1 / (1j * w * c)
        zs = ladder.branch_z(ladder.Branch("shunt", "LC_ser", l=l, c=c), np.array([f]), losses)[0]
        zp = ladder.branch_z(ladder.Branch("shunt", "LC_par", l=l, c=c), np.array([f]), losses)[0]
        assert zs == pytest.approx(zl + zc, rel=1e-12)
        assert zp == pytest.approx(zl * zc / (zl + zc), rel=1e-12)


def test_segment_wrapper_equals_robust_z_at_the_segment_frequency():
    from neohw import inductors
    from tests.robust_spice import fixed_r_z
    for h in (0.12e-6, 0.5e-6, 2.2e-6, 33e-6):
        for f in (1e6, 18e6, 30e6, 60e6):
            assert fixed_r_z(h, f, f) == pytest.approx(inductors.robust_z(h, f), rel=1e-12)


@pytest.mark.integration
def test_matches_ngspice_on_todays_filter_8_in_the_bank():
    """Today's Chebyshev filter 8 in the full bank, with the robust model on both sides: the
    fast model (with the relay bus loading) against ngspice. ngspice AC has no
    frequency-dependent R, so both sides fix each inductor's R per segment (centre in band,
    3·centre over the 3f band): tests.robust_spice.segment_losses."""
    from neohw.blocks import bpf
    from tests.robust_spice import segment_losses, spice_s21_db
    d = bpf.design(8)
    br = ladder.from_positions(d.positions)
    f = np.linspace(d.lo_hz, d.hi_hz, 41)
    f3 = np.linspace(3 * d.lo_hz, 3 * d.hi_hz, 41)
    fast = np.concatenate([
        ladder.db(ladder.s21_s11(br, f, segment_losses(d.centre_hz, d.centre_hz), ladder.bank_bus())[0]),
        ladder.db(ladder.s21_s11(br, f3, segment_losses(3 * d.centre_hz, d.centre_hz), ladder.bank_bus())[0])])
    ref = spice_s21_db(8, np.concatenate([f, f3]))
    assert np.max(np.abs(fast[:41] - ref[:41])) <= 0.01          # without the bus: 0.039 dB
    assert np.max(np.abs(fast[41:] - ref[41:])[ref[41:] > -60]) <= 1.0
    deep = ref[41:] > -80
    assert deep.sum() >= 30
    assert np.max(np.abs(fast[41:] - ref[41:])[deep]) <= 0.05    # without the bus: 0.53 dB


def test_choke_pair_is_two_chokes_in_series_each_with_its_own_losses():
    from neohw import inductors
    f = np.array([5e6, 20e6, 60e6])
    losses = ladder.robust_losses(10e6)
    pair = ladder.Branch("series", "L", l=50.3e-6, parts=((47e-6, 47e-6), (3.3e-6, 3.3e-6)))
    z = ladder.branch_z(pair, f, losses)
    assert np.allclose(z, inductors.robust_z(47e-6, f) + inductors.robust_z(3.3e-6, f))
    assert not np.allclose(z, inductors.robust_z(50.3e-6, f))
    # a selected (scaled) part keeps its bought row's Q, SRF and leads
    sel = ladder.branch_z(ladder.Branch("series", "L", l=48e-6, parts=((47e-6, 48e-6),)), f, losses)
    assert np.allclose(sel, inductors.part_z(47e-6, 48e-6, f))
    with pytest.raises(ValueError):
        ladder.Branch("series", "L", l=40e-6, parts=((47e-6, 47e-6),))


def test_robust_losses_without_choke_capacitance():
    from neohw import inductors
    f = np.array([5e6, 60e6, 150e6])
    r = inductors.nearest_row("tdk_bc", 8.2e-6)
    bare = inductors.robust_r(r, f) + 1j * 2 * math.pi * f * (8.2e-6 + inductors.lead_l("tdk_bc"))
    br = ladder.Branch("series", "L", l=8.2e-6)
    assert np.allclose(ladder.branch_z(br, f, ladder.robust_losses(10e6, cpar=False)), bare)
    assert not np.allclose(ladder.branch_z(br, f, ladder.robust_losses(10e6)), bare)


def test_scaled_draws_equal_explicitly_scaled_ladders():
    br = [ladder.Branch("series", "L", l=50.3e-6, parts=((47e-6, 47e-6), (3.3e-6, 3.3e-6))),
          ladder.Branch("shunt", "C", c=100e-12), ladder.Branch("series", "LC_par", l=1.5e-6, c=30e-12)]
    kl0 = np.array([[1.05, 0.93], [0.97, 1.02]])
    kc1, kl2, kc2 = np.array([1.01, 0.99]), np.array([[0.95], [1.08]]), np.array([1.02, 0.98])
    scale = [(kl0, None), (None, kc1), (kl2, kc2)]
    f = np.linspace(1e6, 60e6, 50)
    losses, bus = ladder.robust_losses(10e6), ladder.bank_bus()
    s21, s11 = ladder.s21_s11(br, f, losses, bus, scale=scale)
    for d in range(2):
        parts = tuple((n, a * k) for (n, a), k in zip(br[0].parts, kl0[d]))
        one = [ladder.Branch("series", "L", l=sum(a for _, a in parts), parts=parts),
               ladder.Branch("shunt", "C", c=100e-12 * kc1[d]),
               ladder.Branch("series", "LC_par", l=1.5e-6 * kl2[d, 0], c=30e-12 * kc2[d])]
        e21, e11 = ladder.s21_s11(one, f, ladder.robust_losses(10e6), bus)
        assert np.allclose(s21[d], e21) and np.allclose(s11[d], e11)   # draws stay on the nominal's row
