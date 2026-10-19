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
    assert bus.shunt_c > 10 * sig["open_c"].default          # ten released relays on the bus


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
    assert m1["att_half_lo_db"] is not None and m1["rej3f_db"] is None   # 0.3 MHz < 0.5 MHz upper edge


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
    assert np.max(np.abs(fast[:41] - ref[:41])) <= 0.05
    assert np.max(np.abs(fast[41:] - ref[41:])[ref[41:] > -60]) <= 1.0
