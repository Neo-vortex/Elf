"""Synthetic characterisation files for twin tests."""
import json
import math

import numpy as np


def _doc(block, **payload):
    return {"schema_version": 1, "block": block, "status": "pass", "generated_by": "test", **payload}


def write_char(d, *, il_db=1.0, pre_gain_db=14.0, pre_nf_db=4.0, mixer_nf_db=7.0, conv_gain_db=36.0,
               harmonics=None, imbalance=(0.0, 0.0), adc_dr_db=108.0, dc=0.0, bypass=1.0,
               tp_gain_db=None, tp_harmonics=None, iip3_dbm=30.0, oip3_dbm=24.0, pn=None):
    """`tp_gain_db` / `tp_harmonics`: (value at the 1 MHz test point, value at 60 MHz); default
    both equal to the top-level (nominal) values."""
    """Flat synthetic characterisation: two filters (1–10 MHz, 10–60 MHz) with S21 = −il_db in
    band and −80 dB outside, flat preamp and attenuator, flat mixer response."""
    d.mkdir(parents=True, exist_ok=True)
    filters = [{"index": 1, "lo_hz": 1e6, "hi_hz": 10e6}, {"index": 2, "lo_hz": 10e6, "hi_hz": 60e6}]
    (d / "bpf_bank.json").write_text(json.dumps(_doc("bpf_bank", filters=filters)))
    for flt in filters:
        lines = ["# Hz S RI R 50"]
        for f in np.geomspace(0.05e6, 400e6, 400):
            inside = flt["lo_hz"] * 0.9 <= f <= flt["hi_hz"] * 1.1
            s21 = 10 ** ((-il_db if inside else -80.0) / 20)
            lines.append(f"{f:.6g} 0 0 {s21:.9g} 0 {s21:.9g} 0 0 0")
        (d / f"bpf_{flt['index']}.s2p").write_text("\n".join(lines) + "\n")
    grid = list(np.geomspace(0.1e6, 300e6, 50))
    (d / "preamp.json").write_text(json.dumps(_doc(
        "preamp", freq_hz=grid, gain_db=[pre_gain_db] * 50, nf_db=[pre_nf_db] * 50,
        bypass_loss_db=0.0, oip3_dbm=oip3_dbm)))
    curve = pn or {"offset_hz": [100.0, 1e3, 1e4, 1e5, 1e6], "dbc_per_hz": [-200.0] * 5}
    (d / "lo_phase_noise.json").write_text(json.dumps(_doc(
        "lo_phase_noise", reference_hz=9e6, source="test", **curve)))
    (d / "attenuator.json").write_text(json.dumps(_doc(
        "attenuator", freq_hz=[0.1e6, 54e6], pad_db=[-20.0, -20.0], bypass_db=[0.0, 0.0])))
    offs = [-60e3, -3e3, 3e3, 60e3]
    harm = {"1": {"db": 0.0, "phase_deg": 0.0, "sideband": "upper"}}
    harm.update(harmonics or {})
    ir = {"p5_gain_err_db": imbalance[0], "p5_phase_err_deg": imbalance[1]}
    (d / "mixer.json").write_text(json.dumps(_doc(
        "mixer", conv_gain_db=conv_gain_db,
        test_points=[{"f_lo_hz": f, "conv_gain_db": g, "harmonics": h} for f, g, h in zip(
            (1e6, 60e6), tp_gain_db or (conv_gain_db, conv_gain_db), tp_harmonics or (harmonics or {}, harmonics or {}))],
        baseband_response={"offset_hz": offs, "re": [1.0] * 4, "im": [0.0] * 4},
        harmonic_responses=harm, image_rejection_db={"direct": ir, "div4": ir},
        nf_db=mixer_nf_db, direct_min_hz=4.8e6, adc_fs_peak_v=2.825, clip_peak_v=2.0, iip3_dbm=iip3_dbm, adc_full_scale_dbm=-20.0,
        adc_noise={"dynamic_range_db": adc_dr_db, "bandwidth_hz": 40e3},
        dc_offset_v={"typical": dc, "worst": dc},
        bypass_correction={str(i): {"offset_hz": offs, "re": [bypass] * 4, "im": [0.0] * 4,
                                    "mean_db": 20 * math.log10(bypass)} for i in (1, 2)})))
    return d
