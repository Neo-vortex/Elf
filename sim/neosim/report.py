"""Self-contained HTML report with embedded plots."""
from __future__ import annotations

import base64
import html
import io
from datetime import datetime
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

from neosim.measure import MDS_AGREEMENT_DB, MDS_BW_HZ, MDS_TARGET_DBM, HardwareChecks  # noqa: E402
from neosim.sweep import BAND_NOISE_DBM_HZ, SweepPoint, implementation_loss_db, wilson_interval  # noqa: E402
from neosim import hardware  # noqa: E402
from neosim.twin import ScenarioResult  # noqa: E402

_STYLE = """
body{font-family:system-ui,sans-serif;max-width:1000px;margin:2rem auto;padding:0 16px;color:#222}
table{border-collapse:collapse;width:100%;margin:.5rem 0 1.5rem}
td,th{border:1px solid #ccc;padding:.35rem .5rem;text-align:left;font-size:.9rem}
.pass{color:#0a7a2f;font-weight:600}.fail{color:#b00020;font-weight:600}
img{max-width:100%}
"""


def _png(fig) -> str:
    buf = io.BytesIO()
    fig.savefig(buf, format="png", dpi=110, bbox_inches="tight")
    plt.close(fig)
    return "data:image/png;base64," + base64.b64encode(buf.getvalue()).decode()


def _badge(ok: bool) -> str:
    return '<span class="pass">PASS</span>' if ok else '<span class="fail">FAIL</span>'


def _scenario_html(r: ScenarioResult) -> str:
    e = html.escape
    fig, ax = plt.subplots(figsize=(9, 3))
    ax.plot(r.psd_freqs_hz / 1e3, r.psd_db, lw=0.7)
    ax.set_xlabel(f"offset from LO {r.scenario.lo_hz / 1e6:.6f} MHz (kHz)")
    ax.set_ylabel("PSD (dBFS/Hz)")
    ax.grid(alpha=0.3)
    rows = "".join(
        f"<tr><td>{e(x.id)}</td><td>{e(x.decoder)}</td><td>{_badge(x.passed)}</td>"
        f"<td>{e(x.output)}</td><td>{e(x.detail)}</td></tr>" for x in r.receives)
    excl = "".join(f"<li>{e(x.station_id)}: {e(x.reason)}</li>" for x in r.excluded) or "<li>none</li>"
    return (f"<h2>{e(r.scenario.name)} {_badge(r.passed)}</h2>"
            f"{_receiver_html(r)}"
            f"<p>ADC: peak {r.adc.peak_dbfs:.1f} dBFS, clipped samples {r.adc.clipped_samples}</p>"
            f"<p>Excluded stations:</p><ul>{excl}</ul>"
            f"<table><tr><th>receive</th><th>decoder</th><th>result</th><th>output</th><th>detail</th></tr>{rows}</table>"
            f'<img alt="spectrum" src="{_png(fig)}">')


def _receiver_html(r: ScenarioResult) -> str:
    rx = r.scenario.receiver
    if r.hardware is None:
        return "<p>Receiver: ideal</p>"
    h = r.hardware
    rows = "".join(f"<tr><td>{html.escape(x['stage'])}</td><td>{x['gain_db']:+.2f}</td><td>{'' if x['nf_db'] != x['nf_db'] else f"{x['nf_db']:.2f}"}</td>"
                   f"<td>{x['added_factor']:.3f}</td></tr>" for x in h.noise["stages"])
    folded = ", ".join(h.folded) or "none"
    return (f"<p>Receiver: hardware model (committed characterisation), filter {h.filter_index}, "
            f"preamp {'on' if rx.preamp else 'off'}, attenuator {rx.attenuator_db} dB; conversion gain "
            f"{h.conv_gain_db:.2f} dB; image rejection {h.image_rejection_db:.1f} dB (5th-percentile imbalance); "
            f"stations through LO harmonics: {html.escape(folded)}</p>"
            f"<p>Noise budget at the LO: receiver NF {h.noise['nf_total_db']:.2f} dB (analog cascade "
            f"{h.noise['nf_db']:.2f} dB, preamp noise folded in from LO harmonics "
            f"{h.noise['harmonic_noise_w_per_hz'] / (hardware.K_B * hardware.T0):.3f} kT0, ADC "
            f"{h.noise['adc_added_factor']:.3f}). Block checks: <a href=\"../hw/report.html\">hardware report</a>.</p>"
            f"<p>IMD3 products in the window: {html.escape(', '.join(f'{i}×{j} at {f / 1e6:.4f} MHz ({p:.1f} dBm)' for i, j, f, p in h.imd3)) or 'none'}; "
            f"reciprocal mixing {h.reciprocal_dbm_per_hz:.1f} dBm/Hz; above full scale at the mixer: "
            f"{html.escape(', '.join(h.overload)) or 'none'}</p>"
            f"<table><tr><th>stage</th><th>gain (dB)</th><th>NF (dB)</th><th>added noise factor (Friis)</th></tr>"
            f"{rows}</table>")


def _sweep_html(points: list[SweepPoint]) -> str:
    fig, ax = plt.subplots(figsize=(7, 3.5))
    snrs = [p.snr_db for p in points]
    ax.plot(snrs, [p.reference_rate for p in points], "o-", label="reference (audio only)")
    ax.plot(snrs, [p.twin_rate for p in points], "s-", label="digital twin, ideal receiver")
    hw = all(p.hardware_rate is not None for p in points)
    if hw:
        ax.plot(snrs, [p.hardware_rate for p in points], "^-", label="digital twin, hardware model")
    ax.set_xlabel("SNR in 2500 Hz (dB)")
    ax.set_ylabel("FT8 decode rate")
    ax.invert_xaxis()
    ax.grid(alpha=0.3)
    ax.legend()
    def loss_txt(curve):
        loss = implementation_loss_db(points, curve)
        return ("not determined (a curve does not cross 50 % inside the swept range)" if loss is None
                else f"{loss:+.1f} dB (50 % decode points, linearly interpolated)")

    def cell(rate: float, trials: int) -> str:
        lo, hi = wilson_interval(round(rate * trials), trials)
        return f"{rate:.2f} <small>[{lo:.2f}–{hi:.2f}]</small>"

    trials = points[0].trials
    rows = "".join(f"<tr><td>{p.snr_db:+.1f}</td><td>{cell(p.reference_rate, p.trials)}</td>"
                   f"<td>{cell(p.twin_rate, p.trials)}</td>"
                   + (f"<td>{cell(p.hardware_rate, p.trials)}</td>" if hw else "") + "</tr>" for p in points)
    hw_loss = (f"<p><b>Implementation loss, hardware model</b> (20 m, band noise {BAND_NOISE_DBM_HZ:g} dBm/Hz, "
               f"preamp off; target ≤ 1 dB): {loss_txt('hardware')}</p>" if hw else "")
    return (f"<h2>FT8 sensitivity sweep</h2><p><b>Implementation loss, ideal receiver:</b> {loss_txt('twin')}</p>{hw_loss}"
            f"<p>{trials} trials per point; brackets show the 95 % Wilson confidence interval of each decode rate. "
            f"Treat loss differences smaller than the SNR step as noise.</p>"
            f"<table><tr><th>SNR (dB)</th><th>reference</th><th>twin</th>{'<th>hardware</th>' if hw else ''}</tr>"
            f"{rows}</table>"
            f'<img alt="sweep" src="{_png(fig)}">')


def _checks_html(c: HardwareChecks) -> str:
    rows = "".join(f"<tr><td>{m.f_lo_hz / 1e6:.1f} MHz</td><td>{'on' if m.preamp else 'off'}</td>"
                   f"<td>{m.nf_total_db:.2f}</td><td>{m.predicted_dbm:.1f}</td><td>{m.measured_dbm:.1f}</td>"
                   f"<td>{_badge(m.passed)}</td></tr>" for m in c.mds)
    b = c.blocker
    decoded = " | ".join(x.output for x in b.with_blocker.receives) or "nothing"
    return (f"<h2>Hardware checks {_badge(c.passed)}</h2>"
            f"<p><b>MDS</b> (CW level for 3 dB (S+N)/N in {MDS_BW_HZ:g} Hz, thermal noise only; target ≤ "
            f"{MDS_TARGET_DBM:g} dBm and within ±{MDS_AGREEMENT_DB:g} dB of the noise budget):</p>"
            f"<table><tr><th>LO</th><th>preamp</th><th>receiver NF (dB)</th><th>budget (dBm)</th>"
            f"<th>measured (dBm)</th><th>result</th></tr>{rows}</table>"
            f"<p><b>Blocker</b> ({html.escape(b.with_blocker.scenario.name)}): {_badge(b.passed)} decoded "
            f"<code>{html.escape(decoded)}</code>; noise floor in the window {b.degradation_db:+.2f} dB with "
            f"{', '.join(map(html.escape, b.blocker_ids))} against without; reciprocal mixing "
            f"{b.with_blocker.hardware.reciprocal_dbm_per_hz:.1f} dBm/Hz at the antenna.</p>")


def write_report(results: list[ScenarioResult], sweep_points: list[SweepPoint] | None, path: Path,
                 checks: HardwareChecks | None = None) -> Path:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    body = "".join(_scenario_html(r) for r in results)
    if sweep_points:
        body += _sweep_html(sweep_points)
    if checks is not None:
        body += _checks_html(checks)
    overall = all(r.passed for r in results) and (checks is None or checks.passed)
    path.write_text(
        f"<!doctype html><html><head><meta charset='utf-8'><title>NeoRadio Sim Report</title>"
        f"<style>{_STYLE}</style></head><body><h1>NeoRadio simulation report {_badge(overall)}</h1>"
        f"<p>Generated {datetime.now():%Y-%m-%d %H:%M}</p>{body}</body></html>",
        encoding="utf-8")
    return path
