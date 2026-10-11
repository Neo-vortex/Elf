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

from neosim.sweep import SweepPoint, implementation_loss_db, wilson_interval  # noqa: E402
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
            f"<p>ADC: peak {r.adc.peak_dbfs:.1f} dBFS, clipped samples {r.adc.clipped_samples}</p>"
            f"<p>Excluded stations:</p><ul>{excl}</ul>"
            f"<table><tr><th>receive</th><th>decoder</th><th>result</th><th>output</th><th>detail</th></tr>{rows}</table>"
            f'<img alt="spectrum" src="{_png(fig)}">')


def _sweep_html(points: list[SweepPoint]) -> str:
    fig, ax = plt.subplots(figsize=(7, 3.5))
    snrs = [p.snr_db for p in points]
    ax.plot(snrs, [p.reference_rate for p in points], "o-", label="reference (audio only)")
    ax.plot(snrs, [p.twin_rate for p in points], "s-", label="digital twin")
    ax.set_xlabel("SNR in 2500 Hz (dB)")
    ax.set_ylabel("FT8 decode rate")
    ax.invert_xaxis()
    ax.grid(alpha=0.3)
    ax.legend()
    loss = implementation_loss_db(points)
    loss_txt = ("not determined (a curve does not cross 50 % inside the swept range)" if loss is None
                else f"{loss:+.1f} dB (50 % decode points, linearly interpolated)")

    def cell(rate: float, trials: int) -> str:
        lo, hi = wilson_interval(round(rate * trials), trials)
        return f"{rate:.2f} <small>[{lo:.2f}–{hi:.2f}]</small>"

    trials = points[0].trials
    rows = "".join(f"<tr><td>{p.snr_db:+.1f}</td><td>{cell(p.reference_rate, p.trials)}</td>"
                   f"<td>{cell(p.twin_rate, p.trials)}</td></tr>" for p in points)
    return (f"<h2>FT8 sensitivity sweep</h2><p><b>Implementation loss:</b> {loss_txt}</p>"
            f"<p>{trials} trials per point; brackets show the 95 % Wilson confidence interval of each decode rate. "
            f"Treat loss differences smaller than the SNR step as noise.</p>"
            f"<table><tr><th>SNR (dB)</th><th>reference</th><th>twin</th></tr>{rows}</table>"
            f'<img alt="sweep" src="{_png(fig)}">')


def write_report(results: list[ScenarioResult], sweep_points: list[SweepPoint] | None, path: Path) -> Path:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    body = "".join(_scenario_html(r) for r in results)
    if sweep_points:
        body += _sweep_html(sweep_points)
    overall = all(r.passed for r in results)
    path.write_text(
        f"<!doctype html><html><head><meta charset='utf-8'><title>NeoRadio Sim Report</title>"
        f"<style>{_STYLE}</style></head><body><h1>NeoRadio simulation report {_badge(overall)}</h1>"
        f"<p>Generated {datetime.now():%Y-%m-%d %H:%M}</p>{body}</body></html>",
        encoding="utf-8")
    return path
