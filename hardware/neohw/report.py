"""Self-contained HTML report for hardware block checks."""
from __future__ import annotations

import base64
import csv
import html
import io
from datetime import datetime
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

from neohw.checks import BlockResult  # noqa: E402

_STYLE = """
body{font-family:system-ui,sans-serif;max-width:1100px;margin:2rem auto;padding:0 16px;color:#222}
table{border-collapse:collapse;margin:.5rem 0 1.5rem}td,th{border:1px solid #ccc;padding:.3rem .5rem;font-size:.9rem}
.pass{color:#0a7a2f;font-weight:600}.fail{color:#b00020;font-weight:600}img,svg{max-width:100%}
"""


def _badge(ok: bool) -> str:
    return '<span class="pass">PASS</span>' if ok else '<span class="fail">FAIL</span>'


def _plot_png(plots: dict, logx: bool = False) -> str:
    fig, ax = plt.subplots(figsize=(9, 3.2))
    for label, (x, y) in plots.items():
        ax.plot(x / 1e6, y, label=label)
    if logx:
        ax.set_xscale("log")
    ax.set_xlabel("frequency (MHz)")
    ax.grid(alpha=0.3)
    ax.legend(fontsize=8)
    buf = io.BytesIO()
    fig.savefig(buf, format="png", dpi=110, bbox_inches="tight")
    plt.close(fig)
    return "data:image/png;base64," + base64.b64encode(buf.getvalue()).decode()


def _block_html(r: BlockResult) -> str:
    e = html.escape
    rows = "".join(f"<tr><td>{e(c.name)}</td><td>{c.value:.3f} {e(c.unit)}</td><td>{e(c.limit)}</td>"
                   f"<td>{_badge(c.passed)}</td></tr>" for c in r.checks)
    out = [f"<h2>{e(r.block)} {_badge(r.passed)}</h2>",
           f"<table><tr><th>check</th><th>value</th><th>limit</th><th>result</th></tr>{rows}</table>"]
    if r.plots:
        out.append(f'<img alt="plots" src="{_plot_png(r.plots, r.logx)}">')
    svg = r.artefacts.get("svg")
    if svg and Path(svg).exists():
        text = Path(svg).read_text()
        out.append("<h3>Connections</h3>" + text[text.find("<svg"):])
    bom = r.artefacts.get("bom")
    if bom and Path(bom).exists():
        rows_bom = list(csv.DictReader(Path(bom).open(encoding="utf-8")))
        if rows_bom:
            head = "".join(f"<th>{e(k)}</th>" for k in rows_bom[0])
            body = "".join("<tr>" + "".join(f"<td>{e(str(v))}</td>" for v in row.values()) + "</tr>"
                           for row in rows_bom)
            out.append(f"<h3>BOM</h3><table><tr>{head}</tr>{body}</table>")
    return "".join(out)


def write_report(results: list[BlockResult], path: Path) -> Path:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    overall = all(r.passed for r in results)
    body = "".join(_block_html(r) for r in results)
    path.write_text(f"<!doctype html><html><head><meta charset='utf-8'><title>NeoRadio Hardware Report</title>"
                    f"<style>{_STYLE}</style></head><body><h1>NeoRadio hardware report {_badge(overall)}</h1>"
                    f"<p>Generated {datetime.now():%Y-%m-%d %H:%M}</p>{body}</body></html>", encoding="utf-8")
    return path
