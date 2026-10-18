# Plan 5.0: Documentation Site Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** A fully self-contained MkDocs Material site, published to GitHub Pages by GitHub Actions, that teaches Elf from first principles (radio basics, SDR, the signal chain) through every board block, component choice and technique, with pictures on every page and result tables generated from the committed characterisation files.

**Architecture:** Markdown pages in `site/docs/`, config `site/mkdocs.yml`. A generator (`site/gen_pages.py`) turns `hardware/characterized/*.json` and `bpf_*.s2p` into result pages and plots before every build, so numbers on the site always equal the committed characterisation. The full HTML reports are copied into the committed `site/docs/reports/` by `make site-reports`. A pytest suite checks coverage (every block has a page, every GIF is referenced) and that a strict build passes. `.github/workflows/pages.yml` builds and deploys on push to `main`.

**Tech Stack:** MkDocs 1.6.1, mkdocs-material 9.7.7, schemdraw 0.23 (circuit drawings), Python 3 (numpy, matplotlib, pytest), MathJax 3 and Mermaid 11 served locally, GitHub Actions (`actions/upload-pages-artifact@v3`, `actions/deploy-pages@v4`).

**Spec:** `docs/superpowers/specs/2026-10-18-rev-a-china-fab-design.md` (§7b; §7a for media)

## Global Constraints

- Site sources: `site/docs/`, config `site/mkdocs.yml`; local build into `build/site/` with `make site`.
- Published by `.github/workflows/pages.yml` on every push to `main`; Pages source "GitHub Actions". **Switching Pages on in the repository settings is confirmed with the user first** (Task 6).
- Every module in `hardware/neohw/blocks/` (except `__init__`) has a page; every GIF in `docs/media/rev-a/` is referenced by a page (tested).
- Numbers on the site come from `hardware/characterized/` through the generator, never typed by hand.
- Pages describe the design as the specs record it. Where a spec of 2026-10-18 supersedes an older one (China primary and Iranian fallback, STM32F411, LM2576HV kept), the page states the current decision and marks parts not yet changed in the code as "planned (plan 5.x)".
- **Self-contained (user, 2026-10-18):** a reader with basic electronics and no radio knowledge can follow the site from the first page to the last without outside sources. Every technical term is defined in the glossary on first use (linked). Every page has at least one picture (circuit drawing, diagram, plot, render or GIF). No script, font or image loads from another site at run time.
- Writing style matches the README: plain, factual sentences; no marketing language.
- Packages on this machine come through the PyPI mirror (`PIP_INDEX_URL=https://mirror-pypi.runflare.com/simple/`); already installed in the Makefile's `python3` (Anaconda): mkdocs 1.6.1, mkdocs-material 9.7.7.

## Review Focus

1. A block module added later without a page: the coverage test must fail and name the module (Task 2 test).
2. A characterisation file with `status: fail` or a failed check: the generated page must show FAIL visibly, not hide the row (Task 2 test).
3. A broken internal link or a page missing from `nav`: `mkdocs build --strict` turns the warning into an error (Task 1 test).
4. The site built in CI without the local `build/` reports: the build must still pass, using the committed copies in `site/docs/reports/` (Task 5 runs the build from a clean checkout path).
5. A GIF added to `docs/media/rev-a/` but not shown on any page: the coverage test fails (Task 2 test).

---

### Task 1: Site skeleton, strict build, `make site`

**Files:**
- Create: `site/mkdocs.yml`, `site/requirements.txt`, `site/docs/index.md`, `site/tests/test_site.py`
- Modify: `Makefile` (targets `site`, `site-reports`; `test` runs the site tests), `.gitignore` (`site/docs/results/`)

**Interfaces:**
- Produces: `make site` → `build/site/index.html`; `site/tests/test_site.py::build_site(dest) -> subprocess.CompletedProcess` used by later tests.

- [ ] **Step 1: Write the failing test**

```python
# site/tests/test_site.py
import subprocess
import sys
from pathlib import Path

SITE = Path(__file__).resolve().parents[1]
ROOT = SITE.parent


def build_site(dest: Path) -> subprocess.CompletedProcess:
    subprocess.run([sys.executable, str(SITE / "gen_pages.py")], check=True, cwd=ROOT)
    return subprocess.run([sys.executable, "-m", "mkdocs", "build", "--strict", "-f", str(SITE / "mkdocs.yml"),
                           "-d", str(dest)], capture_output=True, text=True, cwd=ROOT)


def test_strict_build_passes(tmp_path):
    r = build_site(tmp_path / "out")
    assert r.returncode == 0, r.stderr
    assert (tmp_path / "out" / "index.html").exists()


def test_built_site_loads_nothing_from_other_hosts(tmp_path):
    r = build_site(tmp_path / "out")
    assert r.returncode == 0, r.stderr
    import re as _re
    for page in (tmp_path / "out").rglob("*.html"):
        html = page.read_text(encoding="utf-8", errors="replace")
        for m in _re.finditer(r'<(?:script|link|img)[^>]+(?:src|href)="(https?://[^"]+)"', html):
            assert m.group(1).startswith("https://github.com/Neo-vortex/Elf"), f"{page.name}: {m.group(1)}"
```

- [ ] **Step 2: Run it to see it fail**

Run: `cd site && python3 -m pytest -q tests/test_site.py::test_strict_build_passes`
Expected: FAIL (`gen_pages.py` and `mkdocs.yml` missing).

- [ ] **Step 3: Write the skeleton**

`site/requirements.txt`:
```
mkdocs==1.6.1
mkdocs-material==9.7.7
schemdraw==0.23
numpy
matplotlib
```

Vendor the run-time scripts (committed, so the site loads nothing from other hosts):
```bash
mkdir -p site/docs/assets/vendor
curl -sL -o site/docs/assets/vendor/tex-mml-chtml.js https://cdn.jsdelivr.net/npm/mathjax@3/es5/tex-mml-chtml.js
curl -sL -o site/docs/assets/vendor/mermaid.min.js https://cdn.jsdelivr.net/npm/mermaid@11/dist/mermaid.min.js
```
MathJax's `tex-mml-chtml.js` fetches its fonts relative to its own URL; download `https://cdn.jsdelivr.net/npm/mathjax@3/es5/output/chtml/fonts/woff-v2/` files listed in its source (grep `woff-v2` in the script) into `site/docs/assets/vendor/output/chtml/fonts/woff-v2/`. Material's own fonts: set `theme.font: false` so it uses system fonts instead of Google Fonts.

`site/mkdocs.yml`:
```yaml
site_name: Elf HF receiver
site_description: A general-coverage HF software-defined receiver, designed and verified as code
repo_url: https://github.com/Neo-vortex/Elf
edit_uri: edit/main/site/docs/
docs_dir: docs
theme:
  name: material
  font: false
  features: [navigation.sections, navigation.indexes, navigation.top, search.highlight, content.code.copy]
  palette:
    - media: "(prefers-color-scheme: light)"
      scheme: default
      toggle: {icon: material/weather-night, name: Dark mode}
    - media: "(prefers-color-scheme: dark)"
      scheme: slate
      toggle: {icon: material/weather-sunny, name: Light mode}
markdown_extensions:
  - admonition
  - attr_list
  - tables
  - toc: {permalink: true}
  - pymdownx.arithmatex: {generic: true}
  - pymdownx.superfences:
      custom_fences:
        - {name: mermaid, class: mermaid, format: !!python/name:pymdownx.superfences.fence_code_format}
extra_javascript:
  - assets/vendor/tex-mml-chtml.js
  - assets/vendor/mermaid.min.js
nav:
  - Overview: index.md
```

`site/docs/index.md`: a first version with the project summary from the README's first two paragraphs and a Mermaid block diagram of the signal chain (antenna → bias-T and protection → attenuator → filter bank → preamp → Tayloe mixer → baseband → CS5361 → controller → USB → Linux host; Si5351C LO into the mixer; power block feeding everything).

`site/gen_pages.py` (first version; Task 2 fills it in):
```python
#!/usr/bin/env python3
"""Generate the site's result pages from hardware/characterized/ (run before mkdocs build)."""
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "site" / "docs" / "results"


def main() -> None:
    OUT.mkdir(parents=True, exist_ok=True)


if __name__ == "__main__":
    main()
```

`Makefile` additions (add `site site-reports` to `.PHONY`):
```make
# documentation site: result pages generated from hardware/characterized/, then a strict MkDocs build
site:
	$(PYTHON) site/gen_pages.py
	$(PYTHON) -m mkdocs build --strict -f site/mkdocs.yml -d ../build/site

# copy the latest local reports into the site (committed, so CI publishes them without re-simulating)
site-reports:
	mkdir -p site/docs/reports
	cp build/hw/report.html site/docs/reports/hw-report.html
	cp build/sim/report.html site/docs/reports/sim-report.html
```
In `test:`, append the line `cd site && $(PYTHON) -m pytest -q tests`.

`.gitignore`: add `site/docs/results/`.

- [ ] **Step 4: Run the test**

Run: `cd site && python3 -m pytest -q tests/test_site.py && cd .. && make site`
Expected: PASS; `build/site/index.html` exists.

- [ ] **Step 5: Commit**

```bash
git add site Makefile .gitignore
git commit -m "docs(site): MkDocs Material skeleton with a strict build and make site"
```

### Task 2: Generated result pages and the coverage tests

**Files:**
- Modify: `site/gen_pages.py`, `site/mkdocs.yml` (nav entry "Results")
- Test: `site/tests/test_site.py`

**Interfaces:**
- Consumes: `hardware/characterized/<block>.json` (keys `block`, `status`, `generated_by`, `checks: [{name, value, limit, passed, unit}]`), `hardware/characterized/bpf_<n>.s2p`.
- Produces: `gen_pages.result_page(doc: dict) -> str` (Markdown); pages `site/docs/results/<block>.md`, `site/docs/results/index.md`, plot `site/docs/results/filter-bank-s21.png`; constant `gen_pages.BLOCK_PAGES: dict[str, str]` (block module name → page path under `site/docs/`).

- [ ] **Step 1: Write the failing tests**

```python
# append to site/tests/test_site.py
import importlib.util
import re


def _gen():
    spec = importlib.util.spec_from_file_location("gen_pages", SITE / "gen_pages.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def test_result_page_shows_every_check_and_marks_failures():
    page = _gen().result_page({"block": "demo", "status": "fail", "generated_by": "abc1234",
                               "checks": [{"name": "gain_db", "value": 14.2, "limit": "≥ 14", "passed": True, "unit": "dB"},
                                          {"name": "nf_db", "value": 6.1, "limit": "≤ 5", "passed": False, "unit": "dB"}]})
    assert "gain_db" in page and "nf_db" in page and "FAIL" in page and "abc1234" in page
    assert page.count("| ") >= 3                                   # a table with a header and two rows


def test_every_block_module_has_a_page():
    modules = sorted(p.stem for p in (ROOT / "hardware/neohw/blocks").glob("*.py") if p.stem != "__init__")
    pages = _gen().BLOCK_PAGES
    missing = [m for m in modules if m not in pages]
    assert not missing, f"block modules without a site page: {missing}"
    for page in set(pages.values()):
        assert (SITE / "docs" / page).exists(), page


def test_every_gif_in_media_is_shown_on_a_page():
    gifs = sorted((ROOT / "docs/media/rev-a").glob("*.gif"))
    text = "\n".join(p.read_text(encoding="utf-8") for p in (SITE / "docs").rglob("*.md"))
    unused = [g.name for g in gifs if g.name not in text]
    assert not unused, f"GIFs not referenced by any page: {unused}"


def test_generated_pages_cover_every_characterisation_file(tmp_path):
    _gen().main()
    for f in (ROOT / "hardware/characterized").glob("*.json"):
        assert (SITE / "docs/results" / f"{f.stem}.md").exists(), f.stem
    assert (SITE / "docs/results/filter-bank-s21.png").exists()
```

- [ ] **Step 2: Run them to see them fail**

Run: `cd site && python3 -m pytest -q tests/test_site.py`
Expected: FAIL (`result_page`, `BLOCK_PAGES` missing). `test_every_gif_in_media_is_shown_on_a_page` passes vacuously while `docs/media/rev-a/` is empty; it starts guarding from plan 5.1.

- [ ] **Step 3: Implement the generator**

```python
#!/usr/bin/env python3
"""Generate the site's result pages from hardware/characterized/ (run before mkdocs build):
one Markdown page per characterisation file (every check, PASS/FAIL, the commit that produced it)
and the filter bank's S21 plot from the committed Touchstone files."""
from __future__ import annotations

import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
CHAR = ROOT / "hardware" / "characterized"
OUT = ROOT / "site" / "docs" / "results"

# block module (hardware/neohw/blocks/<name>.py) -> its explanation page under site/docs/
BLOCK_PAGES = {
    "protection": "blocks/protection.md", "bias_tee": "blocks/bias-tee.md", "bias_switch": "blocks/bias-tee.md",
    "attenuator": "blocks/attenuator.md", "bpf": "blocks/filter-bank.md", "bpf_bank": "blocks/filter-bank.md",
    "preamp": "blocks/preamp.md", "mixer": "blocks/mixer.md", "clock": "blocks/clock.md",
    "cal_tone": "blocks/clock.md", "controller": "blocks/controller.md", "power": "blocks/power.md",
    "board": "blocks/board.md",
}


def _fmt(v) -> str:
    return f"{v:.4g}" if isinstance(v, (int, float)) and not isinstance(v, bool) else str(v)


def result_page(doc: dict) -> str:
    status = "PASS" if doc.get("status") == "pass" else "FAIL"
    lines = [f"# {doc['block']} results", "",
             f"Status: **{status}** · generated by commit `{doc.get('generated_by', '?')}` "
             f"(`hardware/characterized/{doc['block']}.json`)", "",
             "| Check | Value | Unit | Limit | Result |", "|---|---|---|---|---|"]
    for c in doc.get("checks", []):
        lines.append(f"| `{c['name']}` | {_fmt(c['value'])} | {c.get('unit', '')} | {c['limit']} | "
                     f"{'PASS' if c['passed'] else '**FAIL**'} |")
    return "\n".join(lines) + "\n"


def _s21_db(path: Path) -> tuple[np.ndarray, np.ndarray]:
    """Frequency (Hz) and |S21| (dB) from a Touchstone v1 file in RI or MA/DB format."""
    unit, fmt, rows = 1.0, "RI", []
    for line in path.read_text().splitlines():
        line = line.strip()
        if line.startswith("#"):
            tok = line[1:].upper().split()
            unit = {"HZ": 1.0, "KHZ": 1e3, "MHZ": 1e6, "GHZ": 1e9}.get(tok[0], 1.0)
            fmt = next((t for t in tok if t in ("RI", "MA", "DB")), "RI")
        elif line and not line.startswith("!"):
            rows.append([float(x) for x in line.split()[:9]])
    a = np.array(rows)
    f, x, y = a[:, 0] * unit, a[:, 3], a[:, 4]
    if fmt == "RI":
        return f, 20 * np.log10(np.maximum(np.hypot(x, y), 1e-30))
    if fmt == "MA":
        return f, 20 * np.log10(np.maximum(x, 1e-30))
    return f, x


def filter_bank_plot(dest: Path) -> None:
    fig, ax = plt.subplots(figsize=(9, 4.5))
    for n in range(1, 12):
        p = CHAR / f"bpf_{n}.s2p"
        if p.exists():
            f, s21 = _s21_db(p)
            ax.semilogx(f / 1e6, s21, lw=1, label=f"{n}")
    ax.set(xlabel="frequency (MHz)", ylabel="S21 (dB)", ylim=(-100, 5), xlim=(0.05, 200),
           title="Band-pass filter bank, each filter selected (simulated)")
    ax.grid(True, which="both", alpha=.3)
    ax.legend(ncol=11, fontsize=7, loc="lower center")
    fig.tight_layout()
    fig.savefig(dest, dpi=120)
    plt.close(fig)


def main() -> None:
    OUT.mkdir(parents=True, exist_ok=True)
    index = ["# Simulation results", "",
             "Generated from the committed characterisation files on every build.", "",
             "| Block | Status | Page |", "|---|---|---|"]
    for path in sorted(CHAR.glob("*.json")):
        doc = json.loads(path.read_text())
        doc.setdefault("block", path.stem)
        (OUT / f"{path.stem}.md").write_text(result_page(doc), encoding="utf-8")
        index.append(f"| {doc['block']} | {'PASS' if doc.get('status') == 'pass' else '**FAIL**'} | "
                     f"[{path.stem}]({path.stem}.md) |")
    index += ["", "![Filter bank S21](filter-bank-s21.png)", ""]
    (OUT / "index.md").write_text("\n".join(index), encoding="utf-8")
    filter_bank_plot(OUT / "filter-bank-s21.png")


if __name__ == "__main__":
    main()
```

Before writing the Touchstone reader, open `hardware/neohw/touchstone.py` and confirm the format `write_s2p` emits (option line and data layout); if it is RI with nine columns per line, the reader above is right; adjust the column indices if not.

Add to `nav` in `mkdocs.yml`: `- Results: results/index.md` (MkDocs lists the per-block result pages under it automatically only when they are in `nav`; add each `results/<block>.md` explicitly, in the order of the signal chain).

Create empty-for-now page files for every `BLOCK_PAGES` value (Task 3 writes them) with a single heading each, so `test_every_block_module_has_a_page` and the strict build pass.

- [ ] **Step 4: Run the tests**

Run: `cd site && python3 -m pytest -q tests && cd .. && make site`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add site
git commit -m "docs(site): result pages and filter-bank plot generated from the characterisation; coverage tests"
```

### Task 2a: Circuit drawings from code

**Files:**
- Create: `site/figures.py` (one function per drawing), output `site/docs/img/circuits/<name>.svg` (generated at build, gitignored)
- Modify: `site/gen_pages.py` (`main()` calls `figures.draw_all(OUT_IMG)`), `.gitignore` (`site/docs/img/circuits/`)

**Interfaces:**
- Produces: `figures.DRAWINGS: dict[str, Callable[[], schemdraw.Drawing]]` with at least the keys `protection`, `bias-tee`, `attenuator`, `bpf-ladder`, `preamp`, `tayloe`, `baseband`, `lo-path`, `power-tree`, `relay-drive`; `figures.draw_all(dest: Path) -> list[Path]`.

- [ ] **Step 1: Failing test**

```python
def _figures():
    spec = importlib.util.spec_from_file_location("figures", SITE / "figures.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def test_every_circuit_drawing_renders(tmp_path):
    fig = _figures()
    required = {"protection", "bias-tee", "attenuator", "bpf-ladder", "preamp", "tayloe", "baseband",
                "lo-path", "power-tree", "relay-drive"}
    assert required <= set(fig.DRAWINGS)
    out = fig.draw_all(tmp_path)
    assert len(out) == len(fig.DRAWINGS)
    for p in out:
        assert p.suffix == ".svg" and p.stat().st_size > 1000 and "<svg" in p.read_text()
```

- [ ] **Step 2: Run to see it fail.** Run: `cd site && python3 -m pytest -q tests -k circuit_drawing`. Expected: FAIL (no `figures.py`).

- [ ] **Step 3: Implement** `figures.py`. Each drawing is a simplified schematic of the block **with the component values read from the code**, not typed: e.g. the attenuator reads its resistor values from `neohw.blocks.attenuator`, the BPF ladder draws filter 7's positions from `neohw.blocks.bpf.design(7).positions` (values formatted with `neohw.units`), the bias-T from `bias_switch.SHUNT, RE, RL, RSET`. Insert `hardware/` on `sys.path` the way `tools/rfq_parts_list.py` does. Pattern:

```python
from __future__ import annotations

import sys
from pathlib import Path
from typing import Callable

import schemdraw
import schemdraw.elements as elm

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "hardware"))

from neohw.blocks import bias_switch  # noqa: E402


def bias_tee() -> schemdraw.Drawing:
    d = schemdraw.Drawing(show=False)
    d += elm.Dot(open=True).label("+5V_BIAS", "left")
    d += elm.Resistor().right().label(f"{bias_switch.SHUNT} Ω shunt")
    d += elm.Ic(pins=[elm.IcPin("IN", side="left"), elm.IcPin("OUT", side="right")], edgepadW=1.5).label("SY6280")
    d += elm.Line().right().length(1)
    d += elm.Dot(open=True).label("bias feed", "right")
    return d


DRAWINGS: dict[str, Callable[[], schemdraw.Drawing]] = {"bias-tee": bias_tee}   # plus the other nine


def draw_all(dest: Path) -> list[Path]:
    dest.mkdir(parents=True, exist_ok=True)
    out = []
    for name, make in DRAWINGS.items():
        path = dest / f"{name}.svg"
        make().save(str(path))
        out.append(path)
    return out
```
Write all ten drawings in this style; each must show the parts that the block page's text explains.

- [ ] **Step 4: Run** `cd site && python3 -m pytest -q tests && cd .. && make site`. Expected: PASS.

- [ ] **Step 5: Commit** `git commit -m "docs(site): circuit drawings generated from the block code with schemdraw"`

### Task 2b: Start here: radio from first principles, and the glossary

**Files:**
- Write: `site/docs/start/{index,radio-waves,bands-and-modes,sdr,signal-chain,reading-the-site}.md`, `site/docs/glossary.md`
- Create: `site/start_figures.py` (matplotlib figures for the start pages, written to `site/docs/img/start/`, generated at build, gitignored); call it from `gen_pages.main()`
- Modify: `site/mkdocs.yml` (nav: "Start here" first, "Glossary" last; enable `abbr` and `pymdownx.snippets` with `auto_append: [glossary-abbr.md]` so every page gets hover definitions)

- [ ] **Step 1: Failing tests**

```python
START = ("radio-waves", "bands-and-modes", "sdr", "signal-chain", "reading-the-site")


def test_start_here_comes_first_and_is_complete():
    nav = (SITE / "mkdocs.yml").read_text()
    assert nav.index("start/index.md") < nav.index("blocks/index.md")
    for name in START:
        page = SITE / "docs/start" / f"{name}.md"
        text = page.read_text(encoding="utf-8")
        assert len(text.split()) >= 400, f"{name}: too short to teach the topic"
        assert "![" in text, f"{name}: no picture"


def test_glossary_defines_the_terms_the_site_uses():
    gloss = (SITE / "docs/glossary.md").read_text(encoding="utf-8")
    terms = ("HF", "SDR", "I/Q", "LO", "SSB", "CW", "FT8", "Tayloe", "band-pass", "Chebyshev", "Monte Carlo",
             "S-parameters", "noise figure", "IP3", "image rejection", "UAC2", "CAT", "DFU", "SPICE", "netlist",
             "Gerber", "stackup", "GCPW", "FDTD", "bias-T", "LDO", "buck converter", "BOM", "LCSC", "Torob")
    missing = [t for t in terms if f"**{t}**" not in gloss]
    assert not missing, f"glossary lacks {missing}"


def test_every_page_has_a_picture():
    for page in (SITE / "docs").rglob("*.md"):
        if page.parent.name == "results" or page.name in ("glossary.md", "glossary-abbr.md"):
            continue
        assert "![" in page.read_text(encoding="utf-8") or "```mermaid" in page.read_text(encoding="utf-8"), page
```

- [ ] **Step 2: Run to see them fail.** Expected: FAIL.

- [ ] **Step 3: Write the pages** (each at least 400 words, one idea per section, a picture per main idea):
- `radio-waves.md`: what an electromagnetic wave is, frequency and wavelength (λ = c/f, with the 3–30 MHz HF range as 100–10 m), how the ionosphere carries HF around the world (with a figure of skywave paths drawn in matplotlib by `start_figures.py`), why antennas are long at HF.
- `bands-and-modes.md`: the band plan Elf covers (a figure of the 0.1–54 MHz spectrum with the amateur and broadcast bands from the BPF band table), modulation: AM, SSB, CW, FT8, each with a figure (spectrum sketch, waveform), and why a receiver needs filters (strong signals elsewhere).
- `sdr.md`: what a software-defined receiver does, mixing down to baseband, why two channels (I and Q) are needed to tell positive from negative frequencies (a figure of a tone at ±f in I/Q), sampling and the 96 kHz window, where the computer takes over.
- `signal-chain.md`: Elf from antenna to decoded text, one section per block in order, each with a one-paragraph explanation, its circuit drawing from Task 2a and a link to its page; the Mermaid chain diagram at the top.
- `reading-the-site.md`: how the site is organised, how the design is verified (simulation first, then the real board), what "China primary / Iranian fallback" means.
- `glossary.md`: every term in the test list (and any other term a page uses) as `**Term**: definition` in alphabetical order; `glossary-abbr.md` with `*[HF]: High frequency, 3–30 MHz` style lines for the hover definitions.

- [ ] **Step 4: Run** `cd site && python3 -m pytest -q tests && cd .. && make site`. Expected: PASS.

- [ ] **Step 5: Commit** `git commit -m "docs(site): start-here path from radio basics to Elf's signal chain; glossary"`

### Task 3: Block pages

**Files:**
- Write: `site/docs/blocks/{protection,bias-tee,attenuator,filter-bank,preamp,mixer,clock,controller,power,board}.md`, `site/docs/blocks/index.md`
- Modify: `site/mkdocs.yml` (nav "Board blocks")

**Interfaces:**
- Consumes: `BLOCK_PAGES` (Task 2); result pages `results/<block>.md`.

- [ ] **Step 1: Add a content test**

```python
def test_block_pages_have_the_required_sections():
    required = ("## What it does", "## Circuit", "## Parts", "## Verification")
    for page in set(_gen().BLOCK_PAGES.values()):
        text = (SITE / "docs" / page).read_text(encoding="utf-8")
        missing = [h for h in required if h not in text]
        assert not missing, f"{page} lacks {missing}"
        assert "](../results/" in text, f"{page} does not link its result page"
```

- [ ] **Step 2: Run it to see it fail**

Run: `cd site && python3 -m pytest -q tests -k block_pages`
Expected: FAIL (pages hold only a heading).

- [ ] **Step 3: Write each page** with exactly these sections, from these sources:

| Page | Sources |
|---|---|
| every page | the block's module docstring and code in `hardware/neohw/blocks/`, its check in `hardware/neohw/checks/`, the README feature list, `2026-10-18-rev-a-china-fab-design.md` §0 and §2 for part changes |
| protection, bias-tee, attenuator | parent spec §4 items 1–4; phase 3 spec §3.3 (bias switch); `bias_switch.py` docstring |
| filter-bank | parent spec §4 item 5 and the band table; phase 2 spec BPF design row and prototype result; `bpf.py` constants (ripple, margins, orders) |
| preamp | phase 2 spec preamp section; `preamp.py` |
| mixer | phase 2 spec mixer and baseband sections; `mixer.py`; README "Simulated performance" mixer row |
| clock | phase 3 spec §3.1 and its amendments (4.8 MHz crossover, skew budget, cal tone) |
| controller | phase 3 spec §3.3; Rev A spec §3 (STM32F411, marked "planned, plan 5.2") |
| power | plan 3.4 spec text in the phase 3 spec; Rev A spec §0 (LM2576HV kept) |
| board | phase 3 spec §3 "Board check"; `checks/board.py` |

Sections per page:
- `## What it does`: two to four sentences, its place in the signal chain.
- `## Circuit`: the topology in words, explained component by component for a reader new to it, with the block's `schemdraw` drawing from Task 2a (`![…](../img/circuits/<name>.svg)`) and, where it helps, a Mermaid diagram.
- `## Parts`: a table, one row per significant part: part, role, why chosen, China primary (MPN, LCSC if known), Iranian fallback. Parts the Rev A spec changes but plan 5.1 has not yet implemented are marked "planned (plan 5.1)".
- `## Verification`: the targets in words, then `See the [results](../results/<block>.md).`
- Optional `## Notes`: decisions and what was rejected and why (from the specs' amendment notes).

`blocks/index.md`: the signal chain diagram again, with each node linking its page.

Plans 5.3 and 5.4 add per-block 3D close-up renders to these pages.

- [ ] **Step 4: Run** `cd site && python3 -m pytest -q tests && cd .. && make site` — Expected: PASS.

- [ ] **Step 5: Commit** `git commit -m "docs(site): a page per board block: function, circuit, parts with China and Iran sources, verification"`

### Task 4: Technique pages, firmware, software, building

**Files:**
- Write: `site/docs/techniques/{index,quadrature-sampling,filter-synthesis,si5351-quadrature,monte-carlo,dual-population,circuits-as-code,layout-as-code,coplanar-lines,em-simulation,digital-twin,virtual-board}.md`, `site/docs/firmware.md`, `site/docs/software.md`, `site/docs/building.md`, `site/docs/layout.md`
- Modify: `site/mkdocs.yml` (nav)

- [ ] **Step 1: Add a test**

```python
TECHNIQUES = ("quadrature-sampling", "filter-synthesis", "si5351-quadrature", "monte-carlo", "dual-population",
              "circuits-as-code", "layout-as-code", "coplanar-lines", "em-simulation", "digital-twin", "virtual-board")


def test_technique_pages_exist_and_are_in_nav():
    nav = (SITE / "mkdocs.yml").read_text()
    for t in TECHNIQUES:
        page = SITE / "docs/techniques" / f"{t}.md"
        assert page.exists() and len(page.read_text().split()) >= 150, t
        assert f"techniques/{t}.md" in nav, t
```

- [ ] **Step 2: Run it to see it fail.** Run: `cd site && python3 -m pytest -q tests -k technique`. Expected: FAIL.

- [ ] **Step 3: Write the pages.** Each technique page (at least 150 words) has: the idea in plain words, the maths where it helps (MathJax, e.g. the Tayloe detector's sampling into four capacitors and the I = 0°−180°, Q = 90°−270° differences; the Chebyshev ladder prototype; `f_out = f_ref · (a + b/c) / MS` for the Si5351), how Elf applies it with links to the code files, and its limits. Sources: phase 1 spec (twin), phase 2 spec (synthesis, Monte Carlo, mixer), phase 3 spec §3.1 (Si5351 phase offset, ÷4 path), phase 4 spec (virtual board, USB/IP), Rev A spec §2, §4, §6 (dual population, layout as code, GCPW, openEMS). `layout-as-code`, `coplanar-lines`, `em-simulation` and `layout.md` describe the method and say the results arrive with plan 5.3.
`firmware.md`: structure (`core/`, `usb/`, `host/`), CAT command list (phase 4 spec), USB audio, the update flow and BOOT switch (Rev A spec §3, planned plan 5.2).
`software.md`: the C# DSP library and CLI (`pi/`), SDR++ and WSJT-X use (README).
`building.md`: tool list from Rev A spec §8 and the `make` targets from the Makefile.

- [ ] **Step 4: Run** `cd site && python3 -m pytest -q tests && cd .. && make site` — Expected: PASS.

- [ ] **Step 5: Commit** `git commit -m "docs(site): techniques, firmware, software and build pages"`

### Task 5: Reports in the site

**Files:**
- Create: `site/docs/reports/index.md`; copies `site/docs/reports/hw-report.html`, `site/docs/reports/sim-report.html` (committed)
- Modify: `site/mkdocs.yml` (nav "Reports")

- [ ] **Step 1: Add a test**

```python
def test_reports_are_published_from_committed_copies(tmp_path):
    for name in ("hw-report.html", "sim-report.html"):
        assert (SITE / "docs/reports" / name).exists(), name
    r = build_site(tmp_path / "out")
    assert r.returncode == 0, r.stderr
    assert (tmp_path / "out/reports/hw-report.html").exists()
```

- [ ] **Step 2: Run it to see it fail.** Expected: FAIL (no copies yet).

- [ ] **Step 3:** Run `make hw` and `make sim` if `build/hw/report.html` or `build/sim/report.html` is older than the last change under `hardware/` or `sim/` (`git log -1 --format=%ct -- hardware sim` against the file times), then `make site-reports`. Write `reports/index.md`: what each report contains, when it was generated (the commit in its header), links to both files.

- [ ] **Step 4: Run** `cd site && python3 -m pytest -q tests` — Expected: PASS.

- [ ] **Step 5: Commit** `git commit -m "docs(site): hardware and simulation reports published with the site"`

### Task 6: GitHub Actions publishing and the README link

**Files:**
- Create: `.github/workflows/pages.yml`
- Modify: `README.md` (link near the top; "Building and running" mentions `make site`)

- [ ] **Step 1: Write the workflow**

```yaml
name: pages
on:
  push:
    branches: [main]
  workflow_dispatch: {}
permissions:
  contents: read
  pages: write
  id-token: write
concurrency:
  group: pages
  cancel-in-progress: true
jobs:
  build:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4
      - uses: actions/setup-python@v5
        with:
          python-version: "3.12"
      - run: pip install -r site/requirements.txt
      - run: python site/gen_pages.py
      - run: mkdocs build --strict -f site/mkdocs.yml -d ../build/site
      - uses: actions/upload-pages-artifact@v3
        with:
          path: build/site
  deploy:
    needs: build
    runs-on: ubuntu-latest
    environment:
      name: github-pages
      url: ${{ steps.deployment.outputs.page_url }}
    steps:
      - id: deployment
        uses: actions/deploy-pages@v4
```

- [ ] **Step 2: Validate locally.** Run: `python3 -c "import yaml; yaml.safe_load(open('.github/workflows/pages.yml'))"` and a clean-path build: `git stash -u` is NOT used; instead `rm -rf build/site site/docs/results && make site`. Expected: no error.

- [ ] **Step 3: Ask the user** to confirm switching Pages on (Settings → Pages → Source: GitHub Actions), or to allow `gh api -X POST repos/Neo-vortex/Elf/pages -f build_type=workflow`. Do not push or enable anything before the answer.

- [ ] **Step 4: README.** Add under the title: `Documentation: https://neo-vortex.github.io/Elf/` (the URL GitHub Pages assigns for this repository; confirm it from `gh api repos/Neo-vortex/Elf/pages --jq .html_url` once enabled) and add `make site` to "Building and running".

- [ ] **Step 5: Commit** `git commit -m "ci: publish the documentation site to GitHub Pages on push to main"`. Pushing to `origin` happens only when the user asks.

## Self-review notes

- Self-contained requirement: start-here path and glossary (Task 2b), pictures on every page (tests in Task 2b, drawings in Task 2a), offline assets (Task 1 test).
- Spec §7b coverage: tool (Task 1), Actions publishing (Task 6, with the user's confirmation), content sections: overview (1), blocks (3), techniques (4), layout and manufacturing (4, `layout.md`, filled by plans 5.3/5.4), firmware and software (4), reports (5), building (4); "kept current" coverage tests (2); three.js viewer is plan 5.4.
