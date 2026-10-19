"""Project-local KiCad 9 footprints (library `NeoRadio`, hardware/footprints/NeoRadio.pretty)."""
from __future__ import annotations

from pathlib import Path

FOOTPRINT_LIB = Path(__file__).resolve().parents[1] / "footprints" / "NeoRadio.pretty"
# SMD pad x and size per chip size, as in KiCad's Inductor_SMD:L_1210_3225Metric and
# L_1812_4532Metric (IPC-7351 nominal): 1210 for TDK NLV32 (< 1 µH), 1812 for TDK NL453232 (≥ 1 µH)
SMD_PADS = {"1210": (1.4, (1.25, 2.65)), "1812": (2.1375, (1.125, 3.4))}
# axial body: Bourns 78F, 7.11 x 2.79 mm max, on a 10.16 mm pitch (as Inductor_THT:L_Axial_L7.0mm_D3.3mm_P10.16mm)
THT_PAD_X, THT_DRILL, THT_PAD = 5.08, 0.8, 1.6
BODY = (7.2, 2.9)
LINK_WIDTH = 0.8        # F.Cu track joining each SMD pad to its THT pad
SILK_W, FAB_W, CRT_W = 0.12, 0.1, 0.05
COURTYARD_MARGIN = 0.25


def _line(x0, y0, x1, y1, layer: str, width: float) -> str:
    return (f'\t(fp_line\n\t\t(start {x0:g} {y0:g})\n\t\t(end {x1:g} {y1:g})\n'
            f'\t\t(stroke\n\t\t\t(width {width:g})\n\t\t\t(type solid)\n\t\t)\n\t\t(layer "{layer}")\n\t)\n')


def _rect(x0, y0, x1, y1, layer: str, width: float) -> str:
    return (f'\t(fp_rect\n\t\t(start {x0:g} {y0:g})\n\t\t(end {x1:g} {y1:g})\n'
            f'\t\t(stroke\n\t\t\t(width {width:g})\n\t\t\t(type solid)\n\t\t)\n\t\t(fill no)\n\t\t(layer "{layer}")\n\t)\n')


def _text(kind: str, value: str, y: float, layer: str) -> str:
    return (f'\t(property "{kind}" "{value}"\n\t\t(at 0 {y:g} 0)\n\t\t(layer "{layer}")\n'
            f'\t\t(effects\n\t\t\t(font\n\t\t\t\t(size 1 1)\n\t\t\t\t(thickness 0.15)\n\t\t\t)\n\t\t)\n\t)\n')


def combined_inductor_name(smd: str) -> str:
    return f"L_{smd}_Axial_P10.16mm"


def combined_inductor_text(smd: str = "1210") -> str:
    """Pads 1 and 2 each as an SMD pad of chip size `smd` (China: wirewound chip) and a THT pad
    for an axial inductor (Iran: Bourns 78F class), joined on F.Cu. One of the two is fitted."""
    name = combined_inductor_name(smd)
    smd_x, smd_size = SMD_PADS[smd]
    half_w, half_h = BODY[0] / 2, BODY[1] / 2
    silk_h = max(half_h, smd_size[1] / 2) + 0.12 + SILK_W / 2    # silk clears the SMD pads
    crt_x = THT_PAD_X + THT_PAD / 2 + COURTYARD_MARGIN
    crt_y = silk_h + SILK_W / 2 + COURTYARD_MARGIN
    out = (f'(footprint "{name}"\n\t(version 20241229)\n\t(generator "neohw.footprints")\n'
           '\t(layer "F.Cu")\n'
           f'\t(descr "Inductor, {smd} SMD pads and axial 7.1x2.8 mm THT pads at 10.16 mm pitch, '
           'one of the two fitted (NeoRadio China/Iran populations)")\n'
           f'\t(tags "inductor {smd} axial dual footprint")\n')
    out += _text("Reference", "REF**", -crt_y - 0.8, "F.SilkS")
    out += _text("Value", name, crt_y + 0.8, "F.Fab")
    out += "\t(attr through_hole)\n"
    out += _rect(-half_w, -silk_h, half_w, silk_h, "F.SilkS", SILK_W)
    out += _rect(-half_w, -half_h, half_w, half_h, "F.Fab", FAB_W)
    out += _line(-THT_PAD_X, 0, -half_w, 0, "F.Fab", FAB_W) + _line(half_w, 0, THT_PAD_X, 0, "F.Fab", FAB_W)
    out += _rect(-crt_x, -crt_y, crt_x, crt_y, "F.CrtYd", CRT_W)
    for sign in (-1, 1):
        out += _line(sign * THT_PAD_X, 0, sign * smd_x, 0, "F.Cu", LINK_WIDTH)
    for num, sign in (("1", -1), ("2", 1)):
        out += (f'\t(pad "{num}" smd roundrect\n\t\t(at {sign * smd_x:g} 0)\n'
                f'\t\t(size {smd_size[0]:g} {smd_size[1]:g})\n'
                '\t\t(layers "F.Cu" "F.Mask" "F.Paste")\n\t\t(roundrect_rratio 0.2)\n\t)\n')
        shape = "rect" if num == "1" else "circle"
        out += (f'\t(pad "{num}" thru_hole {shape}\n\t\t(at {sign * THT_PAD_X:g} 0)\n'
                f'\t\t(size {THT_PAD:g} {THT_PAD:g})\n\t\t(drill {THT_DRILL:g})\n'
                '\t\t(layers "*.Cu" "*.Mask")\n\t)\n')
    return out + "\t(embedded_fonts no)\n)\n"


def combined_inductor(lib: Path = FOOTPRINT_LIB, smd: str = "1210") -> Path:
    """Write NeoRadio:L_<smd>_Axial_P10.16mm into `lib` and return its path."""
    lib = Path(lib)
    lib.mkdir(parents=True, exist_ok=True)
    path = lib / f"{combined_inductor_name(smd)}.kicad_mod"
    path.write_text(combined_inductor_text(smd))
    return path
