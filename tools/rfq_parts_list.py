#!/usr/bin/env python3
"""Parts list for a board maker's stock check and quote, generated from the board netlist:
build/Elf_parts_list_RFQ.xlsx with a main-parts sheet (ICs, semiconductors, relays, crystals,
protection, connectors; English and Persian notes; do-not-substitute and assembly notes) and the
full BOM. Rows for the planned signal meter are marked PLANNED until it is in the netlist."""
from __future__ import annotations

import sys
from collections import defaultdict
from pathlib import Path

from openpyxl import Workbook
from openpyxl.styles import Font, PatternFill

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "hardware"))

from neohw.blocks import board  # noqa: E402

FA = {
    "ESP32-S3-WROOM-1": "ماژول کنترلر و USB (حتماً نسخه N16R8)",
    "Si5351C": "سنتزکننده فرکانس (QFN-16، فاصله پایه ۰٫۵ میلی‌متر، پد حرارتی زیر آی‌سی)",
    "CS5361": "مبدل آنالوگ به دیجیتال ۲۴ بیتی (TSSOP-24، فاصله پایه ۰٫۶۵ میلی‌متر)",
    "FST3253": "سوئیچ میکسر", "LM4562": "آپ‌امپ کم‌نویز", "74LVC74": "تقسیم‌کننده فرکانس",
    "74LVC157": "مالتی‌پلکسر", "74HC595": "شیفت رجیستر رله‌ها", "ULN2803": "درایور رله",
    "LM2576HV-ADJ": "رگولاتور سوئیچینگ ۹ تا ۳۰ ولت — حتماً نسخه HV (۶۰ ولت) و اصل، نه کپی",
    "MIC5205-5.0": "رگولاتور کم‌نویز ۵ ولت", "MIC5205-3.3": "رگولاتور کم‌نویز ۳٫۳ ولت",
    "AMS1117-3.3": "رگولاتور ۳٫۳ ولت", "SY6280": "سوئیچ بایاس آنتن با محدودکننده جریان",
    "LM75A": "سنسور دما", "USBLC6-2SC6": "محافظ ESD پورت USB", "BFR92A": "ترانزیستور RF پیش‌تقویت‌کننده",
    "G6K-2F": "رله امرون ۵ ولت — بیشترین هزینه، موجودی را اول بررسی کنید",
    "25MHz": "کریستال ۲۵ مگاهرتز، بار ۸ پیکوفاراد (CL=8pF) — مقدار بار مهم است",
    "24.576MHz": "اسیلاتور ۲۴٫۵۷۶ مگاهرتز ۳٫۳ ولت", "SS36": "دیود شاتکی ۶۰ ولت ۳ آمپر (پکیج SMC)",
    "SMBJ30A": "دیود TVS محافظ ورودی (جهت‌دار)", "GDT 90V": "برق‌گیر گازی ورودی آنتن",
    "DC jack 5.5/2.1": "جک آداپتور ورودی ۹ تا ۳۰ ولت",
    "PTC 0.75A 33V": "فیوز ری‌ست‌شونده PTC، جریان نگه‌داری ۰٫۷۵ آمپر، حداقل ۳۳ ولت",
    "USB-C 16P": "کانکتور USB-C ۱۶ پین",
}
GROUP = {"U": "1 ICs", "Q": "2 Transistors", "D": "3 Diodes / LEDs", "K": "4 Relays", "X": "5 Crystals / oscillators",
         "GDT": "6 Protection", "F": "6 Protection", "J": "7 Connectors / switches", "L": "8 Inductors",
         "C": "9 Capacitors", "R": "9 Resistors"}
SPECIAL = {"button": "7 Connectors / switches", "USB-C 16P": "7 Connectors / switches", "DC jack 5.5/2.1": "7 Connectors / switches",
           "24.576MHz": "5 Crystals / oscillators", "25MHz": "5 Crystals / oscillators", "SS36": "3 Diodes / LEDs",
           "SMBJ30A": "3 Diodes / LEDs"}
PLANNED = [["1 ICs", "74HC595", "74HC595D", "SOIC-16", 1, "PLANNED: signal meter (pending approval)", "شیفت رجیستر نمایشگر سیگنال — برنامه‌ریزی‌شده", ""],
           ["3 Diodes / LEDs", "LED green", "LED 0805 green", "0805", 5, "PLANNED: signal meter S1–S9", "ال‌ای‌دی سبز ۰۸۰۵ — نمایشگر سیگنال", ""],
           ["3 Diodes / LEDs", "LED yellow", "LED 0805 yellow", "0805", 2, "PLANNED: signal meter S9+10/+20", "ال‌ای‌دی زرد ۰۸۰۵", ""],
           ["3 Diodes / LEDs", "LED red", "LED 0805 red", "0805", 1, "PLANNED: signal meter S9+40 / overload", "ال‌ای‌دی قرمز ۰۸۰۵", ""],
           ["9 Resistors", "1kΩ", "", "0805", 8, "PLANNED: signal meter LED resistors", "", ""]]
NOTES = [
    [], ["Do not substitute / قطعاتی که جایگزین نشوند:"],
    ["• Filter capacitors: C0G/NP0, 2 % (not X7R) — خازن‌های فیلتر حتماً C0G/NP0 با دقت ۲٪"],
    ["• Filter inductors: selected to ±3 % with an LCR meter — سلف‌های فیلتر با دقت ۳٪ انتخاب‌شده"],
    ["• LM2576HV-ADJ: HV version (60 V), genuine — نسخه HV و اصل"],
    ["• Buck inductor 100 µH: Isat ≥ 2 A per its datasheet (12 × 12 mm) — سلف ۱۰۰ میکروهانری با جریان اشباع حداقل ۲ آمپر"],
    ["• 1000 µF / 16 V low-ESR electrolytic at the buck output — خازن الکترولیت ۱۰۰۰ میکروفاراد ESR پایین"],
    ["• Input fuse: resettable PTC, hold 0.75 A, ≥ 33 V — فیوز PTC ری‌ست‌شونده"],
    ["• 25 MHz crystal: CL = 8 pF — کریستال با بار ۸ پیکوفاراد"],
    ["• Relays: Omron G6K-2F 5 VDC — رله امرون"],
    [], ["Assembly notes / نکات مونتاژ:"],
    ["• Fine pitch: Si5351C QFN-16 0.5 mm with exposed pad; CS5361 TSSOP-24 0.65 mm — فاصله پایه ریز"],
    ["• ESP32-S3 module and QFN are moisture sensitive (MSL 3): bake if the package was opened — حساس به رطوبت"],
    ["• Mixed SMD + through-hole (drum choke, axial inductors, jack, USB-C shell) — ترکیب SMD و پایه‌دار"],
    ["• ENIG finish; 4 layers with 50 Ω controlled impedance: please send your stackup — آبکاری ENIG و امپدانس کنترل‌شده ۵۰ اهم"],
    ["• Wash flux near the mixer and baseband amplifiers — شستشوی فلاکس"],
]


def main() -> Path:
    b = board.build()
    groups = defaultdict(list)
    for p in b.circuit.parts:
        pre = p.ref.rstrip("0123456789")
        groups[(pre, p.value, getattr(p, "mpn", "") or "", p.footprint.split(":")[-1], getattr(p, "bom_note", "") or "")].append(p.ref)
    rows = []
    for (pre, val, mpn, fp, note), refs in groups.items():
        cat = SPECIAL.get(val) or ("3 Diodes / LEDs" if val.startswith("LED") else GROUP.get(pre, "1 ICs"))
        rows.append([cat, val, mpn, fp, len(refs), note, FA.get(val, ""), " ".join(sorted(refs, key=lambda r: (len(r), r)))])
    rows.sort(key=lambda r: (r[0], -r[4], r[1]))
    wb = Workbook()
    main_ws, full = wb.active, wb.create_sheet("Full BOM")
    main_ws.title = "Main parts (ICs etc.)"
    for ws in (main_ws, full):
        ws.append([f"Elf HF receiver — parts list for stock check and quote (preliminary). {len(b.circuit.parts)} parts; "
                   "4-layer board, about 140 × 100 mm."])
        ws.append(["فهرست قطعات برای استعلام موجودی و قیمت — نسخه اولیه"])
        ws.append([])
        ws.append(["Category", "Part / value", "Manufacturer part no.", "Package / footprint", "Qty", "Notes", "توضیح فارسی", "References"])
        for c in ws[4]:
            c.font = Font(bold=True, color="FFFFFF")
            c.fill = PatternFill("solid", fgColor="305496")
        ws["A1"].font = Font(bold=True, size=13)
        for col, width in zip("ABCDEFGH", (22, 20, 28, 34, 6, 44, 52, 40)):
            ws.column_dimensions[col].width = width
    for r in rows + PLANNED:
        full.append(r)
    for r in sorted([r for r in rows if not r[0].startswith(("8", "9"))] + [p for p in PLANNED if not p[0].startswith("9")],
                    key=lambda r: (r[0], -r[4])):
        main_ws.append(r[:7])
    for r in NOTES:
        main_ws.append(r)
    out = ROOT / "build" / "Elf_parts_list_RFQ.xlsx"
    out.parent.mkdir(exist_ok=True)
    wb.save(out)
    return out


if __name__ == "__main__":
    print(main())
