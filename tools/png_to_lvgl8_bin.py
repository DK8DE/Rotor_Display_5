#!/usr/bin/env python3
"""
PNG → LVGL-8-Bin (RGB565, LV_COLOR_DEPTH=16, LV_COLOR_16_SWAP=1).

Standard: Alle *.png unter imgs/ (rekursiv) → data/img/ mit gleichem relativen
Pfad und Endung .bin — vorhandene Dateien werden überschrieben.

  python tools/png_to_lvgl8_bin.py

Abhängigkeit: pip install pillow  (siehe requirements.txt)

Optional:
  --no-alpha   TRUE_COLOR (2 B/px), sonst TRUE_COLOR_ALPHA (3 B/px)
  -i / -o      eine einzelne Datei manuell
"""
from __future__ import annotations

import argparse
import struct
import sys
from pathlib import Path

try:
    from PIL import Image
except ImportError:
    print("Bitte installieren: pip install pillow", file=sys.stderr)
    sys.exit(1)

LV_IMG_CF_TRUE_COLOR = 4
LV_IMG_CF_TRUE_COLOR_ALPHA = 5


def pack_header(cf: int, w: int, h: int) -> bytes:
    word = (cf & 0x1F) | (0 << 5) | (0 << 8) | ((w & 0x7FF) << 10) | ((h & 0x7FF) << 21)
    return struct.pack("<I", word)


def rgb888_to_lv_color16_swap1(r: int, g: int, b: int) -> int:
    c = (int(r) << 16) | (int(g) << 8) | int(b)
    full = (
        ((c & 0xF80000) >> 16)
        | ((c & 0xFC00) >> 13)
        | ((c & 0x1C00) << 3)
        | ((c & 0xF8) << 5)
    )
    return full & 0xFFFF


def write_u16_le(buf: bytearray, offset: int, value: int) -> int:
    buf[offset] = value & 0xFF
    buf[offset + 1] = (value >> 8) & 0xFF
    return offset + 2


def convert_png(png_path: Path, out_path: Path, *, use_alpha: bool) -> None:
    im = Image.open(png_path).convert("RGBA")
    w, h = im.size
    pixels = im.load()

    cf = LV_IMG_CF_TRUE_COLOR_ALPHA if use_alpha else LV_IMG_CF_TRUE_COLOR
    bpp = 3 if use_alpha else 2
    out = bytearray(4 + w * h * bpp)
    out[:4] = pack_header(cf, w, h)
    o = 4

    for y in range(h):
        for x in range(w):
            r, g, b, a = pixels[x, y]
            c16 = rgb888_to_lv_color16_swap1(r, g, b)
            o = write_u16_le(out, o, c16)
            if use_alpha:
                out[o] = a
                o += 1

    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_bytes(out)
    rel_out = out_path.as_posix()
    print(f"{png_path.name} ({w}x{h}) -> {rel_out}  cf={cf}  {len(out)} B")


def process_imgs_folder(root: Path, *, use_alpha: bool) -> int:
    imgs = root / "imgs"
    out_root = root / "data" / "img"
    if not imgs.is_dir():
        print(f"Ordner fehlt: {imgs}", file=sys.stderr)
        return 1

    pngs = sorted(imgs.rglob("*.png")) + sorted(imgs.rglob("*.PNG"))
    # doppelte Einträge vermeiden (case-insensitive Doppel)
    seen: set[Path] = set()
    unique: list[Path] = []
    for p in pngs:
        rp = p.resolve()
        if rp not in seen:
            seen.add(rp)
            unique.append(p)

    if not unique:
        print(f"Keine PNGs in {imgs}")
        return 0

    for png_path in unique:
        rel = png_path.relative_to(imgs)
        out_path = out_root / rel.with_suffix(".bin")
        convert_png(png_path, out_path, use_alpha=use_alpha)

    print(f"Fertig: {len(unique)} Datei(en) -> {out_root}")
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description="PNG → LVGL8 .bin (imgs/ → data/img/)")
    ap.add_argument("--input", "-i", type=Path, help="Einzelne Eingabe-PNG")
    ap.add_argument("--output", "-o", type=Path, help="Einzelne Ausgabe-.bin")
    ap.add_argument(
        "--no-alpha",
        action="store_true",
        help="TRUE_COLOR (2 B/px); Standard ist TRUE_COLOR_ALPHA (3 B/px)",
    )
    args = ap.parse_args()

    root = Path(__file__).resolve().parent.parent
    use_alpha = not args.no_alpha

    if args.input and args.output:
        p_in = args.input if args.input.is_absolute() else root / args.input
        p_out = args.output if args.output.is_absolute() else root / args.output
        convert_png(p_in, p_out, use_alpha=use_alpha)
        return 0

    if args.input or args.output:
        print("Entweder beide (-i und -o) oder keines (dann gesamter imgs/-Ordner).", file=sys.stderr)
        return 1

    return process_imgs_folder(root, use_alpha=use_alpha)


if __name__ == "__main__":
    raise SystemExit(main())
