#!/usr/bin/env python3
"""
PNG → LVGL-8-Bin (RGB565, LV_COLOR_DEPTH=16, LV_COLOR_16_SWAP=1).

Standard: Bekannte PNGs unter data/img/ (Kompass_V5, Kompass_EL, windPfeil)
→ data/img/ui_image_*.bin. Sonst: alle *.png unter imgs/ → data/img/.

  python tools/png_to_lvgl8_bin.py
  python tools/png_to_lvgl8_bin.py --from-data-img

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
        # Web-Flasher-Pakete unter IMGs/update|full-install nicht mitkonvertieren
        parts_l = {x.lower() for x in p.parts}
        if "update" in parts_l or "full-install" in parts_l:
            continue
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


# Alias: Quell-PNGs oft unter data/img mit EEZ-/Studio-Namen
# → Ziel-Bins mit Asset-Namen aus screens.c / images.c
DATA_IMG_ALIASES: dict[str, str] = {
    "Kompass_V5.png": "ui_image_kompass_bg.bin",
    "Kompass_EL.png": "ui_image_kompass_el.bin",
    "windPfeil.png": "ui_image_pfeil_wind.bin",
}


def process_data_img_aliases(root: Path, *, use_alpha: bool) -> int:
    """Konvertiert bekannte PNGs in data/img/ auf ui_image_*.bin (in-place)."""
    data_img = root / "data" / "img"
    if not data_img.is_dir():
        print(f"Ordner fehlt: {data_img}", file=sys.stderr)
        return 1
    n = 0
    for src_name, dst_name in DATA_IMG_ALIASES.items():
        src = data_img / src_name
        if not src.is_file():
            # case-insensitive Suche
            found = None
            for p in data_img.iterdir():
                if p.is_file() and p.name.lower() == src_name.lower():
                    found = p
                    break
            if not found:
                print(f"Hinweis: {src_name} fehlt in {data_img}")
                continue
            src = found
        convert_png(src, data_img / dst_name, use_alpha=use_alpha)
        n += 1
    if n == 0:
        print(f"Keine Alias-PNGs in {data_img}")
        return 1
    print(f"Fertig (data/img Aliase): {n} Datei(en)")
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description="PNG → LVGL8 .bin (imgs/ → data/img/)")
    ap.add_argument("--input", "-i", type=Path, help="Einzelne Eingabe-PNG")
    ap.add_argument("--output", "-o", type=Path, help="Einzelne Ausgabe-.bin")
    ap.add_argument(
        "--from-data-img",
        action="store_true",
        help="Bekannte PNGs in data/img (Kompass_V5/EL, windPfeil) → ui_image_*.bin",
    )
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

    if args.from_data_img:
        return process_data_img_aliases(root, use_alpha=use_alpha)

    # Standard: zuerst data/img-Aliase, falls vorhanden; sonst imgs/
    data_img = root / "data" / "img"
    has_alias = data_img.is_dir() and any(
        (data_img / n).is_file()
        or any(p.name.lower() == n.lower() for p in data_img.iterdir() if p.is_file())
        for n in DATA_IMG_ALIASES
    )
    if has_alias:
        return process_data_img_aliases(root, use_alpha=use_alpha)

    return process_imgs_folder(root, use_alpha=use_alpha)


if __name__ == "__main__":
    raise SystemExit(main())
