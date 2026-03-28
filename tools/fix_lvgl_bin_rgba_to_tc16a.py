#!/usr/bin/env python3
"""
Konvertiert EEZ-RGBA8888-Bins für LVGL 8 + LV_COLOR_DEPTH 16.

1) TRUE_COLOR_ALPHA (cf=5, 3 B/px): RGB565 little-endian + Alpha — passt NICHT zu
   LV_COLOR_16_SWAP==1 und EEZ-Alpha ist oft unzuverlässig.

2) TRUE_COLOR (cf=4, 2 B/px): nur RGB565, kein Alpha — zuverlässig für Vollflächen
   (Kompass). Byte-Reihenfolge wie bei LV_COLOR_16_SWAP in lv_conf.h.

Setze unten SWAP = True wenn LV_COLOR_16_SWAP 1 ist.

Verwendung:
  python tools/fix_lvgl_bin_rgba_to_tc16a.py data/img/ui_image_kompass_bg.bin
  (nutzt automatisch .bak_rgba falls vorhanden, sonst 4 B/px-Quelle)
"""
from __future__ import annotations

import struct
import sys
from pathlib import Path

# Zu lv_conf.h: LV_COLOR_16_SWAP — bei 1: High-Byte des RGB565 zuerst im File
SWAP_FOR_LV_COLOR_16_SWAP = True


def pack_header(cf: int, w: int, h: int) -> bytes:
    word = (cf & 0x1F) | (0 << 5) | (0 << 8) | ((w & 0x7FF) << 10) | ((h & 0x7FF) << 21)
    return struct.pack("<I", word)


def rgb565(r: int, g: int, b: int) -> int:
    return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3)


def write_rgb565_bytes(out: bytearray, o: int, c: int) -> int:
    lo = c & 0xFF
    hi = (c >> 8) & 0xFF
    if SWAP_FOR_LV_COLOR_16_SWAP:
        out[o] = hi
        out[o + 1] = lo
    else:
        out[o] = lo
        out[o + 1] = hi
    return o + 2


def main() -> int:
    paths = [Path(p) for p in sys.argv[1:]]
    if not paths:
        print("Keine Dateien.", file=sys.stderr)
        return 1

    # cf=4 LV_IMG_CF_TRUE_COLOR, 2 bytes per pixel (siehe lv_img_buf.h)
    CF_TRUE_COLOR = 4

    for path in paths:
        src_path = path
        bak = path.with_suffix(path.suffix + ".bak_rgba")
        if bak.exists():
            src_path = bak
            print(f"{path}: Quelle {bak.name}")

        data = src_path.read_bytes()
        if len(data) < 4:
            print(f"{path}: zu kurz", file=sys.stderr)
            continue

        (word,) = struct.unpack("<I", data[:4])
        cf_in = word & 0x1F
        w = (word >> 10) & 0x7FF
        h = (word >> 21) & 0x7FF
        payload = data[4:]
        n = w * h

        if len(payload) != n * 4:
            print(
                f"{path}: erwarte RGBA8888 ({n*4} Bytes Payload), habe {len(payload)}",
                file=sys.stderr,
            )
            continue

        # TRUE_COLOR: Header cf=4, 2 Bytes/Pixel
        out = bytearray(4 + n * 2)
        out[:4] = pack_header(CF_TRUE_COLOR, w, h)
        o = 4
        for i in range(n):
            r, g, b, _a = payload[i * 4 : i * 4 + 4]
            c = rgb565(r, g, b)
            o = write_rgb565_bytes(out, o, c)

        path.write_bytes(out)
        print(f"{path}: TRUE_COLOR cf={CF_TRUE_COLOR}, {len(data)} -> {len(out)} B (RGB565, SWAP={SWAP_FOR_LV_COLOR_16_SWAP})")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
