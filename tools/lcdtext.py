#!/usr/bin/env python3
"""Read the Hi-Scan screen as text from an LCD memory dump.

The firmware draws text byte-aligned on a 20-pixel line pitch with the 8x16
font stored in the ROM pack (16 bytes per glyph, indexed by character code).
Matching dump bytes against those glyphs gives exact text. The font is read
from the ROM at runtime; nothing proprietary is stored here.

Usage: tools/lcdtext.py ROM LCDDUMP
"""
import sys

FONT_BASE = 0x168004  # glyph for code c at FONT_BASE + 16 * c (verified for '0'..'9')
LAYER2 = 0x2580
COLS, LINES = 40, 12


def load_font(rom):
    glyphs = {}
    for code in range(0x20, 0x100):
        g = rom[FONT_BASE + 16 * code: FONT_BASE + 16 * code + 16]
        glyphs.setdefault(bytes(g), code)
        glyphs.setdefault(bytes(b ^ 0xFF for b in g), code)  # inverse video
    return glyphs


def screen_lines(rom, mem):
    glyphs = load_font(rom)
    px = lambda y, c: mem[y * COLS + c] ^ mem[LAYER2 + y * COLS + c]
    out = []
    for line in range(LINES):
        best = None
        # the glyph's first row sits a few pixels below the line start
        for shift in range(0, 6):
            top = 20 * line + shift
            if top + 16 > 240:
                continue
            chars, hits = [], 0
            for c in range(COLS):
                cell = bytes(px(top + i, c) for i in range(16))
                code = glyphs.get(cell)
                if code is None:
                    chars.append("?")
                else:
                    chars.append(chr(code) if 0x20 <= code < 0x7F else {0xB0: "°"}.get(code, "?"))
                    hits += code != 0x20
            if best is None or hits > best[0]:
                best = (hits, "".join(chars).rstrip())
        out.append(best[1])
    return out


if __name__ == "__main__":
    rom = open(sys.argv[1], "rb").read()
    mem = open(sys.argv[2], "rb").read()
    print("\n".join(screen_lines(rom, mem)))
