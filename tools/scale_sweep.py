#!/usr/bin/env python3
"""Measure how the firmware converts raw `21 01` block values to the numbers
on the Current Data FULL screen, for Tiburon 2003 Engine L4.

Each run fills every mapped parameter at once, reads the screen as text
(tools/lcdtext.py) and records (raw, shown) pairs. Then it fits
shown = scale * raw + offset per parameter and prints the fit and residuals.

Usage: tools/scale_sweep.py ROM [OUTDIR]
"""
import os, re, subprocess, sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from lcdtext import screen_lines  # noqa: E402

CLI = "./target/release/cascade"
KEYS = ["enter"] + ["down"] * 8 + ["enter", "enter"] + ["down"] * 3 + ["enter", "down", "enter", "f3"]

# (name, offset, size, cell) from protocol/tiburon-2003-engine-l4.md; cells 0..10 left, 11..21 right
PARAMS = [
    ("BATTERY", 0, 1, 6), ("COOL.TEMP", 3, 1, 8), ("AIR TEMP", 5, 1, 3), ("TPS", 7, 1, 4),
    ("ADAPTED TP", 8, 2, 5), ("MAP mV", 10, 1, 2), ("MAP psi", 13, 2, 1), ("VEHIC.SPD", 18, 1, 11),
    ("ENGINE SPD", 19, 2, 9), ("TAR.IDLE", 21, 2, 10), ("FR O2S-B1", 26, 2, 0), ("IGNITION", 30, 1, 18),
    ("INJECTION", 42, 2, 19), ("ISC DUTY", 57, 2, 20), ("IDLE SPD", 59, 2, 21), ("IGN DWELL", 74, 2, 17),
]
BLOCK = 80

# per run: (byte value for 1-byte params, (first, second) bytes for 2-byte params)
VECTORS = [(0, (0, 0)), (1, (0, 1)), (2, (1, 0)), (10, (0, 10)), (50, (0, 100)), (100, (1, 1)),
           (128, (0x10, 0)), (200, (0, 200)), (255, (0xFF, 0xFF)), (64, (0x02, 0x00))]


def run(rom, out, block, tag):
    args = ["--key", "59899906:enter:1572865"]
    t = 300
    for k in KEYS:
        args += ["--key", f"{t * 1000000}:{k}:1572865"]
        t += 30
    dump = f"{out}/{tag}.lcd"
    cmd = [CLI, "run", rom, "-m", str((t + 60) * 1000000), "--iface", "kwp",
           "--local", "0x01=" + block.hex(), "--lcd-dump", dump] + args
    subprocess.run(cmd, check=True, capture_output=True)
    return screen_lines(open(rom, "rb").read(), open(dump, "rb").read())


def cell_text(lines, cell):
    line, col = (cell + 1, 10) if cell < 11 else (cell - 10, 30)
    return lines[line][col:col + 10].strip()


def number(s):
    m = re.match(r"-?\d+(\.\d+)?", s)
    return float(m.group()) if m else None


def main():
    rom = sys.argv[1]
    out = sys.argv[2] if len(sys.argv) > 2 else "traces/sweep"
    os.makedirs(out, exist_ok=True)
    samples = {p[0]: [] for p in PARAMS}
    for n, (v1, (a, b)) in enumerate(VECTORS):
        block = bytearray(BLOCK)
        for name, off, size, cell in PARAMS:
            if size == 1:
                block[off] = v1
            else:
                block[off], block[off + 1] = a, b
        lines = run(rom, out, bytes(block), f"v{n}")
        for name, off, size, cell in PARAMS:
            txt = cell_text(lines, cell)
            samples[name].append(((v1,) if size == 1 else (a, b), txt))

    for name, off, size, cell in PARAMS:
        print(f"{name} (offset {off}, {size} byte{'s' if size > 1 else ''}):")
        for raw, txt in samples[name]:
            print(f"   raw {' '.join(f'{x:02X}' for x in raw):6s} -> {txt}")


if __name__ == "__main__":
    main()
