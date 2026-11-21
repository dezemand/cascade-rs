#!/usr/bin/env python3
"""Map ReadDataByLocalIdentifier block offsets to Current Data cells.

For each offset, run the emulator with an all-zero block except that byte,
open Current Data > FULL, and report which of the 22 screen cells changed
compared to the all-zero baseline. Uses only the stdlib; PNGs come from
`cascade run --png` (stored deflate, 1-bit).

Usage: tools/probe_block.py ROM LENGTH [VALUE] [OUTDIR]\n       env START=N, or OFFSETS=a,b,c [BITS=1] to limit the probe
"""
import subprocess, sys, zlib, struct, os

ROM, LENGTH = sys.argv[1], int(sys.argv[2])
VALUE = int(sys.argv[3], 0) if len(sys.argv) > 3 else 0xFF
OUT = sys.argv[4] if len(sys.argv) > 4 else "traces/probe"
CLI = "./target/release/cascade"
os.makedirs(OUT, exist_ok=True)

# menu path: info screen, Hyundai diag, Tiburon, Engine L4, 2003, Current Data, FULL
KEYS = ["enter"] + ["down"] * 8 + ["enter", "enter"] + ["down"] * 3 + ["enter", "down", "enter", "f3"]

def key_args():
    args = ["--key", "59899906:enter:1572865"]
    t = 300
    for k in KEYS:
        args += ["--key", f"{t * 1000000}:{k}:1572865"]
        t += 30
    return args, (t + 60) * 1000000

def run(block, png):
    args, m = key_args()
    cmd = [CLI, "run", ROM, "-m", str(m), "--iface", "kwp", "--local", "0x01=" + block.hex(), "--png", png] + args
    subprocess.run(cmd, check=True, capture_output=True)

def pixels(png):
    data = open(png, "rb").read()
    pos, idat, w, h = 8, b"", 0, 0
    while pos < len(data):
        n, kind = struct.unpack(">I4s", data[pos:pos + 8])
        body = data[pos + 8:pos + 8 + n]
        if kind == b"IHDR":
            w, h = struct.unpack(">II", body[:8])
        elif kind == b"IDAT":
            idat += body
        pos += 12 + n
    raw = zlib.decompress(idat)
    rb = (w + 7) // 8
    rows = [raw[y * (rb + 1) + 1:(y + 1) * (rb + 1)] for y in range(h)]
    return lambda x, y: not (rows[y][x // 8] >> (7 - x % 8)) & 1

def cells(png):
    px = pixels(png)
    out = []
    for col, (x0, x1) in enumerate([(160, 318), (480, 640)]):
        for r in range(11):
            y0 = 40 + 40 * r
            out.append(tuple(px(x, y) for y in range(y0, y0 + 40, 2) for x in range(x0, x1, 2)))
    return out  # 0..10 left column, 11..21 right column

base_png = f"{OUT}/base.png"
run(bytes(LENGTH), base_png)
base = cells(base_png)
START = int(os.environ.get("START", "0"))
# OFFSETS="25,26,73" and BITS=1 probe single bits of chosen offsets instead
offsets = [int(o, 0) for o in os.environ["OFFSETS"].split(",")] if "OFFSETS" in os.environ else range(START, LENGTH)
values = [1 << b for b in range(8)] if os.environ.get("BITS") else [VALUE]
for i in offsets:
    for v in values:
        b = bytearray(LENGTH)
        b[i] = v
        png = f"{OUT}/off{i:02d}_{v:02x}.png"
        run(bytes(b), png)
        changed = [c for c, (a, z) in enumerate(zip(base, cells(png))) if a != z]
        print(f"offset {i:2d} value {v:02X}: cells {changed}", flush=True)
