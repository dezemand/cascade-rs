#!/usr/bin/env python3
"""Probe a ReadDataByLocalIdentifier block against any screen, by text.

For each offset (and value), run the key sequence with an otherwise zero
block and report which screen lines changed, with old and new text.

Usage: tools/probe_text.py ROM LENGTH OFFSETS VALUES KEY...
  OFFSETS: comma list or a-b ranges, e.g. 1,2,4-6
  VALUES:  comma list of byte values (0x.. allowed), or "bits"
  KEY...:  as tools/screen.py (supports --path NAME)
env PROBE_EXTRA: extra cascade run options (e.g. --kwp-addr 0x28 ...)
"""
import os, sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from screen import PATHS, run  # noqa: E402


def parse_list(s):
    out = []
    for part in s.split(","):
        if "-" in part:
            a, b = part.split("-")
            out += range(int(a, 0), int(b, 0) + 1)
        else:
            out.append(int(part, 0))
    return out


def main():
    rom, length, offs, vals, rest = sys.argv[1], int(sys.argv[2]), sys.argv[3], sys.argv[4], sys.argv[5:]
    offsets = parse_list(offs)
    values = [1 << b for b in range(8)] if vals == "bits" else parse_list(vals)
    keys, it = [], iter(rest)
    for a in it:
        keys += PATHS[next(it)] if a == "--path" else [a]

    def screen(block):
        extra = os.environ.get("PROBE_EXTRA", "").split()
        return run(rom, keys, ["--local", "0x01=" + bytes(block).hex(), *extra], dump="traces/probe_text.lcd")

    base = screen(bytes(length))
    for off in offsets:
        for v in values:
            b = bytearray(length)
            b[off] = v
            lines = screen(b)
            diffs = [f"{base[i].strip()!r} -> {lines[i].strip()!r}" for i in range(len(base)) if base[i] != lines[i]]
            if diffs:
                print(f"offset {off} value {v:02X}: " + "; ".join(diffs), flush=True)


if __name__ == "__main__":
    main()
