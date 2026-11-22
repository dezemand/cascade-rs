#!/usr/bin/env python3
"""Run a key sequence in cascade-rs and print the final screen as text.

Usage: tools/screen.py ROM [options] KEY...
  --path tiburon-l4   prepend the menu path to Tiburon 2003 Engine L4's
                      function menu (info screen ENTER is always added)
  --local ID=HEX      KwpEcu ReadDataByLocalIdentifier block (repeatable)
  --dtc CODE:STATUS   KwpEcu stored DTC (repeatable)
  --respond REQ=RESP  KwpEcu fixed response to an exact request (hex)
  --replay F.jsonl    answer from a kdiag --trace recording
  --kwp-log FILE      write the decoded KWP exchange log
  --tail N            run N million cycles after the last key (default 60)
  --step N            million cycles between keys (default 30)
Keys are cascade key names (enter, down, f1, 3, ...); `wait` adds a gap.
"""
import os, subprocess, sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from lcdtext import screen_lines  # noqa: E402

PATHS = {
    "tiburon-l4": ["enter"] + ["down"] * 8 + ["enter", "enter"] + ["down"] * 3 + ["enter"],
    # ABS function menu; needs --kwp-addr 0x28 --kwp-keybytes EA8F --kwp-length-byte
    "tiburon-abs": ["enter"] + ["down"] * 8 + ["enter"] + ["down"] * 3 + ["enter"] + ["wait"] * 4,
    # SRS 2003-2006 function menu; needs --kwp-addr 0x58 and the same key bytes
    "tiburon-srs": ["enter"] + ["down"] * 8 + ["enter"] + ["down"] * 4 + ["enter", "down", "enter"] + ["wait"] * 4,
}


def run(rom, keys, extra=(), tail=60, step=30, dump="traces/screen.lcd"):
    args = ["--key", "59899906:enter:1572865"]
    t = 300
    for k in keys:
        if k != "wait":  # "wait" only lets time pass (e.g. session setup)
            args += ["--key", f"{t * 1000000}:{k}:1572865"]
        t += step
    cmd = ["./target/release/cascade", "run", rom, "-m", str((t + tail) * 1000000),
           "--iface", "kwp", "--lcd-dump", dump, *extra, *args]
    r = subprocess.run(cmd, capture_output=True, text=True)
    if r.returncode not in (0,):
        sys.stderr.write(r.stderr)
    return screen_lines(open(rom, "rb").read(), open(dump, "rb").read())


def main():
    rom, rest = sys.argv[1], sys.argv[2:]
    keys, extra, tail, step = [], [], 60, 30
    it = iter(rest)
    for a in it:
        if a == "--path":
            keys += PATHS[next(it)]
        elif a in ("--local", "--dtc", "--kwp-log", "--respond", "--line-log", "--kwp-addr", "--kwp-keybytes", "--png", "--replay", "--rom-reads", "--pcs", "--ecu-rom", "--ecu-adc", "--iface", "--exrom"):
            extra += [a, next(it)]
        elif a == "--kwp-length-byte":
            extra.append(a)
        elif False:
            extra += [a, next(it)]
        elif a == "--tail":
            tail = int(next(it))
        elif a == "--step":
            step = int(next(it))
        else:
            keys.append(a)
    print("\n".join(run(rom, keys, extra, tail, step)))


if __name__ == "__main__":
    main()
