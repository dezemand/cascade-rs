#!/usr/bin/env python3
"""Check candidate scaling formulas against the screen readings that
tools/scale_sweep.py produced (re-reads its LCD dumps).

Usage: tools/check_scaling.py ROM [SWEEPDIR]
"""
import os, re, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from lcdtext import screen_lines  # noqa: E402
from scale_sweep import PARAMS, VECTORS, cell_text  # noqa: E402  (module guards below)

ROM = sys.argv[1]
DIR = sys.argv[2] if len(sys.argv) > 2 else "traces/sweep"

def u16(a, b): return a | b << 8
def s16(a, b): v = u16(a, b); return v - 0x10000 if v & 0x8000 else v

# name -> (decode raw tuple, scale, offset, unit)
FORMULAS = {
    "BATTERY": (lambda r: r[0], 26 / 256, 0, "V"),
    "COOL.TEMP": (lambda r: r[0], 0.75 * 1.8, -48 * 1.8 + 32, "°F (= 0.75 °C/bit − 48 °C)"),
    "AIR TEMP": (lambda r: r[0], 0.75 * 1.8, -48 * 1.8 + 32, "°F (= 0.75 °C/bit − 48 °C)"),
    "TPS": (lambda r: r[0], 120 / 256, 0, "°"),
    "ADAPTED TP": (lambda r: u16(*r), 0.0018254, 0, "°"),
    "MAP mV": (lambda r: r[0], 4980 / 255, 0, "mV"),
    "MAP psi": (lambda r: u16(*r), 78.85 / 65535, 0, "psi"),
    "VEHIC.SPD": (lambda r: r[0], 0.621371, 0, "MPH (= 1 km/h/bit)"),
    "ENGINE SPD": (lambda r: s16(*r), 1, 0, "rpm"),
    "TAR.IDLE": (lambda r: s16(*r), 1, 0, "rpm"),
    "FR O2S-B1": (lambda r: u16(*r), 4.87797, 0, "mV"),
    "IGNITION": (lambda r: r[0], -0.375, 72, "°"),
    "INJECTION": (lambda r: u16(*r), 0.004, 0, "ms"),
    "ISC DUTY": (lambda r: u16(*r), 0.00153, 0, "%"),
    "IDLE SPD": (lambda r: s16(*r), 0.00153, 0, "%"),
    "IGN DWELL": (lambda r: u16(*r), 0.004, 0, "ms"),
}

rom = open(ROM, "rb").read()
screens = [screen_lines(rom, open(f"{DIR}/v{n}.lcd", "rb").read()) for n in range(len(VECTORS))]
bad = 0
for name, off, size, cell in PARAMS:
    dec, scale, offset, unit = FORMULAS[name]
    worst = 0.0
    for n, (v1, ab) in enumerate(VECTORS):
        raw = (v1,) if size == 1 else ab
        txt = cell_text(screens[n], cell)
        m = re.match(r"-?\d+(\.\d+)?", txt)
        shown = float(m.group())
        want = dec(raw) * scale + offset
        decimals = len(m.group(1)) - 1 if m.group(1) else 0
        # the screen field is 5 characters wide and truncates longer numbers
        digits = re.sub(r"\D", "", m.group())
        full = f"{want:.{decimals}f}" if decimals else f"{int(round(want))}"
        full_digits = re.sub(r"\D", "", full)
        # the field is 5 characters wide: longer numbers are cut off, so a
        # full-width field only has to be a prefix of the expected number
        if len(m.group().rstrip(".")) >= 5 and f"{int(want)}".startswith(m.group().rstrip(".")) and (want < 0) == (shown < 0 or m.group() == "-0.0"):
            continue  # truncated display that agrees
        err = abs(shown - want)
        # decimal fields round; integer fields (and a trailing '.') truncate
        rounded_ok = err <= 0.5 * 10 ** -decimals + 1e-9
        q = 10 ** decimals
        truncated_ok = abs(shown - int(want * q) / q) < 1e-9
        if not (rounded_ok or truncated_ok):
            bad += 1
            print(f"  MISMATCH {name}: raw {raw} shown {txt!r} expected {want:.3f}")
        worst = max(worst, err)
    print(f"{name:11s} offset {off:2d} size {size}: {scale:.6g} {unit} per bit, offset {offset:+.4g}; worst |err| {worst:.3f}")
print("all samples consistent" if not bad else f"{bad} mismatches")


def validate(n_runs, seed=1):
    """Held-out check: random blocks, predicted vs shown."""
    import random
    from scale_sweep import run, BLOCK
    rnd = random.Random(seed)
    out = "traces/validate"
    os.makedirs(out, exist_ok=True)
    fails = total = 0
    for k in range(n_runs):
        block = bytes(rnd.randrange(256) for _ in range(BLOCK))
        lines = run(ROM, out, block, f"r{k}")
        for name, off, size, cell in PARAMS:
            dec, scale, offset, _ = FORMULAS[name]
            raw = tuple(block[off:off + size])
            want = dec(raw) * scale + offset
            txt = cell_text(lines, cell)
            m = re.match(r"-?\d+(\.\d+)?", txt)
            total += 1
            if not m:
                fails += 1
                print(f"  unreadable {name}: {txt!r}")
                continue
            shown = float(m.group())
            decimals = len(m.group(1)) - 1 if m.group(1) else 0
            q = 10 ** decimals
            ok = (abs(shown - want) <= 0.5 / q + 1e-9 or abs(shown - int(want * q) / q) < 1e-9
                  or (len(m.group().rstrip(".")) >= 5 and f"{int(want)}".startswith(m.group().rstrip("."))))
            if not ok:
                fails += 1
                print(f"  FAIL {name}: raw {raw} shown {txt!r} predicted {want:.3f}")
    print(f"held-out: {total - fails}/{total} predictions match")


if os.environ.get("VALIDATE"):
    validate(int(os.environ["VALIDATE"]))
