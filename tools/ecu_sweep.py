#!/usr/bin/env python3
"""Hi-Scan against the engine ECU's own code: sweep sensor inputs and check
that the protocol CSV decodes every ECU-generated `21 01` block the way the
Hi-Scan displays it.

For each (ADC channel, value) the Hi-Scan firmware runs against the SIMK41
dump (`cascade run --ecu-rom`, ecuc167 inside), goes to the engine Current
Data FULL view, and we read:
  - the raw `21 01` block the ECU sent (from --line-log), and
  - the screen text (tools/lcdtext.py).
Every parameter shown is decoded with protocol/tiburon-2003-engine-l4-current-data.csv
and compared with the screen, allowing the display's rounding. It also
prints, per parameter, the raw range the sweep produced.

Usage: python3 -I tools/ecu_sweep.py HISCAN_ROM ECU_ROM [OUTDIR]
       (stdlib only; runs in parallel, one process per CPU)
"""
import csv, os, re, subprocess, sys
from concurrent.futures import ThreadPoolExecutor

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from lcdtext import screen_lines  # noqa: E402
from scale_sweep import KEYS, cell_text  # noqa: E402

CLI = "./target/release/cascade"
CSV = "protocol/tiburon-2003-engine-l4-current-data.csv"

# FULL-view cell of each CSV parameter (layout as in tools/scale_sweep.py)
CELLS = {
    "battery_voltage": 6, "coolant_temp": 8, "intake_air_temp": 3, "tps": 4,
    "adapted_tps": 5, "map_voltage": 2, "map_pressure": 1, "vehicle_speed": 11,
    "engine_speed": 9, "target_idle": 10, "o2_front_b1": 0, "ignition_timing": 18,
    "injection_time": 19, "isc_duty": 20, "idle_speed_duty": 21, "ign_dwell": 17,
}

# ADC channels with an effect on the block (docs/simk41-notes.md)
CHANNELS = {0: "MAP", 1: "ch1", 2: "O2 front", 3: "O2 rear", 4: "battery",
            10: "throttle", 11: "coolant", 14: "intake air"}
VALUES = list(range(0, 0x400, 0x40)) + [0x3FF]


def load_params():
    out = []
    for row in csv.reader(l for l in open(CSV) if not l.startswith("#")):
        if row[0] == "name" or row[2].startswith("bit"):
            continue
        out.append(dict(name=row[0], off=int(row[1]), size=int(row[2]), signed=row[3] == "yes",
                        scale=float(row[4].lstrip("~")), add=float(row[5] or 0),
                        estimated=row[4].startswith("~")))
    return out


def raw_of(p, block):
    b = block[p["off"]:p["off"] + p["size"]]
    v = int.from_bytes(bytes(b), "little")
    if p["signed"] and p["size"] == 2 and v & 0x8000:
        v -= 0x10000
    return v


def last_block(line_log):
    """The data bytes of the last complete `61 01` long frame the ECU sent."""
    rx = [int(m.group(1)) for m in re.finditer(r"Rx\((\d+)\)", open(line_log).read())]
    for i in range(len(rx) - 6, -1, -1):
        if rx[i] == 0x80 and rx[i + 1] == 0xF1 and rx[i + 2] == 0x11 and rx[i + 4:i + 6] == [0x61, 0x01]:
            n = rx[i + 3]
            if i + 4 + n < len(rx):
                return rx[i + 6:i + 4 + n]
    return None


def run(hiscan, ecu, out, ch, val):
    tag = f"ch{ch}_{val:03x}"
    args = ["--key", "59899906:enter:1572865"]
    t = 300
    for k in KEYS:
        args += ["--key", f"{t * 1000000}:{k}:1572865"]
        t += 30
    dump, log = f"{out}/{tag}.lcd", f"{out}/{tag}.line"
    cmd = [CLI, "run", hiscan, "-m", str((t + 60) * 1000000), "--ecu-rom", ecu,
           "--ecu-adc", f"{ch}=0x{val:x}", "--lcd-dump", dump, "--line-log", log] + args
    subprocess.run(cmd, check=True, capture_output=True)
    return tag, ch, val, last_block(log), dump


def number(s):
    m = re.match(r"-?\d+(\.\d*)?", s)
    return (float(m.group()), len(m.group(1)) - 1 if m.group(1) else 0) if m else (None, 0)


def main():
    hiscan, ecu = sys.argv[1], sys.argv[2]
    out = sys.argv[3] if len(sys.argv) > 3 else "traces/ecu-sweep"
    os.makedirs(out, exist_ok=True)
    params = load_params()
    jobs = [(ch, v) for ch in CHANNELS for v in VALUES]
    with ThreadPoolExecutor(max_workers=os.cpu_count()) as ex:
        results = list(ex.map(lambda j: run(hiscan, ecu, out, *j), jobs))
    rom = open(hiscan, "rb").read()
    raws = {p["name"]: set() for p in params}
    checked = bad = 0
    for tag, ch, val, block, dump in results:
        if block is None:
            print(f"{tag}: no 21 01 block received")
            bad += 1
            continue
        lines = screen_lines(rom, open(dump, "rb").read())
        for p in params:
            raw = raw_of(p, block)
            raws[p["name"]].add(raw)
            cell = CELLS.get(p["name"])
            if cell is None:
                continue
            shown, decimals = number(cell_text(lines, cell))
            if shown is None:
                continue
            want = raw * p["scale"] + p["add"]
            # decimal fields round, integer fields truncate; one display step of slack
            step = 10 ** -decimals
            ok = abs(want - shown) <= step * 1.01 if decimals else -step <= want - shown < 2 * step
            checked += 1
            if not ok:
                bad += 1
                print(f"{tag}: {p['name']} raw {raw} -> CSV {want:.4f}, Hi-Scan shows {shown}")
    print(f"\n{len(results)} runs, {checked} displayed values checked, {bad} mismatches")
    print("\nraw values produced by the ECU across the sweep:")
    for p in params:
        r = sorted(raws[p["name"]])
        est = " (estimated factor)" if p["estimated"] else ""
        if len(r) > 1:
            print(f"  {p['name']:20} {len(r):3} distinct, {r[0]}..{r[-1]}{est}")
        else:
            print(f"  {p['name']:20} constant {r[0] if r else '-'}{est}")


if __name__ == "__main__":
    main()
