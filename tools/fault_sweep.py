#!/usr/bin/env python3
"""Which fault code does an engine ECU dump store for each wiring fault?

Runs the dump in ecuc167 with the engine idling (crank and cam as ecusim
drives them), puts one sensor input at 0 V (short) or 5 V (open) from 3 s
on, and reads `18 00 00 00` at 30 s. Codes already stored by the healthy
baseline run are left out. Runs in parallel.

Usage: python3 -I tools/fault_sweep.py ECU_DUMP [--csv OUT]
       (a 512 KB dump runs at 24 MHz, a 256 KB dump at 16 MHz)
"""
import os, re, subprocess, sys
from concurrent.futures import ThreadPoolExecutor

ECUC167 = "./target/release/ecuc167"
# ADC channel, sensor (as on the GK 2.0 2003 SIMK41; other dumps may differ)
SENSORS = {0: "MAP", 1: "ch1", 2: "front O2", 3: "rear O2", 4: "battery sense",
           5: "ch5", 6: "ch6", 7: "ch7", 8: "EVAP pressure", 9: "ch9",
           10: "throttle", 11: "coolant", 12: "wheel-speed line", 13: "ch13",
           14: "intake air", 15: "ch15"}
# plausible warm idle on every channel that has a known sensor
HEALTHY = {0: 0x108, 2: 0x5C, 3: 0x85, 4: 0x22B, 10: 0x108, 11: 0x103, 14: 0x2F0}


def run(rom, fcpu, faults):
    args = [ECUC167, "run", rom, "--fcpu", str(fcpu), "-m", "31000",
            "--wheel", "cc15+t7in+t0in+capin:60:2:800", "--cam", "cc14:50:110",
            "--init-at", "30000", "--req", "18 00 00 00"]
    for ch, v in HEALTHY.items():
        args += ["--adc", f"{ch}=0x{v:x}"]
    for ch, v in faults:
        args += ["--adc-at", f"3000:{ch}=0x{v:x}"]
    out = subprocess.run(args, capture_output=True, text=True).stdout
    m = re.search(r"<- .. F1 11 58 ([0-9A-F ]+)", out)
    if not m:
        return None
    b = [int(x, 16) for x in m.group(1).split()]
    n = b[0]
    return {f"{'PCBU'[b[1 + 3 * i] >> 6]}{((b[1 + 3 * i] & 0x3F) << 8) | b[2 + 3 * i]:04X}"
            for i in range(n)}


def main():
    rom = sys.argv[1]
    csv = sys.argv[sys.argv.index("--csv") + 1] if "--csv" in sys.argv else None
    fcpu = 24_000_000 if os.path.getsize(rom) > 256 * 1024 else 16_000_000
    jobs = [(ch, v) for ch in SENSORS for v in (0, 0x3FF)]
    with ThreadPoolExecutor(max_workers=os.cpu_count()) as ex:
        base = ex.submit(run, rom, fcpu, [])
        results = list(ex.map(lambda j: (j, run(rom, fcpu, [j])), jobs))
    base = base.result() or set()
    print(f"{os.path.basename(rom)[:18]}: healthy baseline stores {sorted(base) or 'nothing'}")
    rows = []
    for (ch, v), codes in results:
        new = sorted((codes or set()) - base)
        if new:
            kind = "short (0 V)" if v == 0 else "open (5 V)"
            print(f"  ADC {ch:2} {SENSORS[ch]:16} {kind:12} -> {', '.join(new)}")
            rows.append((ch, SENSORS[ch], kind, " ".join(new)))
    if csv:
        with open(csv, "w") as f:
            f.write("adc,sensor,fault,codes\n")
            for r in rows:
                f.write(",".join(map(str, r)) + "\n")


if __name__ == "__main__":
    main()
