#!/usr/bin/env python3
"""Ask the Hi-Scan firmware what each fault code means, for one system.

Runs the emulated Hi-Scan to a system's DTC screen with the KWP2000 ECU
model answering the DTC request with 8 codes at a time, reads the 8 screen
lines (tools/lcdtext.py) and keeps "CODE TEXT" for every code the firmware
knows ("Not Define Code" means it does not). Runs in parallel.

Usage: python3 -I tools/dtc_names.py ROM SYSTEM FIRST LAST [OUT.csv]
       python3 -I tools/dtc_names.py ROM SYSTEM --codes-from CODES.csv [OUT.csv]
  SYSTEM: srs (B codes, 18 00 FF 00 at 0x58), abs (C codes, 13 40 00 at 0x28)
          engine (P codes, 18 00 00 00 at 0x11) or tcs (C codes, 03 at 0x32)
  --codes-from: try only the codes listed in another names CSV
  FIRST/LAST: code numbers as written, e.g. 1000 2999 (decimal digits only)
"""
import csv, os, re, subprocess, sys
from concurrent.futures import ThreadPoolExecutor

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from lcdtext import COLS, LAYER2, ROWS, load_font  # noqa: E402

CLI = "./target/release/cascade"
SYSTEMS = {
    # path keys after boot, the DTC request, letter bits, extra emulator options
    "srs": dict(
        keys=["enter"] + ["down"] * 8 + ["enter"] + ["down"] * 4 + ["enter", "down", "enter"]
        + ["wait"] * 4 + ["enter", "wait", "wait"],
        request="1800FF00", other={"1801FF00": "5800"}, letter=0x8000,
        opts=["--kwp-addr", "0x58", "--kwp-keybytes", "EA8F", "--kwp-length-byte",
              # drawing 8 unknown codes takes the Hi-Scan ~6 s, past the
              # model's 5 s session timeout
              "--kwp-p3-max", "60000"]),
    "abs": dict(
        keys=["enter"] + ["down"] * 8 + ["enter"] + ["down"] * 3 + ["enter"] + ["wait"] * 4
        + ["enter", "wait", "wait"],
        request="134000", other={}, letter=0x4000, record="code",
        opts=["--kwp-addr", "0x28", "--kwp-keybytes", "EA8F", "--kwp-length-byte",
              "--kwp-p3-max", "60000"]),
    "tcs": dict(
        # 5-baud init to 0x32, ISO 9141-2 frames, OBD-style mode 03: at most
        # three codes per response frame
        keys=["enter"] + ["down"] * 8 + ["enter"] + ["down"] * 7 + ["enter", "enter"] + ["wait"] * 4
        + ["enter", "wait", "wait", "wait"],
        request="03", other={"B101": "F101"}, letter=0x4000, record="obd", batch=3,
        opts=["--kwp-addr", "0x32", "--kwp-iso9141", "--kwp-p3-max", "60000"]),
    "engine": dict(
        keys=["enter"] + ["down"] * 8 + ["enter", "enter"] + ["down"] * 3 + ["enter", "wait", "enter",
                                                                             "wait", "wait"],
        request="18000000", other={}, letter=0x0000, opts=["--kwp-p3-max", "60000"]),
}


def scan_lines(rom, mem):
    """Every text line on screen, found by trying all pixel rows and bit
    offsets (the DTC list is not on the usual text grid)."""
    glyphs = load_font(rom)

    def raw(y, c):
        return 0 if c >= COLS else mem[y * COLS + c] ^ mem[LAYER2 + y * COLS + c]

    out = []
    for xs in range(8):
        for top in range(0, 240 - ROWS):
            chars = []
            for c in range(COLS):
                cell = bytes((((raw(top + i, c) << xs) | (raw(top + i, c + 1) >> (8 - xs))) & 0xFF)
                             if xs else raw(top + i, c) for i in range(ROWS))
                code = glyphs.get(cell)
                chars.append(chr(code) if code is not None and 0x20 <= code < 0x7F else "?")
            out.append("".join(chars))
    return out


def run(rom, sys_name, codes, tag, out_dir):
    s = SYSTEMS[sys_name]
    if s.get("record") == "obd":
        resp = "43" + "".join("%04X" % (s["letter"] | c) for c in codes) + "0000" * (3 - len(codes))
    elif s.get("record") == "code":
        # ABS: 53 N + N x (code high, code low), no status byte
        resp = "53%02X" % len(codes) + "".join("%04X" % (s["letter"] | c) for c in codes)
    else:
        resp = "58%02X" % len(codes) + "".join("%04X71" % (s["letter"] | c) for c in codes)
    args = ["--key", "59899906:enter:1572865"]
    t = 300
    for k in s["keys"]:
        if k != "wait":
            args += ["--key", f"{t * 1000000}:{k}:1572865"]
        t += 30
    dump = f"{out_dir}/{tag}.lcd"
    cmd = [CLI, "run", rom, "-m", str((t + 150) * 1000000), "--iface", "kwp",
           "--lcd-dump", dump, "--respond", f"{s['request']}={resp}", *s["opts"]]
    for k, v in s["other"].items():
        cmd += ["--respond", f"{k}={v}"]
    subprocess.run(cmd + args, check=True, capture_output=True)
    return scan_lines(open(rom, "rb").read(), open(dump, "rb").read())


def main():
    rom, sys_name = sys.argv[1], sys.argv[2]
    if sys.argv[3] == "--codes-from":
        rows = csv.reader(l for l in open(sys.argv[4]) if not l.startswith("#"))
        codes = [int(r[0][1:], 16) for r in rows if r and r[0] != "code"]
    else:
        # decimal digits only: code "1346" is 0x1346 on the wire
        codes = [int(f"{n:04d}", 16) for n in range(int(sys.argv[3]), int(sys.argv[4]) + 1)]
    out = sys.argv[5] if len(sys.argv) > 5 else f"protocol/dtc-names-{sys_name}.csv"
    out_dir = "traces/dtc-names"
    os.makedirs(out_dir, exist_ok=True)
    n = SYSTEMS[sys_name].get("batch", 8)
    batches = [codes[i:i + n] for i in range(0, len(codes), n)]
    letter = "PCBU"[SYSTEMS[sys_name]["letter"] >> 14]
    found = {}
    pat = re.compile(r"([PCBU][0-9A-F]{4}) (.+)")

    def one(i_b):
        i, b = i_b
        lines = run(rom, sys_name, b, f"{sys_name}-{i}", out_dir)
        got, seen = {}, set()
        for line in lines:
            m = pat.search(line)
            if m:
                seen.add(m.group(1))
                if "Not Define" not in m.group(2):
                    got[m.group(1)] = m.group(2).rstrip(" ?").strip()
        return b, got, seen

    unread = []
    with ThreadPoolExecutor(max_workers=os.cpu_count()) as ex:
        for b, got, seen in ex.map(one, enumerate(batches)):
            found.update(got)
            # every code of the batch must have been read off the screen
            unread += [f"{letter}{c:04X}" for c in b if f"{letter}{c:04X}" not in seen]
    if unread:
        print(f"WARNING: {len(unread)} codes not read off the screen: {unread[:20]}")
    rows = sorted(found.items())
    with open(out, "w", newline="") as f:
        f.write(f"# {sys_name} fault code texts as the Hi-Scan firmware shows them (HP 701 USA,\n")
        f.write("# 09910-61000), read off its DTC screen in the emulator; Tiburon 2003 path.\n")
        if sys.argv[3] == "--codes-from":
            f.write(f"# Only the codes listed in {os.path.basename(sys.argv[4])} were tried.\n")
        w = csv.writer(f)
        w.writerow(["code", "text"])
        w.writerows(rows)
    print(f"{len(codes)} codes tried, {len(rows)} known -> {out}")


if __name__ == "__main__":
    main()
