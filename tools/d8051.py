#!/usr/bin/env python3
"""Small 8051 disassembler for ECU ROM analysis (stdlib only).

Usage: python3 -I tools/d8051.py ROM [--bank N] [--from ADDR ...] [--list ADDR N]
                                 [--xref ADDR] [--calls]

The ROM is treated as banked: 0x0000-0x7FFF common (file offset = address),
0x8000-0xFFFF from bank N (file offset = N * 0x10000 + address) [assumed
layout for 128 KB images whose halves share the lower 32 KB].

Without --list it does a recursive-descent sweep from the reset and
interrupt vectors (plus any --from addresses) and prints the reached code.
"""
import sys

# (mnemonic, length, operand kinds); kinds: d=direct, i=#imm8, I=#imm16,
# b=bit, r=rel8, a=addr11, A=addr16
OPS = {}


def op(code, m, n, kinds=""):
    OPS[code] = (m, n, kinds)


regs = ["R%d" % i for i in range(8)]
for base, m, k, n in [
    (0x08, "INC", "", 1), (0x18, "DEC", "", 1), (0x28, "ADD A,", "", 1), (0x38, "ADDC A,", "", 1),
    (0x48, "ORL A,", "", 1), (0x58, "ANL A,", "", 1), (0x68, "XRL A,", "", 1),
    (0x78, "MOV {r},", "i", 2), (0x88, "MOV d,{r}", "d", 2), (0x98, "SUBB A,", "", 1),
    (0xA8, "MOV {r},", "d", 2), (0xB8, "CJNE {r},", "ir", 3), (0xC8, "XCH A,", "", 1),
    (0xD8, "DJNZ {r},", "r", 2), (0xE8, "MOV A,", "", 1), (0xF8, "MOV {r},A", "", 1),
]:
    for i in range(8):
        OPS[base + i] = (m, n, k, regs[i])
for base, m, k, n in [
    (0x06, "INC", "", 1), (0x16, "DEC", "", 1), (0x26, "ADD A,", "", 1), (0x36, "ADDC A,", "", 1),
    (0x46, "ORL A,", "", 1), (0x56, "ANL A,", "", 1), (0x66, "XRL A,", "", 1),
    (0x76, "MOV {r},", "i", 2), (0x86, "MOV d,{r}", "d", 2), (0x96, "SUBB A,", "", 1),
    (0xA6, "MOV {r},", "d", 2), (0xB6, "CJNE {r},", "ir", 3), (0xC6, "XCH A,", "", 1),
    (0xD6, "XCHD A,", "", 1), (0xE6, "MOV A,", "", 1), (0xF6, "MOV {r},A", "", 1),
]:
    for i in range(2):
        OPS[base + i] = (m, n, k, "@R%d" % i)
for i in range(8):
    OPS[0x01 + 0x20 * i] = ("AJMP", 2, "a")
    OPS[0x11 + 0x20 * i] = ("ACALL", 2, "a")
for code, m, n, k in [
    (0x00, "NOP", 1, ""), (0x02, "LJMP", 3, "A"), (0x03, "RR A", 1, ""), (0x04, "INC A", 1, ""),
    (0x05, "INC", 2, "d"), (0x10, "JBC", 3, "br"), (0x12, "LCALL", 3, "A"), (0x13, "RRC A", 1, ""),
    (0x14, "DEC A", 1, ""), (0x15, "DEC", 2, "d"), (0x20, "JB", 3, "br"), (0x22, "RET", 1, ""),
    (0x23, "RL A", 1, ""), (0x24, "ADD A,", 2, "i"), (0x25, "ADD A,", 2, "d"), (0x30, "JNB", 3, "br"),
    (0x32, "RETI", 1, ""), (0x33, "RLC A", 1, ""), (0x34, "ADDC A,", 2, "i"), (0x35, "ADDC A,", 2, "d"),
    (0x40, "JC", 2, "r"), (0x42, "ORL", 2, "d", ), (0x43, "ORL", 3, "di"), (0x44, "ORL A,", 2, "i"),
    (0x45, "ORL A,", 2, "d"), (0x50, "JNC", 2, "r"), (0x52, "ANL", 2, "d"), (0x53, "ANL", 3, "di"),
    (0x54, "ANL A,", 2, "i"), (0x55, "ANL A,", 2, "d"), (0x60, "JZ", 2, "r"), (0x62, "XRL", 2, "d"),
    (0x63, "XRL", 3, "di"), (0x64, "XRL A,", 2, "i"), (0x65, "XRL A,", 2, "d"), (0x70, "JNZ", 2, "r"),
    (0x72, "ORL C,", 2, "b"), (0x73, "JMP @A+DPTR", 1, ""), (0x74, "MOV A,", 2, "i"),
    (0x75, "MOV", 3, "di"), (0x80, "SJMP", 2, "r"), (0x82, "ANL C,", 2, "b"),
    (0x83, "MOVC A,@A+PC", 1, ""), (0x84, "DIV AB", 1, ""), (0x85, "MOV", 3, "dd"),
    (0x90, "MOV DPTR,", 3, "I"), (0x92, "MOV", 2, "b"), (0x93, "MOVC A,@A+DPTR", 1, ""),
    (0x94, "SUBB A,", 2, "i"), (0x95, "SUBB A,", 2, "d"), (0xA0, "ORL C,/", 2, "b"),
    (0xA2, "MOV C,", 2, "b"), (0xA3, "INC DPTR", 1, ""), (0xA4, "MUL AB", 1, ""),
    (0xA5, "(A5)", 1, ""), (0xB0, "ANL C,/", 2, "b"), (0xB2, "CPL", 2, "b"), (0xB3, "CPL C", 1, ""),
    (0xB4, "CJNE A,", 3, "ir"), (0xB5, "CJNE A,", 3, "dr"), (0xC0, "PUSH", 2, "d"),
    (0xC2, "CLR", 2, "b"), (0xC3, "CLR C", 1, ""), (0xC4, "SWAP A", 1, ""), (0xC5, "XCH A,", 2, "d"),
    (0xD0, "POP", 2, "d"), (0xD2, "SETB", 2, "b"), (0xD3, "SETB C", 1, ""), (0xD4, "DA A", 1, ""),
    (0xD5, "DJNZ", 3, "dr"), (0xE0, "MOVX A,@DPTR", 1, ""), (0xE2, "MOVX A,@R0", 1, ""),
    (0xE3, "MOVX A,@R1", 1, ""), (0xE4, "CLR A", 1, ""), (0xE5, "MOV A,", 2, "d"),
    (0xF0, "MOVX @DPTR,A", 1, ""), (0xF2, "MOVX @R0,A", 1, ""), (0xF3, "MOVX @R1,A", 1, ""),
    (0xF4, "CPL A", 1, ""), (0xF5, "MOV", 2, "d"),
]:
    OPS[code] = (m, n, k)
# fix-ups for forms whose operand order differs
OPS[0x42] = ("ORL", 2, "d", ",A")
OPS[0x52] = ("ANL", 2, "d", ",A")
OPS[0x62] = ("XRL", 2, "d", ",A")
OPS[0xF5] = ("MOV", 2, "d", ",A")
OPS[0x92] = ("MOV", 2, "b", ",C")
assert len(OPS) == 256, len(OPS)

SFR = {0x80: "P0", 0x81: "SP", 0x82: "DPL", 0x83: "DPH", 0x87: "PCON", 0x88: "TCON", 0x89: "TMOD",
       0x8A: "TL0", 0x8B: "TL1", 0x8C: "TH0", 0x8D: "TH1", 0x90: "P1", 0x98: "SCON", 0x99: "SBUF",
       0xA0: "P2", 0xA8: "IE", 0xB0: "P3", 0xB8: "IP", 0xC8: "T2CON", 0xD0: "PSW", 0xE0: "ACC",
       0xF0: "B"}


class Rom:
    def __init__(self, data, bank):
        self.d, self.bank = data, bank

    def off(self, a):
        return a if a < 0x8000 else self.bank * 0x10000 + a

    def b(self, a):
        o = self.off(a & 0xFFFF)
        return self.d[o] if o < len(self.d) else 0xFF


def dname(v):
    return SFR.get(v, "%02Xh" % v)


def bname(v):
    if v < 0x80:
        return "%02Xh.%d" % (0x20 + v // 8, v % 8)
    base = v & 0xF8
    return "%s.%d" % (SFR.get(base, "%02Xh" % base), v & 7)


def decode(rom, pc):
    """-> (length, text, targets, kind) kind: next|jump|branch|call|ret|ind"""
    o = rom.b(pc)
    ent = OPS[o]
    m, n, kinds = ent[0], ent[1], ent[2]
    extra = ent[3] if len(ent) > 3 else ""
    bs = [rom.b(pc + i) for i in range(n)]
    args, p, tgt = [], 1, []
    reg = extra if extra and not extra.startswith(",") else None
    for k in kinds:
        if k == "d":
            args.append(dname(bs[p])); p += 1
        elif k == "i":
            args.append("#%02Xh" % bs[p]); p += 1
        elif k == "I":
            args.append("#%04Xh" % (bs[p] << 8 | bs[p + 1])); p += 2
        elif k == "b":
            args.append(bname(bs[p])); p += 1
        elif k == "r":
            t = (pc + n + (bs[p] - 256 if bs[p] > 127 else bs[p])) & 0xFFFF
            args.append("%04Xh" % t); tgt.append(t); p += 1
        elif k == "a":
            t = ((pc + n) & 0xF800) | ((o >> 5) << 8) | bs[1]
            args.append("%04Xh" % t); tgt.append(t)
        elif k == "A":
            t = bs[1] << 8 | bs[2]
            args.append("%04Xh" % t); tgt.append(t)
    if o == 0x85:  # MOV dest, src is encoded src, dest
        args = [args[1], args[0]]
    text = m
    if reg:
        if "{r}" in text:
            text = text.replace("{r}", reg)
        else:
            text = text + " " + reg if not text.endswith(",") else text + reg
    if "d" in text and text.startswith("MOV d,"):
        text = "MOV " + args.pop(0) + "," + reg
    if args:
        text = (text + (" " if not text.endswith(",") else "") + ",".join(args))
    if extra.startswith(","):
        text += extra
    kind = "next"
    if m in ("LJMP", "AJMP", "SJMP"):
        kind = "jump"
    elif m in ("LCALL", "ACALL"):
        kind = "call"
    elif m in ("RET", "RETI"):
        kind = "ret"
    elif m == "JMP @A+DPTR":
        kind = "ind"
    elif tgt:
        kind = "branch"
    return n, text, tgt, kind


def sweep(rom, starts):
    seen, todo, calls = {}, list(starts), set()
    while todo:
        pc = todo.pop()
        while pc not in seen:
            n, text, tgt, kind = decode(rom, pc)
            seen[pc] = (n, text, kind)
            if kind == "call":
                calls.update(tgt)
            todo.extend(tgt)
            if kind in ("jump", "ret", "ind"):
                break
            pc = (pc + n) & 0xFFFF
    return seen, calls


def main():
    a = sys.argv[1:]
    rom = Rom(open(a[0], "rb").read(), 0)
    starts = [0x00, 0x03, 0x0B, 0x13, 0x1B, 0x23, 0x2B]
    lst, xref, show_calls = None, None, False
    i = 1
    while i < len(a):
        if a[i] == "--bank":
            rom.bank = int(a[i + 1], 0); i += 2
        elif a[i] == "--from":
            starts.append(int(a[i + 1], 0)); i += 2
        elif a[i] == "--list":
            lst = (int(a[i + 1], 0), int(a[i + 2], 0)); i += 3
        elif a[i] == "--xref":
            xref = int(a[i + 1], 0); i += 2
        elif a[i] == "--calls":
            show_calls = True; i += 1
        else:
            sys.exit("unknown argument " + a[i])
    if lst:
        pc = lst[0]
        for _ in range(lst[1]):
            n, text, _, kind = decode(rom, pc)
            hx = " ".join("%02X" % rom.b(pc + j) for j in range(n))
            print("%05X %04X  %-9s %s" % (rom.off(pc), pc, hx, text))
            if kind in ("jump", "ret", "ind"):
                print()
            pc += n
        return
    seen, calls = sweep(rom, starts)
    if xref is not None:
        for pc in sorted(seen):
            n, text, kind = seen[pc]
            if ("%04Xh" % xref) in text or ("#%04Xh" % xref) in text:
                print("%04X  %s" % (pc, text))
        return
    if show_calls:
        for c in sorted(calls):
            print("%04X" % c)
        return
    prev = None
    for pc in sorted(seen):
        n, text, kind = seen[pc]
        if prev is not None and pc != prev:
            print()
        hx = " ".join("%02X" % rom.b(pc + j) for j in range(n))
        print("%04X  %-9s %s" % (pc, hx, text))
        prev = pc + n
    print("; %d instructions reached, %d call targets" % (len(seen), len(calls)), file=sys.stderr)


if __name__ == "__main__":
    main()
