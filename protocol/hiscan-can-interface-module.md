# Hi-Scan Pro CAN Interface Module

The 2006+ menus in the Hyundai pack (first seen on Sonata NF → Engine L4)
show "PLEASE CONNECT THE CAN INTERFACE MODULE BETWEEN HISCAN PRO AND MAIN
DLC CABLE" and then talk to that module over the Hi-Scan's UART. Read from
the firmware in the emulator; `cascade run … --iface canmod` emulates the
module with a KWP2000-on-CAN engine ECU behind it.

## Selecting it [verified]

- The firmware selects the CAN line by writing **0xDF to I/O 0x250**
  (`Line(42)` / `Can(true)` in cascade's line log). In the pack this
  happens only in the module start-up routine of one code bank (physical
  0xF8000, entry 0xC000, reached from the system dispatcher in bank 0x30);
  the same routine is in the base firmware.
- Start-up: select the line, pulse bits of I/O 0x252, configure, set the
  UART to 0x9600 (38,400) [assumed from the argument], ping the module.

## Messages [verified]

Hi-Scan → module: `F1 mode len payload`. Module → Hi-Scan:
`42 F2 mode len payload`.

| Hi-Scan sends | Meaning | Answer |
|---|---|---|
| `F1 FA 00` | ping | `42 F2 F6 00` |
| `F1 FB 14` + 20 bytes | configuration; the second one contains `FD 00` = 0x7E8 << 5 [assumed: acceptance filter] | `42 F2 FC 0A FC E0 70 01 00…` (upstream's fake; accepted) |
| `F1 FC 0A id_hi id_lo d0..d7` | send one CAN frame: identifier << 5 in two bytes (`FC 00` = 0x7E0), 8 data bytes | each frame from the car: `42 F2 FC 0A id_hi id_lo d0..d7` |
| `F1 F8 00` | close | `42 F2 FF 00` |

The data bytes are ISO-TP (ISO 15765-2) frames that the firmware builds and
parses itself: single frames, and its own flow control `30 00 01` (no block
limit, STmin 1 ms) for long answers.

## What runs over it [verified, Sonata NF engine]

KWP2000 services on 0x7E0/0x7E8: `10 81` (default session), `18 00 FF 00`
(codes by status; answer `58 n (code hi, code lo, status)…`), `21 01`
(the live-data block, 65 items; its layout for this car is not known).
With the emulated module the Hi-Scan lists stored codes with their texts
and fills the Current Data screen.

## Not known

- The real module's answers to `FA` and `FB`, its own frame timing, and
  what the 20 configuration bytes mean beyond the identifier.
- Whether the module needs the I/O 0x252 pulses on real hardware.
- Nothing of this applies to the Tiburon GK: its diagnostics are K-line,
  and its engine CAN bus carries only broadcasts (see the README).
