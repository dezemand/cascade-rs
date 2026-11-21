# cascade-rs

A Rust rewrite of [CASCADE](https://github.com/uli/cascade), Ulrich Hecht's
emulator for the Hyundai Hi-Scan Pro / Carman Scan handheld scan tool, plus
the tools that grew out of using it to learn a car's diagnostic protocol.

The emulator runs the original Hi-Scan firmware and software packs
cycle-accurately, checked instruction by instruction against the C++
original. What the firmware does on the K-line is recorded and turned into
plain data files, which drive `kdiag`, a small native diagnostic tool that
needs neither the emulator nor any Hyundai software.

The first target is a 2002 Hyundai Coupe / Tiburon (GK, 2.0 Beta) over
K-line: engine, ABS and airbag fault codes, and live engine data.

> **Status:** experimental. Everything here has been developed against
> emulated ECUs. **Nothing has been tested on a real car or a real adapter
> yet.** Protocol facts are marked as emulator-derived until confirmed.

## What is in here

| Crate | What it is |
|---|---|
| `cascade-core` | The emulator: Intel 8XC196 CPU, memory banking, HSIO, UART, SED1335 LCD, keypad, EEPROM; plus an 8XC196 disassembler. Deterministic and cycle-counted. |
| `cascade-iface` | The diagnostic-line interface between the emulated UART and the outside world: upstream's fake ECU, a KWP2000 ECU model, ISO 14230 framing, a tracer, and replay of recorded sessions. |
| `cascade-cli` | `cascade`: run the emulator headless, script key presses, save screenshots, diff traces, disassemble. |
| `cascade-diag` | `kdiag`: native KWP2000 client (ISO 14230 timing, read-only guard, CSV-driven parameter decoding), serial K-line backend (macOS), J2534 link, adapter self-test. |
| `cascade-runner` | Runs the emulator on its own thread in real time, for front ends. |
| `cascade-gui` | Desktop front end (GPUI via `gpui-kit`): a flat Hi-Scan layout with a live data view. macOS. |
| `cascade-j2534` | Minimal SAE J2534 PassThru access for ISO 14230; vendor-DLL backend on Windows. |
| `ecuc167` | Infineon C167 disassembler and emulator for running an engine ECU dump offline behind the same interface. |
| `ecu8051` | Siemens 80C517-class 8051 emulator, used to analyse an older engine ECU ROM. |

Also: `oracle/` (a container build of the original C++ CASCADE, used as the
reference), `protocol/` (what the Hi-Scan firmware asks the car and how it
scales the answers), `tools/` (stdlib-only Python helpers for driving the
emulator and reading its screen).

## You need your own software

This repository contains **no ROM packs, ECU dumps, vendor DLLs or other
proprietary material**, and never will. The emulator needs a Hi-Scan
software pack (the Hyundai pack was used here, `HP 701 USA`, part
`09910-61000`); ECU emulation needs an ECU dump. Keep them outside the
repository (`roms/` is git-ignored).

## Building

Rust stable (1.88 or later), no other system dependencies for the core
crates:

```sh
cargo build --release      # cascade, kdiag, the emulators (not the GUI)
cargo test                 # works without any ROM; ROM-dependent tests skip
```

The binaries end up in `target/release/` (`cascade`, `kdiag`, `ecuc167`,
`ecu8051`). The Python tools need Python 3 and nothing else. ROM-dependent
tests run when `CASCADE_ROM` (a Hi-Scan pack) and, for the ECU tests,
`CASCADE_ECU_ROM` (an engine ECU dump) point at your files.

The GUI is built separately because GPUI pulls in about 500 crates:

```sh
tools/bundle-gui.sh                      # builds target/Cascade.app (macOS)
open target/Cascade.app --args path/to/hyundai.his.dat
```

## Quick start

Run the Hi-Scan firmware headless and print the screen:

```sh
cascade run roms/hyundai.his.dat -m 280000000 --key 59899906:enter:1572865 --screen
```

Drive it through the menus to the engine Current Data screen, with the
built-in KWP2000 ECU model on the line (`tools/screen.py` scripts the keys
and reads the LCD as text):

```sh
python3 -I tools/screen.py roms/hyundai.his.dat --path tiburon-l4 down enter wait
```

Try `kdiag` without a car:

```sh
kdiag dtc --sim                         # fault codes from the ECU model
kdiag log --sim --duration-s 5          # live data as CSV
kdiag log --sim --j2534-mock            # the same through the J2534 path
```

On a car, through a K-line adapter (macOS):

```sh
kdiag selftest --port /dev/cu.usbserial-XXXX      # check the adapter first
kdiag dtc --port /dev/cu.usbserial-XXXX --trace dtc.jsonl
kdiag log --port /dev/cu.usbserial-XXXX --out log.csv --trace log.jsonl
```

Every `--trace` recording can be replayed into the emulated Hi-Scan, so the
original firmware shows the car's data:

```sh
python3 -I tools/screen.py roms/hyundai.his.dat --path tiburon-l4 down enter wait --replay log.jsonl
```

## The reference oracle

`oracle/` builds the original C++ CASCADE (Qt 4, Debian buster) in a Linux
container, with two small patches, one of them a real emulation fix. It
expects a clone of upstream in `upstream/uli-cascade` (git-ignored) and uses
Apple's `container` tool; any OCI builder works with small changes.

```sh
git clone https://github.com/uli/cascade upstream/uli-cascade
oracle/build.sh
```

## Safety

- `kdiag` is **read-only**. Every request passes an allowlist (session
  handling and reads) before it reaches the line; resets, writes, actuator
  control, routines, security access and flashing are always refused.
  Clearing fault codes is the single opt-in (`--allow-clear` plus a typed
  confirmation).
- The emulated Hi-Scan, connected to a real adapter, sends whatever its
  original menus send, including actuator tests and resets. Use only the
  read functions on a car.
- This project does not do ECU reprogramming and never will.
- No warranty. You are responsible for what you connect to your car.

## How it was verified

- The CPU core matches the C++ original instruction for instruction (916,040
  trace records from boot), and RAM/ROM dumps match at the main menu and
  along a full diagnosis menu path.
- The disassembler's lengths and branch targets are checked against 145
  million executed instructions.
- The engine parameter scalings were measured by sweeping raw values through
  the Hi-Scan firmware and reading its screen; then checked against an engine
  ECU's own code running in `ecuc167`: for 136 ECU-generated data blocks the
  CSV reproduces all 2,128 values the Hi-Scan displays.

## Credits and licence

cascade-rs is a derivative work of **CASCADE** by Ulrich Hecht
(<https://github.com/uli/cascade>) and is distributed under the **CASCADE
Public License 1.0** (CasPL, see [`LICENSE`](LICENSE)): free to use and
modify, distribution only for non-commercial purposes, under the same
licence. Changes relative to the original are listed in
[`NOTICE.md`](NOTICE.md).

Thanks to the CASCADE forks that carried it forward, to
[GKFlasher / gkbus](https://github.com/Dante383/GKFlasher) for its ISO 14230
implementation notes, and to the [OpenGK](https://opengk.org) project for
documenting the GK platform.
