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
K-line: engine, ABS, airbag and traction control fault codes, and live
engine, ABS and traction control data.

> **Status:** experimental. Everything here has been developed against
> emulated ECUs. **Nothing has been tested on a real car or a real adapter
> yet.** Protocol facts are marked as emulator-derived until confirmed.

## What is in here

| Crate | What it is |
|---|---|
| `cascade-core` | The emulator: Intel 8XC196 CPU, memory banking, HSIO, UART, SED1335 LCD, keypad, EEPROM; plus an 8XC196 disassembler. Deterministic and cycle-counted. |
| `cascade-iface` | The diagnostic-line interface between the emulated UART and the outside world: upstream's fake ECU, a KWP2000 ECU model, ISO 14230 framing, a tracer, and replay of recorded sessions. |
| `cascade-cli` | `cascade`: run the emulator headless, script key presses, save screenshots, diff traces, disassemble. |
| `cascade-diag` | `kdiag`: native KWP2000 and ISO 9141-2 client (ISO 14230 timing, 5-baud init, read-only guard, CSV-driven parameter decoding), serial K-line backend (macOS), J2534 link, adapter self-test. |
| `cascade-runner` | Runs the emulator on its own thread in real time, for front ends. |
| `cascade-gui` | Desktop front end (GPUI via `gpui-kit`): a flat Hi-Scan layout with a live data view. macOS. |
| `cascade-can` | CAN diagnostics transport: ISO-TP over a bus trait, Linux SocketCAN (`can0`, `vcan0`), an in-memory bus and a simulated OBD-II ECU. |
| `cascade-j2534` | Minimal SAE J2534 PassThru access for ISO 14230; vendor-DLL backend on Windows. |
| `ecuc167` | Infineon C167 disassembler and emulator for running an engine ECU dump offline behind the same interface. |
| `ecusim` | The simulated car: an engine ECU dump running in real time as its own process (`ecusim serve`), ABS/TCS/SRS models, an engine model that runs on the ECU's own fuel and spark, faults, scenarios, and a socket testers connect to. |
| `ecusim-gui` | Control panel for `ecusim`: ignition, starter, throttle, temperatures, battery, road speed (GPUI). macOS. |
| `ecu8051` | Siemens 80C517-class 8051 emulator, used to analyse an older engine ECU ROM. |

Also: `protocol/` (what the Hi-Scan firmware asks the car and how it scales
the answers), `tools/` (stdlib-only Python helpers for driving the
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
`CASCADE_ECU_ROM` (an engine ECU dump) point at your files;
`CASCADE_ECU_ROM_NOIMMO` (a dump without immobiliser) runs the crank test.

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
kdiag dtc --sim --system srs            # airbag codes with the Hi-Scan's texts
kdiag ident --sim --system srs          # airbag module identification
kdiag log --sim --system tcs --duration-s 2   # traction control (ISO 9141-2)
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

Hyundai packs can show official troubleshooting flowcharts (DTC screen →
HELP → FLOW). They live in a separate "TROUBLE SHOOTING PACK" image, which
the emulator loads as an extended ROM, as the original's `-x` option does:

```sh
cascade run roms/hyundai.his.dat --exrom roms/i-info.his.dat …
open target/Cascade.app --args roms/hyundai.his.dat --exrom roms/i-info.his.dat
```

Without it the firmware reports "TROUBLE SHOOTING PACK NOT FOUND!".

The handheld keeps settings and some state in a small serial EEPROM. The GUI
saves it next to the ROM as `ROM.eep`, as the original does (`--eeprom PATH`
for another file, `--no-eeprom` to keep it in memory), and keeps it across
power cycles. `cascade run` starts blank unless given `--eeprom FILE`, so
headless runs stay reproducible.

## A simulated car

`ecusim` runs a GK 2.0 engine ECU dump in real time, as a separate process,
and puts models of the ABS, traction control and airbag modules beside it.
It powers the ECU from the battery, turns a 60-2 crank wheel and the cam,
and drives the sensors. Testers connect to it like to a car: the Hi-Scan GUI
(line option "Process"), `kdiag --ecusim auto`, `cascade run --ecusim auto`,
and, through `kdiag elm`, ELM327 OBD apps.

```sh
tools/bundle-ecusim.sh
open target/EcuSim.app --args --rom path/to/simk4x-without-immobiliser.bin
kdiag log --ecusim auto --duration-s 5
```

Headless: `ecusim serve --rom ROM`, then `ecusim set ignition on`,
`ecusim set starter on`, `ecusim set throttle 20`, `ecusim status`.

**The engine.** With the GK 2.0 2003 SIMK41 dump the engine runs on the
ECU's own outputs: its injection time, ignition advance and idle-air duty
(read through the ECU's own data-block address table) drive a small engine
model, so the engine catches on the ECU's fuel, its idle controller holds
it at its 830 rpm target, the front O2 switches as its fuel feedback trims,
and a fuel cut stalls it. The physics is invented and calibrated to that
dump [assumed]; other dumps use an open-loop model (engine speed follows the
starter and throttle). `ecusim set model open` forces open loop.

**The dumps.** All from the OpenGK collection, immobiliser disabled:

| Dump | Family | Notes |
|---|---|---|
| GK 2.0 2003 | SIMK41, 256 KB, 16 MHz | fully mapped: closed loop, memory readout, fault catalogue |
| Elantra XD 2001 | SIMK41 | older code set (P0105, P0115, P0120); stores P1602 (automatic gearbox link) |
| GK 2.0 CVVT 2003 | SIMK43, 512 KB, 24 MHz | MAF instead of MAP, oil temperature input; stores P1602 |
| GK 2.0 CVVT 2005, Tucson 2005 | SIMK43 | as above; store P0600 (no CAN bus emulated) |

Each identifies itself, keeps crank sync and sets and clears its own codes.
The clock and the sensor wiring follow the dump size.
`tools/fault_sweep.py DUMP --csv DUMP.faults.csv` maps which code a dump
stores for each sensor fault; `ecusim` reads that profile from beside the
dump and the panel shows its codes. For dumps whose memory is not mapped,
an internal tester reads the ECU over its K-line while no outside tester
uses it.

- Use a dump **without immobiliser**. With one, the ECU stops crank
  processing when its immobiliser does not answer; this project does not
  work around that.
- Crank: 60-2 on CC15/T7IN, T0IN and GPT2 CAPIN; cam on CC14, low from tooth
  50 to 110 of the 720° cycle (the one pattern every dump syncs to without a
  cam code). Road speed goes to the input the running software reads (CC30
  or CC13).
- On the Hi-Scan use TIBURON → ENGINE L4 → **2003** for the SIMK41 and
  **2004** for the SIMK43 data layout; other entries expect another layout.

**Fault codes.** The panel (or `ecusim set fault.coolant_open on`) injects
wiring faults: engine sensors and the cam signal, the ABS front-left wheel
sensor and pump motor, the driver and passenger airbags. Faults can be on,
off or intermittent. The ECU's own code (or the module model) stores the
code, and the Hi-Scan reads and erases it like on the car (ERAS with key
on, engine off; STOP ENGINE stalls the engine with the key on). Codes stay
through key cycles; a battery disconnect wipes them, as the ECU is modelled
as powered from the battery (its own non-volatile store is not emulated).
The panel shows what the ECU holds, read from its memory: stored codes and
its own readings.

**Scenarios and time.** `ecusim set time 4` runs the simulator four times
faster while no tester is connected. `ecusim set snapshot NAME` /
`restore NAME` save and restore the whole state (in memory). `ecusim play
scenarios/loose-coolant-connector.txt` runs a timed scenario; see
`scenarios/`.

**Tools against the simulated car.**

- `tools/car_day_rehearsal.sh`: every read-only kdiag step of the first car
  session, for all four systems, against the running simulator.
- `kdiag elm --ecusim auto` serves ELM327 on 127.0.0.1:35000 for OBD apps,
  through kdiag's read-only guard (`--allow-clear` lets mode 04 through).
- `kdiag ... --ecusim auto --j2534-mock` runs kdiag's J2534 path against
  the simulator. A virtual J2534 DLL for Windows software was not built: it
  could not be tested here.

## CAN (OBD-II on ISO 15765-4 cars)

The Tiburon GK has no CAN diagnostics: all its modules are on the K line.
kdiag can also talk OBD-II over CAN, for other cars and for testing tools:

- `--can IFACE` uses a Linux SocketCAN interface. kdiag does not configure
  it; bring it up first:
  `sudo ip link set can0 up type can bitrate 500000`, or a virtual bus with
  `sudo ip link add dev vcan0 type vcan && sudo ip link set vcan0 up`.
- `--can sim` uses an in-memory bus with a simulated OBD-II engine ECU, on
  any OS.
- `dtc`, `ident`, `log`, `clear` and `elm` then use OBD-II modes 03/07,
  09 (VIN, ECU name), 01 and 04, through the same read-only guard.
  `--can-ids TX:RX` (default `7E0:7E8`; `7DF:7E8` for a functional
  request, 29-bit ids as 8 hex digits) and `--can-pad XX|none` (default AA).
- `kdiag can-ecu --can vcan0` answers on the bus as the simulated ECU;
  `kdiag can-ecu --can vcan0 --ecusim auto` answers as the simulated car's
  K-line engine ECU (a gateway, behind the read-only guard), so CAN tools
  can be tried against the real ECU code.

ISO-TP is done in user space, so all of it except the SocketCAN socket
itself is tested on any OS. **Status:** the SocketCAN path compiles for
Linux but has not been run on `can0` or `vcan0` yet [assumed to work]. The
Hi-Scan firmware's own CAN mode is not wired up (its adapter's CAN framing
is unknown).

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

- The CPU core matches the original C++ CASCADE (debug build, with the ROM
  bank fix from `NOTICE.md`) instruction for instruction (916,040
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
