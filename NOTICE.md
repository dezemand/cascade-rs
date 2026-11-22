# Notice

cascade-rs is a derivative work of CASCADE (Car Scanner Device Emulator),
(C) Copyright 2014 Ulrich Hecht, https://github.com/uli/cascade,
and is distributed under the CASCADE Public License 1.0 (see `LICENSE`).
The whole repository, including the parts written from scratch, is under
the same licence.

No ROM packs, ECU dumps, vendor libraries or other proprietary material are
included.

## Changes relative to the original

All dated 2026-11-21 (the first published version).

### Rewritten

- The emulator (CPU, memory map and banking, HSIO, UART, LCD, keypad,
  EEPROM, the fake interface) is a translation of the C++ sources into Rust.
  It follows the original's debug build: interrupts and the event pump are
  checked before every instruction.
- The Qt user interface is replaced by a headless command-line runner
  (`cascade`) and a new GPUI desktop front end (`cascade-gui`).

### Behaviour that differs from upstream

- **ROM data-window bound** (an emulation fix): upstream accepts a bank at
  `phys` only when `phys <= rom_size - 0xc000`; this port uses
  `phys + 0x4000 <= rom_size`. Upstream redirects the last two 16 KB banks to
  offset 0, so the Hyundai pack's self-check fails and the firmware resets.
  The same fix was applied to the C++ build used as the reference.
- Undefined behaviour in upstream (out-of-bounds data/code pointers, EEPROM
  words ≥ 128, a bad HSO software-timer index) stops the emulator with an
  error instead of reading or writing out of bounds.
- Upstream's `exit(1)` paths become reported stops; an endless loop resets
  the machine and is reported as an event.
- The interface API passes the emulated cycle count to every call and gives
  backends the UART receive buffer explicitly.
- Key presses are stamped with emulated cycles, so scripted sessions are
  reproducible; front ends release held keys after a firmware reset.
- Upstream's side-effect-free mapped read (used by its disassembler) indexes
  the extended ROM without subtracting its base address, an out-of-bounds
  read; this port subtracts it.
- EEPROM files hold all 128 words (256 bytes). Upstream writes and reads
  only the first 128 bytes of its `.eep` files, so the second half was lost
  between runs; its files still load (into the first half). Headless runs
  start blank unless an EEPROM file is given.
- The disassembler decodes instruction lengths correctly (byte immediates,
  the `FE` prefix); upstream's printer does not, which does not matter for
  its traces.

Kept on purpose, because the original defines correct behaviour here:
add/sub never set V; INC never sets Z; the short-indexed offset is not
sign-extended; register shift counts are masked to the operand width; XCHB
reads 16 bits.

### Added

- Extended ROM (upstream `-x`; map_hi 0x10), ported 2026-11-22: Hyundai
  "TROUBLE SHOOTING PACK" images for the DTC HELP → FLOW screens.
- EEPROM persistence (upstream's `ROM.eep`), ported 2026-11-22; the GUI also
  keeps the EEPROM across power cycles.
- Diagnostic-line backends: a KWP2000 ECU model (also 5-baud init and ISO 9141-2 framing), replay of recorded
  sessions, a real serial K-line (a port of upstream's `iface_kl_tty`), and
  an emulated engine ECU (`ecuc167`), and `ecusim`, which runs that ECU in
  real time as a separate process with an engine model and models of the
  car's other modules.
- `kdiag elm`, an ELM327 front for OBD apps.
- CAN diagnostics (`cascade-can`): ISO-TP, Linux SocketCAN, a simulated
  OBD-II ECU, and `kdiag --can` / `kdiag can-ecu`.
- Tracing and analysis: oracle trace format and diff, line-event logs, ROM
  read and code coverage probes, a disassembler command.
- `kdiag`, a native diagnostic tool that uses the protocol data learned with
  the emulator, with serial and J2534 links.
- Protocol data files in `protocol/`.
