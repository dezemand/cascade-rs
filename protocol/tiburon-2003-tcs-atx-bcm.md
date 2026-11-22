# Tiburon (GK) traction control, automatic transaxle, body control: what the Hi-Scan firmware asks for

Same source and caveats as `tiburon-2003-engine-l4.md`: everything is
**derived from the firmware's behaviour in the emulator** and has not been
seen on the car. The menu (09 TIBURON) also lists ENGINE V6, CRUISE and EPS
(up to 2001, not the GK) and TRANSMITTER CODE SAVING (immobiliser key
programming: not used, project rules).

## Traction control (TCS, 2003-, address 0x32) [emulator-derived]

Menu: 09 TIBURON > 08 TRACTION CONTROL SYSTEM. Function menu: DTCs, Current
Data, Flight Record, Actuation Test, Simu-Scan, Data Setup. Actuation drives
the hydraulic unit: not to be used (project rules).

### Line and session

- OBD pin 8 (as for ABS), 10,400 baud.
- **5-baud init** to address **0x32** (not a fast init). The module answers
  `55`, key bytes **08 08**; the Hi-Scan sends the inverted second key byte
  (`F7`), the module the inverted address (`CD`). This is the ISO 9141-2
  handshake.
- Frames are **ISO 9141-2**, no length byte: requests `68 6A F1 data sum`,
  responses `48 6B 32 data sum` (sum = 8-bit sum of the preceding bytes).
  A frame ends on an inter-byte gap.
- Keep-alive: `B1 01`, answered `F1 01`. The Hi-Scan sends it between the
  other requests.

### Fault codes

- Request `03` (OBD-style mode 03). Response `43` + three codes of two
  bytes, `00 00` for an empty slot; the top two bits are the letter (C =
  01, so C1200 is `52 00`). The Hi-Scan reads at most three codes from one
  frame; a module with more sends more frames (ISO 9141-2 mode 03; one frame
  per request is all the model does).
- Code texts: `dtc-names-tcs.csv`, read off the Hi-Scan's TCS screen. It is
  **not** the ABS table: about 30 shared codes have other texts (C1112
  "5V SOURCE OVER RANGE" vs "SENSOR POWER VOLTAGE", C1235 "FRONT HEIGHT
  SENSOR" vs "PRESSURE SNSR(1ST)-ELECTR.", …).
- Clearing (mode 04) was not looked at; kdiag refuses it for this module.

### Current data

Six items, from two manufacturer requests polled in turn:

| Request | Response | Items |
|---|---|---|
| `B1 0E` | `F1 0E` + 4 bytes (at least) | wheel speed FL, FR, RL, RR: 1 km/h per bit, shown in MPH |
| `B1 2C` | `F1 2C` + 1 byte (at least) | TC FL valve = bit 0, TC FR valve = bit 2 (set = ON) |

Definitions: `tiburon-2003-tcs-wheel-speeds.csv`, `tiburon-2003-tcs-valves.csv`.
Speeds checked at raw 10, 20, 100, 200, 255 (6, 12, 62, 124, 158 MPH). The
Hi-Scan accepts responses with only these bytes; real modules probably send
more. `B1` also carries the actuation tests, so kdiag allows exactly `B1 01`,
`B1 0E` and `B1 2C` (`guard::check_tcs`).

## Automatic transaxle (2003-2004, address 0x18) [emulator-derived, unresolved]

- K line (pin 7), fast init, `StartCommunication` to 0x18.
- The firmware rejects the model's StartCommunication answer whatever the
  key bytes tried, so nothing
  after the session start was seen. Only relevant for an automatic car.

## Body control (BCM, 2003-) [emulator-derived, not mapped]

- The firmware sends proprietary frames without a KWP header, e.g.
  `05 EA 52 23 45 00 A9`: the first byte is the length, the last an 8-bit
  sum. Line value 0xFF in the 0x250 register (pin decoding unknown).
- Not answered in the model; nothing more is known.
