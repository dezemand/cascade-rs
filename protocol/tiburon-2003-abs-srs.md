# Tiburon (GK) ABS and SRS airbag: what the Hi-Scan firmware asks for

Same source and caveats as `tiburon-2003-engine-l4.md`: everything is
**derived from the firmware's behaviour in the emulator** and has not been
seen on the car. Menu paths: 09 TIBURON > 04 ANTI-LOCK BRAKE SYSTEM, and
09 TIBURON > 05 SRS-AIRBAG > 02 2003-2006.

## Physical line and session [emulator-derived]

| System | Line (I/O 0x250) | Address | StartCommunication as sent |
|---|---|---|---|
| Engine | 7 (K) | 0x11 | `81 11 F1 81 04` |
| ABS | 8 (OBD pin 8, ABS/TCS) | 0x28 | `80 28 F1 01 81 1B` |
| SRS | 12 (OBD pin 12), retried on 10 | 0x58 | `80 58 F1 01 81 4B` |

- All use 10,400 baud and the 25 ms fast init.
- ABS and SRS send their requests with the separate length byte. They only
  accept the session if the ECU answers with key bytes **EA 8F** and also
  uses the length-byte header (`80 F1 28 03 C1 EA 8F cs`). EF 8F or a short
  header makes them retry. The engine accepts EF 8F with short headers.
- The pin numbers are upstream's decoding of the 0x250 value; the cable must
  connect those OBD pins to the adapter's K input.

## ABS (0x28)

- Function menu: DTCs, Current Data, Flight Record, Actuation Test,
  Simu-Scan, Air Bleeding Mode, Identification Check, Data Setup.
  Actuation and air bleeding drive hardware: not to be used (project rules).
- DTCs: `13 40 00` (ReadDiagnosticTroubleCodes, group 0x4000). Response
  `53 N` + N × (high, low), no status byte; the top two bits give the
  letter (`50 1F` shows C101F, `40 15` shows C0015).
- Current Data: `21 01`, a 17-byte block (and once `21 00 01`, which the
  firmware tolerates being refused). Offsets and scaling:
  `tiburon-2003-abs-current-data.csv`. Highlights: engine speed u16 LE ÷ 4,
  vehicle and four wheel speeds 1 km/h per bit, G sensors raw × 4/255 − 2 g,
  warning lamps and switches as bit pairs (state, not-available).
- Not mapped: transmission range byte (offset 4: 0 shows "-", FF "P,N").

## SRS (0x58)

- Function menu: DTCs, Crashing Information, Identification Check.
- DTCs: two requests, `18 00 FF 00` and `18 01 FF 00` (ReadDTCsByStatus,
  all groups, status filters 0x00 and 0x01). `58 00` shows NO TROUBLE CODE.
  Record format with codes present: not yet checked.
- Crashing Information (front/side): starts with `21 00 01`; not explored
  (it shows COMMUNICATION ERROR when refused).
