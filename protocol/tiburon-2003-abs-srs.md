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
- Transmission range (TR SWITCH), offset 4: the lowest set bit wins.
  bit0 P,N; bit1 R; bit2 D; bit3 4; bit4 3; bit5 2; bit6 L; bit7 SPORTS;
  0x00 shows "-" (swept 0x00-0x08 and every single bit; 0x03 shows P,N,
  0x06 shows R). Not in the CSV, which has no enum type; kdiag logs it raw.

## SRS (0x58)

- Function menu: DTCs, Crashing Information, Identification Check.
- DTCs: two requests, `18 00 FF 00` and `18 01 FF 00` (ReadDTCsByStatus,
  all groups, status filters 0x00 and 0x01). `58 00` shows NO TROUBLE CODE.
- Record format with codes present: as the engine, `58 N` + N × (code
  high, code low, status). The letter is in the top two bits of the code
  (00 P, 01 C, 10 B, 11 U), so B1102 is `91 02` and shows "BATTERY VOLTAGE
  LOW"; `11 02` would be P1102.
- Code texts: `dtc-names-srs.csv`, 433 B codes, read off the Hi-Scan's
  Tiburon SRS DTC screen for every code B1000-B2999
  (`tools/dtc_names.py`); unknown codes show "%s Not Define Code". They
  include the airbag circuits (DAB/PAB driver/passenger airbag, DSAB/PSAB
  side airbags, DBPT/PBPT belt pretensioners, firing-circuit crosses,
  impact sensors, B2500 SRI lamp).
- Drawing many unknown codes takes the Hi-Scan about 6 s; with only
  TesterPresent pending, a module with a 5 s P3max would end the session
  ("COMMUNICATION ERROR"). Whether the real module does is unknown.
  Codes from both requests are listed and counted ("2 ITEMS" for one of
  each). Codes from `18 01` are marked H (history) whatever their status
  byte; codes from `18 00` are not, so `18 00` reads current and `18 01`
  stored codes.
- Identification Check: `1A 80`. The Hi-Scan shows fields at these bytes of
  the response (`5A` = 0): VEHICLE 1-6 (starting with the option byte
  `80` itself), PART NO 7-17, PART NAME 18-37, ACU VERSION 68 "." 69.
  Found by answering it with a position pattern; unverified on a module.
- Crashing Information (front/side): `21 00 01` first; when answered
  `61 00 …` the firmware polls `21 00` continuously (refused: four tries,
  then `20`, an unsupported service, and back). Answered with a 40-byte
  record of all 00 or all FF, the screen stays blank, so the record layout
  is unknown. Only the car's reply will tell; the trace is what matters.
