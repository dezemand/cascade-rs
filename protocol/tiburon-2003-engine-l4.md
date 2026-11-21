# Tiburon 2003, Engine L4: what the Hi-Scan firmware asks for

Source: Hyundai pack `S-System 09-07-2007` (HP 701 USA, part 09910-61000),
menu path 01 Hyundai Vehicle Diagnosis > 09 TIBURON > 01 ENGINE L4 > 04 2003,
run in cascade-rs against the `KwpEcu` model (`--iface kwp`).

**Everything here is derived from the firmware's behaviour in the emulator.
None of it has been seen on the car yet.** Mark items verified only after
checking them on the real ECU.

## Session [emulator-derived; the init sequence also matches the C++ oracle]

- 10,400 baud (UART divisor 0x77).
- KWP2000 fast init: K low about 25 ms, then high.
- `81 11 F1 81 04`: StartCommunication to address 0x11 from tester F1.
  The firmware accepts key bytes `EF 8F` (any others not tried).
- `81 11 F1 3E C1` TesterPresent, about every 125 ms while idle in menus.
- ISO 14230 framing (format byte, target, source, [length], data, sum checksum).

## Diagnostic trouble codes [emulator-derived]

- Request `18 00 00 00` (ReadDTCsByStatus, status 0x00, group 0x0000),
  polled about every 0.8 s.
- Response `58 N` followed by N × (code high, code low, status) is decoded:
  `58 02 03 01 E0 01 30 20` shows "P0301 CYL.NO.1,MISFIRE DETECTED" and
  "P0130 O2 SNSR CIRCUIT-MAL(B1/S1)". Status bits: not yet investigated.

### DTC detail (DTAL key on the DTC screen) [emulator-derived]

- Request `17 hi lo` (ReadStatusOfDTC for one code, e.g. `17 03 01`).
- Response `57 01 hi lo status` + record. The firmware reads:
  - status bit 7: MIL on; bit 6: pending; bit 5: history; both bits 5 and 6
    (as in `E0`): present; bit 4: readiness not complete. Bits 0-3: unused.
  - record byte +5 (counting from the byte after status): statistic counter.
  - +7..8, +9..10, +11..12: **big-endian** 16-bit counters in 6-minute units:
    time stamp at detection, time stamp at erasure, and the current counter.
    Shown "op. hour after detection" = (current − detection stamp) × 6 min,
    "after erasure" = (current − erasure stamp) × 6 min, wrapped to signed 16 bit.
- So the status byte in the `58` list response uses the same bits.

## Readiness test [emulator-derived]

- Request `01 01`: OBD-II mode 01 PID 01 carried over KWP2000. Response
  `41 01 A B C D` decoded exactly as SAE J1979 (A: MIL bit 7 + DTC count;
  B: misfire/fuel/component supported bits 0-2, incomplete bits 4-6;
  C: supported catalyst, heated catalyst, EVAP, secondary air, A/C, O2,
  O2 heater, EGR in bits 0-7; D: the same bits, set = not complete).

## Freeze frame [emulator-derived]

- Request `12 00 00`. Response `52 00` followed by J1979-style values:
  rpm (big-endian, ÷ 4), vehicle speed (km/h), calculated load (× 100/255),
  coolant (A − 40 °C), fuel system status bank 1, one unused byte, short-term
  fuel trim (A − 128) × 100/128, one unused byte, long-term trim (same).
  Example `52 00 0C 80 ...` shows 800 rpm.

## Identification check [emulator-derived]

- Request `1A 8E` (ReadEcuIdentification option 0x8E). The firmware accepts
  `5A 8E` + ASCII and shows it in a large font under MODEL / SYSTEM.
  With the 11-character reply `39110-23700` (an ECU part number, made up)
  it showed `39110-23`: the first 8 characters only, or cut by the screen
  width at that font size (not distinguished). One request, no polling.
- Function 11 VERSION CONFIGURATION writes a configuration into the ECM and
  10 RESETTING ADAPTIVE VALUES resets the ECU: not explored, not to be used.

## Current Data [emulator-derived]

- Request `21 01` (ReadDataByLocalIdentifier 0x01), about every 0.17 s.
  This single block feeds all 27 parameters on the Current Data screen.
- The firmware reads at least 76 bytes after `61 01`, so the real response
  must use the ISO 14230 extra length byte (`80 F1 11 LEN 61 01 ...`).
  Offsets below are relative to the first byte after `61 01`, with that
  framing. Short frames (≤ 63 bytes) are misparsed by one byte.
- Method: `tools/probe_block.py` sets one byte (or bit) of an otherwise zero
  80-byte block and compares the 22 cells of the FULL view.

Scaling **[emulator-derived]**: little-endian, `value = raw × scale + add`.
Machine-readable copy: `tiburon-2003-engine-l4-current-data.csv`.

| Offset | Size | Parameter (FULL view label) | Scale | Add | Shown unit |
|---|---|---|---|---|---|
| 0 | u8 | BATTERY | 0.1015625 (26/256) | 0 | V |
| 3 | u8 | COOL.TEMP. | 1.35 (0.75 °C) | −54.4 (−48 °C) | °F |
| 5 | u8 | AIR TEMP. | 1.35 (0.75 °C) | −54.4 (−48 °C) | °F |
| 7 | u8 | TPS | 0.46875 (120/256) | 0 | ° |
| 8–9 | u16 | ADAPTED TP | ≈0.0018254 | 0 | ° |
| 10 | u8 | MAP SENSOR | ≈19.5294 (4980/255) | 0 | mV |
| 13–14 | u16 | MAP SENSOR | ≈0.00120317 | 0 | psi |
| 18 | u8 | VEHIC.SPD | 0.621371 (1 km/h) | 0 | MPH |
| 19–20 | s16 | ENGINE SPD | 1 | 0 | rpm |
| 21–22 | s16 | TAR.IDLE | 1 | 0 | rpm |
| 24 bit 0 | flag | TRAN.RANG. (set = DRIVE, clear = P,N) | | | |
| 25 bit 0 | flag | A/C SWITCH | | | |
| 25 bit 2 | flag | A/C RELAY | | | |
| 26–27 | u16 | FR O2S-B1 | ≈4.87797 | 0 | mV |
| 30 | u8 | IGNITION | −0.375 | 72 | ° |
| 42–43 | u16 | INJECTION | 0.004 (4 µs) | 0 | ms |
| 57–58 | u16 | ISC DUTY | ≈0.00153 | 0 | % |
| 59–60 | s16 | IDLE SPD | ≈0.00153 | 0 | % |
| 73 bit 1 | flag | CRANK.SIG. | | | |
| 73 bit 2 | flag | CLOSED TP | | | |
| 73 bit 3 | flag | PART LOAD | | | |
| 74–75 | u16 | IGN DWELL | 0.004 (4 µs) | 0 | ms |
| 28–29 | u16 | RR OXYGEN SENSOR | ≈4.87797 | 0 | mV |
| 46 bit 0 | flag | A/F CLOSE LOOP | | | |
| 51–52 | s16 | LONG FUEL-IDLE | 0.004 | 0 | ms |
| 53–54 | s16 | LONG FUEL-PART | ≈0.00153 | 0 | % |
| 67–68 | u16 | EVAP.PURGE VALVE | ≈0.0030518 (200/65536) | 0 | % |

Checked against the ECU's own code **[verified in emulation, not on the car]**:
a GK-20 SIMK41 engine ECU dump (not in this repository) runs in `ecuc167` behind the
Hi-Scan emulator and behind `kdiag --sim-ecu`.
- StartCommunication answers key bytes EF 8F; `21 01` returns 78 data bytes
  (the Hi-Scan reads up to offset 75).
- `tools/ecu_sweep.py` swept eight ADC inputs (MAP, O2 front/rear, battery,
  throttle, coolant, intake air, channel 1) over 17 levels each: 136 blocks,
  and for every one the CSV decode matches the Hi-Scan's FULL screen (2,128
  values, 0 mismatches, within display rounding). That covers the full
  real range of each sensor, so the formulas are linear there with no kinks.
  The ≈ factors stay bracketed: the screen cannot pin them closer.
- Not exercised (constant in the emulator: no crank signal, no switch
  inputs): vehicle and engine speed, ignition timing, injection, fuel trims,
  purge, the flag bits.
- Sensor behaviour seen in the ECU code:
  - MAP pressure is linear in sensor voltage (about 24.7 kPa/V, about 0 at
    0 V) and clamped to exactly 10.0 kPa (word 1206) and 115.0 kPa (word
    13869). If those clamps are round kPa values, the exact factor is 1/120.6
    kPa = 0.00120264 psi per bit, 0.04 % below the bracketed 0.00120317
    **[assumed]**; the screen cannot tell them apart.
  - TPS byte = ADC/4 − 64 (0 at 0.5 V and below); a full-scale (faulty)
    sensor reads 0. Adapted TP stays at its default, 16384.
  - O2 words are the raw 10-bit ADC values; battery is ADC/4.
  - With ADC channel 1 at 0.6 V or less the ECU still answers StartComm and
    some TesterPresents but not `21 01` (a supply or ignition sense input
    **[assumed]**).

How the scaling was found:
- `tools/scale_sweep.py` sets all parameters to 10 raw values (one byte of the 16-bit ones at a time, which settles byte order).
- It reads the screen exactly: `tools/lcdtext.py` matches LCD memory against the 8×16 font in the ROM pack.
- `tools/check_scaling.py` checks the formulas against all 160 readings.
- With `VALIDATE=8` it also predicts 8 random blocks it was not fitted on: 128 of 128 predictions matched.
- Factors marked ≈ are bracketed by boundary samples (for example, front O2 lies between 4.87793 and 4.87801). Treat them as good to about 0.01 %.

Display rules (needed to read the screen, not part of the protocol):
- Decimal fields round.
- Integer fields truncate.
- Fields are 5 characters wide, and longer numbers are cut off.

Limits:
- Linear behaviour is confirmed only at the tested points (single bytes 0, 1, 2, 10, 50, 64, 100, 128, 200, 255, plus random held-out values). A lookup table with kinks between those points cannot be ruled out, though the held-out checks make one unlikely.
- Signedness of engine speed, target idle and idle-speed duty comes from 0xC800 showing as negative.

Open:
- The true block length and the bytes no parameter on this page uses.
- Flight Record reuses `21 01`; Simu-Scan and Symptom Analysis not explored.
  Actuation Test, EVAP leakage test, adaptive reset and version
  configuration act on the car and are not to be driven (project rules).
