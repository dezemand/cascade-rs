# Scenarios for `ecusim`

Each file is a timed list of controls: `SECONDS NAME VALUE` per line (wall
time from the start), `#` comments. Run one against a running ECU process:

```sh
ecusim play scenarios/loose-coolant-connector.txt
```

Then diagnose with the Hi-Scan (line "Process") or `kdiag --ecusim auto`.
The codes named here are what the GK 2.0 2003 SIMK41 dump stores; other
dumps may store others (see `tools/fault_sweep.py`).
