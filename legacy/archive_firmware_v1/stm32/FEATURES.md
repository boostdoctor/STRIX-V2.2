# TorquEFI extended features

## Serial commands

| Command | Description |
|---------|-------------|
| `GETDTC` | List active DTCs → `DTC:n,0x....` |
| `CLRDTC` | Clear all DTCs |
| `SET:O2MODE,m` | 0=off 1=narrowband 2=wideband |
| `SET:WB,afrMin,afrMax,vMax` | WB scale (default 10–20 AFR @ 0–3.3 V) |
| `SET:TARGETAFR,x.x` | WB closed-loop target (default 14.7) |
| `GETAFR` | `AFR:..,MODE:..,O2V:..,TGT:..` |
| `SET:CYLTRIM,cyl,pct` | Per-cylinder fuel trim −25..+25 % |
| `GETCYLTRIM` | Trims for cyl 1..N |
| `SET:IDLEPID,kp,ki,kd` | Closed-loop idle gains |
| `GETIDLEPID` | Gains + throttle + active |


## DTC codes

| Code | Meaning |
|------|---------|
| 0x0100 / 0101 | MAP low / high |
| 0x0110 | TPS cal range |
| 0x0120 / 0121 | ECT open / overtemp |
| 0x0122 | IAT open |
| 0x0130 / 0131 | Battery low / high |
| 0x0200 / 0201 | Sync / cam loss |
| 0x0300 / 0301 | O2 stuck / AFR range |

## Wideband wiring

- Controller 0–5 V out → divider to **PA5 (ADC O2)** 0–3.3 V.
- `SET:O2MODE,2` then `SET:WB,10,20,3.3` (adjust to controller curve).


## Lambda units

| Command | Description |
|---------|-------------|
| `SET:STOICH,14.7` | Fuel stoich AFR (petrol 14.7, E85 ~9.8) |
| `SET:TARGETAFR,14.7` | CL target as AFR → reply includes `LAM:` |
| `SET:TARGETLAMBDA,1.00` | CL target as λ (alias `SET:LAMBDA,`) |
| `SET:WBL,0.68,1.36,3.3` | WB scale in λ (converts to AFR internally) |
| `GETAFR` / `GETLAMBDA` | `AFR:..,LAM:..,TGT_AFR:..,TGT_LAM:..,STOICH:..` |

Internal control still uses AFR; λ = AFR / STOICH.
