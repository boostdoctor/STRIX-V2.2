# Applied improvement pack

## Firmware
- `GETPROTO` / `PROTO?` → `PROTO:2,NAME:STRIXV2,MAP:12x22`
- `SYNCQ` telemetry (0–100 sync quality from tooth errors / losses / cam)
- Start injector prime scaled by ECT (less fuel when warm)
- Optional `HAL_IWDG_Refresh` in `ECU_Loop` when IWDG enabled in CubeMX

## Tuner
- Verified flash: SAVE → wait OK:SAVE → GETMAPSUM cross-check
- Connection health LED (telem age)
- Map undo stack (Ctrl+Z)
- Diff highlight after GETMAP vs previous UI tables
- CSV session log (30 s buffer)
- DBW apply interlock warning

## Tests
- `tests/test_interp.py` — bilinear / tenths semantics

## Not fully split (deferred)
- Full `ecu_app.c` module split (large refactor)
- Closed-loop idle PID / per-cyl trim UI pages (firmware hooks only as before)
