# Legacy development (pre–STRIX V2.2)

Historical code kept for reference. **Active development is STRIX V2.2**
(`STRIX_V2/` firmware + `STRIX_V2_py/` tuner).

| Path | Description |
|------|-------------|
| `TorquEFI/` | Early package: Arduino, first STM32 port, simple tuner, docs |
| `TorquEFI/arduino_legacy/` | Arduino `ecu_firmware.ino` + `ecu_config.h` |
| `arduino/` | Standalone Arduino firmware copy |
| `TorquEFI_STM32F411/` | Early CubeIDE snapshot |
| `ecu_firmware_stm32/` | Early STM32 Core notes/sources |
| `archive_firmware_v1/` | Archived firmware snapshot (if present) |
| `archive_tuner_v1/` | Archived tuner snapshot (if present) |

Do **not** flash these for new builds — use `STRIX_V2/STRIX_V2` and `STRIX_V2_py`.

Refresh this tree:

```bash
./scripts/archive_legacy.sh /path/to/artifacts
./scripts/push_legacy.sh      # optional: commit + push
```

Last assembled: 2026-08-24T10:51:37Z
