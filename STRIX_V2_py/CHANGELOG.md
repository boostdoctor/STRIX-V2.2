# STRIX V2 Changelog

## 2026-08-11 — Improvement pack

### Safety
- Flash (SAVE) blocked when RPM > 0 (firmware `ERR:SAVE,RPM` + UI)
- DBW apply requires TPS open/closed ADC span
- Busy flag blocks map cell TX during Flash

### Reliability
- USB telemetry loss → auto-reconnect attempt
- Offline queue for SET:/CFG: when disconnected; flush on connect
- PROTO:2 handshake warning if ECU version mismatches
- Verified Flash via GETMAPSUM (existing)

### UI
- Strip presets: Street / Dyno / Debug
- Map diff count after GETMAP vs previous UI tables
- Setup wizard + engine settings (existing)
- Link health LED (existing)

### Firmware
- Flash blob v8 with full engine settings
- Closed-loop idle SET:IDLEEN / IDLERPM / IDLEPID
- Module stubs: ecu_settings, ecu_idle, ecu_maps

### Docs
- STRIX_V2_User_Manual.docx
- STRIX_V2_BlackPill_Pinout.pdf
