# Scripts

| Script | Purpose |
|--------|---------|
| `archive_legacy.sh` | Assemble/refresh `legacy/` from TorquEFI, Arduino, early STM32, and `archive/*` snapshots |
| `push_legacy.sh` | Commit `legacy/` + `scripts/` and push to `origin` (`GH_TOKEN` or `GITHUB_TOKEN`) |
| `legacy_inventory.py` | Print file/size inventory of `legacy/` |

## Typical workflow

```bash
# From repo root, with artifacts path containing TorquEFI / ecu_firmware
./scripts/archive_legacy.sh /path/to/artifacts
python3 scripts/legacy_inventory.py

export GH_TOKEN=ghp_...   # or github_pat_...
./scripts/push_legacy.sh
```

Environment for `push_legacy.sh`:

- `GH_TOKEN` / `GITHUB_TOKEN` — GitHub PAT with `repo` scope
- `LEGACY_MSG` — optional commit message
- `GIT_NAME` / `GIT_EMAIL` — optional commit identity
