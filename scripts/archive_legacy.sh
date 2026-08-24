#!/usr/bin/env bash
# archive_legacy.sh — assemble legacy/ from TorquEFI, Arduino, early STM32 snapshots
# Usage:
#   ./scripts/archive_legacy.sh [SOURCE_ROOT] [DEST_REPO_ROOT]
# Defaults: SOURCE_ROOT=../.. or cwd parents; DEST=repo root (script's ../)
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${2:-$SCRIPT_DIR/..}" && pwd)"
SRC_ROOT="${1:-}"

# Auto-detect source root (artifacts tree with TorquEFI / ecu_firmware)
if [[ -z "$SRC_ROOT" ]]; then
  for cand in \
    "$REPO_ROOT/.." \
    "$REPO_ROOT/../artifacts" \
    "/home/workdir/artifacts" \
    "$PWD" \
    "$PWD/artifacts"; do
    if [[ -d "$cand/TorquEFI" || -d "$cand/ecu_firmware" ]]; then
      SRC_ROOT="$(cd "$cand" && pwd)"
      break
    fi
  done
fi

if [[ -z "${SRC_ROOT:-}" || ! -d "$SRC_ROOT" ]]; then
  echo "ERROR: cannot find source root with TorquEFI / ecu_firmware" >&2
  echo "Usage: $0 /path/to/artifacts [repo_root]" >&2
  exit 1
fi

LEGACY="$REPO_ROOT/legacy"
STAMP="$(date -u +%Y-%m-%dT%H:%M:%SZ)"
echo "==> Source:  $SRC_ROOT"
echo "==> Dest:    $LEGACY"
echo "==> Stamp:   $STAMP"

mkdir -p "$LEGACY"
rsync_opts=(-a --delete --exclude '__pycache__' --exclude '*.pyc' --exclude '.git' --exclude '*.bak')

copy_tree() {
  local src="$1" dst="$2"
  if [[ -d "$src" ]]; then
    mkdir -p "$dst"
    if command -v rsync >/dev/null 2>&1; then
      rsync "${rsync_opts[@]}" "$src"/ "$dst"/
    else
      rm -rf "$dst"
      mkdir -p "$(dirname "$dst")"
      cp -a "$src" "$dst"
    fi
    echo "  + $src  →  ${dst#$REPO_ROOT/}"
  else
    echo "  - skip (missing): $src"
  fi
}

copy_files() {
  local srcdir="$1" dstdir="$2"
  shift 2
  mkdir -p "$dstdir"
  for f in "$@"; do
    if [[ -e "$srcdir/$f" ]]; then
      cp -a "$srcdir/$f" "$dstdir/"
      echo "  + $srcdir/$f"
    fi
  done
}

# TorquEFI package
copy_tree "$SRC_ROOT/TorquEFI" "$LEGACY/TorquEFI"

# Standalone Arduino
if [[ -d "$SRC_ROOT/ecu_firmware" ]]; then
  mkdir -p "$LEGACY/arduino"
  copy_files "$SRC_ROOT/ecu_firmware" "$LEGACY/arduino" ecu_firmware.ino ecu_config.h
elif [[ -d "$SRC_ROOT/TorquEFI/arduino_legacy" ]]; then
  copy_tree "$SRC_ROOT/TorquEFI/arduino_legacy" "$LEGACY/arduino"
fi

# Early CubeIDE snapshots
copy_tree "$SRC_ROOT/TorquEFI_STM32F411" "$LEGACY/TorquEFI_STM32F411"
copy_tree "$SRC_ROOT/ecu_firmware_stm32" "$LEGACY/ecu_firmware_stm32"

# Local archive/ snapshots (v1 firmware/tuner) if present
if [[ -d "$SRC_ROOT/archive/firmware_v1" ]]; then
  copy_tree "$SRC_ROOT/archive/firmware_v1" "$LEGACY/archive_firmware_v1"
fi
if [[ -d "$SRC_ROOT/archive/tuner_v1" ]]; then
  copy_tree "$SRC_ROOT/archive/tuner_v1" "$LEGACY/archive_tuner_v1"
fi

# Manifest
{
  echo "# Legacy archive manifest"
  echo "Generated: $STAMP"
  echo "Source:    $SRC_ROOT"
  echo
  echo "## Trees"
  find "$LEGACY" -mindepth 1 -maxdepth 1 -type d | sort | while read -r d; do
    n=$(find "$d" -type f | wc -l)
    sz=$(du -sh "$d" | cut -f1)
    echo "- $(basename "$d"): $n files, $sz"
  done
  echo
  echo "## File list"
  find "$LEGACY" -type f | sed "s|^$LEGACY/||" | sort
} > "$LEGACY/MANIFEST.txt"

# README
cat > "$LEGACY/README.md" << EOF
# Legacy development (pre–STRIX V2.2)

Historical code kept for reference. **Active development is STRIX V2.2**
(\`STRIX_V2/\` firmware + \`STRIX_V2_py/\` tuner).

| Path | Description |
|------|-------------|
| \`TorquEFI/\` | Early package: Arduino, first STM32 port, simple tuner, docs |
| \`TorquEFI/arduino_legacy/\` | Arduino \`ecu_firmware.ino\` + \`ecu_config.h\` |
| \`arduino/\` | Standalone Arduino firmware copy |
| \`TorquEFI_STM32F411/\` | Early CubeIDE snapshot |
| \`ecu_firmware_stm32/\` | Early STM32 Core notes/sources |
| \`archive_firmware_v1/\` | Archived firmware snapshot (if present) |
| \`archive_tuner_v1/\` | Archived tuner snapshot (if present) |

Do **not** flash these for new builds — use \`STRIX_V2/STRIX_V2\` and \`STRIX_V2_py\`.

Refresh this tree:

\`\`\`bash
./scripts/archive_legacy.sh /path/to/artifacts
./scripts/push_legacy.sh      # optional: commit + push
\`\`\`

Last assembled: $STAMP
EOF

echo "==> Done. Files: $(find "$LEGACY" -type f | wc -l)"
echo "    Manifest: legacy/MANIFEST.txt"
