#!/usr/bin/env bash
# push_legacy.sh — commit and push legacy/ (and scripts/) to origin
# Env:
#   GH_TOKEN or GITHUB_TOKEN  — PAT with repo scope (required for HTTPS push)
#   LEGACY_MSG                — optional commit message override
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
cd "$REPO_ROOT"

if [[ ! -d .git ]]; then
  echo "ERROR: not a git repo: $REPO_ROOT" >&2
  exit 1
fi

if [[ ! -d legacy ]]; then
  echo "ERROR: legacy/ missing — run scripts/archive_legacy.sh first" >&2
  exit 1
fi

TOKEN="${GH_TOKEN:-${GITHUB_TOKEN:-}}"
REMOTE_URL="$(git remote get-url origin 2>/dev/null || true)"
if [[ -z "$REMOTE_URL" ]]; then
  REMOTE_URL="https://github.com/boostdoctor/STRIX-V2.2.git"
  git remote add origin "$REMOTE_URL" 2>/dev/null || git remote set-url origin "$REMOTE_URL"
fi

git add legacy scripts
# Root README mention
if [[ -f README.md ]] && ! grep -q 'legacy/' README.md 2>/dev/null; then
  cat >> README.md << 'EOF'

## Legacy (archived)

Pre–STRIX V2.2 development (TorquEFI, Arduino sketch, early STM32) lives under [`legacy/`](legacy/).
Refresh with `./scripts/archive_legacy.sh` then `./scripts/push_legacy.sh`.
EOF
  git add README.md
fi

if git diff --cached --quiet; then
  echo "Nothing new to commit under legacy/ or scripts/."
  exit 0
fi

MSG="${LEGACY_MSG:-Update legacy archive ($(date -u +%Y-%m-%d))}"
git -c user.email="${GIT_EMAIL:-strix@boostdoctor.local}" \
    -c user.name="${GIT_NAME:-STRIX V2}" \
    commit -m "$MSG"

BRANCH="$(git rev-parse --abbrev-ref HEAD)"
echo "==> Pushing $BRANCH → origin"

if [[ -n "$TOKEN" ]]; then
  # HTTPS with PAT
  PUSH_URL="https://x-access-token:${TOKEN}@github.com/boostdoctor/STRIX-V2.2.git"
  git push "$PUSH_URL" "HEAD:refs/heads/$BRANCH"
else
  echo "Note: GH_TOKEN not set — using default remote credentials"
  git push origin "HEAD:refs/heads/$BRANCH"
fi

echo "==> Pushed $(git rev-parse --short HEAD)"
