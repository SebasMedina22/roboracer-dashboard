#!/usr/bin/env bash
# Guard: the professor's repository (rrstack) must never be modified by us.
# Exits 1 (and prints what changed) if it has local changes, 0 if it is clean.
#
# Usage: bash scripts/check_rrstack_clean.sh [path]
#        default path: $RRSTACK_DIR, else ~/rrstack
set -euo pipefail

DIR="${1:-${RRSTACK_DIR:-$HOME/rrstack}}"

if [ ! -d "$DIR/.git" ]; then
  echo "[guard] $DIR is not a git repository (set RRSTACK_DIR or pass the path)" >&2
  exit 2
fi

# Tracked changes and untracked files; build/ install/ log/ are ignored by rrstack's .gitignore
changes="$(git -C "$DIR" status --porcelain)"
if [ -n "$changes" ]; then
  echo "[guard] STOP: $DIR has local changes:" >&2
  echo "$changes" >&2
  echo "[guard] We never edit the professor's repo. Inspect with: git -C $DIR status / git -C $DIR diff" >&2
  exit 1
fi

echo "[guard] rrstack clean at $(git -C "$DIR" rev-parse --short HEAD)"
