#!/usr/bin/env bash
# Copy what scripts/car.sh saved on the Jetson to this laptop. Run it in WSL ON THE LAPTOP.
#
# Usage: bash scripts/fetch_car.sh [options]
#   (no option)      only the small files: info_*.txt, *.log, bags/*/bag_info.txt
#   --bag NAME       also the rosbag NAME (all days); repeatable
#   --all-bags       also every rosbag (can be many GB)
#   --host USER@IP   default chocapinto@192.168.3.2
#   -o DIR           destination (default: $AVAI_BAGS_DIR, else /mnt/c/Users/Asus/Documents/avai_bags)
#
# One ssh connection (asks the Jetson password once). The folder layout is kept:
#   DIR/<date>/info_*.txt, DIR/<date>/bags/<NAME>/...
set -uo pipefail

HOST="chocapinto@192.168.3.2"
DEST="${AVAI_BAGS_DIR:-/mnt/c/Users/Asus/Documents/avai_bags}"
REMOTE_DIR="avai_car"                 # relative to the Jetson home (car.sh default)
BAGS=(); ALL=false

usage() { sed -n '2,13p' "$0" | sed 's/^# \{0,1\}//'; exit "${1:-0}"; }
while [ $# -gt 0 ]; do
  case "$1" in
    --bag) [[ "${2:-}" =~ ^[A-Za-z0-9_-]+$ ]] || { echo "--bag needs a NAME" >&2; exit 1; }
           BAGS+=("$2"); shift 2 ;;
    --all-bags) ALL=true; shift ;;
    --host) HOST="$2"; shift 2 ;;
    -o) DEST="$2"; shift 2 ;;
    -h|--help) usage 0 ;;
    *) echo "unknown option: $1" >&2; usage 1 ;;
  esac
done

mkdir -p "$DEST" || { echo "cannot create $DEST" >&2; exit 1; }

# tar on the Jetson decides what to send; everything travels in one ssh connection.
# Bags are excluded unless asked for; their bag_info.txt is always included.
if $ALL; then
  SELECT="find . -mindepth 1 -maxdepth 1 -type d"
else
  SELECT="{ find . -type f \\( -name 'info_*.txt' -o -name '*.log' -o -name bag_info.txt \\)"
  for b in "${BAGS[@]}"; do SELECT="$SELECT; find . -path './*/bags/$b' -type d"; done
  SELECT="$SELECT; }"
fi
REMOTE="cd ~/$REMOTE_DIR 2>/dev/null || { echo 'no ~/$REMOTE_DIR on the Jetson: run car.sh there first' >&2; exit 3; }; \
$SELECT | tar cf - -T -"

if $ALL; then which="all"; elif [ ${#BAGS[@]} -gt 0 ]; then which="${BAGS[*]}"; else which="none"; fi
echo "[fetch] $HOST:~/$REMOTE_DIR -> $DEST  (bags: $which)"
# shellcheck disable=SC2029  # REMOTE is built here on purpose and runs on the Jetson
ssh "$HOST" "$REMOTE" | tar xf - -C "$DEST"
status=("${PIPESTATUS[@]}")
if [ "${status[0]}" -ne 0 ] || [ "${status[1]}" -ne 0 ]; then
  echo "[fetch] ERROR: copy failed (ssh ${status[0]}, tar ${status[1]})" >&2
  exit 1
fi
echo "[fetch] done:"
(cd "$DEST" && find . -maxdepth 3 -mindepth 1 | sort | sed 's/^/[fetch]   /')
du -sh "$DEST" | sed 's/^/[fetch] total /'
