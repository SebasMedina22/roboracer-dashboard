#!/usr/bin/env bash
# Evaluated simulation runs of the course's cone pipeline, one after another.
# Each run: guard -> eval.launch.py (Gazebo headless + race_monitor) -> results -> plot -> guard.
#
# Usage: bash scripts/run_eval.sh [options]
#   -n N            number of runs (default 1)
#   -l LAPS         laps per run (default 3)
#   --rtf X         simulation speed limit: 1 = real time (world default), 2 = twice, ...
#                   "max" = no practical limit. Only matters if the PC can go faster than 1x.
#   --max-wall MIN  real minutes per run before it is stopped (default 10)
#   --keep-going    do not stop a run at the first cone contact
#   --gui           show the Gazebo window (to watch a run)
#   -o DIR          results folder (default: $AVAI_RUNS_DIR, else ~/avai_runs)
#
# Results: DIR/<date_time>_runK/{summary.json,trajectory.csv,events.csv,run.png,launch.log}
#          DIR/batch_<date_time>.csv  (one line per run)
set -uo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
RRSTACK_DIR="${RRSTACK_DIR:-$HOME/rrstack}"

RUNS=1; LAPS=3; RTF=1; MAX_WALL=10; STOP_ON_CONTACT=true; GUI=false
OUT="${AVAI_RUNS_DIR:-$HOME/avai_runs}"

usage() { sed -n '2,16p' "$0" | sed 's/^# \{0,1\}//'; exit "${1:-0}"; }
while [ $# -gt 0 ]; do
  case "$1" in
    -n) RUNS="$2"; shift 2 ;;
    -l) LAPS="$2"; shift 2 ;;
    --rtf) RTF="$2"; shift 2 ;;
    --max-wall) MAX_WALL="$2"; shift 2 ;;
    --keep-going) STOP_ON_CONTACT=false; shift ;;
    --gui) GUI=true; shift ;;
    -o) OUT="$2"; shift 2 ;;
    -h|--help) usage 0 ;;
    *) echo "unknown option: $1" >&2; usage 1 ;;
  esac
done
[ "$RTF" = "max" ] && RTF=100
for v in "$RUNS" "$LAPS" "$MAX_WALL"; do
  [[ "$v" =~ ^[0-9]+$ ]] || { echo "-n, -l and --max-wall need whole numbers (got '$v')" >&2; exit 1; }
done
[[ "$RTF" =~ ^[0-9]+([.][0-9]+)?$ ]] || { echo "--rtf needs a number or 'max' (got '$RTF')" >&2; exit 1; }

log() { printf '\033[1;34m[eval]\033[0m %s\n' "$*"; }
warn() { printf '\033[1;33m[eval]\033[0m %s\n' "$*"; }

# ---------------------------------------------------------------- environment (same as rr / rd)
# shellcheck disable=SC1090
ros_source() { set +u; source "$1"; set -u; }      # ROS setup files break under set -u
ros_source /opt/ros/humble/setup.bash
ros_source "$RRSTACK_DIR/ros_ws/install/setup.bash"
ros_source "$REPO_DIR/install/setup.bash"
grep -qi microsoft /proc/version && export LIBGL_ALWAYS_SOFTWARE=1
if [ -n "${CONTAINER_ID:-}" ] || [ -f /run/.containerenv ]; then
  export LIBGL_ALWAYS_SOFTWARE=1 LIBGL_DRI3_DISABLE=1 QT_QPA_PLATFORM=xcb
fi
export IGN_IP=127.0.0.1 GZ_IP=127.0.0.1

ros2 pkg executables chocapinto_eval 2>/dev/null | grep -q race_monitor || {
  echo "chocapinto_eval is not built: cd $REPO_DIR && colcon build --symlink-install" >&2; exit 1; }

WORLD_FILE="$(ros2 pkg prefix roboracer_gazebo)/share/roboracer_gazebo/worlds/flw_cone_track_from_walls.world"
WORLD_NAME="$(sed -n 's/.*<world[[:space:]]\+name="\([^"]*\)".*/\1/p' "$WORLD_FILE" | head -1)"
[ -n "$WORLD_NAME" ] || { echo "could not read the world name from $WORLD_FILE" >&2; exit 1; }

mkdir -p "$OUT"
BATCH="$(date +%Y%m%d_%H%M%S)"

SIM_PATTERN="ign gazebo|gz sim|ros2 launch chocapinto_eval|ros2 launch roboracer_planning"

cleanup_sim() {   # leftovers from a previous or interrupted run
  pkill -f "$SIM_PATTERN" 2>/dev/null
  local _
  for _ in 1 2 3 4 5 6 7 8 9 10; do                     # up to 5 s to exit politely
    pgrep -f "$SIM_PATTERN" > /dev/null || return 0
    sleep 0.5
  done
  # A hung Gazebo ignores SIGTERM; a second Gazebo on the same world then never starts.
  warn "simulation processes still alive after 5 s, forcing them to stop:"
  pgrep -af "$SIM_PATTERN" | sed 's/^/[eval]   /' >&2
  pkill -9 -f "$SIM_PATTERN" 2>/dev/null
  sleep 1
  if pgrep -f "$SIM_PATTERN" > /dev/null; then
    warn "could not stop them (kill -9 them by hand, or restart WSL: wsl --shutdown in PowerShell)"
    return 1
  fi
}

set_rtf_when_ready() {   # background: wait for Gazebo, then change its speed limit
  local _
  for _ in $(seq 1 120); do
    if ign service -l 2>/dev/null | grep -q "/world/$WORLD_NAME/set_physics"; then
      if ign service -s "/world/$WORLD_NAME/set_physics" --reqtype ignition.msgs.Physics \
           --reptype ignition.msgs.Boolean --timeout 3000 \
           --req "max_step_size: 0.001, real_time_factor: $RTF" >/dev/null 2>&1; then
        echo "[eval] simulation speed limit set to ${RTF}x"
      else
        echo "[eval] could not set the speed limit (run continues at 1x)"
      fi
      return
    fi
    sleep 1
  done
  echo "[eval] Gazebo did not offer set_physics within 120 s (run continues at 1x)"
}

# Ctrl+C: ros2 launch gets it too and race_monitor saves what it measured. Bash runs this
# trap only after the launch has finished, so the results are written before we stop.
INTERRUPTED=0
trap 'INTERRUPTED=1' INT

log "results -> $OUT   ($RUNS run(s), $LAPS lap(s) each, max ${MAX_WALL} min, speed limit ${RTF}x)"
RUN_DIRS=()
for k in $(seq 1 "$RUNS"); do
  RUN="${BATCH}_run${k}"
  RUN_DIR="$OUT/$RUN"
  echo
  log "=== run $k/$RUNS: $RUN ==="
  bash "$REPO_DIR/scripts/check_rrstack_clean.sh" || { warn "guard failed: stopping"; exit 1; }
  cleanup_sim || { warn "a previous simulation is still running: not starting $RUN"; exit 1; }
  mkdir -p "$RUN_DIR"

  RTF_PID=""
  if [ "$RTF" != "1" ]; then
    set_rtf_when_ready & RTF_PID=$!
  fi

  # Safety net: the monitor stops itself at MAX_WALL; this only fires if something hangs.
  timeout --foreground -s INT -k 30 $(( MAX_WALL * 60 + 180 )) \
    ros2 launch chocapinto_eval eval.launch.py \
      laps:="$LAPS" gui:="$GUI" rviz:=false stop_on_contact:="$STOP_ON_CONTACT" \
      max_wall_time:="$(( MAX_WALL * 60 )).0" out_dir:="$OUT" run_name:="$RUN" \
    2>&1 | tee "$RUN_DIR/launch.log" \
         | grep --line-buffered -E "race_monitor\]|\[stage|\[eval\]|ERROR|process has died"

  [ -n "$RTF_PID" ] && kill "$RTF_PID" 2>/dev/null
  cleanup_sim
  bash "$REPO_DIR/scripts/check_rrstack_clean.sh" || { warn "guard failed after the run: stopping"; exit 1; }

  if [ -f "$RUN_DIR/summary.json" ]; then
    python3 "$REPO_DIR/scripts/plot_run.py" "$RUN_DIR" || warn "plot failed (see above)"
    RUN_DIRS+=("$RUN_DIR")
  else
    warn "no summary.json for $RUN: see $RUN_DIR/launch.log"
  fi
  if [ "$INTERRUPTED" = 1 ]; then
    warn "stopped by user after run $k"
    break
  fi
done

[ ${#RUN_DIRS[@]} -gt 0 ] && python3 - "$OUT/batch_${BATCH}.csv" "${RUN_DIRS[@]}" <<'EOF'
import csv, json, os, sys
out, dirs = sys.argv[1], sys.argv[2:]
cols = ['run', 'end_reason', 'laps_completed', 'sim_time', 'avg_speed', 'contacts',
        'min_clearance', 'odom_drift_xy', 'rtf_avg', 'lap_times']
rows = []
for d in dirs:
    s = json.load(open(os.path.join(d, 'summary.json')))
    s['run'] = os.path.basename(d)
    s['lap_times'] = ' '.join(f"{lap['time']:.1f}" for lap in s['laps'])
    rows.append({c: s.get(c) for c in cols})
with open(out, 'w', newline='') as f:
    w = csv.DictWriter(f, fieldnames=cols); w.writeheader(); w.writerows(rows)
print('\n' + ' | '.join(cols))
for r in rows:
    print(' | '.join('' if r[c] is None else str(r[c]) for c in cols))
print(f'\n[eval] batch table -> {out}')
EOF
