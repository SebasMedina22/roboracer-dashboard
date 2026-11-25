#!/usr/bin/env bash
# Helper for the REAL CAR. Run it ON THE JETSON (after: ssh chocapinto@192.168.3.2).
# Finds the real topic names by itself, so nobody has to type them.
#
# Usage: bash scripts/car.sh <command> [args]
#   info                  save topics, types, rates, nodes and free disk to the session folder;
#                         prints the detected camera topics
#   dashboard             start the dashboard and print the URL to open on the laptop
#   record NAME [SEC]     record a rosbag NAME (stops after SEC seconds, or with ONE Ctrl+C)
#   yolo                  level B: only YOLO on the real camera (nothing publishes /drive)
#   drive [MAX_SPEED]     level C: the course's autonomous pipeline (default 0.5 m/s)
#   help                  this text
#
# Everything is saved in  $AVAI_CAR_DIR/<date>/  (default ~/avai_car/<date>/):
#   info_<time>.txt, <command>_<time>.log, bags/<NAME>/ (+ bags/<NAME>/bag_info.txt)
# Copy it to the laptop with scripts/fetch_car.sh (run in WSL on the laptop).
#
# Overrides (only if the detection is wrong): CAMERA_PREFIX=/camera/xyz bash scripts/car.sh ...
#   RRSTACK_SETUP (default ~/rrstack/ros_ws/install/setup.bash),
#   DASHBOARD_SETUP (default <this repo>/install/setup.bash)
set -uo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SESSION="${AVAI_CAR_DIR:-$HOME/avai_car}/$(date +%Y%m%d)"
STAMP="$(date +%H%M%S)"
RRSTACK_SETUP="${RRSTACK_SETUP:-$HOME/rrstack/ros_ws/install/setup.bash}"
DASHBOARD_SETUP="${DASHBOARD_SETUP:-$REPO_DIR/install/setup.bash}"
MIN_FREE_GB=5

say()  { echo "[car] $*"; }
warn() { echo "[car] WARNING: $*" >&2; }
die()  { echo "[car] ERROR: $*" >&2; exit 1; }
usage() { sed -n '2,21p' "$0" | sed 's/^# \{0,1\}//'; exit "${1:-0}"; }

# ROS 2 in this shell (ssh sessions often have nothing sourced)
if ! command -v ros2 > /dev/null 2>&1; then
  # shellcheck disable=SC1091
  [ -f /opt/ros/humble/setup.bash ] && source /opt/ros/humble/setup.bash
fi
command -v ros2 > /dev/null 2>&1 || die "ros2 not found (is ROS 2 Humble installed on this machine?)"

mkdir -p "$SESSION" || die "cannot create $SESSION"

# ---------------------------------------------------------------- topic detection
TOPICS_T=""                      # "name [type]" lines, filled once
load_topics() {
  [ -n "$TOPICS_T" ] && return
  TOPICS_T="$(timeout 15 ros2 topic list -t 2> /dev/null)"
  [ -n "$TOPICS_T" ] || die "no topics: is the car's base (drivers) running? (ros2 topic list is empty)"
}
topic_names() { load_topics; echo "$TOPICS_T" | awk '{print $1}'; }
has_topic()   { topic_names | grep -qx -- "$1"; }
# Number of publishers. A topic can be listed only because someone SUBSCRIBES to it
# (e.g. the dashboard): then nothing is actually sent on it.
publishers()  { timeout 10 ros2 topic info "$1" 2> /dev/null | awk '/Publisher count/ {print $3}'; }

# Messages per second over 4 s, or 0 if nothing arrived.
rate_of() {
  local r
  r="$(timeout -s INT 4 ros2 topic hz "$1" 2>&1 | awk '/average rate/ {v=$3} END {print v}')"
  echo "${r:-0}"
}

# Camera prefix: the part before /color/image_raw (RealSense: /camera/realsense2_camera,
# /camera/camera, ... ; simulator: /camera). CAMERA_PREFIX overrides.
camera_prefix() {
  if [ -n "${CAMERA_PREFIX:-}" ]; then echo "${CAMERA_PREFIX%/}"; return; fi
  local t
  t="$(topic_names | grep -E '/color/image_raw$' | grep -v -i depth | head -n 1)"
  [ -n "$t" ] && echo "${t%/color/image_raw}"
}

# Depth image that matches the colour image pixel by pixel if possible.
depth_topic() {
  local p="$1" t
  for t in "$p/aligned_depth_to_color/image_raw" "$p/depth/image_rect_raw" "$p/depth/image_raw"; do
    has_topic "$t" && { echo "$t"; return; }
  done
  topic_names | grep -E "^$p/.*depth.*image" | head -n 1
}

detect() {
  CAM="$(camera_prefix)"
  if [ -z "$CAM" ]; then
    warn "no '<prefix>/color/image_raw' topic: camera driver not running?"
    COLOR=""; INFO=""; DEPTH=""
    return
  fi
  COLOR="$CAM/color/image_raw"
  INFO="$CAM/color/camera_info"
  DEPTH="$(depth_topic "$CAM")"
  has_topic "$INFO" || warn "no $INFO"
  [ -n "$DEPTH" ] || warn "no depth image under $CAM (YOLO needs it for distances)"
}

topic_state() {   # "yes (N publishers)" / "listed, but NO publisher" / "no"
  local n
  if ! has_topic "$1"; then echo "no"; return; fi
  n="$(publishers "$1")"
  if [ "${n:-0}" -gt 0 ]; then echo "yes ($n publisher$([ "$n" -gt 1 ] && echo s))"
  else echo "listed, but NO publisher (only subscribers)"; fi
}

print_detected() {
  say "camera prefix : ${CAM:-NOT FOUND}"
  say "colour image  : ${COLOR:--}"
  say "camera info   : ${INFO:--}"
  say "depth image   : ${DEPTH:--}"
  local t
  for t in /scan /odom /drive /joy /cones/blue /cones/yellow /waypoints; do
    say "$(printf '%-14s: %s' "$t" "$(topic_state "$t")")"
  done
}

free_gb() { df -Pk "$SESSION" | awk 'NR==2 {printf "%d", $4 / 1048576}'; }

source_rrstack() {
  [ -f "$RRSTACK_SETUP" ] || die "rrstack not found at $RRSTACK_SETUP (ask the supervisors where it is; then RRSTACK_SETUP=/path/to/install/setup.bash bash scripts/car.sh ...)"
  # shellcheck disable=SC1090
  source "$RRSTACK_SETUP"
}

# ---------------------------------------------------------------- commands
cmd_info() {
  local out="$SESSION/info_$STAMP.txt" t r detected rates="" nodata=0
  load_topics
  detect
  detected="$(print_detected 2>&1)"
  echo "$detected"
  say "measuring data rates (4 s per topic) ..."
  for t in /scan /odom "$COLOR" "$DEPTH"; do
    if [ -z "$t" ] || ! has_topic "$t"; then continue; fi
    r="$(rate_of "$t")"
    if [ "$r" = "0" ]; then
      rates+="$(printf '[car] %-40s NO DATA in 4 s' "$t")"$'\n'; nodata=$((nodata + 1))
    else
      rates+="$(printf '[car] %-40s %s Hz' "$t" "$r")"$'\n'
    fi
  done
  printf '%s' "$rates"
  say "free disk     : $(free_gb) GB"
  if [ "$nodata" -gt 0 ]; then
    warn "$nodata sensor topic(s) send NO DATA: drivers/simulation not running yet? Recording now would be empty."
  fi
  {
    echo "# car info $(date -Iseconds) on $(hostname)"
    echo; echo "## detected"; echo "$detected"
    echo; echo "## rates (4 s each)"; printf '%s' "$rates"
    echo; echo "## free disk"; df -h "$SESSION"
    echo; echo "## nodes"; timeout 15 ros2 node list 2>&1
    echo; echo "## topics (name [type])"; echo "$TOPICS_T"
  } > "$out"
  say "saved -> $out"
}

cmd_dashboard() {
  [ -f "$DASHBOARD_SETUP" ] || die "dashboard not built: cd $REPO_DIR && colcon build --symlink-install --packages-select roboracer_dashboard"
  # shellcheck disable=SC1090
  source "$DASHBOARD_SETUP"
  local ip
  ip="$(hostname -I 2> /dev/null | awk '{print $1}')"
  say "open on the laptop:  http://${ip:-<jetson-ip>}:8000/?car"
  say "(panels pick the real topic names by themselves; Ctrl+C here stops the dashboard)"
  ros2 launch roboracer_dashboard dashboard.launch.py 2>&1 | tee "$SESSION/dashboard_$STAMP.log"
}

cmd_record() {
  local name="${1:-}" secs="${2:-}" dir t
  [ -n "$name" ] || die "usage: car.sh record NAME [SECONDS]"
  [[ "$name" =~ ^[A-Za-z0-9_-]+$ ]] || die "NAME: only letters, digits, _ and -"
  [ -z "$secs" ] || [[ "$secs" =~ ^[0-9]+$ ]] || die "SECONDS must be a whole number"
  dir="$SESSION/bags/$name"
  [ -e "$dir" ] && die "$dir already exists: choose another NAME"
  mkdir -p "$SESSION/bags"

  load_topics
  detect
  local topics=()
  for t in /scan /odom /tf /tf_static /drive /joy "$COLOR" "$INFO" "$DEPTH" \
           /cones/blue /cones/yellow /waypoints; do
    [ -n "$t" ] || continue
    if has_topic "$t"; then topics+=("$t"); else
      case "$t" in /joy|/drive|/cones/*|/waypoints) ;; *) warn "not recording $t (topic does not exist)" ;; esac
    fi
  done
  [ ${#topics[@]} -gt 0 ] || die "none of the topics to record exist"

  local free
  free="$(free_gb)"
  [ "$free" -ge "$MIN_FREE_GB" ] || die "only $free GB free on the Jetson (need >= $MIN_FREE_GB): delete old bags first"
  say "recording ${#topics[@]} topics -> $dir   (free: $free GB; images take a few GB per minute)"
  printf '[car]   %s\n' "${topics[@]}"
  if [ -n "$secs" ]; then say "stops by itself after $secs s (or ONE Ctrl+C)"; else say "stop with ONE Ctrl+C"; fi

  # Ctrl+C reaches ros2 bag record, which closes the bag cleanly; the script carries on.
  trap 'echo; say "stopping ..."' INT
  if [ -n "$secs" ]; then
    timeout --foreground -s INT "$secs" ros2 bag record -o "$dir" "${topics[@]}"
  else
    ros2 bag record -o "$dir" "${topics[@]}"
  fi
  trap - INT
  sleep 1
  if [ -d "$dir" ]; then
    ros2 bag info "$dir" > "$dir/bag_info.txt" 2>&1
    grep -E 'Duration|Messages|Bag size' "$dir/bag_info.txt" | sed 's/^/[car]   /'
    say "saved -> $dir ($(du -sh "$dir" | cut -f1))"
    # sensor topics that got nothing: the bag is useless for them
    local empty
    empty="$(grep -oE 'Topic: [^ ]+ \| Type: [^ ]+ \| Count: 0\b' "$dir/bag_info.txt" \
             | awk '{print $2}' | grep -vE '^/(tf_static|drive|joy|cones/.*|waypoints)$')"
    if [ -n "$empty" ]; then
      warn "NO messages recorded on: $(echo "$empty" | tr '\n' ' ')"
      warn "were the sensors publishing? check with: bash scripts/car.sh info"
    fi
  else
    warn "no bag was written"
  fi
}

cmd_yolo() {
  load_topics
  detect
  if [ -z "$COLOR" ] || [ -z "$DEPTH" ] || ! has_topic "$INFO"; then
    die "camera topics incomplete (see warnings): is the camera driver running?"
  fi
  source_rrstack
  say "YOLO only (level B): $COLOR + $DEPTH. Nothing is sent to the motor."
  say "check: dashboard 'LiDAR + cones' panel and Logs; Ctrl+C to stop"
  ros2 run roboracer_perception yolo_node --ros-args -r __node:=yolo_cone_debug_node \
    -p image_topic:="$COLOR" -p depth_topic:="$DEPTH" -p camera_info_topic:="$INFO" \
    -p use_sim_time:=false 2>&1 | tee "$SESSION/yolo_$STAMP.log"
}

cmd_drive() {
  local speed="${1:-0.5}" ok
  [[ "$speed" =~ ^[0-9]+(\.[0-9]+)?$ ]] || die "MAX_SPEED must be a number like 0.5"
  [[ "$speed" == *.* ]] || speed="$speed.0"           # the launch needs the decimal point
  awk -v s="$speed" 'BEGIN { exit !(s <= 1.5) }' || die "MAX_SPEED $speed > 1.5 m/s: not allowed"
  load_topics
  detect
  [ -n "$CAM" ] || die "no camera topics: is the camera driver running?"
  # the dashboard subscribes to /cones/blue, so only a PUBLISHER means another YOLO is running
  if has_topic /cones/blue && [ "$(publishers /cones/blue)" -gt 0 ] 2> /dev/null; then
    warn "something already publishes /cones/blue: is 'car.sh yolo' still running? Stop it first (the launch starts its own YOLO)."
  fi
  source_rrstack
  echo "[car] AUTONOMOUS DRIVING, max $speed m/s, camera $CAM"
  echo "[car]   - supervisors agreed?  - track clear?  - someone holding the joypad, ready to release?"
  read -r -p "[car] type yes to start: " ok
  [ "$ok" = "yes" ] || die "not started"
  ros2 launch roboracer_planning cone_drive_car.launch.py camera_prefix:="$CAM" max_speed:="$speed" \
    2>&1 | tee "$SESSION/drive_$STAMP.log"
}

case "${1:-help}" in
  info) cmd_info ;;
  dashboard) cmd_dashboard ;;
  record) shift; cmd_record "$@" ;;
  yolo) cmd_yolo ;;
  drive) shift; cmd_drive "$@" ;;
  help|-h|--help) usage 0 ;;
  *) echo "unknown command: $1" >&2; usage 1 ;;
esac
