#!/usr/bin/env bash
# One-shot setup for the AVAI Lab stack (isselab/rrstack) + our dashboard.
# Target: Ubuntu 22.04 (native, WSL2, or a distrobox on a newer Ubuntu).
#
# Usage (from inside this cloned repo):
#   bash scripts/install.sh
#
# Safe to run again: every step skips what is already done.
# Override the rrstack location with: RRSTACK_DIR=/some/path bash scripts/install.sh
set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
RRSTACK_DIR="${RRSTACK_DIR:-$HOME/rrstack}"

step() { printf '\n\033[1;34m==> %s\033[0m\n' "$*"; }
warn() { printf '\033[1;33m[!] %s\033[0m\n' "$*"; }
ros_source() { set +u; source "$1"; set -u; }   # ROS setup files break under `set -u`

# --- 0. Checks ---------------------------------------------------------------
. /etc/os-release
if [ "${VERSION_CODENAME:-}" != "jammy" ]; then
  echo "ROS 2 Humble needs Ubuntu 22.04 (jammy). Found: ${PRETTY_NAME:-unknown}." >&2
  echo "On a newer Ubuntu, run this inside a distrobox (see INSTALL.md)." >&2
  exit 1
fi
IS_WSL=false
grep -qi microsoft /proc/version && IS_WSL=true
HAS_NVIDIA=false
command -v nvidia-smi >/dev/null 2>&1 && nvidia-smi >/dev/null 2>&1 && HAS_NVIDIA=true
echo "repo: $REPO_DIR | rrstack: $RRSTACK_DIR | WSL: $IS_WSL | NVIDIA GPU: $HAS_NVIDIA"

# --- 1. ROS 2 Humble ---------------------------------------------------------
step "1/6 ROS 2 Humble"
if [ -f /opt/ros/humble/setup.bash ]; then
  echo "already installed"
else
  sudo apt-get update
  sudo apt-get install -y software-properties-common curl
  sudo add-apt-repository -y universe
  ver=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
        | grep -F '"tag_name"' | awk -F\" '{print $4}')
  [ -n "$ver" ] || { echo "Could not read the ros-apt-source version from GitHub." >&2; exit 1; }
  curl -fL -o /tmp/ros2-apt-source.deb \
    "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ver}/ros2-apt-source_${ver}.${VERSION_CODENAME}_all.deb"
  sudo dpkg -i /tmp/ros2-apt-source.deb
  sudo apt-get update
  sudo apt-get install -y ros-humble-desktop
fi

# --- 2. Gazebo Fortress + ROS packages ---------------------------------------
step "2/6 Gazebo Fortress + ROS packages"
sudo apt-get install -y \
  ros-humble-ros-gz ros-humble-robot-state-publisher ros-humble-xacro \
  ros-humble-ackermann-msgs ros-humble-cv-bridge ros-humble-slam-toolbox \
  ros-humble-nav2-map-server ros-humble-teleop-twist-keyboard \
  ros-humble-rosbridge-suite ros-humble-web-video-server \
  python3-colcon-common-extensions python3-rosdep python3-pip git

# --- 3. Python: YOLO ---------------------------------------------------------
step "3/6 Python packages (YOLO)"
if $HAS_NVIDIA; then
  echo "NVIDIA GPU found -> PyTorch with CUDA"
  pip3 install ultralytics "numpy<2"
else
  echo "No NVIDIA GPU -> CPU-only PyTorch (much smaller download)"
  pip3 install torch torchvision --index-url https://download.pytorch.org/whl/cpu
  pip3 install ultralytics "numpy<2"
fi
# numpy 2 breaks cv_bridge on Humble; keep it below 2 even if something upgraded it
pip3 install "numpy<2"

# --- 4. rrstack (professor's repo: we only clone/pull, never edit) -----------
step "4/6 rrstack"
if [ -d "$RRSTACK_DIR/.git" ]; then
  if [ -z "$(git -C "$RRSTACK_DIR" status --porcelain --untracked-files=no)" ]; then
    git -C "$RRSTACK_DIR" pull --ff-only
  else
    warn "rrstack has local changes -> not pulling. Run 'git -C $RRSTACK_DIR status' to check."
  fi
else
  git clone https://github.com/isselab/rrstack.git "$RRSTACK_DIR"
fi
ros_source /opt/ros/humble/setup.bash
[ -f /etc/ros/rosdep/sources.list.d/20-default.list ] || sudo rosdep init
rosdep update
( cd "$RRSTACK_DIR/ros_ws" \
  && rosdep install --from-paths src --ignore-src -r -y \
  && colcon build --symlink-install )

# --- 5. Our dashboard --------------------------------------------------------
step "5/6 Dashboard"
ros_source "$RRSTACK_DIR/ros_ws/install/setup.bash"
if [ -f "$REPO_DIR/package.xml" ]; then
  # This repo IS the package -> build it into ~/dashboard_ws
  DASH_WS="$HOME/dashboard_ws"
  mkdir -p "$DASH_WS"
  ( cd "$DASH_WS" && colcon build --symlink-install --base-paths "$REPO_DIR" )
else
  # This repo is a workspace (packages under src/) -> build in place
  DASH_WS="$REPO_DIR"
  ( cd "$DASH_WS" && colcon build --symlink-install )
fi

# --- 6. Shortcuts rr / rd ----------------------------------------------------
step "6/6 Shortcuts rr / rd in ~/.bashrc"
GL=""
$IS_WSL && GL=" && export LIBGL_ALWAYS_SOFTWARE=1"   # WSL: render Gazebo/RViz on the CPU
sed -i '/# >>> avai-lab >>>/,/# <<< avai-lab <<</d' ~/.bashrc
cat >> ~/.bashrc <<EOF
# >>> avai-lab >>>
alias rr='cd $RRSTACK_DIR/ros_ws && source /opt/ros/humble/setup.bash && source install/setup.bash$GL'
alias rd='cd $DASH_WS && source /opt/ros/humble/setup.bash && source $RRSTACK_DIR/ros_ws/install/setup.bash && source install/setup.bash$GL'
# <<< avai-lab <<<
EOF

python3 -c "import ultralytics, numpy; print('ultralytics', ultralytics.__version__, '| numpy', numpy.__version__)"
step "Done. Open a NEW terminal, then:"
cat <<'EOF'
  T1:  rr  ->  ros2 launch roboracer_planning cone_drive.launch.py
  T2:  rd  ->  ros2 launch roboracer_dashboard dashboard.launch.py
  Browser: http://localhost:8000
EOF
