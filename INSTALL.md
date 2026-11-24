# Install & run

Everything runs on **Ubuntu 22.04**, because ROS 2 Humble only exists for 22.04. Native Ubuntu, WSL2 on Windows, or a distrobox on a newer Ubuntu all work.

## 1. Install (one command)

```bash
git clone https://github.com/SebasMedina22/roboracer-dashboard.git ~/dashboard_ws
cd ~/dashboard_ws
bash scripts/install.sh
```

Any clone location works; the `rd` shortcut points to wherever you ran the script from.

It asks for your `sudo` password and takes 10–30 min the first time (ROS + PyTorch downloads). It is safe to run again: steps that are already done are skipped.

| Step | What it does |
|---|---|
| 1 | ROS 2 Humble (`ros-humble-desktop`) |
| 2 | Gazebo Fortress, `ros_gz`, `slam_toolbox`, `rosbridge`, `web_video_server`, `colcon`, `rosdep` |
| 3 | YOLO (`ultralytics`). PyTorch with CUDA if `nvidia-smi` works, otherwise CPU-only. Keeps `numpy<2` (numpy 2 breaks `cv_bridge` on Humble) |
| 4 | Clones [isselab/rrstack](https://github.com/isselab/rrstack) to `~/rrstack` (or pulls if the clone is clean), runs `rosdep`, builds it |
| 5 | Builds this dashboard |
| 6 | Adds two shortcuts to `~/.bashrc`: **`rr`** (rrstack) and **`rd`** (dashboard). On WSL they also set `LIBGL_ALWAYS_SOFTWARE=1` |

When it finishes, **open a new terminal** so the shortcuts exist.

## 2. Run

One terminal per line. Type the shortcut first, on its own line.

| Terminal | Commands | What you get |
|---|---|---|
| T1 | `rr` then `ros2 launch roboracer_planning cone_drive.launch.py` | Gazebo with cones + YOLO + Delaunay + pure pursuit. The car drives by itself |
| T2 | `rd` then `ros2 launch roboracer_dashboard dashboard.launch.py` | rosbridge (:9090), camera stream (:8080), page (:8000) |
| Browser | <http://localhost:8000> | The dashboard |

Other scenarios (T1 only, all after `rr`):

```bash
ros2 launch roboracer_gazebo gazebo.launch.py world:=cones        # just the world; drive with teleop below
ros2 run teleop_twist_keyboard teleop_twist_keyboard               # keyboard driving (never together with an autonomous controller)
ros2 launch roboracer_wallfollowing wall_follow.launch.py          # wall following
ros2 launch roboracer_slam sim_slam_rviz.launch.py                 # SLAM map in RViz (+ /map for the dashboard)
```

Only **one** launch that opens Gazebo at a time.

## 3. Newer Ubuntu (24.04, 26.04, …): use distrobox

```bash
sudo apt install -y distrobox podman
distrobox create --name ros-humble --image ubuntu:22.04 --nvidia   # drop --nvidia without an NVIDIA GPU
distrobox enter ros-humble
# now you are in Ubuntu 22.04 -> follow section 1 from here
```

Your home folder is shared and GUI windows open on your desktop. Run `distrobox enter ros-humble` in **every** new terminal before `rr` / `rd`.

## 4. Troubleshooting

| Symptom | Fix |
|---|---|
| `Unable to locate package ros-humble-desktop` | Not Ubuntu 22.04 → section 3 |
| `ros2: command not found` / `Package 'roboracer_…' not found` | Run `rr` (or `rd`) in that terminal |
| `ModuleNotFoundError: No module named 'ultralytics'` | `pip3 install ultralytics "numpy<2"` |
| `pip3: command not found` | `sudo apt install -y python3-pip` |
| `colcon build` fails with `canonicalize_version() got an unexpected keyword argument 'strip_trailing_zero'` | A newer setuptools in `~/.local` shadows Ubuntu's: `pip3 uninstall -y setuptools`, then build again (the script does this) |
| `rosdep`: `Cannot locate rosdep definition for [ament_python]` | Harmless warning, the install continues |
| Gazebo closes or is black (WSL) | The shortcuts set `LIBGL_ALWAYS_SOFTWARE=1`; check you used `rr` |
| Car jumps around, `Moved backwards in time` | Two Gazebo launches running → `pkill -f "ign gazebo"; pkill -f ros2; ros2 daemon stop` |
| Dashboard map panel says "waiting for /map" | Normal: only the SLAM launch publishes `/map` |
| Gazebo texture error `RealSense_Albedo.png` | Cosmetic, ignore |

## Project rules

- Never edit files inside `~/rrstack` (only `git pull` there). Change parameters with our own YAML or launch arguments.
- Never run teleop and an autonomous controller at the same time (both publish `/cmd_vel`).
- Real car: low speed first, someone holding the joypad ready to stop it.
