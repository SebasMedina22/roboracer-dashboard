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

## 2b. Real car (Jetson)

The dashboard runs **on the Jetson**, and any laptop on the lab network opens it in a browser. (A dashboard inside WSL2 usually cannot see the car's topics: WSL2's NAT blocks DDS discovery.)

```bash
ssh chocapinto@192.168.3.2
ros2 pkg list | grep -E "rosbridge_server|web_video_server"   # both must appear
# if missing (needs sudo + internet on the Jetson):
#   sudo apt install -y ros-humble-rosbridge-suite ros-humble-web-video-server
git clone https://github.com/SebasMedina22/roboracer-dashboard.git ~/dashboard_ws
#   no internet on the Jetson? from the laptop: scp -r ~/dashboard_ws/src chocapinto@192.168.3.2:~/dashboard_ws/
cd ~/dashboard_ws && source /opt/ros/humble/setup.bash && colcon build --symlink-install
source install/setup.bash
ros2 topic list                                   # note the real topic names first
ros2 launch roboracer_dashboard dashboard.launch.py
```

On the laptop open **<http://192.168.3.2:8000/?car>**. The camera, LiDAR, odometry and map panels **detect their topic automatically**: if the expected name does not exist, the page asks ROS (through `rosapi`) for all topics and picks one of the right message type (the camera prefers colour images, never depth). An auto-picked topic is shown in orange, and each panel header has a drop-down with every topic of that type. A topic can still be forced in the URL, e.g. `?car&scan=/scan_filtered` or `?camera=/some/image_raw` (keys: `camera`, `scan`, `odom`, `map`, `tf`, `status`, `rosout`, `blue`, `yellow`, `waypoints`).

The LiDAR panel also draws what the cone pipeline sees: YOLO's cones (`/cones/blue`, `/cones/yellow`) as blue/yellow dots and Delaunay's path (`/waypoints`) as a magenta line. Detections older than 1 s disappear, and the cones are drawn even without `/scan` (e.g. only `yolo_node` running).

### Helper on the Jetson: `scripts/car.sh`

Finds the real topic names by itself (camera prefix, depth image) and saves everything in
`~/avai_car/<date>/` so it can be copied to the laptop afterwards.

```bash
bash scripts/car.sh info                 # topics, types, rates, nodes, free disk -> info_<time>.txt
bash scripts/car.sh dashboard            # dashboard + the URL to open on the laptop
bash scripts/car.sh record vuelta1 90    # rosbag of the right topics, stops after 90 s (or ONE Ctrl+C)
bash scripts/car.sh yolo                 # only YOLO on the real camera (nothing drives)
bash scripts/car.sh drive 0.5            # the course's autonomous pipeline, max 0.5 m/s (asks "yes")
```

On the laptop (WSL), copy the results to `C:\Users\Asus\Documents\avai_bags`:

```bash
bash scripts/fetch_car.sh                # small files only (info, logs, bag summaries)
bash scripts/fetch_car.sh --bag vuelta1  # plus one rosbag (can be GBs)
```

Manual equivalent of `car.sh record` (stop with Ctrl+C):

```bash
ros2 bag record -o lap1 /scan /odom /tf /tf_static \
  /camera/realsense2_camera/color/image_raw /camera/realsense2_camera/color/camera_info \
  /camera/realsense2_camera/aligned_depth_to_color/image_raw
```

## 2c. Evaluated simulation runs (no watching needed)

`scripts/run_eval.sh` runs the course's cone pipeline headless, measures every run against
Gazebo's true pose and writes the results to a folder. Nothing in `~/rrstack` is modified;
the guard (`scripts/check_rrstack_clean.sh`) checks that before and after every run.

```bash
# once: where results go (here a Windows folder, readable from Windows)
echo 'export AVAI_RUNS_DIR=/mnt/c/Users/<you>/Documents/avai_runs' >> ~/.bashrc && source ~/.bashrc

bash scripts/run_eval.sh                 # 1 run, 3 laps, max 10 real minutes
bash scripts/run_eval.sh -n 5 -l 3       # 5 runs in a row
bash scripts/run_eval.sh --gui -l 1      # watch one lap in the Gazebo window
```

| Option | Meaning | Default |
|---|---|---|
| `-n N` | runs in a row | 1 |
| `-l LAPS` | laps per run | 3 |
| `--rtf X` | simulation speed limit (`1` = real time, `max` = no limit); only matters if the PC runs faster than 1x | 1 |
| `--max-wall MIN` | real minutes per run | 10 |
| `--keep-going` | do not stop at the first cone contact | stop |
| `--gui` | show Gazebo | headless |
| `-o DIR` | results folder | `$AVAI_RUNS_DIR` or `~/avai_runs` |

A run ends by itself when the laps are done, at a cone contact, when the car is stuck (< 0.05 m/s
for 5 s), or at the time limit. Each run folder has `summary.json` (laps, lap times, speeds,
contacts, minimum clearance, odometry drift), `trajectory.csv`, `events.csv`, `run.png` and
`launch.log`; each batch adds `batch_<date>.csv` with one line per run.

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
| Gazebo window black / "Gazebo GUI is not responding" (distrobox, Wayland, NVIDIA laptop) | Re-run `bash scripts/install.sh` so `rr` sets `LIBGL_ALWAYS_SOFTWARE=1 LIBGL_DRI3_DISABLE=1 QT_QPA_PLATFORM=xcb IGN_IP=127.0.0.1`, open a new terminal, `pkill -f "ign gazebo"`, launch again |
| Car jumps around, `Moved backwards in time` | Two Gazebo launches running → `pkill -f "ign gazebo"; pkill -f ros2; ros2 daemon stop` |
| Dashboard map panel says "waiting for /map" | Normal: only the SLAM launch publishes `/map` |
| Gazebo texture error `RealSense_Albedo.png` | Cosmetic, ignore |

## Project rules

- Never edit files inside `~/rrstack` (only `git pull` there). Change parameters with our own YAML or launch arguments.
- Never run teleop and an autonomous controller at the same time (both publish `/cmd_vel`).
- Real car: low speed first, someone holding the joypad ready to stop it.
