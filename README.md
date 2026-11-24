# roboracer_dashboard

Web dashboard for the RoboRacer (AVAI Lab, RUB). Shows the 4 things Task 3 asks for:

| Panel | Topic | Type |
|---|---|---|
| Camera | `/camera/color/image_raw` | `sensor_msgs/Image` (via web_video_server, MJPEG) |
| LiDAR | `/scan` | `sensor_msgs/LaserScan` |
| Odometry | `/odom` | `nav_msgs/Odometry` |
| Occupancy grid | `/map` + `/tf` | `nav_msgs/OccupancyGrid`, `tf2_msgs/TFMessage` |

## How it works

```text
ROS topics ──► rosbridge_websocket (:9090, JSON over WebSocket) ──► browser (web/js/*.js, canvas)
/camera/... ──► web_video_server (:8080, MJPEG over HTTP) ─────────► <img>
web/ files ──► web_server node (:8000, plain HTTP) ───────────────► the page itself
```

The page uses the same host it was loaded from, so on the real car you open
`http://<jetson-ip>:8000` from a laptop and everything points at the Jetson.

## Quick start

Ubuntu 22.04 (native, WSL2, or distrobox). Full guide and troubleshooting: **[INSTALL.md](INSTALL.md)**.

```bash
git clone https://github.com/SebasMedina22/roboracer-dashboard.git ~/dashboard_ws
cd ~/dashboard_ws
bash scripts/install.sh        # ROS 2 Humble, Gazebo, YOLO, rrstack, this dashboard, shortcuts rr / rd
```

Then, in a **new** terminal for each line:

| Terminal | Commands |
|---|---|
| T1 | `rr` then `ros2 launch roboracer_planning cone_drive.launch.py` (simulation; only ONE launch that opens Gazebo) |
| T2 | `rd` then `ros2 launch roboracer_dashboard dashboard.launch.py` |
| Browser | <http://localhost:8000> |

The occupancy-grid panel needs `/map`, which only the SLAM launch publishes: `ros2 launch roboracer_slam sim_slam_rviz.launch.py`.

## Repository layout

This repo is a colcon workspace: the package lives in `src/roboracer_dashboard/`, and `build/ install/ log/` are git-ignored. `scripts/install.sh` sets up everything else.

## Files

Paths relative to `src/roboracer_dashboard/`.

| File | What it does |
|---|---|
| `launch/dashboard.launch.py` | Starts rosbridge, web_video_server and the web server |
| `roboracer_dashboard/web_server.py` | ROS node serving `web/` over HTTP |
| `web/js/rosbridge.js` | Tiny rosbridge client (subscribe + Hz counters) |
| `web/js/lidar.js` | LaserScan polar → x/y plot |
| `web/js/odom.js` | Speed, heading, position, trajectory |
| `web/js/map.js` | OccupancyGrid → image, car pose via TF (map→odom→base_link) |
| `web/js/main.js` | Wires topics to panels |
