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

## Requirements

```bash
sudo apt install -y ros-humble-rosbridge-suite ros-humble-web-video-server
```

## Build

```bash
cd ~/dashboard_ws
colcon build --symlink-install
source install/setup.bash
```

## Run

1. A simulation (only ONE launch that opens Gazebo), e.g. `ros2 launch roboracer_slam sim_slam_rviz.launch.py`
2. `ros2 launch roboracer_dashboard dashboard.launch.py`
3. Open <http://localhost:8000>

## Files

| File | What it does |
|---|---|
| `launch/dashboard.launch.py` | Starts rosbridge, web_video_server and the web server |
| `roboracer_dashboard/web_server.py` | ROS node serving `web/` over HTTP |
| `web/js/rosbridge.js` | Tiny rosbridge client (subscribe + Hz counters) |
| `web/js/lidar.js` | LaserScan polar → x/y plot |
| `web/js/odom.js` | Speed, heading, position, trajectory |
| `web/js/map.js` | OccupancyGrid → image, car pose via TF (map→odom→base_link) |
| `web/js/main.js` | Wires topics to panels |
