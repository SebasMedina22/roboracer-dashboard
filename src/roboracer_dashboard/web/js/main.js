// Entry point: connect to rosbridge, wire each topic to its panel.

import { RosBridge } from './rosbridge.js';
import { createLidarPanel } from './lidar.js';
import { createOdomPanel } from './odom.js';
import { createMapPanel } from './map.js';
import { createMonitorPanel } from './monitor.js';
import { createLogsPanel } from './logs.js';

// Default topic names (simulation). Any of them can be overridden from the URL,
// without editing code:
//   http://<host>:8000/?camera=/camera/realsense2_camera/color/image_raw&scan=/scan
// Keys: camera, scan, odom, map, tf, status, rosout, blue, yellow, waypoints.
// `?car` switches to the real car's RealSense camera topic (other topics stay the same).
const DEFAULT_TOPICS = {
  camera: '/camera/color/image_raw',
  scan: '/scan',
  odom: '/odom',
  map: '/map',
  tf: '/tf',
  status: '/eval/status',
  rosout: '/rosout',
  blue: '/cones/blue',          // yolo_node detections, drawn on the LiDAR panel
  yellow: '/cones/yellow',
  waypoints: '/waypoints',      // delaunay_node path, drawn on the LiDAR panel
};
const CAR_TOPICS = { camera: '/camera/realsense2_camera/color/image_raw' };

const params = new URLSearchParams(window.location.search);
const TOPICS = { ...DEFAULT_TOPICS, ...(params.has('car') ? CAR_TOPICS : {}) };
for (const key of Object.keys(DEFAULT_TOPICS)) {
  const value = params.get(key);
  if (value) TOPICS[key] = value.startsWith('/') ? value : `/${value}`;
}

// Same host as the page: localhost in WSL, the Jetson's IP on the real car.
const HOST = window.location.hostname || 'localhost';
const ROSBRIDGE_URL = `ws://${HOST}:9090`;
const VIDEO_URL = `http://${HOST}:8080/stream?topic=${TOPICS.camera}&type=mjpeg&quality=60`;

const $ = (id) => document.getElementById(id);

// Show the topic names actually in use in the panel headers.
for (const key of ['camera', 'scan', 'odom', 'map', 'status', 'rosout']) {
  const el = $(`topic-${key}`);
  if (el) el.textContent = TOPICS[key];
}
$('lidar-info').textContent = `waiting for ${TOPICS.scan} …`;
$('map-info').textContent = `waiting for ${TOPICS.map} …`;

// ---------- Camera (MJPEG stream from web_video_server, no rosbridge) ----------
const cam = $('camera');
const camMsg = $('camera-msg');
function startCamera() {
  cam.src = `${VIDEO_URL}&t=${Date.now()}`;   // t= avoids a cached broken stream
}
// An MJPEG stream never "finishes loading", so instead of onload we check
// whether a first frame has arrived (naturalWidth > 0).
setInterval(() => { camMsg.hidden = cam.naturalWidth > 0; }, 500);
cam.onerror = () => {
  camMsg.hidden = false;
  setTimeout(startCamera, 3000);              // retry until web_video_server is up
};
startCamera();

// ---------- ROS topics through rosbridge ----------
const ros = new RosBridge(ROSBRIDGE_URL);
ros.onStatus = (ok) => {
  const el = $('status');
  el.textContent = ok ? `rosbridge connected · ${ROSBRIDGE_URL}` : `rosbridge disconnected · retrying ${ROSBRIDGE_URL}`;
  el.className = ok ? 'pill ok' : 'pill bad';
};

const lidar = createLidarPanel($('lidar'), $('lidar-info'));
const odom = createOdomPanel($('odom'), {
  speed: $('odom-speed'), kmh: $('odom-kmh'), yaw: $('odom-yaw'),
  yawRate: $('odom-yawrate'), pos: $('odom-pos'),
});
const map = createMapPanel($('map'), $('map-info'));
const monitor = createMonitorPanel({
  state: $('mon-state'), run: $('mon-run'), body: $('mon-body'), idle: $('mon-idle'),
  simt: $('mon-simt'), rtf: $('mon-rtf'), lap: $('mon-lap'), dist: $('mon-dist'),
  speed: $('mon-speed'), clear: $('mon-clear'), minclear: $('mon-minclear'), mode: $('mon-mode'),
  contacts: $('mon-contacts'), drift: $('mon-drift'), laps: $('mon-laps'),
});
const logs = createLogsPanel($('log-list'), {
  all: $('log-all'), warn: $('log-warn'), monitor: $('log-monitor'), clear: $('log-clear'),
}, $('log-count'));

//            topic        message type                     callback          max every (ms)
ros.subscribe(TOPICS.scan, 'sensor_msgs/msg/LaserScan',     lidar.update,     100);
ros.subscribe(TOPICS.odom, 'nav_msgs/msg/Odometry',         odom.update,      50);
ros.subscribe(TOPICS.map,  'nav_msgs/msg/OccupancyGrid',    map.updateMap,    2000);
ros.subscribe(TOPICS.tf,   'tf2_msgs/msg/TFMessage',        map.updateTf,     50);
ros.subscribe(TOPICS.blue,   'geometry_msgs/msg/PoseArray',   (m) => lidar.updateCones('blue', m),   100);
ros.subscribe(TOPICS.yellow, 'geometry_msgs/msg/PoseArray',   (m) => lidar.updateCones('yellow', m), 100);
ros.subscribe(TOPICS.waypoints, 'geometry_msgs/msg/PoseArray', lidar.updateWaypoints, 100);
ros.subscribe(TOPICS.status, 'std_msgs/msg/String',         monitor.update,   0);
// every log line matters: no throttling, and a queue so bursts are not dropped
ros.subscribe(TOPICS.rosout, 'rcl_interfaces/msg/Log',      logs.add,         0, 200);

$('odom-reset').onclick = () => odom.reset();

// Messages per second shown in each panel header
setInterval(() => {
  $('hz-scan').textContent = `${ros.rate(TOPICS.scan)} Hz`;
  $('hz-odom').textContent = `${ros.rate(TOPICS.odom)} Hz`;
  $('hz-map').textContent = `${ros.rate(TOPICS.map)} Hz`;
}, 1000);

window.addEventListener('resize', () => { lidar.redraw(); odom.redraw(); map.redraw(); });
