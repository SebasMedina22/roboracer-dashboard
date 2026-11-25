// Entry point: connect to rosbridge, wire each topic to its panel.

import { RosBridge } from './rosbridge.js';
import { createLidarPanel } from './lidar.js';
import { createOdomPanel } from './odom.js';
import { createMapPanel } from './map.js';
import { createMonitorPanel } from './monitor.js';
import { createLogsPanel } from './logs.js';
import { TopicSelector, listTopics } from './topics.js';

// Default topic names (simulation). On the real car the names may differ: the camera,
// LiDAR, odometry and map panels then pick a topic of the right type automatically
// (see topics.js) and offer the alternatives in a drop-down. Any name can also be
// forced from the URL, without editing code:
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
const PINNED = new Set();              // topics forced in the URL: never auto-replaced
for (const key of Object.keys(DEFAULT_TOPICS)) {
  const value = params.get(key);
  if (value) {
    TOPICS[key] = value.startsWith('/') ? value : `/${value}`;
    PINNED.add(key);
  }
}

// Same host as the page: localhost in WSL, the Jetson's IP on the real car.
const HOST = window.location.hostname || 'localhost';
const ROSBRIDGE_URL = `ws://${HOST}:9090`;
const videoUrl = () => `http://${HOST}:8080/stream?topic=${TOPICS.camera}&type=mjpeg&quality=60`;

const $ = (id) => document.getElementById(id);

// Fixed topic names in the headers (the others get a drop-down, see below).
for (const key of ['status', 'rosout']) {
  const el = $(`topic-${key}`);
  if (el) el.textContent = TOPICS[key];
}
$('lidar-info').textContent = `waiting for ${TOPICS.scan} …`;
$('map-info').textContent = `waiting for ${TOPICS.map} …`;

// ---------- Camera (MJPEG stream from web_video_server, no rosbridge) ----------
const cam = $('camera');
const camMsg = $('camera-msg');
function startCamera() {
  cam.src = `${videoUrl()}&t=${Date.now()}`;  // t= avoids a cached broken stream
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

// Panels whose topic can change at runtime (auto-detection or the drop-down).
//           key       message type                    callback        max every (ms)
const SWITCHABLE = {
  scan: ['sensor_msgs/msg/LaserScan', lidar.update, 100],
  odom: ['nav_msgs/msg/Odometry', odom.update, 50],
  map: ['nav_msgs/msg/OccupancyGrid', map.updateMap, 2000],
};
for (const [key, [type, cb, ms]] of Object.entries(SWITCHABLE)) ros.subscribe(TOPICS[key], type, cb, ms);

function switchTopic(key, topic) {
  const old = TOPICS[key];
  if (topic === old) return;
  TOPICS[key] = topic;
  if (key === 'camera') {
    startCamera();
  } else {
    const [type, cb, ms] = SWITCHABLE[key];
    ros.unsubscribe(old);
    ros.subscribe(topic, type, cb, ms);
    if (key === 'odom') odom.reset();
  }
  console.info(`[topics] ${key}: ${old} -> ${topic}`);
}

const selectors = ['camera', 'scan', 'odom', 'map'].map((key) => new TopicSelector(
  key, $(`topic-${key}`), TOPICS[key], PINNED.has(key), (topic) => switchTopic(key, topic)));

// Every few seconds: which topics exist now? (new ones appear while the stack starts up)
async function refreshTopics() {
  const list = ros.connected ? await listTopics(ros) : null;
  if (list) for (const s of selectors) s.update(list);
}
setInterval(refreshTopics, 5000);
setTimeout(refreshTopics, 1000);

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
