// Entry point: connect to rosbridge, wire each topic to its panel.

import { RosBridge } from './rosbridge.js';
import { createLidarPanel } from './lidar.js';
import { createOdomPanel } from './odom.js';
import { createMapPanel } from './map.js';

// Change topic names here if the stack uses different ones.
const TOPICS = {
  camera: '/camera/color/image_raw',
  scan: '/scan',
  odom: '/odom',
  map: '/map',
  tf: '/tf',
};

// Same host as the page: localhost in WSL, the Jetson's IP on the real car.
const HOST = window.location.hostname || 'localhost';
const ROSBRIDGE_URL = `ws://${HOST}:9090`;
const VIDEO_URL = `http://${HOST}:8080/stream?topic=${TOPICS.camera}&type=mjpeg&quality=60`;

const $ = (id) => document.getElementById(id);

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

//            topic        message type                     callback          max every (ms)
ros.subscribe(TOPICS.scan, 'sensor_msgs/msg/LaserScan',     lidar.update,     100);
ros.subscribe(TOPICS.odom, 'nav_msgs/msg/Odometry',         odom.update,      50);
ros.subscribe(TOPICS.map,  'nav_msgs/msg/OccupancyGrid',    map.updateMap,    2000);
ros.subscribe(TOPICS.tf,   'tf2_msgs/msg/TFMessage',        map.updateTf,     50);

$('odom-reset').onclick = () => odom.reset();

// Messages per second shown in each panel header
setInterval(() => {
  $('hz-scan').textContent = `${ros.rate(TOPICS.scan)} Hz`;
  $('hz-odom').textContent = `${ros.rate(TOPICS.odom)} Hz`;
  $('hz-map').textContent = `${ros.rate(TOPICS.map)} Hz`;
}, 1000);

window.addEventListener('resize', () => { lidar.redraw(); odom.redraw(); map.redraw(); });
