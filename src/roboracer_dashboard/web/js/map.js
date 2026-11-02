// Map panel: nav_msgs/OccupancyGrid from SLAM (/map) + where the car is on it.
//
// OccupancyGrid: info.width x info.height cells of info.resolution meters.
// data[] is row-major, row 0 at info.origin (bottom of the map).
//   -1 = unknown, 0 = free, 100 = occupied
// Cell (col,row) -> world: x = origin.x + (col + 0.5)*res, y = origin.y + (row + 0.5)*res
//
// The car pose in the map frame comes from TF:
//   map -> odom        (published by slam_toolbox: the drift correction)
//   odom -> base_link  (published by the simulator / wheel odometry)
//   map -> base_link = (map -> odom) ∘ (odom -> base_link)

import { fitCanvas, cssVar, drawCar, yawFromQuaternion } from './utils.js';

// 2D transform {x, y, yaw}; compose a ∘ b (apply b, then a)
function compose(a, b) {
  const c = Math.cos(a.yaw);
  const s = Math.sin(a.yaw);
  return { x: a.x + c * b.x - s * b.y, y: a.y + s * b.x + c * b.y, yaw: a.yaw + b.yaw };
}

function to2D(t) {
  return { x: t.translation.x, y: t.translation.y, yaw: yawFromQuaternion(t.rotation) };
}

const strip = (frame) => frame.replace(/^\//, '');

export function createMapPanel(canvas, infoEl) {
  const mapImage = document.createElement('canvas');   // the grid, 1 pixel per cell
  let info = null;
  let mapToOdom = null;
  let odomToBase = null;
  const trail = [];

  function carPose() {
    if (!odomToBase) return null;
    // Before SLAM publishes map->odom, assume they coincide.
    return compose(mapToOdom || { x: 0, y: 0, yaw: 0 }, odomToBase);
  }

  function draw() {
    const { ctx, w, h } = fitCanvas(canvas);
    ctx.fillStyle = cssVar('--panel-bg');
    ctx.fillRect(0, 0, w, h);
    if (!info) {
      ctx.fillStyle = cssVar('--muted');
      ctx.font = '13px system-ui, sans-serif';
      ctx.fillText('Waiting for /map … (run sim_slam_rviz.launch.py and drive)', 12, 24);
      return;
    }

    // Fit the map in the panel keeping its aspect ratio
    const scale = Math.min(w / info.width, h / info.height);     // screen px per cell
    const offX = (w - info.width * scale) / 2;
    const offY = (h - info.height * scale) / 2;
    ctx.imageSmoothingEnabled = false;
    ctx.drawImage(mapImage, offX, offY, info.width * scale, info.height * scale);

    // world (map frame) -> screen
    const res = info.resolution;
    const ox = info.origin.position.x;
    const oy = info.origin.position.y;
    const sx = (x) => offX + ((x - ox) / res) * scale;
    const sy = (y) => offY + (info.height - (y - oy) / res) * scale;

    if (trail.length > 1) {
      ctx.strokeStyle = cssVar('--odom');
      ctx.lineWidth = 1.5;
      ctx.beginPath();
      trail.forEach((p, i) => (i ? ctx.lineTo(sx(p.x), sy(p.y)) : ctx.moveTo(sx(p.x), sy(p.y))));
      ctx.stroke();
    }

    const pose = carPose();
    if (pose) drawCar(ctx, sx(pose.x), sy(pose.y), -pose.yaw, 8, cssVar('--car'));
  }

  function updateMap(msg) {
    info = msg.info;
    mapImage.width = info.width;
    mapImage.height = info.height;
    const mctx = mapImage.getContext('2d');
    const img = mctx.createImageData(info.width, info.height);
    let known = 0;
    let occupied = 0;
    for (let row = 0; row < info.height; row++) {
      const flipped = info.height - 1 - row;              // image row 0 is the TOP
      for (let col = 0; col < info.width; col++) {
        const v = msg.data[row * info.width + col];
        let shade;
        if (v < 0) shade = 70;                             // unknown: dark gray
        else { shade = 255 - Math.round(v * 2.55); known++; if (v > 50) occupied++; }
        const k = (flipped * info.width + col) * 4;
        img.data[k] = img.data[k + 1] = img.data[k + 2] = shade;
        img.data[k + 3] = 255;
      }
    }
    mctx.putImageData(img, 0, 0);
    infoEl.textContent =
      `${info.width}×${info.height} cells · ${(info.resolution * 100).toFixed(0)} cm/cell · ` +
      `${(known * info.resolution ** 2).toFixed(1)} m² explored · ${occupied} occupied`;
    draw();
  }

  function updateTf(msg) {
    let changed = false;
    for (const t of msg.transforms) {
      const parent = strip(t.header.frame_id);
      const child = strip(t.child_frame_id);
      if (parent === 'map' && child === 'odom') { mapToOdom = to2D(t.transform); changed = true; }
      if (parent === 'odom' && child === 'base_link') { odomToBase = to2D(t.transform); changed = true; }
    }
    if (!changed) return;
    const pose = carPose();
    const prev = trail[trail.length - 1];
    if (pose && (!prev || Math.hypot(pose.x - prev.x, pose.y - prev.y) > 0.05)) {
      trail.push({ x: pose.x, y: pose.y });
      if (trail.length > 3000) trail.shift();
    }
    draw();
  }

  return { updateMap, updateTf, redraw: draw };
}
