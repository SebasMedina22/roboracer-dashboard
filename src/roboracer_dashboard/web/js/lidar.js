// LiDAR panel: draws a sensor_msgs/LaserScan seen from above.
//
// Each beam i has angle   theta_i = angle_min + i * angle_increment
// and range r_i. In the car frame (x forward, y left):
//   x = r cos(theta),  y = r sin(theta)
// On screen the car points UP, so:  screen_x = cx - y*scale,  screen_y = cy - x*scale
//
// On top of the scan it draws what the cone pipeline sees (same axes, x forward, y left):
//   - /cones/blue, /cones/yellow (yolo_node): positions relative to the camera
//   - /waypoints (delaunay_node): relative to base_link, CAMERA_OFFSET behind the camera
// In the simulator the camera and the LiDAR sit at the same x on the car; on the real
// car the difference is a few cm, fine for a visual check.

import { fitCanvas, cssVar, drawCar } from './utils.js';

const CAMERA_OFFSET = 0.37;   // camera ahead of base_link [m] (planning_params.yaml)
const STALE_MS = 1000;        // detections older than this are not drawn
const CONE_RADIUS = 0.117;    // [m], drawn at least 4 px

export function createLidarPanel(canvas, infoEl) {
  let viewRange = 5;      // meters from the car to the panel edge (mouse wheel to zoom)
  let lastScan = null;
  // latest PoseArray points per layer: {pts: [[x, y], ...], at: ms}
  const layers = { blue: null, yellow: null, waypoints: null };

  const points = (msg) => (msg.poses || []).map((p) => [p.position.x, p.position.y]);
  const fresh = (layer) => layer && performance.now() - layer.at < STALE_MS;

  canvas.addEventListener('wheel', (e) => {
    e.preventDefault();
    viewRange = Math.min(30, Math.max(1, viewRange * (e.deltaY > 0 ? 1.15 : 0.87)));
    draw();
  }, { passive: false });

  function draw() {
    const scan = lastScan;
    const { ctx, w, h } = fitCanvas(canvas);
    const cx = w / 2;
    const cy = h / 2;
    const scale = Math.min(w, h) / 2 / viewRange;   // pixels per meter

    ctx.fillStyle = cssVar('--panel-bg');
    ctx.fillRect(0, 0, w, h);

    // Range rings every meter (every 5 m when zoomed out)
    const step = viewRange > 10 ? 5 : 1;
    ctx.strokeStyle = cssVar('--grid');
    ctx.fillStyle = cssVar('--muted');
    ctx.font = '11px system-ui, sans-serif';
    ctx.lineWidth = 1;
    for (let r = step; r <= viewRange * 1.5; r += step) {
      ctx.beginPath();
      ctx.arc(cx, cy, r * scale, 0, 2 * Math.PI);
      ctx.stroke();
      ctx.fillText(`${r} m`, cx + 3, cy - r * scale - 3);
    }

    // Beams -> points
    let closest = { r: Infinity, x: 0, y: 0 };
    ctx.fillStyle = cssVar('--lidar');
    for (let i = 0; scan && i < scan.ranges.length; i++) {
      const r = scan.ranges[i];
      if (r === null || !Number.isFinite(r) || r < scan.range_min || r > scan.range_max) continue;
      const theta = scan.angle_min + i * scan.angle_increment;
      const x = r * Math.cos(theta);
      const y = r * Math.sin(theta);
      const sx = cx - y * scale;
      const sy = cy - x * scale;
      ctx.fillRect(sx - 1.5, sy - 1.5, 3, 3);
      if (r < closest.r) closest = { r, x: sx, y: sy };
    }

    // Closest obstacle
    if (Number.isFinite(closest.r)) {
      ctx.strokeStyle = cssVar('--warn');
      ctx.setLineDash([4, 4]);
      ctx.beginPath();
      ctx.moveTo(cx, cy);
      ctx.lineTo(closest.x, closest.y);
      ctx.stroke();
      ctx.setLineDash([]);
    }

    // Planned path: car -> waypoints (shifted from base_link to the camera/LiDAR origin)
    if (fresh(layers.waypoints) && layers.waypoints.pts.length) {
      ctx.strokeStyle = cssVar('--path');
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.moveTo(cx, cy);
      for (const [x, y] of layers.waypoints.pts) {
        ctx.lineTo(cx - y * scale, cy - (x - CAMERA_OFFSET) * scale);
      }
      ctx.stroke();
      ctx.lineWidth = 1;
    }

    // Detected cones
    const coneR = Math.max(4, CONE_RADIUS * scale);
    for (const color of ['blue', 'yellow']) {
      if (!fresh(layers[color])) continue;
      ctx.fillStyle = cssVar(color === 'blue' ? '--cone-blue' : '--cone-yellow');
      for (const [x, y] of layers[color].pts) {
        ctx.beginPath();
        ctx.arc(cx - y * scale, cy - x * scale, coneR, 0, 2 * Math.PI);
        ctx.fill();
      }
    }

    drawCar(ctx, cx, cy, -Math.PI / 2, 10, cssVar('--car'));

    const n = (layer) => (fresh(layer) ? layer.pts.length : 0);
    const cones = `cones ${n(layers.blue)} blue / ${n(layers.yellow)} yellow`;
    const beams = scan
      ? `${scan.ranges.filter((v) => v !== null && Number.isFinite(v)).length}/${scan.ranges.length} beams · `
        + `closest ${Number.isFinite(closest.r) ? closest.r.toFixed(2) + ' m' : '—'}`
      : 'no /scan';
    infoEl.textContent = `${beams} · ${cones} · zoom ±${viewRange.toFixed(1)} m`;
  }

  // Without /scan (e.g. only YOLO running) the cones still need redrawing,
  // and stale detections must disappear: a slow timer covers both.
  setInterval(() => {
    if (Object.values(layers).some((l) => l !== null)) draw();
  }, 250);

  return {
    update(scan) {
      lastScan = scan;
      draw();
    },
    updateCones(color, msg) {
      layers[color] = { pts: points(msg), at: performance.now() };
      if (!lastScan) draw();
    },
    updateWaypoints(msg) {
      layers.waypoints = { pts: points(msg), at: performance.now() };
    },
    redraw() { draw(); },
  };
}
