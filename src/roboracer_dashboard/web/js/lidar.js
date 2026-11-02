// LiDAR panel: draws a sensor_msgs/LaserScan seen from above.
//
// Each beam i has angle   theta_i = angle_min + i * angle_increment
// and range r_i. In the car frame (x forward, y left):
//   x = r cos(theta),  y = r sin(theta)
// On screen the car points UP, so:  screen_x = cx - y*scale,  screen_y = cy - x*scale

import { fitCanvas, cssVar, drawCar } from './utils.js';

export function createLidarPanel(canvas, infoEl) {
  let viewRange = 5;      // meters from the car to the panel edge (mouse wheel to zoom)
  let lastScan = null;

  canvas.addEventListener('wheel', (e) => {
    e.preventDefault();
    viewRange = Math.min(30, Math.max(1, viewRange * (e.deltaY > 0 ? 1.15 : 0.87)));
    if (lastScan) draw(lastScan);
  }, { passive: false });

  function draw(scan) {
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
    for (let i = 0; i < scan.ranges.length; i++) {
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

    drawCar(ctx, cx, cy, -Math.PI / 2, 10, cssVar('--car'));

    const valid = scan.ranges.filter((r) => r !== null && Number.isFinite(r)).length;
    infoEl.textContent =
      `${valid}/${scan.ranges.length} beams · closest ${Number.isFinite(closest.r) ? closest.r.toFixed(2) + ' m' : '—'} · zoom ±${viewRange.toFixed(1)} m`;
  }

  return {
    update(scan) {
      lastScan = scan;
      draw(scan);
    },
    redraw() { if (lastScan) draw(lastScan); },
  };
}
