// Odometry panel: numbers (speed, heading, position) + the path driven so far,
// from nav_msgs/Odometry on /odom (frame: odom).

import { fitCanvas, cssVar, drawCar, yawFromQuaternion } from './utils.js';

const MAX_TRAIL = 3000;   // points kept for the trajectory

export function createOdomPanel(canvas, fields) {
  const trail = [];
  let last = null;

  function draw() {
    const { ctx, w, h } = fitCanvas(canvas);
    ctx.fillStyle = cssVar('--panel-bg');
    ctx.fillRect(0, 0, w, h);
    if (!last) return;

    // Fit the whole trajectory in the canvas, same scale for x and y
    let minX = Infinity; let maxX = -Infinity; let minY = Infinity; let maxY = -Infinity;
    for (const p of trail) {
      minX = Math.min(minX, p.x); maxX = Math.max(maxX, p.x);
      minY = Math.min(minY, p.y); maxY = Math.max(maxY, p.y);
    }
    const pad = 24;
    const spanX = Math.max(maxX - minX, 2);
    const spanY = Math.max(maxY - minY, 2);
    const scale = Math.min((w - 2 * pad) / spanX, (h - 2 * pad) / spanY);
    const midX = (minX + maxX) / 2;
    const midY = (minY + maxY) / 2;
    // world -> screen (x right, y up)
    const sx = (x) => w / 2 + (x - midX) * scale;
    const sy = (y) => h / 2 - (y - midY) * scale;

    // Start point
    ctx.fillStyle = cssVar('--muted');
    ctx.beginPath();
    ctx.arc(sx(trail[0].x), sy(trail[0].y), 4, 0, 2 * Math.PI);
    ctx.fill();

    // Path
    ctx.strokeStyle = cssVar('--odom');
    ctx.lineWidth = 2;
    ctx.beginPath();
    trail.forEach((p, i) => (i ? ctx.lineTo(sx(p.x), sy(p.y)) : ctx.moveTo(sx(p.x), sy(p.y))));
    ctx.stroke();

    // Car (screen y is flipped, so heading on screen = -yaw)
    drawCar(ctx, sx(last.x), sy(last.y), -last.yaw, 9, cssVar('--car'));

    ctx.fillStyle = cssVar('--muted');
    ctx.font = '11px system-ui, sans-serif';
    ctx.fillText(`${(spanX).toFixed(1)} m × ${(spanY).toFixed(1)} m`, 8, h - 8);
  }

  return {
    update(msg) {
      const p = msg.pose.pose.position;
      const yaw = yawFromQuaternion(msg.pose.pose.orientation);
      const v = msg.twist.twist.linear.x;           // forward speed [m/s]
      const wz = msg.twist.twist.angular.z;         // yaw rate [rad/s]

      last = { x: p.x, y: p.y, yaw };
      const prev = trail[trail.length - 1];
      if (!prev || Math.hypot(p.x - prev.x, p.y - prev.y) > 0.02) {
        trail.push({ x: p.x, y: p.y });
        if (trail.length > MAX_TRAIL) trail.shift();
      }

      fields.speed.textContent = `${v.toFixed(2)} m/s`;
      fields.kmh.textContent = `${(v * 3.6).toFixed(1)} km/h`;
      fields.yaw.textContent = `${(yaw * 180 / Math.PI).toFixed(0)}°`;
      fields.yawRate.textContent = `${wz.toFixed(2)} rad/s`;
      fields.pos.textContent = `(${p.x.toFixed(2)}, ${p.y.toFixed(2)})`;
      draw();
    },
    reset() { trail.length = 0; draw(); },
    redraw: draw,
  };
}
