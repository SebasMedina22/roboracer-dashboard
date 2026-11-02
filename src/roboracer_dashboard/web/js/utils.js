// Small helpers shared by the panels.

// Yaw (heading) in radians from a ROS quaternion {x, y, z, w}.
// psi = atan2(2(wz + xy), 1 - 2(y^2 + z^2))
export function yawFromQuaternion(q) {
  return Math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));
}

// Make a canvas' drawing buffer match its size on screen (sharp on HiDPI).
// Returns {ctx, w, h} in CSS pixels.
export function fitCanvas(canvas) {
  const dpr = window.devicePixelRatio || 1;
  const w = canvas.clientWidth;
  const h = canvas.clientHeight;
  if (canvas.width !== Math.round(w * dpr) || canvas.height !== Math.round(h * dpr)) {
    canvas.width = Math.round(w * dpr);
    canvas.height = Math.round(h * dpr);
  }
  const ctx = canvas.getContext('2d');
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  return { ctx, w, h };
}

// Read a CSS variable so canvases use the same colors as the page theme.
export function cssVar(name) {
  return getComputedStyle(document.documentElement).getPropertyValue(name).trim();
}

// Draw the car as a triangle pointing in `heading` (radians, screen coords:
// 0 = right, -PI/2 = up).
export function drawCar(ctx, x, y, heading, size, color) {
  ctx.save();
  ctx.translate(x, y);
  ctx.rotate(heading);
  ctx.beginPath();
  ctx.moveTo(size, 0);
  ctx.lineTo(-size * 0.7, size * 0.6);
  ctx.lineTo(-size * 0.7, -size * 0.6);
  ctx.closePath();
  ctx.fillStyle = color;
  ctx.fill();
  ctx.restore();
}
