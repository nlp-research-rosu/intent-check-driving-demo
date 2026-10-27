// 2D geometry helpers. World frame: x east (along the highway), y north (left of travel), heading CCW from +x.

export const clamp = (v, lo, hi) => (v < lo ? lo : v > hi ? hi : v);

export function wrapAngle(a) {
  a = (a + Math.PI) % (2 * Math.PI);
  if (a < 0) a += 2 * Math.PI;
  return a - Math.PI;
}

/** Corners of a vehicle rectangle centred at (x, y), counter-clockwise: FL, RL, RR, FR. */
export function vehicleCorners(x, y, heading, length, width) {
  const c = Math.cos(heading), s = Math.sin(heading);
  const hl = length / 2, hw = width / 2;
  return [
    [x + c * hl - s * hw, y + s * hl + c * hw],
    [x - c * hl - s * hw, y - s * hl + c * hw],
    [x - c * hl + s * hw, y - s * hl - c * hw],
    [x + c * hl + s * hw, y + s * hl - c * hw],
  ];
}

/** Points along the footprint boundary: corners plus points every <= spacing metres along each edge. */
export function footprintSamples(x, y, heading, length, width, spacing) {
  const corners = vehicleCorners(x, y, heading, length, width);
  const pts = [];
  for (let i = 0; i < 4; i++) {
    const [x1, y1] = corners[i];
    const [x2, y2] = corners[(i + 1) % 4];
    const n = Math.max(1, Math.ceil(Math.hypot(x2 - x1, y2 - y1) / spacing));
    for (let k = 0; k < n; k++) pts.push([x1 + ((x2 - x1) * k) / n, y1 + ((y2 - y1) * k) / n]);
  }
  return pts;
}

/** Separating-axis test for two convex polygons. Touching counts as overlapping. */
export function convexOverlap(a, b) {
  for (const poly of [a, b]) {
    for (let i = 0; i < poly.length; i++) {
      const [x1, y1] = poly[i];
      const [x2, y2] = poly[(i + 1) % poly.length];
      const nx = y2 - y1, ny = x1 - x2;
      let minA = Infinity, maxA = -Infinity, minB = Infinity, maxB = -Infinity;
      for (const [px, py] of a) { const d = nx * px + ny * py; if (d < minA) minA = d; if (d > maxA) maxA = d; }
      for (const [px, py] of b) { const d = nx * px + ny * py; if (d < minB) minB = d; if (d > maxB) maxB = d; }
      if (maxA < minB || maxB < minA) return false;
    }
  }
  return true;
}

/** Point in convex polygon (either winding). Points on the boundary count as inside. */
export function pointInConvex(poly, px, py, eps = 1e-9) {
  let sign = 0;
  for (let i = 0; i < poly.length; i++) {
    const [x1, y1] = poly[i];
    const [x2, y2] = poly[(i + 1) % poly.length];
    const cross = (x2 - x1) * (py - y1) - (y2 - y1) * (px - x1);
    if (cross > eps) { if (sign < 0) return false; sign = 1; }
    else if (cross < -eps) { if (sign > 0) return false; sign = -1; }
  }
  return true;
}

export function polygonArea(poly) {
  let a = 0;
  for (let i = 0; i < poly.length; i++) {
    const [x1, y1] = poly[i];
    const [x2, y2] = poly[(i + 1) % poly.length];
    a += x1 * y2 - x2 * y1;
  }
  return a / 2;
}

export function isConvex(poly) {
  if (poly.length < 3) return false;
  let sign = 0;
  for (let i = 0; i < poly.length; i++) {
    const [x1, y1] = poly[i];
    const [x2, y2] = poly[(i + 1) % poly.length];
    const [x3, y3] = poly[(i + 2) % poly.length];
    const cross = (x2 - x1) * (y3 - y2) - (y2 - y1) * (x3 - x2);
    if (Math.abs(cross) < 1e-12) continue;
    const s = Math.sign(cross);
    if (sign !== 0 && s !== sign) return false;
    sign = s;
  }
  return sign !== 0;
}

export function polygonCentroid(poly) {
  let x = 0, y = 0;
  for (const p of poly) { x += p[0]; y += p[1]; }
  return [x / poly.length, y / poly.length];
}
