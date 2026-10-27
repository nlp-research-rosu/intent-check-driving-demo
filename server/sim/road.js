// Static road geometry: a straight three-lane highway (travel direction +x) with an entrance ramp,
// acceleration lane, deceleration lane and exit ramp on the right-hand side (negative y).
//
// Lanes are numbered 0 (rightmost, y in [0, 3.7]) to 2 (leftmost, y in [7.4, 11.1]).
// The drivable surface is the union of convex region polygons.

import { pointInConvex, footprintSamples } from './geometry.js';
import { FOOTPRINT_EDGE_SPACING } from './constants.js';

export const LANE_WIDTH = 3.7;
export const NUM_LANES = 3;
export const HIGHWAY_WIDTH = LANE_WIDTH * NUM_LANES;
export const HIGHWAY_X_MIN = -400;
export const HIGHWAY_X_MAX = 1600;
export const HIGHWAY_SPEED_LIMIT = 30; // m/s (108 km/h)
export const RAMP_SPEED_LIMIT = 22; // m/s (79 km/h)
export const RAMP_WIDTH = 4.0;

export const ENTRANCE_RAMP = Object.freeze({ p0: [-60, -52], p1: [160, -LANE_WIDTH / 2], extendStart: 0, extendEnd: 4 });
export const ACCEL_LANE = Object.freeze({ x0: 150, x1: 380, taperEnd: 440 });
export const DECEL_LANE = Object.freeze({ taperStart: 820, x0: 880, x1: 1000 });
export const EXIT_RAMP = Object.freeze({ p0: [990, -LANE_WIDTH / 2], p1: [1200, -50], extendStart: 4, extendEnd: 0 });

export const REGION_KINDS = Object.freeze(['highway', 'entrance_ramp', 'accel_lane', 'taper', 'decel_lane', 'exit_ramp']);

function rampFrame(r) {
  const dx = r.p1[0] - r.p0[0], dy = r.p1[1] - r.p0[1];
  const len = Math.hypot(dx, dy);
  const ux = dx / len, uy = dy / len;
  return { len, ux, uy, nx: -uy, ny: ux, heading: Math.atan2(dy, dx) };
}

/** Ramp rectangle, counter-clockwise: start-right, end-right, end-left, start-left. */
function rampQuad(r) {
  const f = rampFrame(r);
  const hw = RAMP_WIDTH / 2;
  const a = [r.p0[0] - f.ux * r.extendStart, r.p0[1] - f.uy * r.extendStart];
  const b = [r.p1[0] + f.ux * r.extendEnd, r.p1[1] + f.uy * r.extendEnd];
  return [
    [a[0] - f.nx * hw, a[1] - f.ny * hw],
    [b[0] - f.nx * hw, b[1] - f.ny * hw],
    [b[0] + f.nx * hw, b[1] + f.ny * hw],
    [a[0] + f.nx * hw, a[1] + f.ny * hw],
  ];
}

/** Pose at distance s along a ramp centreline (from p0), with lateral offset (left positive). */
export function rampPose(which, s, lateral = 0) {
  const r = which === 'entrance' ? ENTRANCE_RAMP : EXIT_RAMP;
  const f = rampFrame(r);
  return { x: r.p0[0] + f.ux * s + f.nx * lateral, y: r.p0[1] + f.uy * s + f.ny * lateral, heading: f.heading };
}

export function rampLength(which) {
  return rampFrame(which === 'entrance' ? ENTRANCE_RAMP : EXIT_RAMP).len;
}

/** Quad covering ramp centreline distances [s0, s1] at full ramp width. */
export function rampSection(which, s0, s1) {
  const a = rampPose(which, s0), b = rampPose(which, s1);
  const nx = -Math.sin(a.heading), ny = Math.cos(a.heading);
  const hw = RAMP_WIDTH / 2;
  return [
    [a.x - nx * hw, a.y - ny * hw],
    [b.x - nx * hw, b.y - ny * hw],
    [b.x + nx * hw, b.y + ny * hw],
    [a.x + nx * hw, a.y + ny * hw],
  ];
}

const yAtX = (p, q, x) => p[1] + ((q[1] - p[1]) * (x - p[0])) / (q[0] - p[0]);
const xAtY = (p, q, y) => p[0] + ((q[0] - p[0]) * (y - p[1])) / (q[1] - p[1]);

const LW = LANE_WIDTH, HW = HIGHWAY_WIDTH, XMIN = HIGHWAY_X_MIN, XMAX = HIGHWAY_X_MAX;
const entQ = rampQuad(ENTRANCE_RAMP);
const extQ = rampQuad(EXIT_RAMP);

// Region order matters only for speed-limit lookup: the first containing region wins.
export const REGIONS = Object.freeze([
  { id: 'highway', kind: 'highway', speedLimit: HIGHWAY_SPEED_LIMIT, polygon: [[XMIN, 0], [XMAX, 0], [XMAX, HW], [XMIN, HW]] },
  { id: 'accel_lane', kind: 'accel_lane', speedLimit: HIGHWAY_SPEED_LIMIT, polygon: [[ACCEL_LANE.x0, -LW], [ACCEL_LANE.x1, -LW], [ACCEL_LANE.x1, 0], [ACCEL_LANE.x0, 0]] },
  { id: 'accel_taper', kind: 'taper', speedLimit: HIGHWAY_SPEED_LIMIT, polygon: [[ACCEL_LANE.x1, -LW], [ACCEL_LANE.taperEnd, 0], [ACCEL_LANE.x1, 0]] },
  { id: 'decel_taper', kind: 'taper', speedLimit: HIGHWAY_SPEED_LIMIT, polygon: [[DECEL_LANE.x0, -LW], [DECEL_LANE.x0, 0], [DECEL_LANE.taperStart, 0]] },
  { id: 'decel_lane', kind: 'decel_lane', speedLimit: HIGHWAY_SPEED_LIMIT, polygon: [[DECEL_LANE.x0, -LW], [DECEL_LANE.x1, -LW], [DECEL_LANE.x1, 0], [DECEL_LANE.x0, 0]] },
  { id: 'entrance_ramp', kind: 'entrance_ramp', speedLimit: RAMP_SPEED_LIMIT, polygon: entQ },
  { id: 'exit_ramp', kind: 'exit_ramp', speedLimit: RAMP_SPEED_LIMIT, polygon: extQ },
].map((r) => Object.freeze({ ...r, polygon: Object.freeze(r.polygon.map((p) => Object.freeze(p))) })));

// Painted lines for rendering only (they have no effect on the simulation).
const entRightEnd = [xAtY(entQ[0], entQ[1], -LW), -LW];
const entLeftGore = [ACCEL_LANE.x0, yAtX(entQ[3], entQ[2], ACCEL_LANE.x0)];
const extRightStart = [xAtY(extQ[0], extQ[1], -LW), -LW];
const extLeftGore = [DECEL_LANE.x1, yAtX(extQ[3], extQ[2], DECEL_LANE.x1)];

export const MARKINGS = Object.freeze([
  { style: 'solid', pts: [[XMIN, HW], [XMAX, HW]] },
  { style: 'dashed', pts: [[XMIN, LW], [XMAX, LW]] },
  { style: 'dashed', pts: [[XMIN, 2 * LW], [XMAX, 2 * LW]] },
  { style: 'solid', pts: [[XMIN, 0], [ACCEL_LANE.x0, 0]] },
  { style: 'dashed', pts: [[ACCEL_LANE.x0, 0], [ACCEL_LANE.taperEnd, 0]] },
  { style: 'solid', pts: [[ACCEL_LANE.taperEnd, 0], [DECEL_LANE.taperStart, 0]] },
  { style: 'dashed', pts: [[DECEL_LANE.taperStart, 0], [DECEL_LANE.x1, 0]] },
  { style: 'solid', pts: [[DECEL_LANE.x1, 0], [XMAX, 0]] },
  { style: 'solid', pts: [[XMIN, 0], [XMIN, HW]] },
  { style: 'solid', pts: [[XMAX, 0], [XMAX, HW]] },
  // Entrance ramp and acceleration lane.
  { style: 'solid', pts: [entQ[3], entQ[0]] },
  { style: 'solid', pts: [entQ[0], entRightEnd, [ACCEL_LANE.x1, -LW], [ACCEL_LANE.taperEnd, 0]] },
  { style: 'solid', pts: [entQ[3], entLeftGore, [ACCEL_LANE.x0, 0]] },
  // Deceleration lane and exit ramp.
  { style: 'solid', pts: [[DECEL_LANE.taperStart, 0], [DECEL_LANE.x0, -LW], extRightStart, extQ[1]] },
  { style: 'solid', pts: [[DECEL_LANE.x1, 0], extLeftGore, extQ[2]] },
  { style: 'solid', pts: [extQ[1], extQ[2]] },
].map((m) => Object.freeze(m)));

/** Road description shared with the controller and the browser. */
export const ROAD = Object.freeze({
  laneWidth: LANE_WIDTH,
  numLanes: NUM_LANES,
  xMin: HIGHWAY_X_MIN,
  xMax: HIGHWAY_X_MAX,
  regions: REGIONS,
});

export function regionAt(x, y) {
  for (const r of REGIONS) if (pointInConvex(r.polygon, x, y)) return r;
  return null;
}

export const isDrivable = (x, y) => regionAt(x, y) !== null;

export function speedLimitAt(x, y) {
  const r = regionAt(x, y);
  return r ? r.speedLimit : null;
}

export const laneCenterY = (k) => (k + 0.5) * LANE_WIDTH;

/** Highway lane containing lateral position y, or -1 when outside the main carriageway. */
export function laneAt(y) {
  if (y < 0 || y > HIGHWAY_WIDTH) return -1;
  return Math.min(NUM_LANES - 1, Math.floor(y / LANE_WIDTH));
}

/** Highway lanes whose corridor overlaps the vehicle body laterally by at least minOverlap metres. */
export function lanesOccupied(v, minOverlap = 0.3) {
  if (v.x < HIGHWAY_X_MIN || v.x > HIGHWAY_X_MAX) return [];
  const half = Math.abs(Math.sin(v.heading)) * v.length / 2 + Math.abs(Math.cos(v.heading)) * v.width / 2;
  const lo = v.y - half, hi = v.y + half;
  const out = [];
  for (let k = 0; k < NUM_LANES; k++) {
    const overlap = Math.min(hi, (k + 1) * LANE_WIDTH) - Math.max(lo, k * LANE_WIDTH);
    if (overlap >= minOverlap) out.push(k);
  }
  return out;
}

/** First footprint boundary point that is off the drivable surface, or null if the footprint is on-road. */
export function footprintOffRoad(x, y, heading, length, width) {
  for (const p of footprintSamples(x, y, heading, length, width, FOOTPRINT_EDGE_SPACING)) {
    if (!isDrivable(p[0], p[1])) return p;
  }
  return null;
}
