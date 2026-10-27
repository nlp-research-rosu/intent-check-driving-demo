// Validation of starting configurations. A configuration is rejected (never silently repaired)
// if any rule below fails. See docs/ASSUMPTIONS.md for the rationale and ranges.

import { VALIDATION, EGO_LIMITS } from './constants.js';
import { vehicleCorners, convexOverlap, isConvex, polygonArea, polygonCentroid } from './geometry.js';
import { lanesOccupied, footprintOffRoad, laneCenterY, isDrivable, NUM_LANES } from './road.js';
import { isValidSeed } from './rng.js';

const isNum = (v) => typeof v === 'number' && Number.isFinite(v);

/**
 * Minimum bumper-to-bumper gap so that a follower at vf, reacting after VALIDATION.reactionTime and
 * braking at VALIDATION.brakeDecel, can avoid a leader at vl that brakes equally hard.
 */
export function requiredGap(vf, vl) {
  const closing = Math.max(0, vf * vf - vl * vl) / (2 * VALIDATION.brakeDecel);
  return VALIDATION.minGap + VALIDATION.reactionTime * vf + closing;
}

const nameOf = (v) => (v.id === 0 ? 'ego' : `vehicle ${v.id}`);

function inflatedCorners(v, m) {
  return vehicleCorners(v.x, v.y, v.heading ?? 0, v.length + 2 * m, v.width + 2 * m);
}

/** Describes why vehicles a and b may not start in this arrangement, or null if they may. */
export function pairConflict(a, b) {
  const reach = (Math.hypot(a.length, a.width) + Math.hypot(b.length, b.width)) / 2 + 2 * VALIDATION.overlapMargin;
  const far = Math.hypot(a.x - b.x, a.y - b.y) > reach;
  if (!far && convexOverlap(inflatedCorners(a, VALIDATION.overlapMargin), inflatedCorners(b, VALIDATION.overlapMargin))) {
    return `${nameOf(a)} and ${nameOf(b)} overlap (within ${VALIDATION.overlapMargin} m margin)`;
  }
  const la = lanesOccupied(a), lb = lanesOccupied(b);
  if (!la.some((k) => lb.includes(k))) return null;
  const [rear, front] = a.x <= b.x ? [a, b] : [b, a];
  const gap = front.x - rear.x - (rear.length + front.length) / 2;
  const need = requiredGap(rear.speed, front.speed);
  if (gap < need) {
    return `${nameOf(rear)} is ${gap.toFixed(1)} m behind ${nameOf(front)} in the same lane; at ${rear.speed.toFixed(1)} -> ${front.speed.toFixed(1)} m/s at least ${need.toFixed(1)} m is required`;
  }
  return null;
}

function checkRange(errors, label, v, lo, hi) {
  if (!isNum(v)) errors.push(`${label} must be a finite number`);
  else if (v < lo || v > hi) errors.push(`${label} = ${v} is outside [${lo}, ${hi}]`);
}

function checkDims(errors, label, v, kind) {
  const ranges = kind === 'truck'
    ? { length: [6, 20], width: [2.0, 2.6] }
    : { length: [3, 6], width: [1.5, 2.2] };
  checkRange(errors, `${label}.length`, v.length, ...ranges.length);
  checkRange(errors, `${label}.width`, v.width, ...ranges.width);
  checkRange(errors, `${label}.wheelbase`, v.wheelbase, 1.5, isNum(v.length) ? v.length * 0.9 : 0);
}

export const EVENT_TYPES = Object.freeze(['force_lane_change', 'brake']);

/** Validate a complete scenario configuration. Returns {ok, errors}. */
export function validateConfig(cfg) {
  const errors = [];
  if (!cfg || typeof cfg !== 'object') return { ok: false, errors: ['configuration must be an object'] };
  if (!isValidSeed(cfg.seed)) errors.push(`seed must be an integer in [0, 4294967295] (got ${cfg.seed})`);
  checkRange(errors, 'timeLimit', cfg.timeLimit, VALIDATION.timeLimitMin, VALIDATION.timeLimitMax);

  // Ego vehicle.
  const ego = cfg.ego;
  if (!ego || typeof ego !== 'object') {
    errors.push('ego vehicle is missing');
  } else {
    for (const k of ['x', 'y', 'heading', 'speed']) if (!isNum(ego[k])) errors.push(`ego.${k} must be a finite number`);
    checkDims(errors, 'ego', ego, 'car');
    checkRange(errors, 'ego.speed', ego.speed, 0, EGO_LIMITS.speedMax);
    if (isNum(ego.x) && isNum(ego.y) && isNum(ego.heading) && isNum(ego.length) && isNum(ego.width)) {
      const off = footprintOffRoad(ego.x, ego.y, ego.heading, ego.length, ego.width);
      if (off) errors.push(`ego footprint is off the road near (${off[0].toFixed(1)}, ${off[1].toFixed(1)})`);
    }
  }

  // Surrounding vehicles.
  const npcs = Array.isArray(cfg.npcs) ? cfg.npcs : null;
  if (!npcs) errors.push('npcs must be an array');
  else if (npcs.length > VALIDATION.maxVehicles) errors.push(`too many vehicles (${npcs.length} > ${VALIDATION.maxVehicles})`);
  const ids = new Set();
  const npcErrorsBefore = errors.length;
  for (const [i, n] of (npcs ?? []).entries()) {
    const label = `npc[${i}]`;
    if (!n || typeof n !== 'object') { errors.push(`${label} must be an object`); continue; }
    if (!Number.isInteger(n.id) || n.id <= 0) errors.push(`${label}.id must be a positive integer`);
    else if (ids.has(n.id)) errors.push(`${label}.id ${n.id} is duplicated`);
    ids.add(n.id);
    if (n.kind !== 'car' && n.kind !== 'truck') errors.push(`${label}.kind must be 'car' or 'truck'`);
    for (const k of ['x', 'y', 'heading', 'speed', 'desiredSpeed']) if (!isNum(n[k])) errors.push(`${label}.${k} must be a finite number`);
    checkDims(errors, label, n, n.kind);
    checkRange(errors, `${label}.speed`, n.speed, 0, 40);
    checkRange(errors, `${label}.desiredSpeed`, n.desiredSpeed, 1, 40);
    checkRange(errors, `${label}.heading`, n.heading, -0.1, 0.1);
    if (n.idmT !== undefined) checkRange(errors, `${label}.idmT`, n.idmT, 0.8, 2.5);
    if (n.politeness !== undefined) checkRange(errors, `${label}.politeness`, n.politeness, 0, 1);
    if (!Number.isInteger(n.lane) || n.lane < 0 || n.lane >= NUM_LANES) errors.push(`${label}.lane must be an integer in [0, ${NUM_LANES - 1}]`);
    else if (isNum(n.y) && Math.abs(n.y - laneCenterY(n.lane)) > 0.6) errors.push(`${label} is not centred in lane ${n.lane} (|dy| > 0.6 m)`);
    if (isNum(n.x) && isNum(n.y) && isNum(n.heading) && isNum(n.length) && isNum(n.width)) {
      const off = footprintOffRoad(n.x, n.y, n.heading, n.length, n.width);
      if (off) errors.push(`${label} footprint is off the road`);
    }
    if (errors.length - npcErrorsBefore > 20) { errors.push('... further vehicle errors suppressed'); break; }
  }

  // Pairwise conflicts (only when every vehicle is individually valid).
  if (errors.length === 0) {
    const all = [{ ...ego, id: 0 }, ...npcs].sort((a, b) => a.x - b.x);
    let count = 0;
    for (let i = 0; i < all.length && count < 10; i++) {
      for (let j = i + 1; j < all.length && all[j].x - all[i].x < 400; j++) {
        const c = pairConflict(all[i], all[j]);
        if (c) { errors.push(c); if (++count >= 10) break; }
      }
    }
  }

  // Scripted scenario events.
  if (!Array.isArray(cfg.events)) errors.push('events must be an array');
  for (const [i, e] of (cfg.events ?? []).entries()) {
    const label = `event[${i}]`;
    if (!EVENT_TYPES.includes(e?.type)) { errors.push(`${label}.type must be one of ${EVENT_TYPES.join(', ')}`); continue; }
    checkRange(errors, `${label}.t`, e.t, 0, isNum(cfg.timeLimit) ? cfg.timeLimit : 0);
    const npc = (npcs ?? []).find((n) => n?.id === e.vehicleId);
    if (!npc) { errors.push(`${label} refers to unknown vehicle ${e.vehicleId}`); continue; }
    if (e.type === 'force_lane_change') {
      if (!Number.isInteger(e.targetLane) || Math.abs(e.targetLane - npc.lane) !== 1 || e.targetLane < 0 || e.targetLane >= NUM_LANES) {
        errors.push(`${label}.targetLane must be a lane adjacent to vehicle ${npc.id}'s lane ${npc.lane}`);
      }
    } else if (e.type === 'brake') {
      checkRange(errors, `${label}.decel`, e.decel, 0.5, 9);
      checkRange(errors, `${label}.targetSpeed`, e.targetSpeed, 0, 40);
      checkRange(errors, `${label}.hold`, e.hold, 0, 120);
    }
  }

  // Destination.
  const d = cfg.destination;
  if (!d || !Array.isArray(d.polygon) || d.polygon.length < 3 || d.polygon.length > 16 ||
      !d.polygon.every((p) => Array.isArray(p) && isNum(p[0]) && isNum(p[1]))) {
    errors.push('destination.polygon must have 3..16 finite [x, y] points');
  } else if (!isConvex(d.polygon) || Math.abs(polygonArea(d.polygon)) < 1) {
    errors.push('destination.polygon must be convex with area >= 1 m^2');
  } else {
    const [cx, cy] = polygonCentroid(d.polygon);
    if (!isDrivable(cx, cy)) errors.push('destination centre must be on the road');
  }

  // Background traffic inflow.
  const tr = cfg.traffic;
  if (!tr || !Array.isArray(tr.inflow) || !Array.isArray(tr.laneSpeeds) || tr.inflow.length !== NUM_LANES || tr.laneSpeeds.length !== NUM_LANES) {
    errors.push(`traffic.inflow and traffic.laneSpeeds must each have ${NUM_LANES} entries`);
  } else {
    tr.inflow.forEach((r, k) => checkRange(errors, `traffic.inflow[${k}]`, r, 0, 1));
    tr.laneSpeeds.forEach((s, k) => checkRange(errors, `traffic.laneSpeeds[${k}]`, s, 5, 40));
  }

  return { ok: errors.length === 0, errors };
}

/** Validate user-supplied scenario parameters against the scenario's declared ranges. */
export function validateParams(scenario, params) {
  const errors = [];
  const out = {};
  const given = params ?? {};
  if (typeof given !== 'object' || Array.isArray(given)) return { ok: false, errors: ['params must be an object'], params: null };
  for (const k of Object.keys(given)) {
    if (!scenario.params.some((p) => p.key === k)) errors.push(`unknown parameter '${k}' for scenario ${scenario.id}`);
  }
  for (const p of scenario.params) {
    const v = given[p.key] === undefined ? p.default : given[p.key];
    if (!isNum(v)) errors.push(`${p.key} must be a finite number`);
    else if (v < p.min || v > p.max) errors.push(`${p.key} = ${v} is outside [${p.min}, ${p.max}]`);
    else if (p.integer && !Number.isInteger(v)) errors.push(`${p.key} must be an integer`);
    out[p.key] = v;
  }
  return { ok: errors.length === 0, errors, params: out };
}

