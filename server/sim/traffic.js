// Surrounding-traffic behaviour:
//  - longitudinal: Intelligent Driver Model (IDM) against the nearest vehicle ahead in any lane the
//    vehicle occupies or is moving into (the ego counts like any other vehicle);
//  - lane changes: MOBIL (incentive + safety criterion for the new follower, keep-right bias,
//    courtesy toward a vehicle merging from the acceleration lane);
//  - lateral: heading-based lane tracking through the same bounded bicycle model as the ego;
//  - scripted overrides from scenario events (forced lane change, hard braking).

import { CAR_LIMITS, TRUCK_LIMITS, DIMS } from './constants.js';
import { clamp, wrapAngle } from './geometry.js';
import { createVehicle } from './vehicle.js';
import { NUM_LANES, laneCenterY, lanesOccupied, ACCEL_LANE, HIGHWAY_X_MIN } from './road.js';
import { requiredGap } from './validate.js';

export const IDM = Object.freeze({ s0: 2.0, delta: 4, defaultT: 1.4, carA: 1.4, truckA: 0.8, b: 2.0 });
export const MOBIL = Object.freeze({
  threshold: 0.2,
  keepRightBias: 0.25,
  bSafe: 4.0,
  minGap: 2.0,
  decisionPeriodSteps: 25,
  cooldown: 4.0,
  mergeCourtesy: 0.6,
});
export const LATERAL = Object.freeze({ kY: 0.6, maxHeading: 0.07, kHeading: 2.0, doneTolerance: 0.15 });

export function createNpc(spec) {
  const kind = spec.kind ?? 'car';
  const v = createVehicle({ ...spec, kind, limits: kind === 'truck' ? TRUCK_LIMITS : CAR_LIMITS });
  return Object.assign(v, {
    lane: spec.lane,
    targetLane: null,
    desiredSpeed: spec.desiredSpeed,
    laneChange: spec.laneChange !== false,
    idm: { T: spec.idmT ?? IDM.defaultT, a: kind === 'truck' ? IDM.truckA : IDM.carA, b: IDM.b },
    politeness: spec.politeness ?? 0.3,
    cooldownUntil: 0,
    scripted: null,
    forced: false,
  });
}

/** IDM acceleration for vehicle v given leader info {gap, speed} (or null for free road). */
export function idmAccel(v, lead) {
  // Vehicles without IDM parameters (the ego) are modelled as wanting to keep their current speed.
  const p = v.idm ?? { T: IDM.defaultT, a: IDM.carA, b: IDM.b };
  const v0 = Math.max(1, v.desiredSpeed ?? v.speed + 1);
  const free = 1 - Math.pow(v.speed / v0, IDM.delta);
  if (!lead) return p.a * free;
  const dv = v.speed - lead.speed;
  const sStar = IDM.s0 + Math.max(0, v.speed * p.T + (v.speed * dv) / (2 * Math.sqrt(p.a * p.b)));
  const s = Math.max(lead.gap, 0.1);
  return p.a * (free - (sStar / s) ** 2);
}

const gapBetween = (rear, front) => front.x - rear.x - (rear.length + front.length) / 2;
const isAhead = (o, v) => o.x > v.x || (o.x === v.x && o.id > v.id);

/** Lanes a vehicle occupies physically, plus the lane it is moving into (like a turn signal). */
export function lanesClaimed(v) {
  const lanes = lanesOccupied(v);
  if (v.targetLane != null && !lanes.includes(v.targetLane)) lanes.push(v.targetLane);
  return lanes;
}

/** Per-lane lists of vehicles claiming each lane, sorted by x then id. */
export function buildLaneIndex(vehicles) {
  const lanes = Array.from({ length: NUM_LANES }, () => []);
  for (const v of vehicles) for (const k of lanesClaimed(v)) lanes[k].push(v);
  for (const l of lanes) l.sort((a, b) => a.x - b.x || a.id - b.id);
  return lanes;
}

export function leaderIn(list, v) {
  let best = null;
  for (const o of list) if (o !== v && isAhead(o, v) && (!best || o.x < best.x)) best = o;
  return best ? { veh: best, gap: gapBetween(v, best), speed: best.speed } : null;
}

export function followerIn(list, v) {
  let best = null;
  for (const o of list) if (o !== v && !isAhead(o, v) && (!best || o.x > best.x)) best = o;
  return best ? { veh: best, gap: gapBetween(best, v), speed: best.speed } : null;
}

function leaderAcrossLanes(v, lanes, laneIndex) {
  let lead = null;
  for (const k of lanes) {
    const l = leaderIn(laneIndex[k], v);
    if (l && (!lead || l.gap < lead.gap)) lead = l;
  }
  return lead;
}

function applyScript(npc, aIdm, t) {
  const s = npc.scripted;
  if (!s) return aIdm;
  if (s.mode === 'brake') {
    if (npc.speed > s.targetSpeed + 0.05) return Math.min(-s.decel, aIdm);
    s.mode = 'hold';
    s.holdUntil = t + s.hold;
  }
  if (t >= s.holdUntil) { npc.scripted = null; return aIdm; }
  const a = s.targetSpeed <= 0.05 ? -s.decel : clamp(s.targetSpeed - npc.speed, -s.decel, npc.idm.a);
  return Math.min(a, aIdm);
}

function mergePressure(npc, ego) {
  if (!ego) return false;
  const onAccelLane = ego.y < 0 && ego.x >= ACCEL_LANE.x0 - 20 && ego.x <= ACCEL_LANE.taperEnd;
  return onAccelLane && Math.abs(ego.x - npc.x) < 50;
}

function decideLaneChange(npc, aCur, ctx) {
  let best = null, bestGain = MOBIL.threshold;
  for (const k of [npc.lane - 1, npc.lane + 1]) {
    if (k < 0 || k >= NUM_LANES) continue;
    const list = ctx.laneIndex[k];
    const lead = leaderIn(list, npc), fol = followerIn(list, npc);
    if ((lead && lead.gap < MOBIL.minGap) || (fol && fol.gap < MOBIL.minGap)) continue;
    const aNew = idmAccel(npc, lead);
    let folGain = 0;
    if (fol) {
      const aFolNew = idmAccel(fol.veh, { gap: fol.gap, speed: npc.speed });
      if (aFolNew < -MOBIL.bSafe) continue;
      folGain = aFolNew - idmAccel(fol.veh, leaderIn(list, fol.veh));
    }
    let gain = aNew - aCur + npc.politeness * folGain;
    if (k < npc.lane) gain += MOBIL.keepRightBias;
    if (npc.lane === 0 && k === 1 && mergePressure(npc, ctx.ego)) gain += MOBIL.mergeCourtesy;
    if (gain > bestGain) { bestGain = gain; best = k; }
  }
  if (best !== null) npc.targetLane = best;
}

function lateralSteer(npc) {
  const lane = npc.targetLane ?? npc.lane;
  const ey = laneCenterY(lane) - npc.y;
  const psiD = clamp(Math.atan2(LATERAL.kY * ey, Math.max(npc.speed, 3)), -LATERAL.maxHeading, LATERAL.maxHeading);
  const yawRate = LATERAL.kHeading * wrapAngle(psiD - npc.heading);
  return Math.atan((yawRate * npc.wheelbase) / Math.max(npc.speed, 1));
}

/**
 * Compute {accel, steer} for every NPC from the current (pre-step) state. Lane-change decisions
 * are stored on the NPC. Returns Map id -> command.
 * ctx: {t, step, ego, npcs, laneIndex}
 */
export function computeNpcCommands(ctx) {
  const cmds = new Map();
  for (const npc of ctx.npcs) {
    if (npc.targetLane !== null && Math.abs(laneCenterY(npc.targetLane) - npc.y) < LATERAL.doneTolerance && Math.abs(npc.heading) < 0.02) {
      npc.lane = npc.targetLane;
      npc.targetLane = null;
      npc.cooldownUntil = ctx.t + MOBIL.cooldown;
    }
    const lanes = new Set(lanesOccupied(npc));
    lanes.add(npc.lane);
    if (npc.targetLane !== null) lanes.add(npc.targetLane);
    const aIdm = idmAccel(npc, leaderAcrossLanes(npc, lanes, ctx.laneIndex));
    const accel = applyScript(npc, aIdm, ctx.t);
    if (npc.targetLane === null && npc.laneChange && !npc.forced && !npc.scripted && ctx.t >= npc.cooldownUntil &&
        (ctx.step + npc.id * 7) % MOBIL.decisionPeriodSteps === 0) {
      decideLaneChange(npc, aIdm, ctx);
    }
    cmds.set(npc.id, { accel, steer: lateralSteer(npc) });
  }
  return cmds;
}

/** Random background vehicle for a lane. Always consumes the same number of random draws. */
export function randomNpcSpec(rng, { id, lane, x, laneSpeed, truckProb = 0 }) {
  const truckDraw = rng.next();
  const truck = lane === 0 && truckDraw < truckProb;
  const dims = truck ? DIMS.truck : DIMS.car;
  const d1 = rng.uniform(-2, 2.5), d2 = rng.uniform(-1, 1), d3 = rng.uniform(1.1, 1.7), d4 = rng.uniform(0.1, 0.5);
  const desiredSpeed = truck ? Math.min(laneSpeed, 24) - Math.abs(d1) : laneSpeed + d1;
  const speed = clamp(Math.min(desiredSpeed, laneSpeed) + d2, 0, 40);
  return {
    id, kind: truck ? 'truck' : 'car', x, y: laneCenterY(lane), heading: 0, speed, lane, desiredSpeed,
    length: dims.length, width: dims.width, wheelbase: dims.wheelbase, laneChange: true, idmT: d3, politeness: d4,
  };
}

export const SPAWN_X = HIGHWAY_X_MIN + 15;

/** Upstream inflow: spawn a vehicle in lane k when its spawn time has come and there is room. */
export function spawnTraffic(sim) {
  const tr = sim.traffic;
  for (let k = 0; k < NUM_LANES; k++) {
    if (!(tr.inflow[k] > 0) || sim.t < tr.nextSpawn[k]) continue;
    let rear = null;
    for (const v of [sim.ego, ...sim.npcs]) {
      if (lanesClaimed(v).includes(k) && (!rear || v.x < rear.x)) rear = v;
    }
    const speed = Math.min(tr.laneSpeeds[k], rear ? rear.speed : Infinity);
    if (rear && rear.x - SPAWN_X - (rear.length + DIMS.truck.length) / 2 < requiredGap(speed, rear.speed) + 0.6 * speed) {
      tr.nextSpawn[k] = sim.t + 0.5;
      continue;
    }
    const spec = randomNpcSpec(sim.rng, { id: sim.nextId++, lane: k, x: SPAWN_X, laneSpeed: tr.laneSpeeds[k], truckProb: 0.06 });
    spec.speed = Math.min(spec.speed, speed);
    sim.npcs.push(createNpc(spec));
    tr.nextSpawn[k] = sim.t + Math.max(0.5, sim.rng.exp(1 / tr.inflow[k]));
  }
}
