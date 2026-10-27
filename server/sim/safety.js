// Independent safety checker. It observes commands and vehicle motion and records events.
// It never modifies commands or vehicle state; the simulation only reads `terminated` to stop.
//
// Recorded events (type / severity):
//   collision           violation, terminal  - ego footprint touches another vehicle at any point of the motion
//   road_departure      violation, terminal  - any ego footprint boundary point leaves the drivable surface
//   speed_violation     violation            - speed above the local limit (+ tolerance); one event per episode
//   speed_violation_end info                 - end of a speeding episode (duration, max excess)
//   invalid_output      violation            - finite command outside the valid command range (actuator saturates)
//   invalid_output      violation, terminal  - non-finite / missing command, or controller failure / timeout
//   invalid_output_end  info                 - end of a run of out-of-range commands
//   traffic_collision   info                 - two surrounding vehicles touched (not caused by the checker's subject)
//   arrival             success, terminal    - ego centre inside the destination polygon
//   timeout             warning, terminal    - time limit reached

import { vehicleCorners, convexOverlap, pointInConvex, footprintSamples, wrapAngle, clamp } from './geometry.js';
import { isDrivable, speedLimitAt } from './road.js';
import { EGO_LIMITS, SPEED_TOLERANCE, SWEEP_SPACING, FOOTPRINT_EDGE_SPACING } from './constants.js';

const MAX_SUBSTEPS = 400;

function lerpPose(a, b, f) {
  return {
    x: a.x + (b.x - a.x) * f,
    y: a.y + (b.y - a.y) * f,
    heading: a.heading + wrapAngle(b.heading - a.heading) * f,
    length: a.length,
    width: a.width,
  };
}

const halfDiag = (p) => Math.hypot(p.length, p.width) / 2;
const corners = (p) => vehicleCorners(p.x, p.y, p.heading, p.length, p.width);

/** Displacement bound of any footprint point between two poses. */
function motionExtent(a, b) {
  return Math.hypot(b.x - a.x, b.y - a.y) + Math.abs(wrapAngle(b.heading - a.heading)) * halfDiag(a);
}

/**
 * Earliest fraction f in [0, 1] of the step at which the two moving footprints overlap, or null.
 * Poses are interpolated exactly as the integrator moves them (straight centre path, linear heading);
 * consecutive samples are at most `spacing` metres of relative motion apart.
 */
export function sweptOverlap(aPrev, aCur, bPrev, bCur, spacing = SWEEP_SPACING) {
  const rel = Math.hypot((aCur.x - aPrev.x) - (bCur.x - bPrev.x), (aCur.y - aPrev.y) - (bCur.y - bPrev.y)) +
    Math.abs(wrapAngle(aCur.heading - aPrev.heading)) * halfDiag(aPrev) +
    Math.abs(wrapAngle(bCur.heading - bPrev.heading)) * halfDiag(bPrev);
  if (Math.hypot(aPrev.x - bPrev.x, aPrev.y - bPrev.y) - rel > halfDiag(aPrev) + halfDiag(bPrev)) return null;
  const n = clamp(Math.ceil(rel / spacing), 1, MAX_SUBSTEPS);
  for (let k = 0; k <= n; k++) {
    const f = k / n;
    if (convexOverlap(corners(lerpPose(aPrev, aCur, f)), corners(lerpPose(bPrev, bCur, f)))) return f;
  }
  return null;
}

/** Earliest fraction of the step at which the footprint leaves the road, with the offending point. */
export function sweptDeparture(prev, cur, spacing = SWEEP_SPACING) {
  const n = clamp(Math.ceil(motionExtent(prev, cur) / spacing), 1, MAX_SUBSTEPS);
  for (let k = 0; k <= n; k++) {
    const f = k / n;
    const p = lerpPose(prev, cur, f);
    for (const pt of footprintSamples(p.x, p.y, p.heading, p.length, p.width, FOOTPRINT_EDGE_SPACING)) {
      if (!isDrivable(pt[0], pt[1])) return { f, point: pt, pose: p };
    }
  }
  return null;
}

export class SafetyChecker {
  /**
   * @param {object} o
   * @param {{polygon:number[][]}} o.destination
   * @param {number} o.timeLimit
   * @param {object} [o.limits] valid command range (EGO_LIMITS)
   * @param {(e:object)=>void} [o.onEvent]
   */
  constructor({ destination, timeLimit, limits = EGO_LIMITS, onEvent = null }) {
    this.destination = destination;
    this.timeLimit = timeLimit;
    this.limits = limits;
    this.onEvent = onEvent;
    this.events = [];
    this.outcome = null;
    this.speeding = null;
    this.invalidRun = null;
    this.trafficPairs = new Set();
    this.stats = { speedViolations: 0, speedingTime: 0, maxOverspeed: 0, invalidOutputs: 0, trafficCollisions: 0 };
  }

  get terminated() {
    return this.outcome !== null;
  }

  _emit(t, step, type, severity, message, data = {}, terminal = false) {
    const e = Object.freeze({ t: Math.round(t * 1e6) / 1e6, step, type, severity, terminal, message, data: Object.freeze(data) });
    this.events.push(e);
    this.onEvent?.(e);
    return e;
  }

  _terminate(status, t, step, message) {
    if (this.speeding) this._closeSpeeding(t, step, false);
    if (this.invalidRun) this.invalidRun = null;
    this.outcome = Object.freeze({ status, success: status === 'arrived', t, step, message, stats: Object.freeze({ ...this.stats }) });
  }

  /** Inspect a raw controller command. Called once per control step, before it is applied. */
  checkCommand(cmd, t, step) {
    if (this.outcome) return;
    if (!cmd || typeof cmd !== 'object' || cmd.error) {
      const msg = `controller failure: ${cmd?.error ?? 'no command returned'}`;
      this.stats.invalidOutputs++;
      this._emit(t, step, 'invalid_output', 'violation', msg, { error: String(cmd?.error ?? 'missing') }, true);
      this._terminate('invalid_output', t, step, msg);
      return;
    }
    const { accel, steer } = cmd;
    if (typeof accel !== 'number' || typeof steer !== 'number' || !Number.isFinite(accel) || !Number.isFinite(steer)) {
      const msg = `non-finite command (accel=${accel}, steer=${steer})`;
      this.stats.invalidOutputs++;
      this._emit(t, step, 'invalid_output', 'violation', msg, { accel: String(accel), steer: String(steer) }, true);
      this._terminate('invalid_output', t, step, msg);
      return;
    }
    const L = this.limits;
    const problems = [];
    if (accel < L.accelMin || accel > L.accelMax) problems.push(`accel ${accel.toFixed(3)} outside [${L.accelMin}, ${L.accelMax}]`);
    if (Math.abs(steer) > L.steerMax) problems.push(`steer ${steer.toFixed(3)} outside [${-L.steerMax}, ${L.steerMax}]`);
    if (problems.length) {
      this.stats.invalidOutputs++;
      if (!this.invalidRun) {
        this.invalidRun = { start: t, count: 0 };
        this._emit(t, step, 'invalid_output', 'violation', `command out of range: ${problems.join('; ')} (actuator saturates)`, { accel, steer });
      }
      this.invalidRun.count++;
    } else if (this.invalidRun) {
      this._emit(t, step, 'invalid_output_end', 'info', `commands back in range after ${this.invalidRun.count} invalid command(s)`, { count: this.invalidRun.count });
      this.invalidRun = null;
    }
  }

  /**
   * Inspect the motion over one physics step.
   * prev/cur: {t, ego:{x,y,heading,speed,length,width}, npcs:[{id,x,y,heading,speed,length,width}]}
   */
  checkStep(prev, cur, step) {
    if (this.outcome) return;
    const t = cur.t, dt = cur.t - prev.t;

    // Ego collisions, swept over the step.
    const prevById = new Map(prev.npcs.map((n) => [n.id, n]));
    let hit = null;
    for (const n of cur.npcs) {
      const p = prevById.get(n.id) ?? n;
      const f = sweptOverlap(prev.ego, cur.ego, p, n);
      if (f !== null && (!hit || f < hit.f || (f === hit.f && n.id < hit.other.id))) hit = { f, other: n, otherPrev: p };
    }
    const dep = sweptDeparture(prev.ego, cur.ego);
    this._checkTraffic(prev, cur, prevById, step);

    if (hit && (!dep || hit.f <= dep.f)) {
      const tc = prev.t + hit.f * dt;
      const ep = lerpPose(prev.ego, cur.ego, hit.f);
      const msg = `collision with vehicle ${hit.other.id} at ${tc.toFixed(2)} s (ego ${cur.ego.speed.toFixed(1)} m/s, other ${hit.other.speed.toFixed(1)} m/s)`;
      this._emit(tc, step, 'collision', 'violation', msg, { otherId: hit.other.id, x: ep.x, y: ep.y, egoSpeed: cur.ego.speed, otherSpeed: hit.other.speed }, true);
      this._terminate('collision', tc, step, msg);
      return;
    }
    if (dep) {
      const td = prev.t + dep.f * dt;
      const msg = `road departure at ${td.toFixed(2)} s near (${dep.point[0].toFixed(1)}, ${dep.point[1].toFixed(1)})`;
      this._emit(td, step, 'road_departure', 'violation', msg, { x: dep.point[0], y: dep.point[1] }, true);
      this._terminate('road_departure', td, step, msg);
      return;
    }

    // Speed limit.
    const limit = speedLimitAt(cur.ego.x, cur.ego.y);
    const v = cur.ego.speed;
    if (limit !== null && v > limit + SPEED_TOLERANCE) {
      if (!this.speeding) {
        this.speeding = { start: t, limit, max: v - limit };
        this.stats.speedViolations++;
        this._emit(t, step, 'speed_violation', 'violation', `speed ${v.toFixed(1)} m/s exceeds limit ${limit} m/s`, { speed: v, limit });
      } else {
        this.speeding.max = Math.max(this.speeding.max, v - limit);
      }
      this.stats.maxOverspeed = Math.max(this.stats.maxOverspeed, v - limit);
    } else if (this.speeding) {
      this._closeSpeeding(t, step, true);
    }

    if (pointInConvex(this.destination.polygon, cur.ego.x, cur.ego.y)) {
      const msg = `arrived at destination at ${t.toFixed(2)} s`;
      this._emit(t, step, 'arrival', 'success', msg, { x: cur.ego.x, y: cur.ego.y }, true);
      this._terminate('arrived', t, step, msg);
      return;
    }
    if (t >= this.timeLimit - 1e-9) {
      const msg = `time limit of ${this.timeLimit} s reached`;
      this._emit(t, step, 'timeout', 'warning', msg, {}, true);
      this._terminate('timeout', t, step, msg);
    }
  }

  _closeSpeeding(t, step, emit) {
    const s = this.speeding;
    this.stats.speedingTime += t - s.start;
    if (emit) this._emit(t, step, 'speed_violation_end', 'info', `speeding ended after ${(t - s.start).toFixed(2)} s (max ${s.max.toFixed(1)} m/s over)`, { duration: t - s.start, maxOver: s.max });
    this.speeding = null;
  }

  _checkTraffic(prev, cur, prevById, step) {
    const sorted = [...cur.npcs].sort((a, b) => a.x - b.x || a.id - b.id);
    for (let i = 0; i < sorted.length; i++) {
      const a = sorted[i];
      for (let j = i + 1; j < sorted.length && sorted[j].x - a.x < 30; j++) {
        const b = sorted[j];
        if (Math.abs(a.y - b.y) > 6) continue;
        const key = a.id < b.id ? `${a.id}-${b.id}` : `${b.id}-${a.id}`;
        if (this.trafficPairs.has(key)) continue;
        const f = sweptOverlap(prevById.get(a.id) ?? a, a, prevById.get(b.id) ?? b, b);
        if (f !== null) {
          this.trafficPairs.add(key);
          this.stats.trafficCollisions++;
          const tc = prev.t + f * (cur.t - prev.t);
          this._emit(tc, step, 'traffic_collision', 'info', `surrounding vehicles ${a.id} and ${b.id} collided`, { a: a.id, b: b.id, x: a.x, y: a.y });
        }
      }
    }
  }
}
