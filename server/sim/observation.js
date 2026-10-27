// Builds the controller input for one control step.

import { CONTROL_DT, EGO_LIMITS, SENSOR_RANGE, MAX_OBJECTS } from './constants.js';
import { effectiveSteerMax } from './vehicle.js';
import { ROAD } from './road.js';

export function buildObservation(sim) {
  const e = sim.ego;
  const seen = [];
  for (const n of sim.npcs) {
    const d = Math.hypot(n.x - e.x, n.y - e.y);
    if (d <= SENSOR_RANGE) seen.push({ d, n });
  }
  seen.sort((a, b) => a.d - b.d || a.n.id - b.n.id);
  return {
    time: sim.t,
    step: sim.stepCount,
    controlDt: CONTROL_DT,
    ego: {
      x: e.x, y: e.y, heading: e.heading, speed: e.speed, accel: e.accel, steer: e.steer,
      length: e.length, width: e.width, wheelbase: e.wheelbase,
    },
    steerMaxNow: effectiveSteerMax(e.limits, e.wheelbase, e.speed),
    objects: seen.slice(0, MAX_OBJECTS).map(({ n }) => ({
      id: n.id, x: n.x, y: n.y, heading: n.heading, speed: n.speed, accel: n.accel, length: n.length, width: n.width,
    })),
    road: ROAD,
    destination: sim.destination,
    limits: EGO_LIMITS,
  };
}
