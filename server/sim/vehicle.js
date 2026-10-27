// Kinematic bicycle model referenced at the body centre, with bounded acceleration,
// bounded steering angle (speed dependent, see latAccelMax) and bounded steering rate.

import { clamp, wrapAngle } from './geometry.js';

export function createVehicle(spec) {
  return {
    id: spec.id,
    kind: spec.kind,
    x: spec.x,
    y: spec.y,
    heading: spec.heading ?? 0,
    speed: spec.speed ?? 0,
    accel: 0,
    steer: spec.steer ?? 0,
    length: spec.length,
    width: spec.width,
    wheelbase: spec.wheelbase,
    limits: spec.limits,
  };
}

/** Largest steering angle the tyres can use at this speed. */
export function effectiveSteerMax(limits, wheelbase, speed) {
  const v2 = speed * speed;
  if (v2 < 1e-9) return limits.steerMax;
  return Math.min(limits.steerMax, Math.atan((limits.latAccelMax * wheelbase) / v2));
}

/**
 * Advance one vehicle by dt under (accelCmd, steerCmd). Commands outside the physical limits are
 * saturated by the actuator model. Mutates v and returns the applied {accel, steer}.
 * Within a step the centre moves on a straight segment while the heading changes linearly,
 * which is exactly what the safety checker interpolates when sweeping.
 */
export function stepVehicle(v, accelCmd, steerCmd, dt) {
  const L = v.limits;
  const a = clamp(accelCmd, L.accelMin, L.accelMax);

  const steerLimit = effectiveSteerMax(L, v.wheelbase, v.speed);
  const target = clamp(steerCmd, -steerLimit, steerLimit);
  const maxDelta = L.steerRateMax * dt;
  v.steer = clamp(v.steer + clamp(target - v.steer, -maxDelta, maxDelta), -L.steerMax, L.steerMax);

  const v0 = v.speed;
  const v1 = clamp(v0 + a * dt, 0, L.speedMax);
  const vAvg = 0.5 * (v0 + v1);
  const lr = v.wheelbase / 2;
  const beta = Math.atan((lr / v.wheelbase) * Math.tan(v.steer));

  v.x += vAvg * Math.cos(v.heading + beta) * dt;
  v.y += vAvg * Math.sin(v.heading + beta) * dt;
  v.heading = wrapAngle(v.heading + (vAvg / lr) * Math.sin(beta) * dt);
  v.speed = v1;
  v.accel = (v1 - v0) / dt;
  return { accel: v.accel, steer: v.steer };
}
