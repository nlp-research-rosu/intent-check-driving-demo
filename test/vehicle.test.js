import { test } from 'node:test';
import assert from 'node:assert/strict';

import { createVehicle, stepVehicle, effectiveSteerMax } from '../server/sim/vehicle.js';
import { EGO_LIMITS, PHYS_DT, DIMS } from '../server/sim/constants.js';
import { convexOverlap, vehicleCorners, pointInConvex, wrapAngle } from '../server/sim/geometry.js';

const ego = (over = {}) => createVehicle({ id: 0, kind: 'ego', x: 0, y: 0, heading: 0, speed: 20, ...DIMS.ego, limits: EGO_LIMITS, ...over });

test('zero command keeps speed and heading on a straight line', () => {
  const v = ego();
  for (let i = 0; i < 500; i++) stepVehicle(v, 0, 0, PHYS_DT);
  assert.equal(v.speed, 20);
  assert.equal(v.heading, 0);
  assert.equal(v.y, 0);
  assert.ok(Math.abs(v.x - 200) < 1e-9);
});

test('acceleration is saturated to the physical limits', () => {
  const v = ego();
  const r = stepVehicle(v, 100, 0, PHYS_DT);
  assert.ok(Math.abs(r.accel - EGO_LIMITS.accelMax) < 1e-9);
  const w = ego();
  const r2 = stepVehicle(w, -100, 0, PHYS_DT);
  assert.ok(Math.abs(r2.accel - EGO_LIMITS.accelMin) < 1e-9);
});

test('braking stops the vehicle without reversing', () => {
  const v = ego({ speed: 5 });
  for (let i = 0; i < 200; i++) stepVehicle(v, EGO_LIMITS.accelMin, 0, PHYS_DT);
  assert.equal(v.speed, 0);
  const x = v.x;
  stepVehicle(v, EGO_LIMITS.accelMin, 0, PHYS_DT);
  assert.equal(v.x, x);
});

test('speed never exceeds speedMax', () => {
  const v = ego({ speed: 44.99 });
  for (let i = 0; i < 100; i++) stepVehicle(v, EGO_LIMITS.accelMax, 0, PHYS_DT);
  assert.equal(v.speed, EGO_LIMITS.speedMax);
});

test('steering angle is rate limited and bounded', () => {
  const v = ego({ speed: 1 });
  stepVehicle(v, 0, 10, PHYS_DT);
  assert.ok(Math.abs(v.steer - EGO_LIMITS.steerRateMax * PHYS_DT) < 1e-12);
  for (let i = 0; i < 500; i++) stepVehicle(v, 0, 10, PHYS_DT);
  assert.ok(v.steer <= EGO_LIMITS.steerMax + 1e-12);
});

test('usable steering shrinks with speed so lateral acceleration stays bounded', () => {
  const v = ego({ speed: 30 });
  for (let i = 0; i < 200; i++) stepVehicle(v, 0, 0.5, PHYS_DT);
  const yawRate = (v.speed / v.wheelbase) * Math.tan(v.steer);
  assert.ok(v.speed * yawRate <= EGO_LIMITS.latAccelMax * 1.01, `lateral accel ${v.speed * yawRate}`);
  assert.ok(effectiveSteerMax(EGO_LIMITS, 2.85, 30) < effectiveSteerMax(EGO_LIMITS, 2.85, 10));
});

test('positive steer turns left (counter-clockwise)', () => {
  const v = ego({ speed: 10 });
  for (let i = 0; i < 100; i++) stepVehicle(v, 0, 0.1, PHYS_DT);
  assert.ok(v.heading > 0);
  assert.ok(v.y > 0);
});

test('geometry helpers', () => {
  const a = vehicleCorners(0, 0, 0, 4, 2);
  assert.ok(convexOverlap(a, vehicleCorners(3.9, 0, 0, 4, 2)));
  assert.ok(!convexOverlap(a, vehicleCorners(4.1, 0, 0, 4, 2)));
  assert.ok(!convexOverlap(a, vehicleCorners(0, 2.1, 0, 4, 2)));
  assert.ok(convexOverlap(a, vehicleCorners(2.5, 1.5, Math.PI / 4, 4, 2)));
  assert.ok(pointInConvex(a, 0, 0));
  assert.ok(!pointInConvex(a, 3, 0));
  assert.ok(Math.abs(wrapAngle(3 * Math.PI) - Math.PI) < 1e-12 || Math.abs(wrapAngle(3 * Math.PI) + Math.PI) < 1e-12);
});
