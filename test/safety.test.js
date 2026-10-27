import { test } from 'node:test';
import assert from 'node:assert/strict';

import { SafetyChecker, sweptOverlap, sweptDeparture } from '../server/sim/safety.js';
import { EGO_LIMITS, DIMS, PHYS_DT } from '../server/sim/constants.js';
import { convexOverlap, vehicleCorners } from '../server/sim/geometry.js';
import { laneCenterY, footprintOffRoad, HIGHWAY_SPEED_LIMIT } from '../server/sim/road.js';

const DEST = { polygon: [[1000, 0], [1100, 0], [1100, 11.1], [1000, 11.1]], target: [1050, 5.55] };
const checker = (o = {}) => new SafetyChecker({ destination: DEST, timeLimit: 60, ...o });

const pose = (x, y, extra = {}) => ({ id: 0, x, y, heading: 0, speed: 20, length: DIMS.ego.length, width: DIMS.ego.width, ...extra });
const npc = (id, x, y, extra = {}) => ({ ...pose(x, y, extra), id, length: DIMS.car.length, width: DIMS.car.width, ...extra });
const snap = (t, ego, npcs = []) => ({ t, ego, npcs });
const deepFreeze = (o) => { Object.freeze(o); for (const v of Object.values(o)) if (v && typeof v === 'object') deepFreeze(v); return o; };

const L1 = laneCenterY(1);

test('no events for normal driving', () => {
  const c = checker();
  c.checkCommand({ accel: 0.5, steer: 0.01 }, 0, 0);
  c.checkStep(snap(0, pose(0, L1), [npc(1, 50, L1)]), snap(PHYS_DT, pose(0.4, L1), [npc(1, 50.4, L1)]), 1);
  assert.equal(c.events.length, 0);
  assert.equal(c.outcome, null);
});

test('detects a collision at the end of a step', () => {
  const c = checker();
  c.checkStep(snap(0, pose(0, L1), [npc(1, 4.75, L1, { speed: 0 })]), snap(PHYS_DT, pose(0.4, L1), [npc(1, 4.75, L1, { speed: 0 })]), 1);
  assert.equal(c.outcome.status, 'collision');
  const e = c.events.at(-1);
  assert.equal(e.type, 'collision');
  assert.equal(e.terminal, true);
  assert.equal(e.data.otherId, 1);
});

test('detects a collision that happens only in the middle of a step (no tunnelling)', () => {
  // Ego moves 20 m in one step straight through a stationary car; both endpoints are clear.
  const prev = snap(0, pose(0, L1, { speed: 1000 }), [npc(7, 10, L1, { speed: 0 })]);
  const cur = snap(PHYS_DT, pose(20, L1, { speed: 1000 }), [npc(7, 10, L1, { speed: 0 })]);
  const c0 = (p) => vehicleCorners(p.x, p.y, p.heading, p.length, p.width);
  assert.equal(convexOverlap(c0(prev.ego), c0(prev.npcs[0])), false);
  assert.equal(convexOverlap(c0(cur.ego), c0(cur.npcs[0])), false);
  const c = checker();
  c.checkStep(prev, cur, 1);
  assert.equal(c.outcome?.status, 'collision');
  const tc = c.events.at(-1).t;
  assert.ok(tc > 0 && tc < PHYS_DT, `contact time ${tc} should be inside the step`);
});

test('swept overlap reports the first contact fraction', () => {
  const f = sweptOverlap(pose(0, 0), pose(10, 0), npc(1, 10, 0, { speed: 0 }), npc(1, 10, 0, { speed: 0 }));
  // Contact when the gap of 10 - 4.65 = 5.35 m is closed, i.e. at ~53.5 % of the step (to sample resolution).
  assert.ok(f !== null && Math.abs(f - 0.535) < 0.011, `f = ${f}`);
});

test('vehicles side by side in adjacent lanes do not collide', () => {
  const c = checker();
  c.checkStep(snap(0, pose(0, laneCenterY(1)), [npc(1, 0, laneCenterY(0)), npc(2, 0, laneCenterY(2))]),
    snap(PHYS_DT, pose(0.4, laneCenterY(1)), [npc(1, 0.4, laneCenterY(0)), npc(2, 0.4, laneCenterY(2))]), 1);
  assert.equal(c.outcome, null);
});

test('detects leaving the road', () => {
  const c = checker();
  c.checkStep(snap(0, pose(0, 10.0)), snap(PHYS_DT, pose(0.4, 10.3)), 1);
  assert.equal(c.outcome.status, 'road_departure');
  assert.equal(c.events.at(-1).type, 'road_departure');
});

test('detects a departure that occurs only between step endpoints (gore area)', () => {
  // From the entrance ramp straight to lane 0 across the unpaved gore before the acceleration lane.
  const a = pose(120, -10.5, { heading: 0.4, length: 1, width: 1 });
  const b = pose(150, 1.5, { heading: 0.4, length: 1, width: 1 });
  assert.equal(footprintOffRoad(a.x, a.y, a.heading, a.length, a.width), null);
  assert.equal(footprintOffRoad(b.x, b.y, b.heading, b.length, b.width), null);
  assert.notEqual(sweptDeparture(a, b), null);
  const c = checker();
  c.checkStep(snap(0, a), snap(PHYS_DT, b), 1);
  assert.equal(c.outcome.status, 'road_departure');
});

test('collision earlier in the step takes priority over a later departure', () => {
  const c = checker();
  c.checkStep(snap(0, pose(0, 9.25, { speed: 500 }), [npc(3, 6, 9.25, { speed: 0 })]),
    snap(PHYS_DT, pose(10, 11.5, { speed: 500 }), [npc(3, 6, 9.25, { speed: 0 })]), 1);
  assert.equal(c.outcome.status, 'collision');
});

test('records speed violations as episodes without ending the run', () => {
  const c = checker();
  const lim = HIGHWAY_SPEED_LIMIT;
  let t = 0;
  const go = (v) => { c.checkStep(snap(t, pose(0, L1, { speed: v })), snap(t + PHYS_DT, pose(0.1, L1, { speed: v })), 1); t += PHYS_DT; };
  go(lim - 1);
  go(lim + 2);
  go(lim + 3);
  go(lim + 0.05); // within tolerance
  go(lim + 4);
  assert.equal(c.outcome, null);
  const types = c.events.map((e) => e.type);
  assert.deepEqual(types, ['speed_violation', 'speed_violation_end', 'speed_violation']);
  assert.equal(c.stats.speedViolations, 2);
  assert.ok(Math.abs(c.stats.maxOverspeed - 4) < 1e-9);
  assert.equal(c.events[0].severity, 'violation');
});

test('out-of-range commands are reported but do not end the run', () => {
  const c = checker();
  c.checkCommand({ accel: EGO_LIMITS.accelMax + 1, steer: 0 }, 0, 0);
  c.checkCommand({ accel: 0, steer: EGO_LIMITS.steerMax + 0.1 }, 0.1, 5);
  c.checkCommand({ accel: 0, steer: 0 }, 0.2, 10);
  assert.equal(c.outcome, null);
  assert.deepEqual(c.events.map((e) => e.type), ['invalid_output', 'invalid_output_end']);
  assert.equal(c.stats.invalidOutputs, 2);
});

for (const [name, cmd] of [
  ['NaN accel', { accel: NaN, steer: 0 }],
  ['infinite steer', { accel: 0, steer: -Infinity }],
  ['missing field', { accel: 0 }],
  ['string value', { accel: '1', steer: 0 }],
  ['controller error', { error: 'controller did not respond within 2000 ms' }],
  ['no command', null],
]) {
  test(`invalid controller output ends the run: ${name}`, () => {
    const c = checker();
    c.checkCommand(cmd, 1, 50);
    assert.equal(c.outcome.status, 'invalid_output');
    assert.equal(c.events.at(-1).type, 'invalid_output');
    assert.equal(c.events.at(-1).terminal, true);
  });
}

test('arrival at the destination', () => {
  const c = checker();
  c.checkStep(snap(10, pose(999.5, L1)), snap(10 + PHYS_DT, pose(1000.1, L1)), 501);
  assert.equal(c.outcome.status, 'arrived');
  assert.equal(c.outcome.success, true);
});

test('timeout at the time limit', () => {
  const c = checker({ timeLimit: 1 });
  c.checkStep(snap(1 - PHYS_DT, pose(0, L1)), snap(1, pose(0.4, L1)), 50);
  assert.equal(c.outcome.status, 'timeout');
});

test('records nothing after the run has terminated', () => {
  const c = checker({ timeLimit: 1 });
  c.checkStep(snap(1 - PHYS_DT, pose(0, L1)), snap(1, pose(0.4, L1)), 50);
  const n = c.events.length;
  c.checkCommand({ accel: NaN, steer: 0 }, 1, 50);
  c.checkStep(snap(1, pose(0, 50)), snap(1 + PHYS_DT, pose(0, 50)), 51);
  assert.equal(c.events.length, n);
});

test('the checker never modifies commands or vehicle state', () => {
  const c = checker();
  const cmd = deepFreeze({ accel: 99, steer: -3 });
  const prev = deepFreeze(snap(0, pose(0, L1), [npc(1, 4.7, L1)]));
  const cur = deepFreeze(snap(PHYS_DT, pose(0.4, L1), [npc(1, 4.7, L1)]));
  // Frozen inputs: any write attempt would throw in strict-mode ES modules.
  c.checkCommand(cmd, 0, 0);
  c.checkStep(prev, cur, 1);
  assert.equal(cmd.accel, 99);
  assert.equal(c.outcome.status, 'collision');
});

test('surrounding-vehicle collisions are recorded as information only', () => {
  const c = checker();
  c.checkStep(snap(0, pose(-200, L1), [npc(1, 0, L1), npc(2, 4.7, L1, { speed: 0 })]),
    snap(PHYS_DT, pose(-199.6, L1), [npc(1, 0.4, L1), npc(2, 4.7, L1, { speed: 0 })]), 1);
  assert.equal(c.outcome, null);
  assert.equal(c.events[0].type, 'traffic_collision');
  assert.equal(c.events[0].severity, 'info');
});
