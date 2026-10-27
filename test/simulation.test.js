import { test } from 'node:test';
import assert from 'node:assert/strict';

import { Simulation } from '../server/sim/simulation.js';
import { buildScenario, SCENARIOS } from '../server/sim/scenarios.js';
import { PHYS_DT, CONTROL_PERIOD_STEPS, CONTROL_DT, EGO_LIMITS, DIMS, SENSOR_RANGE, MAX_OBJECTS } from '../server/sim/constants.js';
import { laneCenterY, rampPose } from '../server/sim/road.js';
import { makeRng } from '../server/sim/rng.js';
import { ZeroController } from '../server/controllers/zero.js';
import { ManualController } from '../server/controllers/manual.js';

const fn = (step) => ({ async reset() {}, step: async (obs) => step(obs) });

async function started(config, controller = new ZeroController()) {
  const sim = new Simulation(config, controller);
  await sim.start();
  return sim;
}

/** Minimal valid configuration: stationary ego parked on the entrance ramp, no inflow. */
function baseConfig(npcs, extra = {}) {
  const p = rampPose('entrance', 30);
  return {
    scenarioId: 'test', name: 'test', seed: 1, params: {}, timeLimit: 120,
    ego: { x: p.x, y: p.y, heading: p.heading, speed: 0, ...DIMS.ego },
    npcs: npcs.map((n, i) => ({
      id: i + 1, kind: 'car', y: laneCenterY(n.lane), heading: 0, desiredSpeed: n.speed, laneChange: false,
      idmT: 1.4, politeness: 0.3, ...DIMS.car, ...n,
    })),
    events: [],
    destination: { polygon: [[1500, 0], [1590, 0], [1590, 11.1], [1500, 11.1]], target: [1545, 5.55] },
    traffic: { inflow: [0, 0, 0], laneSpeeds: [20, 20, 20] },
    ...extra,
  };
}

const gapAhead = (rear, front) => front.x - rear.x - (rear.length + front.length) / 2;

test('time advances in exact fixed steps', async () => {
  const sim = await started(buildScenario('cut_in', 1, {}).config);
  for (let i = 0; i < 123; i++) await sim.step();
  assert.equal(sim.stepCount, 123);
  assert.equal(sim.t, 123 * PHYS_DT);
});

test('controller is queried once per control period with zero-order hold', async () => {
  const calls = [];
  const sim = await started(buildScenario('cut_in', 1, {}).config, fn((o) => { calls.push(o.step); return { accel: 0, steer: 0 }; }));
  await sim.runUntil(2);
  assert.deepEqual(calls, Array.from({ length: 2 / CONTROL_DT }, (_, i) => i * CONTROL_PERIOD_STEPS));
});

test('reset restores the exact starting configuration and reproduces the run', async () => {
  for (const sc of SCENARIOS) {
    const cfg = buildScenario(sc.id, 42, {}).config;
    const sim = await started(cfg);
    const initial = sim.stateFingerprint();
    const marks = [];
    for (let i = 0; i < 6; i++) { await sim.runUntil((i + 1) * 2); marks.push(sim.stateFingerprint()); }
    await sim.reset();
    assert.equal(sim.stateFingerprint(), initial, `${sc.id}: reset state differs`);
    assert.equal(sim.log.length, 0);
    for (let i = 0; i < 6; i++) { await sim.runUntil((i + 1) * 2); assert.equal(sim.stateFingerprint(), marks[i], `${sc.id}: diverged after reset at ${(i + 1) * 2}s`); }
    // A brand-new simulation from the same seed is identical as well.
    const fresh = await started(buildScenario(sc.id, 42, {}).config);
    await fresh.runUntil(12);
    assert.equal(fresh.stateFingerprint(), marks[5], `${sc.id}: fresh run differs`);
  }
});

test('results do not depend on how stepping is chunked (frame-rate independence)', async () => {
  const cfg = buildScenario('exit_traffic', 9, {}).config;
  const a = await started(cfg);
  await a.runUntil(8);
  const b = await started(cfg);
  while (b.t < 8 - 1e-9) await b.stepControlPeriod();
  const c = await started(cfg);
  const rng = makeRng(5);
  while (c.t < 8 - 1e-9) { const n = rng.int(1, 17); for (let i = 0; i < n && c.t < 8 - 1e-9; i++) await c.step(); }
  assert.equal(b.stateFingerprint(), a.stateFingerprint());
  assert.equal(c.stateFingerprint(), a.stateFingerprint());
});

test('different seeds produce different traffic', async () => {
  const a = await started(buildScenario('merge_busy', 1, {}).config);
  const b = await started(buildScenario('merge_busy', 2, {}).config);
  await a.runUntil(3);
  await b.runUntil(3);
  assert.notEqual(a.stateFingerprint(), b.stateFingerprint());
});

test('observation contains ego state, road, destination and nearby traffic only', async () => {
  let obs = null;
  const sim = await started(buildScenario('merge_busy', 3, { density: 45 }).config, fn((o) => { obs ??= o; return { accel: 0, steer: 0 }; }));
  await sim.step();
  assert.equal(obs.time, 0);
  assert.equal(obs.controlDt, CONTROL_DT);
  assert.equal(obs.ego.length, DIMS.ego.length);
  assert.ok(obs.road.regions.length >= 7);
  assert.ok(obs.destination.polygon.length >= 3);
  assert.deepEqual(obs.limits, EGO_LIMITS);
  assert.ok(obs.objects.length > 0 && obs.objects.length <= MAX_OBJECTS);
  let last = 0;
  for (const o of obs.objects) {
    const d = Math.hypot(o.x - obs.ego.x, o.y - obs.ego.y);
    assert.ok(d <= SENSOR_RANGE);
    assert.ok(d >= last);
    last = d;
  }
});

test('controller commands are applied unchanged except for physical saturation, and reported', async () => {
  const sim = await started(buildScenario('slow_vehicle', 1, {}).config, fn(() => ({ accel: 10, steer: 0 })));
  await sim.step();
  assert.equal(sim.lastCommand.accel, 10);
  assert.equal(sim.heldCommand.accel, 10);
  assert.ok(Math.abs(sim.applied.accel - EGO_LIMITS.accelMax) < 1e-9);
  assert.equal(sim.log[0].type, 'invalid_output');
  assert.equal(sim.done, false);
});

test('non-finite controller output ends the episode', async () => {
  const sim = await started(buildScenario('slow_vehicle', 1, {}).config, fn((o) => (o.time >= 1 ? { accel: NaN, steer: 0 } : { accel: 0, steer: 0 })));
  await sim.runUntil(5);
  assert.equal(sim.done, true);
  assert.equal(sim.outcome.status, 'invalid_output');
  assert.ok(Math.abs(sim.t - 1) < 1e-9);
});

test('a throwing controller ends the episode with invalid_output', async () => {
  const sim = await started(buildScenario('slow_vehicle', 1, {}).config, fn(() => { throw new Error('boom'); }));
  await sim.step();
  assert.equal(sim.outcome.status, 'invalid_output');
  assert.match(sim.outcome.message, /boom/);
});

test('placeholder controller runs into the slow vehicle and the collision is recorded', async () => {
  const sim = await started(buildScenario('slow_vehicle', 4, { density: 0 }).config);
  await sim.runUntil(70);
  assert.equal(sim.outcome.status, 'collision');
  assert.equal(sim.log.filter((e) => e.terminal).length, 1);
});

test('placeholder controller keeps going straight off the ramp and the departure is recorded', async () => {
  const sim = await started(buildScenario('merge_busy', 4, { density: 5 }).config);
  await sim.runUntil(60);
  assert.ok(['road_departure', 'collision'].includes(sim.outcome.status), sim.outcome.status);
});

test('timeout when the ego never reaches the destination', async () => {
  const sim = await started(baseConfig([], { timeLimit: 5 }));
  await sim.runUntil(10);
  assert.equal(sim.outcome.status, 'timeout');
  assert.ok(Math.abs(sim.t - 5) < 1e-9);
});

test('a lane-keeping controller reaches the destination', async () => {
  const cfg = buildScenario('cut_in', 1, { density: 0 }).config;
  // Remove the cut-in vehicle so the lane ahead is free.
  cfg.npcs = [];
  cfg.events = [];
  const sim = await started(cfg, fn((o) => ({ accel: 0, steer: -0.5 * (o.ego.y - laneCenterY(1)) * 0.02 - 0.5 * o.ego.heading })));
  await sim.runUntil(50);
  assert.equal(sim.outcome.status, 'arrived');
});

test('traffic: follower keeps a safe distance behind a slower vehicle', async () => {
  const sim = await started(baseConfig([
    { lane: 1, x: 300, speed: 15 },
    { lane: 1, x: 200, speed: 28, desiredSpeed: 28 },
  ]));
  let minGap = Infinity;
  while (sim.t < 40) { await sim.step(); minGap = Math.min(minGap, gapAhead(sim.npcs[1], sim.npcs[0])); }
  assert.equal(sim.checker.stats.trafficCollisions, 0);
  assert.ok(minGap > 5, `min gap ${minGap}`);
  assert.ok(Math.abs(sim.npcs[1].speed - 15) < 1, 'follower should match the leader speed');
});

test('traffic: follower stops behind a hard-braking leader', async () => {
  const sim = await started(baseConfig([
    { lane: 2, x: 340, speed: 25 },
    { lane: 2, x: 300, speed: 25 },
  ], { events: [{ t: 2, type: 'brake', vehicleId: 1, decel: 7, targetSpeed: 0, hold: 30 }] }));
  await sim.runUntil(20);
  assert.equal(sim.checker.stats.trafficCollisions, 0);
  assert.ok(sim.npcs[0].speed < 0.01 && sim.npcs[1].speed < 0.5);
  assert.ok(gapAhead(sim.npcs[1], sim.npcs[0]) > 0.5);
});

test('traffic: faster vehicle changes lanes to pass a slow one', async () => {
  const sim = await started(baseConfig([
    { lane: 1, x: 300, speed: 12 },
    { lane: 1, x: 200, speed: 25, desiredSpeed: 28, laneChange: true },
  ]));
  let changed = false;
  while (sim.t < 30 && !changed) { await sim.step(); changed = sim.npcs[1].lane !== 1; }
  assert.ok(changed, 'expected a lane change');
  assert.equal(sim.checker.stats.trafficCollisions, 0);
});

test('traffic: vehicles react to the ego (braking behind a slow ego)', async () => {
  const cfg = baseConfig([{ lane: 1, x: 100, speed: 28, desiredSpeed: 28 }]);
  cfg.ego = { x: 200, y: laneCenterY(1), heading: 0, speed: 10, ...DIMS.ego };
  const sim = await started(cfg);
  let minAccel = 0;
  while (sim.t < 20 && !sim.done) { await sim.step(); minAccel = Math.min(minAccel, sim.npcs[0].accel); }
  assert.ok(minAccel < -1, 'NPC should brake for the ego');
  assert.notEqual(sim.outcome?.status, 'collision');
  assert.ok(gapAhead(sim.npcs[0], sim.ego) > 2);
});

test('traffic: forced cut-in moves the vehicle into the target lane', async () => {
  const sim = await started(buildScenario('cut_in', 11, { density: 0 }).config);
  const cutter = sim.npcs[0];
  const startLane = cutter.lane;
  await sim.runUntil(3 + 6);
  assert.notEqual(startLane, 1);
  assert.equal(cutter.lane, 1);
  assert.ok(Math.abs(cutter.y - laneCenterY(1)) < 0.3);
});

test('traffic: busy scenarios run without collisions between surrounding vehicles', async () => {
  for (const id of ['merge_busy', 'exit_traffic']) {
    for (const seed of [1, 2, 3]) {
      const cfg = buildScenario(id, seed, { density: id === 'merge_busy' ? 40 : 35 }).config;
      // Park the ego on the ramp so traffic runs for the full minute.
      const p = rampPose('entrance', 30);
      cfg.ego = { ...cfg.ego, x: p.x, y: p.y, heading: p.heading, speed: 0 };
      const sim = await started(cfg);
      let laneChanges = 0;
      const changing = new Set();
      while (sim.t < 30) {
        await sim.step();
        for (const n of sim.npcs) {
          if (n.targetLane !== null && !changing.has(n.id)) { laneChanges++; changing.add(n.id); }
          if (n.targetLane === null) changing.delete(n.id);
        }
      }
      assert.equal(sim.checker.stats.trafficCollisions, 0, `${id} seed ${seed}`);
      assert.ok(laneChanges > 0, `${id} seed ${seed}: expected some lane changes`);
      assert.ok(sim.npcs.length > 50, 'inflow keeps traffic populated');
    }
  }
});

test('manual controller maps keys to bounded commands', async () => {
  const m = new ManualController();
  const sim = await started(buildScenario('cut_in', 1, {}).config, m);
  m.setKeys({ up: true, left: true });
  await sim.stepControlPeriod();
  assert.ok(sim.lastCommand.accel > 0);
  assert.ok(sim.lastCommand.steer > 0);
  assert.ok(Math.abs(sim.lastCommand.steer) <= EGO_LIMITS.steerMax);
  m.setKeys({ up: false, left: false, brake: true });
  await sim.stepControlPeriod();
  assert.equal(sim.lastCommand.accel, EGO_LIMITS.accelMin);
  assert.equal(sim.log.filter((e) => e.type === 'invalid_output').length, 0);
});
