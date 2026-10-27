import { test } from 'node:test';
import assert from 'node:assert/strict';

import { SCENARIOS, buildScenario } from '../server/sim/scenarios.js';
import { validateConfig, requiredGap } from '../server/sim/validate.js';
import { laneCenterY, rampPose } from '../server/sim/road.js';
import { Simulation, ConfigError } from '../server/sim/simulation.js';
import { ZeroController } from '../server/controllers/zero.js';

const clone = (o) => structuredClone(o);
const valid = () => buildScenario('sudden_brake', 7, {}).config;

test('every scenario builds a valid configuration for many seeds', () => {
  for (const sc of SCENARIOS) {
    for (let seed = 0; seed < 25; seed++) {
      const r = buildScenario(sc.id, seed, {});
      assert.ok(r.ok, `${sc.id} seed ${seed}: ${r.errors?.join('; ')}`);
      assert.ok(validateConfig(r.config).ok);
    }
  }
});

test('scenarios build at the extremes of every parameter range', () => {
  for (const sc of SCENARIOS) {
    for (const p of sc.params) {
      for (const v of [p.min, p.max]) {
        const r = buildScenario(sc.id, 3, { [p.key]: v });
        // Some extreme combinations are legitimately rejected; they must be rejected with reasons.
        assert.ok(r.ok || r.errors.length > 0, `${sc.id} ${p.key}=${v}`);
      }
    }
  }
});

test('same seed gives an identical configuration; different seeds differ', () => {
  for (const sc of SCENARIOS) {
    const a = buildScenario(sc.id, 1234, {}).config;
    const b = buildScenario(sc.id, 1234, {}).config;
    const c = buildScenario(sc.id, 1235, {}).config;
    assert.deepEqual(a, b);
    assert.notDeepEqual(a, c);
  }
});

test('invalid seeds and parameters are rejected', () => {
  for (const seed of [-1, 1.5, 2 ** 32, NaN, '5', null]) {
    assert.equal(buildScenario('merge_busy', seed, {}).ok, false, `seed ${seed}`);
  }
  assert.equal(buildScenario('nope', 1, {}).ok, false);
  const r1 = buildScenario('merge_busy', 1, { density: 100 });
  assert.equal(r1.ok, false);
  assert.match(r1.errors[0], /density/);
  assert.equal(buildScenario('merge_busy', 1, { bogus: 1 }).ok, false);
  assert.equal(buildScenario('merge_busy', 1, { density: 'NaN' }).ok, false);
});

test('parameter combinations that make a collision unavoidable are rejected', () => {
  // 30 m/s following a stopped-speed truck 30 m ahead cannot be avoided with 6 m/s² braking.
  const r = buildScenario('slow_vehicle', 1, { ego_speed: 30, slow_speed: 6, slow_gap: 30 });
  assert.equal(r.ok, false);
  assert.ok(r.errors.some((e) => /ego is .* behind vehicle/.test(e)), r.errors.join('; '));
  // Same-speed follower at 10 m / 30 m/s violates the reaction-time gap.
  assert.equal(buildScenario('sudden_brake', 1, { ego_speed: 30, lead_gap: 10 }).ok, false);
});

test('hand-made invalid configurations are rejected with reasons', () => {
  const cases = {
    'overlapping vehicles': (c) => { c.npcs[0].x = c.ego.x + 1; c.npcs[0].y = c.ego.y; c.npcs[0].lane = 1; },
    'ego off road': (c) => { c.ego.y = 30; },
    'ego partly off road': (c) => { c.ego.y = 10.5; },
    'ego missing': (c) => { delete c.ego; },
    'non-finite speed': (c) => { c.npcs[1].speed = NaN; },
    'negative speed': (c) => { c.ego.speed = -1; },
    'npc not centred in lane': (c) => { c.npcs[1].y += 1.2; },
    'duplicate ids': (c) => { c.npcs[1].id = c.npcs[0].id; },
    'unknown event vehicle': (c) => { c.events[0].vehicleId = 99999; },
    'event after time limit': (c) => { c.events[0].t = c.timeLimit + 1; },
    'bad destination': (c) => { c.destination.polygon = [[0, 0], [1, 0]]; },
    'destination off road': (c) => { c.destination.polygon = [[0, -100], [10, -100], [10, -90], [0, -90]]; },
    'bad time limit': (c) => { c.timeLimit = 0; },
    'bad inflow': (c) => { c.traffic.inflow = [0.1, 0.1]; },
    'too-short gap behind leader': (c) => {
      const lead = c.npcs.find((n) => n.id === c.events[0].vehicleId);
      lead.x = c.ego.x + (c.ego.length + lead.length) / 2 + requiredGap(c.ego.speed, lead.speed) - 0.5;
    },
    'oversized vehicle': (c) => { c.npcs[2].length = 40; },
  };
  for (const [name, mutate] of Object.entries(cases)) {
    const c = clone(valid());
    mutate(c);
    const v = validateConfig(c);
    assert.equal(v.ok, false, `${name} should be rejected`);
    assert.ok(v.errors.length > 0);
  }
});

test('Simulation refuses to start from an invalid configuration', () => {
  const c = clone(valid());
  c.ego.y = laneCenterY(1) + 50;
  assert.throws(() => new Simulation(c, new ZeroController()), ConfigError);
});

test('merge scenario places the ego on the entrance ramp', () => {
  const c = buildScenario('merge_busy', 5, { ego_ramp_pos: 40 }).config;
  const p = rampPose('entrance', 40);
  assert.ok(Math.abs(c.ego.x - p.x) < 1e-9 && Math.abs(c.ego.y - p.y) < 1e-9);
  assert.ok(c.npcs.length > 50, 'busy traffic expected');
});

test('cut-in and sudden-brake scenarios carry their scripted events', () => {
  const ci = buildScenario('cut_in', 2, { cut_time: 4 }).config;
  assert.equal(ci.events[0].type, 'force_lane_change');
  assert.equal(ci.events[0].t, 4);
  assert.equal(ci.events[0].targetLane, 1);
  const sb = buildScenario('sudden_brake', 2, { brake_decel: 8 }).config;
  assert.equal(sb.events[0].type, 'brake');
  assert.equal(sb.events[0].decel, 8);
});
