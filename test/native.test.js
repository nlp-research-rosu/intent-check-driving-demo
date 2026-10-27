import { test } from 'node:test';
import assert from 'node:assert/strict';
import path from 'node:path';
import { fileURLToPath } from 'node:url';

import { NativeController, parseCNumber, serializeReset, serializeStep } from '../server/controllers/native.js';
import { ZeroController } from '../server/controllers/zero.js';
import { Simulation } from '../server/sim/simulation.js';
import { buildScenario } from '../server/sim/scenarios.js';
import { buildObservation } from '../server/sim/observation.js';

const ROOT = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '..');
const EXE = path.join(ROOT, 'build', process.platform === 'win32' ? 'placeholder_controller.exe' : 'placeholder_controller');
const BAD_EXE = path.join(ROOT, 'build', process.platform === 'win32' ? 'invalid_controller.exe' : 'invalid_controller');
const skipNative = NativeController.exists(EXE) ? false : 'native test fixtures not built (run npm run build:fixtures)';
const skipBad = NativeController.exists(BAD_EXE) ? false : 'native test fixtures not built (run npm run build:fixtures)';

test('parses C number spellings', () => {
  assert.equal(parseCNumber('0'), 0);
  assert.equal(parseCNumber('-2.5'), -2.5);
  assert.equal(parseCNumber('1.0000000000000001e-07'), 1.0000000000000001e-7);
  assert.equal(parseCNumber('inf'), Infinity);
  assert.equal(parseCNumber('-inf'), -Infinity);
  assert.ok(Number.isNaN(parseCNumber('nan')));
  assert.ok(Number.isNaN(parseCNumber('-nan(ind)')));
  assert.ok(Number.isNaN(parseCNumber('')));
  assert.ok(Number.isNaN(parseCNumber('abc')));
});

test('protocol lines carry the full road, destination and observation', async () => {
  const sim = new Simulation(buildScenario('merge_busy', 1, {}).config, new ZeroController());
  const ctx = sim.controllerContext();
  const r = serializeReset(ctx).split(' ');
  assert.equal(r[0], 'R');
  const expectedReset = 1 + 1 + 1 + 6 + 4 + 1 +
    ctx.road.regions.reduce((n, reg) => n + 3 + 2 * reg.polygon.length, 0) + 1 + 2 * ctx.destination.polygon.length + 2;
  assert.equal(r.length, expectedReset);
  const obs = buildObservation(sim);
  const s = serializeStep(obs).split(' ');
  assert.equal(s.length, 1 + 2 + 9 + 1 + 1 + 8 * obs.objects.length);
  // Numbers survive the text round trip exactly.
  assert.equal(Number(s[3]), obs.ego.x);
});

test('native placeholder controller returns zero commands', { skip: skipNative }, async () => {
  const c = new NativeController(EXE);
  try {
    const sim = new Simulation(buildScenario('cut_in', 1, {}).config, c);
    await sim.start();
    assert.match(c.name, /placeholder/);
    await sim.step();
    assert.deepEqual(sim.lastCommand, { accel: 0, steer: 0 });
  } finally {
    c.close();
  }
});

test('native placeholder run is identical to the in-process zero controller, including after reset', { skip: skipNative }, async () => {
  const cfg = buildScenario('sudden_brake', 3, {}).config;
  const ref = new Simulation(cfg, new ZeroController());
  await ref.start();
  await ref.runUntil(6);
  const c = new NativeController(EXE);
  try {
    const sim = new Simulation(cfg, c);
    await sim.start();
    await sim.runUntil(6);
    assert.equal(sim.stateFingerprint(), ref.stateFingerprint());
    await sim.reset();
    await sim.runUntil(6);
    assert.equal(sim.stateFingerprint(), ref.stateFingerprint());
  } finally {
    c.close();
  }
});

test('native controller invalid outputs are detected (demo controller)', { skip: skipBad }, async () => {
  const c = new NativeController(BAD_EXE);
  try {
    const sim = new Simulation(buildScenario('cut_in', 1, { density: 0, cut_gap: 40 }).config, c);
    await sim.start();
    await sim.runUntil(10);
    const types = sim.log.map((e) => e.type);
    assert.ok(types.includes('invalid_output'));
    assert.equal(sim.outcome.status, 'invalid_output');
    assert.ok(Math.abs(sim.outcome.t - 4) < 1e-9);
  } finally {
    c.close();
  }
});

test('a missing controller executable is reported clearly', async () => {
  const c = new NativeController(path.join(ROOT, 'build', 'does-not-exist.exe'));
  const sim = new Simulation(buildScenario('cut_in', 1, {}).config, c);
  await assert.rejects(sim.start(), /not found/);
});
