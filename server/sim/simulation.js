// Fixed-timestep simulation. One call to step() advances exactly PHYS_DT of simulated time,
// independent of wall-clock time or rendering. The controller is queried every
// CONTROL_PERIOD_STEPS steps and its command is held in between.

import { PHYS_DT, CONTROL_PERIOD_STEPS, CONTROL_DT, EGO_LIMITS } from './constants.js';
import { makeRng } from './rng.js';
import { createVehicle, stepVehicle } from './vehicle.js';
import { createNpc, buildLaneIndex, computeNpcCommands, spawnTraffic } from './traffic.js';
import { SafetyChecker } from './safety.js';
import { buildObservation } from './observation.js';
import { validateConfig } from './validate.js';
import { ROAD, HIGHWAY_X_MAX, regionAt, laneAt, speedLimitAt } from './road.js';

export class ConfigError extends Error {
  constructor(errors) {
    super(`invalid starting configuration:\n - ${errors.join('\n - ')}`);
    this.errors = errors;
  }
}

function deepFreeze(o) {
  if (o && typeof o === 'object' && !Object.isFrozen(o)) {
    Object.freeze(o);
    for (const v of Object.values(o)) deepFreeze(v);
  }
  return o;
}

/** Raw command as returned by the controller; values are passed through unchanged. */
function normalizeCommand(cmd) {
  if (!cmd || typeof cmd !== 'object') return { error: 'controller returned no command' };
  if (cmd.error) return { error: String(cmd.error) };
  return { accel: cmd.accel, steer: cmd.steer };
}

export class Simulation {
  /**
   * @param {object} config validated starting configuration (see scenarios.js)
   * @param {{reset(ctx):Promise<void>, step(obs):Promise<{accel,steer}|{error}>}} controller
   */
  constructor(config, controller) {
    const v = validateConfig(config);
    if (!v.ok) throw new ConfigError(v.errors);
    this.initialConfig = deepFreeze(structuredClone(config));
    this.controller = controller;
    this._initState();
  }

  _initState() {
    const c = structuredClone(this.initialConfig);
    this.t = 0;
    this.stepCount = 0;
    this.done = false;
    this.rng = makeRng((c.seed ^ 0x5bd1e995) >>> 0);
    this.ego = createVehicle({ ...c.ego, id: 0, kind: 'ego', limits: EGO_LIMITS });
    this.npcs = c.npcs.map(createNpc);
    this.nextId = c.npcs.reduce((m, n) => Math.max(m, n.id), 0) + 1;
    this.scenarioEvents = c.events.map((e, i) => ({ ...e, index: i, done: false })).sort((a, b) => a.t - b.t || a.index - b.index);
    this.traffic = {
      inflow: c.traffic.inflow,
      laneSpeeds: c.traffic.laneSpeeds,
      nextSpawn: c.traffic.inflow.map((r) => (r > 0 ? this.rng.exp(1 / r) : Infinity)),
    };
    this.destination = c.destination;
    this.log = [];
    this.checker = new SafetyChecker({
      destination: c.destination, timeLimit: c.timeLimit, limits: EGO_LIMITS, onEvent: (e) => this.log.push(e),
    });
    this.lastCommand = null;
    this.heldCommand = { accel: 0, steer: 0 };
    this.applied = { accel: 0, steer: 0 };
    this.trajectory = [[0, this.ego.x, this.ego.y]];
  }

  get config() {
    return this.initialConfig;
  }

  get outcome() {
    return this.checker.outcome;
  }

  controllerContext() {
    return { seed: this.initialConfig.seed, road: ROAD, destination: this.destination, limits: EGO_LIMITS, controlDt: CONTROL_DT };
  }

  /** Prepare the controller for a fresh episode. Must be awaited before the first step. */
  async start() {
    await this.controller.reset(this.controllerContext());
  }

  /** Restore the exact starting configuration and restart the controller. */
  async reset() {
    this._initState();
    await this.start();
  }

  _poseSnapshot() {
    const pose = (v) => ({ id: v.id, x: v.x, y: v.y, heading: v.heading, speed: v.speed, length: v.length, width: v.width });
    return { t: this.t, ego: pose(this.ego), npcs: this.npcs.map(pose) };
  }

  _applyScenarioEvents() {
    for (const ev of this.scenarioEvents) {
      if (ev.done || this.t < ev.t - 1e-9) continue;
      ev.done = true;
      const npc = this.npcs.find((n) => n.id === ev.vehicleId);
      if (!npc) continue;
      if (ev.type === 'force_lane_change') {
        npc.targetLane = ev.targetLane;
        npc.forced = true;
        npc.laneChange = false;
        this.log.push({ t: this.t, step: this.stepCount, type: 'scenario', severity: 'info', terminal: false, message: `vehicle ${npc.id} starts changing into lane ${ev.targetLane}`, data: { vehicleId: npc.id } });
      } else if (ev.type === 'brake') {
        npc.scripted = { mode: 'brake', decel: ev.decel, targetSpeed: ev.targetSpeed, hold: ev.hold, holdUntil: Infinity };
        this.log.push({ t: this.t, step: this.stepCount, type: 'scenario', severity: 'info', terminal: false, message: `vehicle ${npc.id} brakes at ${ev.decel} m/s² to ${ev.targetSpeed} m/s`, data: { vehicleId: npc.id } });
      }
    }
  }

  /** Advance one physics step (PHYS_DT). */
  async step() {
    if (this.done) return;

    if (this.stepCount % CONTROL_PERIOD_STEPS === 0) {
      let raw;
      try {
        raw = await this.controller.step(buildObservation(this));
      } catch (err) {
        raw = { error: String(err?.message ?? err) };
      }
      this.lastCommand = normalizeCommand(raw);
      this.checker.checkCommand(this.lastCommand, this.t, this.stepCount);
      if (this.checker.terminated) { this.done = true; return; }
      this.heldCommand = { accel: this.lastCommand.accel, steer: this.lastCommand.steer };
    }

    this._applyScenarioEvents();
    const laneIndex = buildLaneIndex([this.ego, ...this.npcs]);
    const npcCmds = computeNpcCommands({ t: this.t, step: this.stepCount, ego: this.ego, npcs: this.npcs, laneIndex });

    const prev = this._poseSnapshot();
    this.applied = stepVehicle(this.ego, this.heldCommand.accel, this.heldCommand.steer, PHYS_DT);
    for (const n of this.npcs) {
      const c = npcCmds.get(n.id);
      stepVehicle(n, c.accel, c.steer, PHYS_DT);
    }
    this.stepCount++;
    this.t = this.stepCount * PHYS_DT;
    this.checker.checkStep(prev, this._poseSnapshot(), this.stepCount);
    this.trajectory.push([this.t, this.ego.x, this.ego.y]);

    this.npcs = this.npcs.filter((n) => n.x < HIGHWAY_X_MAX - 10);
    spawnTraffic(this);
    if (this.checker.terminated) this.done = true;
  }

  /** Advance to the next control step boundary (one controller period) or until done. */
  async stepControlPeriod() {
    do {
      await this.step();
    } while (!this.done && this.stepCount % CONTROL_PERIOD_STEPS !== 0);
  }

  /** Run until simulated time >= tEnd or done. */
  async runUntil(tEnd) {
    while (!this.done && this.t < tEnd - 1e-9) await this.step();
  }

  /** Compact state used by the UI. */
  frame() {
    const e = this.ego;
    const region = regionAt(e.x, e.y);
    return {
      t: this.t,
      step: this.stepCount,
      done: this.done,
      ego: { x: e.x, y: e.y, heading: e.heading, speed: e.speed, accel: e.accel, steer: e.steer, length: e.length, width: e.width },
      npcs: this.npcs.map((n) => [
        n.id, n.x, n.y, n.heading, n.length, n.width, n.speed, n.accel, n.kind === 'truck' ? 1 : 0,
        (n.accel < -1 ? 1 : 0) | (n.targetLane !== null && n.targetLane > n.lane ? 2 : 0) |
          (n.targetLane !== null && n.targetLane < n.lane ? 4 : 0) | (n.scripted || n.forced ? 8 : 0),
      ]),
      command: this.lastCommand,
      applied: this.applied,
      region: region ? region.id : null,
      speedLimit: speedLimitAt(e.x, e.y),
      lane: region?.id === 'highway' ? laneAt(e.y) : null,
      outcome: this.outcome,
    };
  }

  /** Deterministic fingerprint of the full dynamic state (used by tests). */
  stateFingerprint() {
    const r = (v) => v.toPrecision(15);
    const parts = [this.stepCount, r(this.t)];
    for (const v of [this.ego, ...this.npcs]) parts.push(v.id, r(v.x), r(v.y), r(v.heading), r(v.speed), r(v.steer));
    parts.push(this.log.length, this.outcome?.status ?? '-');
    return parts.join('|');
  }
}
