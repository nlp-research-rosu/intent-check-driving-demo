// Scenario generators. Each scenario turns (seed, params) into a starting configuration
// deterministically; the configuration is then validated and rejected if invalid.

import { DIMS } from './constants.js';
import { clamp } from './geometry.js';
import { makeRng, isValidSeed } from './rng.js';
import { laneCenterY, rampPose, rampSection, HIGHWAY_WIDTH, HIGHWAY_X_MIN, HIGHWAY_X_MAX, DECEL_LANE, NUM_LANES } from './road.js';
import { randomNpcSpec } from './traffic.js';
import { validateConfig, validateParams, pairConflict } from './validate.js';

const laneSpeedsFor = (base) => [base - 3, base, base + 3].map((s) => clamp(s, 5, 40));
const inflowFor = (density, laneSpeeds) => laneSpeeds.map((s) => clamp((density * s) / 1000, 0, 1));

function highwayDestination(x0, x1, label) {
  return { polygon: [[x0, 0], [x1, 0], [x1, HIGHWAY_WIDTH], [x0, HIGHWAY_WIDTH]], target: [(x0 + x1) / 2, HIGHWAY_WIDTH / 2], label };
}

function egoSpec(x, y, heading, speed) {
  return { x, y, heading, speed, length: DIMS.ego.length, width: DIMS.ego.width, wheelbase: DIMS.ego.wheelbase };
}

function laneEgo(lane, x, speed) {
  return egoSpec(x, laneCenterY(lane), 0, speed);
}

function specialNpc(cfg, kind, lane, x, speed, extra = {}) {
  const dims = DIMS[kind];
  const npc = {
    id: cfg.nextId++, kind, x, y: laneCenterY(lane), heading: 0, speed, lane, desiredSpeed: speed,
    length: dims.length, width: dims.width, wheelbase: dims.wheelbase, laneChange: false, idmT: 1.4, politeness: 0.3, ...extra,
  };
  cfg.npcs.push(npc);
  return npc;
}

function fits(spec, cfg) {
  const ego = { ...cfg.ego, id: 0 };
  if (Math.abs(ego.x - spec.x) < 250 && pairConflict(spec, ego)) return false;
  for (const v of cfg.npcs) if (Math.abs(v.x - spec.x) < 250 && pairConflict(spec, v)) return false;
  return true;
}

/**
 * Fill lanes with background traffic from downstream to upstream. Mean spacing follows the density
 * (vehicles per km per lane); every vehicle keeps at least a ~0.8 s headway plus the validation gap.
 */
function fillTraffic(rng, cfg, { density, laneSpeeds, exclusions = [], truckProb = 0.06 }) {
  if (density <= 0) return;
  const mean = 1000 / density;
  const xMin = HIGHWAY_X_MIN + 20, xMax = HIGHWAY_X_MAX - 30;
  for (let lane = 0; lane < NUM_LANES; lane++) {
    let x = xMax - rng.uniform(0, mean);
    while (x > xMin) {
      const spec = randomNpcSpec(rng, { id: cfg.nextId, lane, x, laneSpeed: laneSpeeds[lane], truckProb });
      const excluded = exclusions.some((e) => e.lane === lane && x + spec.length / 2 > e.x0 && x - spec.length / 2 < e.x1);
      if (!excluded && fits(spec, cfg)) {
        cfg.npcs.push(spec);
        cfg.nextId++;
        const minGap = 3 + 0.8 * spec.speed;
        x -= spec.length + minGap + rng.exp(Math.max(1, mean - spec.length - minGap));
      } else {
        x -= 4;
      }
    }
  }
}

const P = (key, label, min, max, def, unit = '', step = 0.5) => ({ key, label, min, max, default: def, unit, step });

export const SCENARIOS = [
  {
    id: 'merge_busy',
    name: 'Merge into busy traffic',
    description: 'The ego starts on the entrance ramp and must merge from the acceleration lane into dense traffic, then reach the highway section 600-700 m.',
    timeLimit: 60,
    params: [
      P('density', 'Traffic density', 5, 45, 28, 'veh/km/lane', 1),
      P('traffic_speed', 'Traffic speed (middle lane)', 15, 30, 24, 'm/s'),
      P('ego_speed', 'Ego initial speed', 8, 22, 15, 'm/s'),
      P('ego_ramp_pos', 'Ego distance along ramp', 20, 120, 50, 'm', 5),
    ],
    build(rng, p, cfg) {
      const pose = rampPose('entrance', p.ego_ramp_pos);
      cfg.ego = egoSpec(pose.x, pose.y, pose.heading, p.ego_speed);
      const laneSpeeds = laneSpeedsFor(p.traffic_speed);
      fillTraffic(rng, cfg, { density: p.density, laneSpeeds });
      cfg.traffic = { inflow: inflowFor(p.density, laneSpeeds), laneSpeeds };
      cfg.destination = highwayDestination(600, 700, 'Highway 600-700 m');
    },
  },
  {
    id: 'slow_vehicle',
    name: 'Slower vehicle ahead',
    description: 'The ego cruises in the middle lane and approaches a slow truck that never changes lanes. Adjacent lanes carry traffic.',
    timeLimit: 70,
    params: [
      P('slow_speed', 'Slow vehicle speed', 6, 22, 14, 'm/s'),
      P('slow_gap', 'Initial gap to slow vehicle', 30, 200, 90, 'm', 5),
      P('ego_speed', 'Ego initial speed', 15, 30, 26, 'm/s'),
      P('density', 'Traffic density', 0, 30, 12, 'veh/km/lane', 1),
    ],
    build(rng, p, cfg) {
      const ex = rng.uniform(60, 100);
      cfg.ego = laneEgo(1, ex, p.ego_speed);
      const tx = ex + (DIMS.ego.length + DIMS.truck.length) / 2 + p.slow_gap;
      specialNpc(cfg, 'truck', 1, tx, p.slow_speed, { idmT: 1.5, politeness: 0 });
      const laneSpeeds = laneSpeedsFor(p.ego_speed);
      fillTraffic(rng, cfg, { density: p.density, laneSpeeds, exclusions: [{ lane: 1, x0: ex - 30, x1: tx + 60 }] });
      cfg.traffic = { inflow: inflowFor(p.density, laneSpeeds), laneSpeeds };
      cfg.destination = highwayDestination(1100, 1200, 'Highway 1100-1200 m');
    },
  },
  {
    id: 'cut_in',
    name: 'Car changes lanes nearby',
    description: 'A slightly slower car in an adjacent lane (side chosen by the seed) cuts into the ego lane at a set time, leaving a short gap.',
    timeLimit: 50,
    params: [
      P('ego_speed', 'Ego initial speed', 15, 30, 25, 'm/s'),
      P('cut_gap', 'Gap in front of ego at cut-in', 5, 40, 12, 'm'),
      P('cut_time', 'Cut-in start time', 1, 10, 3, 's'),
      P('density', 'Traffic density', 0, 30, 10, 'veh/km/lane', 1),
    ],
    build(rng, p, cfg) {
      const side = rng.chance(0.5) ? 0 : 2;
      const ex = rng.uniform(80, 120);
      cfg.ego = laneEgo(1, ex, p.ego_speed);
      const vc = Math.max(3, p.ego_speed - rng.uniform(1, 3));
      const xc = ex + (DIMS.ego.length + DIMS.car.length) / 2 + p.cut_gap + (p.ego_speed - vc) * p.cut_time;
      const cutter = specialNpc(cfg, 'car', side, xc, vc);
      cfg.events.push({ t: p.cut_time, type: 'force_lane_change', vehicleId: cutter.id, targetLane: 1 });
      const laneSpeeds = laneSpeedsFor(p.ego_speed);
      fillTraffic(rng, cfg, {
        density: p.density, laneSpeeds,
        exclusions: [{ lane: 1, x0: ex - 40, x1: xc + 80 }, { lane: side, x0: xc - 50, x1: xc + 50 }],
      });
      cfg.traffic = { inflow: inflowFor(p.density, laneSpeeds), laneSpeeds };
      cfg.destination = highwayDestination(900, 1000, 'Highway 900-1000 m');
    },
  },
  {
    id: 'sudden_brake',
    name: 'Sudden braking ahead',
    description: 'The ego follows a car in the middle lane that brakes hard at a set time down to a final speed, holds it, then resumes.',
    timeLimit: 60,
    params: [
      P('ego_speed', 'Ego and leader initial speed', 10, 30, 25, 'm/s'),
      P('lead_gap', 'Initial gap to leader', 10, 80, 28, 'm'),
      P('brake_decel', 'Leader braking deceleration', 2, 9, 7, 'm/s²'),
      P('brake_time', 'Braking start time', 1, 15, 4, 's'),
      P('final_speed', 'Leader speed after braking', 0, 20, 0, 'm/s'),
      P('hold_time', 'Time held at final speed', 0, 30, 8, 's'),
      P('density', 'Traffic density', 0, 30, 12, 'veh/km/lane', 1),
    ],
    build(rng, p, cfg) {
      const ex = rng.uniform(80, 120);
      cfg.ego = laneEgo(1, ex, p.ego_speed);
      const lx = ex + (DIMS.ego.length + DIMS.car.length) / 2 + p.lead_gap;
      const leader = specialNpc(cfg, 'car', 1, lx, p.ego_speed);
      cfg.events.push({ t: p.brake_time, type: 'brake', vehicleId: leader.id, decel: p.brake_decel, targetSpeed: p.final_speed, hold: p.hold_time });
      const laneSpeeds = laneSpeedsFor(p.ego_speed);
      fillTraffic(rng, cfg, { density: p.density, laneSpeeds, exclusions: [{ lane: 1, x0: ex - 40, x1: lx + 80 }] });
      cfg.traffic = { inflow: inflowFor(p.density, laneSpeeds), laneSpeeds };
      cfg.destination = highwayDestination(1000, 1100, 'Highway 1000-1100 m');
    },
  },
  {
    id: 'exit_traffic',
    name: 'Reach the exit through traffic',
    description: 'The ego starts in the leftmost lane and must cross two lanes of traffic to take the exit (deceleration lane from 880 m) and reach the end of the exit ramp.',
    timeLimit: 75,
    params: [
      P('distance_to_exit', 'Distance to deceleration lane', 250, 800, 550, 'm', 10),
      P('ego_speed', 'Ego initial speed', 15, 30, 26, 'm/s'),
      P('density', 'Traffic density', 0, 40, 24, 'veh/km/lane', 1),
      P('traffic_speed', 'Traffic speed (middle lane)', 15, 30, 25, 'm/s'),
    ],
    build(rng, p, cfg) {
      const ex = DECEL_LANE.x0 - p.distance_to_exit;
      cfg.ego = laneEgo(2, ex, p.ego_speed);
      const laneSpeeds = laneSpeedsFor(p.traffic_speed);
      fillTraffic(rng, cfg, { density: p.density, laneSpeeds, exclusions: [{ lane: 2, x0: ex - 40, x1: ex + 40 }] });
      cfg.traffic = { inflow: inflowFor(p.density, laneSpeeds), laneSpeeds };
      const tgt = rampPose('exit', 172.5);
      cfg.destination = { polygon: rampSection('exit', 150, 195), target: [tgt.x, tgt.y], label: 'End of exit ramp' };
    },
  },
];

export function getScenario(id) {
  return SCENARIOS.find((s) => s.id === id) ?? null;
}

export function listScenarios() {
  return SCENARIOS.map(({ id, name, description, timeLimit, params }) => ({ id, name, description, timeLimit, params }));
}

/**
 * Build and validate a starting configuration. Returns {ok, config} or {ok: false, errors}.
 * Identical (scenarioId, seed, params) always produce an identical configuration.
 */
export function buildScenario(scenarioId, seed, params = {}) {
  const sc = getScenario(scenarioId);
  if (!sc) return { ok: false, errors: [`unknown scenario '${scenarioId}'`] };
  if (!isValidSeed(seed)) return { ok: false, errors: [`seed must be an integer in [0, 4294967295] (got ${seed})`] };
  const pv = validateParams(sc, params);
  if (!pv.ok) return { ok: false, errors: pv.errors };
  const cfg = {
    scenarioId: sc.id, name: sc.name, seed, params: pv.params, timeLimit: sc.timeLimit,
    ego: null, npcs: [], events: [], destination: null, traffic: null, nextId: 1,
  };
  sc.build(makeRng(seed), pv.params, cfg);
  delete cfg.nextId;
  const v = validateConfig(cfg);
  if (!v.ok) return { ok: false, errors: v.errors, config: cfg };
  return { ok: true, config: cfg };
}
