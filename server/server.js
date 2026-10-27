// Local HTTP server: serves the browser UI, runs the simulation in real time (scaled by the
// playback speed) and streams state to the browser with Server-Sent Events.
//
// The simulation always advances in fixed PHYS_DT steps; wall-clock time only decides how many
// steps to run, so results do not depend on the server tick rate or the browser frame rate.

import http from 'node:http';
import fs from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import { performance } from 'node:perf_hooks';

import { PHYS_DT, CONTROL_DT, EGO_LIMITS } from './sim/constants.js';
import { listScenarios, buildScenario } from './sim/scenarios.js';
import { Simulation } from './sim/simulation.js';
import { ROAD, MARKINGS } from './sim/road.js';
import { NativeController } from './controllers/native.js';
import { ManualController } from './controllers/manual.js';

const ROOT = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '..');
const PUBLIC_DIR = path.join(ROOT, 'public');
const PORT = Number(process.env.PORT) || 8080;
const HOST = process.env.HOST || '127.0.0.1';
const CONTROLLER_EXE = process.env.CONTROLLER_EXE
  ? path.resolve(process.env.CONTROLLER_EXE)
  : path.join(ROOT, 'build', process.platform === 'win32' ? 'controller.exe' : 'controller');
const SPEEDS = [0.25, 0.5, 1, 2, 4, 8, 16];
const MAX_STEPS_PER_TICK = 1000;

// ---------------------------------------------------------------------------------------------
// Session state. All mutations go through `serial` so stepping never interleaves with reset/load.

let session = null; // { id, config, controllerType, controller, sim }
let sessionId = 0;
let running = false;
let speed = 1;
let budget = 0;
let lastError = null;
let queue = Promise.resolve();

function serial(fn) {
  const p = queue.then(fn);
  queue = p.catch(() => {});
  return p;
}

function makeController(type) {
  if (type === 'manual') return new ManualController();
  if (type === 'native') return new NativeController(CONTROLLER_EXE);
  throw new Error(`unknown controller type '${type}'`);
}

async function loadSession({ scenarioId, seed, params, controller }) {
  const built = buildScenario(scenarioId, seed, params ?? {});
  if (!built.ok) return { ok: false, errors: built.errors };
  const ctl = makeController(controller ?? 'native');
  const sim = new Simulation(built.config, ctl);
  try {
    await sim.start();
  } catch (err) {
    ctl.close();
    return { ok: false, errors: [err.message] };
  }
  session?.controller.close();
  session = { id: ++sessionId, config: built.config, controllerType: controller ?? 'native', controller: ctl, sim };
  running = false;
  budget = 0;
  lastError = null;
  for (const c of clients) sendSession(c);
  return { ok: true };
}

// ---------------------------------------------------------------------------------------------
// Real-time loop.

let lastTick = performance.now();

async function tick() {
  const now = performance.now();
  const wall = Math.min((now - lastTick) / 1000, 0.25);
  lastTick = now;
  if (session && running && !session.sim.done) {
    budget += wall * speed;
    await serial(async () => {
      const sim = session.sim;
      let n = 0;
      while (budget >= PHYS_DT && !sim.done && n < MAX_STEPS_PER_TICK) {
        await sim.step();
        budget -= PHYS_DT;
        n++;
      }
      if (sim.done) running = false;
    }).catch((err) => { lastError = err.message; running = false; });
  } else {
    budget = 0;
  }
  setTimeout(tick, 5);
}

// ---------------------------------------------------------------------------------------------
// Server-Sent Events.

const clients = new Set();

const jsonReplacer = (_k, v) => (typeof v === 'number' && !Number.isFinite(v) ? String(v) : v);
const toJson = (v) => JSON.stringify(v, jsonReplacer);

function sse(client, event, data) {
  client.res.write(`event: ${event}\ndata: ${toJson(data)}\n\n`);
}

function sendSession(client) {
  client.trajSent = 0;
  client.logSent = 0;
  if (!session) {
    sse(client, 'session', { id: 0 });
    return;
  }
  const c = session.config;
  sse(client, 'session', {
    id: session.id,
    scenarioId: c.scenarioId,
    name: c.name,
    seed: c.seed,
    params: c.params,
    timeLimit: c.timeLimit,
    destination: c.destination,
    controllerType: session.controllerType,
    controllerName: session.controller.name,
    npcCount: c.npcs.length,
  });
  sendFrame(client);
}

function sendFrame(client) {
  if (!session) return;
  const sim = session.sim;
  const traj = [];
  for (let i = client.trajSent; i < sim.trajectory.length; i++) traj.push(sim.trajectory[i][1], sim.trajectory[i][2]);
  const frame = {
    sessionId: session.id,
    ...sim.frame(),
    running,
    speed,
    error: lastError,
    trajFrom: client.trajSent,
    traj,
    logFrom: client.logSent,
    log: sim.log.slice(client.logSent),
    keys: session.controllerType === 'manual' ? session.controller.keys : null,
  };
  client.trajSent = sim.trajectory.length;
  client.logSent = sim.log.length;
  sse(client, 'frame', frame);
}

let lastBroadcastStep = -1;
let lastBroadcastState = '';
setInterval(() => {
  if (!session) return;
  const state = `${session.id}|${session.sim.stepCount}|${running}|${speed}|${lastError}`;
  if (state === lastBroadcastState && session.sim.stepCount === lastBroadcastStep) return;
  lastBroadcastState = state;
  lastBroadcastStep = session.sim.stepCount;
  for (const c of clients) sendFrame(c);
}, 33);

setInterval(() => { for (const c of clients) c.res.write(': keep-alive\n\n'); }, 15000).unref();

// ---------------------------------------------------------------------------------------------
// HTTP.

const MIME = { '.html': 'text/html; charset=utf-8', '.js': 'text/javascript; charset=utf-8', '.css': 'text/css; charset=utf-8', '.svg': 'image/svg+xml', '.json': 'application/json' };

function sendJson(res, status, body) {
  res.writeHead(status, { 'Content-Type': 'application/json', 'Cache-Control': 'no-store' });
  res.end(toJson(body));
}

function readBody(req) {
  return new Promise((resolve, reject) => {
    let data = '';
    req.setEncoding('utf8');
    req.on('data', (d) => {
      data += d;
      if (data.length > 1e6) { reject(new Error('request body too large')); req.destroy(); }
    });
    req.on('end', () => {
      if (!data) return resolve({});
      try { resolve(JSON.parse(data)); } catch { reject(new Error('invalid JSON body')); }
    });
    req.on('error', reject);
  });
}

function serveStatic(req, res) {
  const rel = decodeURIComponent(new URL(req.url, 'http://x').pathname);
  const file = path.normalize(path.join(PUBLIC_DIR, rel === '/' ? 'index.html' : rel));
  if (!file.startsWith(PUBLIC_DIR)) { res.writeHead(403); res.end(); return; }
  fs.readFile(file, (err, buf) => {
    if (err) { res.writeHead(404); res.end('not found'); return; }
    res.writeHead(200, { 'Content-Type': MIME[path.extname(file)] ?? 'application/octet-stream', 'Cache-Control': 'no-cache' });
    res.end(buf);
  });
}

const needSession = (res) => {
  if (session) return false;
  sendJson(res, 409, { ok: false, errors: ['no scenario loaded'] });
  return true;
};

const routes = {
  'GET /api/info': (_req, res) => sendJson(res, 200, {
    scenarios: listScenarios(),
    road: { ...ROAD, markings: MARKINGS },
    limits: EGO_LIMITS,
    physDt: PHYS_DT,
    controlDt: CONTROL_DT,
    speeds: SPEEDS,
    nativeController: { path: CONTROLLER_EXE, exists: NativeController.exists(CONTROLLER_EXE) },
  }),

  'GET /api/stream': (req, res) => {
    res.writeHead(200, { 'Content-Type': 'text/event-stream', 'Cache-Control': 'no-cache', Connection: 'keep-alive' });
    const client = { res, trajSent: 0, logSent: 0 };
    clients.add(client);
    sendSession(client);
    req.on('close', () => clients.delete(client));
  },

  'POST /api/load': async (req, res) => {
    const body = await readBody(req);
    const seed = typeof body.seed === 'string' && body.seed.trim() !== '' ? Number(body.seed) : body.seed;
    const r = await serial(() => loadSession({ ...body, seed }));
    sendJson(res, r.ok ? 200 : 400, r);
  },

  'POST /api/reset': async (_req, res) => {
    if (needSession(res)) return;
    await serial(async () => {
      running = false;
      budget = 0;
      lastError = null;
      await session.sim.reset();
      for (const c of clients) sendSession(c);
    });
    sendJson(res, 200, { ok: true });
  },

  'POST /api/start': async (_req, res) => {
    if (needSession(res)) return;
    if (!session.sim.done) { running = true; lastTick = performance.now(); }
    sendJson(res, 200, { ok: true, running });
  },

  'POST /api/pause': async (_req, res) => {
    running = false;
    sendJson(res, 200, { ok: true });
  },

  'POST /api/step': async (_req, res) => {
    if (needSession(res)) return;
    running = false;
    await serial(() => session.sim.stepControlPeriod());
    sendJson(res, 200, { ok: true, t: session.sim.t });
  },

  'POST /api/speed': async (req, res) => {
    const body = await readBody(req);
    if (!SPEEDS.includes(body.speed)) return sendJson(res, 400, { ok: false, errors: [`speed must be one of ${SPEEDS.join(', ')}`] });
    speed = body.speed;
    sendJson(res, 200, { ok: true, speed });
  },

  'POST /api/manual': async (req, res) => {
    const body = await readBody(req);
    if (session?.controllerType === 'manual') session.controller.setKeys(body);
    sendJson(res, 200, { ok: true });
  },
};

const server = http.createServer(async (req, res) => {
  const key = `${req.method} ${new URL(req.url, 'http://x').pathname}`;
  const route = routes[key];
  if (!route) {
    if (req.method === 'GET') return serveStatic(req, res);
    return sendJson(res, 404, { ok: false, errors: ['not found'] });
  }
  try {
    await route(req, res);
  } catch (err) {
    if (!res.headersSent) sendJson(res, 500, { ok: false, errors: [err.message] });
  }
});

server.listen(PORT, HOST, () => {
  console.log(`Driving simulation running at http://${HOST === '0.0.0.0' ? 'localhost' : HOST}:${PORT}/`);
  console.log(`Native controller: ${CONTROLLER_EXE}${NativeController.exists(CONTROLLER_EXE) ? '' : '  (NOT BUILT - run npm run build:controller)'}`);
  tick();
});

function shutdown() {
  session?.controller.close();
  process.exit(0);
}
process.on('SIGINT', shutdown);
process.on('SIGTERM', shutdown);
