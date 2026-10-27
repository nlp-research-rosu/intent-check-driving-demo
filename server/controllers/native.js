// Runs a compiled C controller (controller_host.c + your controller) as a child process.
// The process is restarted for every episode so that no state survives a reset.

import { spawn } from 'node:child_process';
import fs from 'node:fs';
import { REGION_KINDS } from '../sim/road.js';

const num = (v) => (Number.isFinite(v) ? String(v) : 'nan');

/** Parse a number printed by C's printf("%.17g"), including inf/nan spellings. */
export function parseCNumber(tok) {
  const t = String(tok).trim().toLowerCase();
  if (t === '') return NaN;
  if (/^[+-]?inf/.test(t)) return t.startsWith('-') ? -Infinity : Infinity;
  if (t.includes('nan')) return NaN;
  const v = Number(t);
  return v;
}

export function serializeReset(ctx) {
  const L = ctx.limits;
  const parts = ['R', ctx.seed >>> 0, num(ctx.controlDt),
    num(L.accelMin), num(L.accelMax), num(L.steerMax), num(L.steerRateMax), num(L.latAccelMax), num(L.speedMax),
    num(ctx.road.laneWidth), ctx.road.numLanes, num(ctx.road.xMin), num(ctx.road.xMax), ctx.road.regions.length];
  for (const r of ctx.road.regions) {
    parts.push(REGION_KINDS.indexOf(r.kind), num(r.speedLimit), r.polygon.length);
    for (const [x, y] of r.polygon) parts.push(num(x), num(y));
  }
  const d = ctx.destination;
  parts.push(d.polygon.length);
  for (const [x, y] of d.polygon) parts.push(num(x), num(y));
  parts.push(num(d.target[0]), num(d.target[1]));
  return parts.join(' ');
}

export function serializeStep(obs) {
  const e = obs.ego;
  const parts = ['S', num(obs.time), obs.step,
    num(e.x), num(e.y), num(e.heading), num(e.speed), num(e.accel), num(e.steer), num(e.length), num(e.width), num(e.wheelbase),
    num(obs.steerMaxNow), obs.objects.length];
  for (const o of obs.objects) parts.push(o.id, num(o.x), num(o.y), num(o.heading), num(o.speed), num(o.accel), num(o.length), num(o.width));
  return parts.join(' ');
}

export class NativeController {
  /**
   * @param {string} exePath compiled controller executable
   * @param {{timeoutMs?:number, log?:(s:string)=>void}} [opts]
   */
  constructor(exePath, { timeoutMs = 2000, log = (s) => process.stderr.write(s) } = {}) {
    this.exePath = exePath;
    this.timeoutMs = timeoutMs;
    this.log = log;
    this.proc = null;
    this.name = null;
    this.buffer = '';
    this.waiter = null;
  }

  static exists(exePath) {
    return fs.existsSync(exePath);
  }

  _readLine() {
    return new Promise((resolve, reject) => {
      const timer = setTimeout(() => {
        this.waiter = null;
        this._kill();
        reject(new Error(`controller did not respond within ${this.timeoutMs} ms`));
      }, this.timeoutMs);
      this.waiter = {
        resolve: (l) => { clearTimeout(timer); resolve(l); },
        reject: (e) => { clearTimeout(timer); reject(e); },
      };
      this._drain();
    });
  }

  _drain() {
    if (!this.waiter) return;
    const i = this.buffer.indexOf('\n');
    if (i < 0) return;
    const line = this.buffer.slice(0, i).replace(/\r$/, '');
    this.buffer = this.buffer.slice(i + 1);
    const w = this.waiter;
    this.waiter = null;
    w.resolve(line);
  }

  async _request(line) {
    if (!this.proc) throw new Error('controller process is not running');
    const p = this._readLine();
    this.proc.stdin.write(line + '\n');
    return p;
  }

  _kill() {
    if (this.proc) {
      const p = this.proc;
      this.proc = null;
      try { p.stdin.end('Q\n'); } catch { /* already closed */ }
      setTimeout(() => { try { p.kill(); } catch { /* gone */ } }, 200).unref();
    }
  }

  async _start() {
    this._kill();
    if (!NativeController.exists(this.exePath)) throw new Error(`controller executable not found: ${this.exePath} (run npm run build:controller)`);
    this.buffer = '';
    const proc = spawn(this.exePath, [], { stdio: ['pipe', 'pipe', 'pipe'], windowsHide: true });
    this.proc = proc;
    proc.stdout.setEncoding('utf8');
    proc.stdout.on('data', (d) => { if (this.proc === proc) { this.buffer += d; this._drain(); } });
    proc.stderr.setEncoding('utf8');
    proc.stderr.on('data', (d) => this.log(`[controller] ${d}`));
    proc.stdin.on('error', () => { /* reported via exit */ });
    const onGone = (why) => {
      if (this.proc !== proc) return;
      this.proc = null;
      this.waiter?.reject(new Error(`controller process ${why}`));
      this.waiter = null;
    };
    proc.on('exit', (code, sig) => onGone(`exited (code ${code}${sig ? `, signal ${sig}` : ''})`));
    proc.on('error', (err) => onGone(`failed: ${err.message}`));
    const ready = await this._readLine();
    const m = /^READY (\d+) ?(.*)$/.exec(ready);
    if (!m) throw new Error(`unexpected controller greeting: ${ready}`);
    if (m[1] !== '1') throw new Error(`controller API version ${m[1]} is not supported (expected 1)`);
    this.name = m[2] || 'native controller';
  }

  async reset(ctx) {
    await this._start();
    const resp = await this._request(serializeReset(ctx));
    if (resp !== 'OK') throw new Error(`controller rejected reset: ${resp}`);
  }

  async step(obs) {
    let resp;
    try {
      resp = await this._request(serializeStep(obs));
    } catch (err) {
      return { error: err.message };
    }
    if (resp.startsWith('E ')) return { error: `controller host: ${resp.slice(2)}` };
    const tok = resp.split(/\s+/);
    if (tok[0] !== 'C' || tok.length !== 3) return { error: `malformed controller response: ${resp.slice(0, 80)}` };
    return { accel: parseCNumber(tok[1]), steer: parseCNumber(tok[2]) };
  }

  close() {
    this._kill();
  }
}
