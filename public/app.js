'use strict';
// Browser client: renders state streamed from the backend. All simulation happens on the server;
// rendering rate has no influence on simulation results.

const $ = (id) => document.getElementById(id);
const S = {
  info: null,
  session: null,
  frame: null,
  traj: [],
  log: [],
  zoom: 7,
  keys: { up: false, down: false, left: false, right: false, brake: false },
};

async function api(path, body) {
  const opts = body === undefined ? { method: 'GET' } : { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(body) };
  const r = await fetch(path, opts);
  return r.json().catch(() => ({ ok: false, errors: [`HTTP ${r.status}`] }));
}

const fmt = (v, d = 2) => (typeof v === 'number' ? v.toFixed(d) : v == null ? '–' : String(v));
const scenarioById = (id) => S.info.scenarios.find((s) => s.id === id);

// ---------------------------------------------------------------------------------------------
// Controls

function renderParams() {
  const sc = scenarioById($('scenario').value);
  $('scenario-desc').textContent = `${sc.description} Time limit ${sc.timeLimit} s.`;
  const box = $('params');
  box.innerHTML = '';
  for (const p of sc.params) {
    const div = document.createElement('div');
    div.className = 'param';
    div.innerHTML = `<span>${p.label}${p.unit ? ` (${p.unit})` : ''}</span>` +
      `<input type="number" data-key="${p.key}" min="${p.min}" max="${p.max}" step="${p.step}" value="${p.default}">` +
      `<span class="range muted">${p.key} ∈ [${p.min}, ${p.max}], default ${p.default}</span>`;
    box.appendChild(div);
  }
}

/** Show the loaded session's settings in the form (e.g. after a page reload). */
function syncForm(s) {
  if ($('scenario').value !== s.scenarioId) { $('scenario').value = s.scenarioId; renderParams(); }
  for (const inp of $('params').querySelectorAll('input')) if (s.params[inp.dataset.key] !== undefined) inp.value = s.params[inp.dataset.key];
  $('seed').value = s.seed;
  $('controller').value = s.controllerType;
  showErrors(null);
}

function showErrors(errors) {
  const el = $('load-errors');
  if (!errors || !errors.length) { el.hidden = true; el.textContent = ''; return; }
  el.hidden = false;
  el.textContent = `Configuration rejected:\n• ${errors.join('\n• ')}`;
}

async function load() {
  const params = {};
  for (const inp of $('params').querySelectorAll('input')) params[inp.dataset.key] = inp.value === '' ? NaN : Number(inp.value);
  const seedText = $('seed').value.trim();
  const body = { scenarioId: $('scenario').value, seed: seedText === '' ? null : Number(seedText), params, controller: $('controller').value };
  // NaN is not representable in JSON; send it as a string so the server reports it as invalid.
  for (const k of Object.keys(params)) if (Number.isNaN(params[k])) params[k] = 'NaN';
  $('load').disabled = true;
  try {
    const r = await api('/api/load', body);
    showErrors(r.ok ? null : r.errors);
  } finally {
    $('load').disabled = false;
  }
}

async function command(name) {
  const r = await api(`/api/${name}`, {});
  if (!r.ok) showErrors(r.errors);
}

function updateButtons() {
  const f = S.frame;
  const has = !!S.session?.id;
  const done = !!f?.done;
  $('start').disabled = !has || done || f?.running;
  $('pause').disabled = !has || !f?.running;
  $('step').disabled = !has || done;
  $('reset').disabled = !has;
  $('manual-help').hidden = S.session?.controllerType !== 'manual';
}

// ---------------------------------------------------------------------------------------------
// Keyboard (manual controller)

const KEYMAP = { ArrowUp: 'up', ArrowDown: 'down', ArrowLeft: 'left', ArrowRight: 'right', ' ': 'brake' };

function onKey(e, down) {
  const k = KEYMAP[e.key];
  if (!k || S.session?.controllerType !== 'manual') return;
  if (document.activeElement && ['INPUT', 'SELECT'].includes(document.activeElement.tagName)) return;
  e.preventDefault();
  if (S.keys[k] === down) return;
  S.keys[k] = down;
  api('/api/manual', S.keys);
}

// ---------------------------------------------------------------------------------------------
// Stream

function connect() {
  const es = new EventSource('/api/stream');
  es.addEventListener('session', (ev) => {
    const d = JSON.parse(ev.data);
    S.session = d.id ? d : null;
    if (S.session) syncForm(S.session);
    S.frame = null;
    S.traj = [];
    S.log = [];
    $('events').innerHTML = '';
    updatePanels();
  });
  es.addEventListener('frame', (ev) => {
    const f = JSON.parse(ev.data);
    if (!S.session || f.sessionId !== S.session.id) return;
    if (f.trajFrom === 0) S.traj = [];
    for (const v of f.traj) S.traj.push(v);
    if (f.logFrom === 0) { S.log = []; $('events').innerHTML = ''; }
    for (const e of f.log) { S.log.push(e); appendEvent(e); }
    S.frame = f;
    updatePanels();
  });
  es.onerror = () => { $('status').textContent = 'Disconnected from server – retrying…'; };
}

function appendEvent(e) {
  const li = document.createElement('li');
  li.className = e.severity;
  li.innerHTML = `<span class="t">${Number(e.t).toFixed(2)}s</span><span><span class="type"></span><span class="msg"></span></span>`;
  li.querySelector('.type').textContent = e.type.replace(/_/g, ' ');
  li.querySelector('.msg').textContent = e.message;
  const list = $('events');
  list.appendChild(li);
  list.scrollTop = list.scrollHeight;
  $('event-count').textContent = `(${S.log.length})`;
}

function setBar(id, valId, raw, lo, hi) {
  const bar = $(id), i = bar.querySelector('i');
  const v = typeof raw === 'number' ? raw : NaN;
  const ok = Number.isFinite(v) && v >= lo && v <= hi;
  bar.classList.toggle('invalid', !ok);
  $(valId).textContent = raw == null ? '–' : typeof raw === 'number' ? raw.toFixed(3) : String(raw);
  if (!Number.isFinite(v)) { i.style.left = '50%'; i.style.width = '0'; return; }
  const frac = v >= 0 ? Math.min(v / hi, 1) : -Math.min(v / lo, 1);
  const w = Math.abs(frac) * 50;
  i.style.left = frac >= 0 ? '50%' : `${50 - w}%`;
  i.style.width = `${w}%`;
}

const STATUS_CLASS = { arrived: 'success', timeout: 'warning' };

function updatePanels() {
  updateButtons();
  const s = S.session, f = S.frame;
  if (!s) {
    $('status').textContent = 'No scenario loaded – choose one and press "Load scenario".';
    $('outcome').className = 'outcome pending';
    $('outcome').textContent = 'Not started';
    $('telemetry').innerHTML = '';
    return;
  }
  if (!f) return;
  if (document.activeElement !== $('speed') && Number($('speed').value) !== f.speed) $('speed').value = String(f.speed);
  $('status').textContent = `${s.name} · seed ${s.seed} · ${s.controllerName} · t = ${f.t.toFixed(2)} / ${s.timeLimit} s · ${f.running ? 'running' : f.done ? 'finished' : 'paused'} · ×${f.speed}` + (f.error ? ` · ERROR: ${f.error}` : '');

  const e = f.ego, cmd = f.command;
  const rows = [
    ['Speed', `${e.speed.toFixed(2)} m/s (${(e.speed * 3.6).toFixed(0)} km/h)`],
    ['Speed limit', f.speedLimit == null ? 'off road' : `${f.speedLimit} m/s`],
    ['Region', f.region ? f.region.replace(/_/g, ' ') + (f.lane != null ? `, lane ${f.lane}` : '') : 'off road'],
    ['Position', `x ${e.x.toFixed(1)}, y ${e.y.toFixed(2)}`],
    ['Heading', `${(e.heading * 180 / Math.PI).toFixed(1)}°`],
    ['Applied accel', `${f.applied.accel.toFixed(2)} m/s²`],
    ['Applied steer', `${f.applied.steer.toFixed(4)} rad`],
    ['Controller', cmd?.error ? `error: ${cmd.error}` : s.controllerName],
  ];
  if (f.keys) rows.push(['Keys', Object.entries(f.keys).filter(([, v]) => v).map(([k]) => k).join(' ') || '–']);
  $('telemetry').innerHTML = rows.map(([k, v]) => `<dt>${k}</dt><dd>${v}</dd>`).join('');
  const L = S.info.limits;
  setBar('bar-accel', 'val-accel', cmd && !cmd.error ? cmd.accel : null, L.accelMin, L.accelMax);
  setBar('bar-steer', 'val-steer', cmd && !cmd.error ? cmd.steer : null, -L.steerMax, L.steerMax);

  const o = f.outcome, box = $('outcome');
  if (o) {
    box.className = `outcome ${STATUS_CLASS[o.status] ?? 'failure'}`;
    const st = o.stats;
    box.innerHTML = `${o.status.replace(/_/g, ' ').toUpperCase()} at ${o.t.toFixed(2)} s<small></small>`;
    box.querySelector('small').textContent = `${o.message}. Speed violations: ${st.speedViolations} (${st.speedingTime.toFixed(1)} s, max +${st.maxOverspeed.toFixed(1)} m/s); invalid outputs: ${st.invalidOutputs}; traffic collisions: ${st.trafficCollisions}.`;
  } else {
    box.className = `outcome ${f.running ? 'running' : 'pending'}`;
    box.textContent = f.running ? 'Running…' : f.t > 0 ? 'Paused' : 'Ready';
  }
}

// ---------------------------------------------------------------------------------------------
// Rendering

const view = $('view'), ctx = view.getContext('2d');
const ov = $('overview'), octx = ov.getContext('2d');

function fitCanvas(c) {
  const dpr = window.devicePixelRatio || 1;
  const w = Math.round(c.clientWidth * dpr), h = Math.round(c.clientHeight * dpr);
  if (c.width !== w || c.height !== h) { c.width = w; c.height = h; }
  return dpr;
}

function pathPoly(g, pts, tx, ty) {
  g.beginPath();
  pts.forEach(([x, y], i) => (i ? g.lineTo(tx(x), ty(y)) : g.moveTo(tx(x), ty(y))));
  g.closePath();
}

function drawVehicle(g, x, y, h, len, wid, z, color, opts = {}) {
  g.save();
  g.translate(x, y);
  g.rotate(-h);
  const L = len * z, W = wid * z;
  g.fillStyle = color;
  g.fillRect(-L / 2, -W / 2, L, W);
  if (opts.outline) { g.lineWidth = Math.max(1.5, 0.25 * z); g.strokeStyle = opts.outline; g.strokeRect(-L / 2, -W / 2, L, W); }
  g.fillStyle = 'rgba(20,30,45,0.75)';
  g.fillRect(L / 2 - Math.min(1.7, len * 0.3) * z, -W / 2 + 0.2 * z, 0.6 * z, W - 0.4 * z);
  if (opts.braking) { g.fillStyle = '#ff3030'; g.fillRect(-L / 2, -W / 2, 0.35 * z, 0.5 * z); g.fillRect(-L / 2, W / 2 - 0.5 * z, 0.35 * z, 0.5 * z); }
  if (opts.blink) {
    g.fillStyle = '#ffb000';
    const yy = opts.blink === 'left' ? -W / 2 : W / 2 - 0.45 * z;
    g.fillRect(L / 2 - 0.6 * z, yy, 0.6 * z, 0.45 * z);
    g.fillRect(-L / 2, yy, 0.6 * z, 0.45 * z);
  }
  g.restore();
}

function draw() {
  requestAnimationFrame(draw);
  const dpr = fitCanvas(view);
  const W = view.width, H = view.height;
  ctx.fillStyle = '#1c2a1e';
  ctx.fillRect(0, 0, W, H);
  if (!S.info) return;
  const f = S.frame;
  const z = S.zoom * dpr;
  const cx = f ? f.ego.x + Math.min(30, W / z / 5) : 120, cy = f ? f.ego.y : 0;
  const tx = (x) => (x - cx) * z + W / 2, ty = (y) => H / 2 - (y - cy) * z;

  // Road surface and markings.
  ctx.fillStyle = '#3b4048';
  for (const r of S.info.road.regions) { pathPoly(ctx, r.polygon, tx, ty); ctx.fill(); }
  if (S.session?.destination) {
    pathPoly(ctx, S.session.destination.polygon, tx, ty);
    ctx.fillStyle = 'rgba(62,207,142,0.22)'; ctx.fill();
    ctx.strokeStyle = '#3ecf8e'; ctx.lineWidth = 2; ctx.setLineDash([6, 4]); ctx.stroke(); ctx.setLineDash([]);
    const [dx, dy] = S.session.destination.target;
    ctx.fillStyle = '#3ecf8e'; ctx.font = `${12 * dpr}px system-ui`; ctx.textAlign = 'center';
    ctx.fillText('DESTINATION', tx(dx), ty(dy));
  }
  ctx.strokeStyle = '#e8e8e8';
  ctx.lineWidth = Math.max(1, 0.15 * z);
  for (const m of S.info.road.markings) {
    ctx.setLineDash(m.style === 'dashed' ? [3 * z, 9 * z] : []);
    ctx.beginPath();
    m.pts.forEach(([x, y], i) => (i ? ctx.lineTo(tx(x), ty(y)) : ctx.moveTo(tx(x), ty(y))));
    ctx.stroke();
  }
  ctx.setLineDash([]);

  // Ego trajectory.
  if (S.traj.length >= 4) {
    ctx.strokeStyle = '#33d6ff';
    ctx.lineWidth = Math.max(1.5, 0.25 * z);
    ctx.beginPath();
    for (let i = 0; i < S.traj.length; i += 2) (i ? ctx.lineTo(tx(S.traj[i]), ty(S.traj[i + 1])) : ctx.moveTo(tx(S.traj[i]), ty(S.traj[i + 1])));
    ctx.stroke();
  }
  if (!f) return;

  // Vehicles.
  const blinkOn = Math.floor(performance.now() / 350) % 2 === 0;
  ctx.font = `${10 * dpr}px system-ui`;
  ctx.textAlign = 'center';
  for (const [id, x, y, h, len, wid, , , kind, flags] of f.npcs) {
    const sx = tx(x), sy = ty(y);
    if (sx < -100 || sx > W + 100 || sy < -100 || sy > H + 100) continue;
    const color = flags & 8 ? '#c77dff' : kind === 1 ? '#c98a4b' : '#8fa3bf';
    const blink = blinkOn && (flags & 2 ? 'left' : flags & 4 ? 'right' : null);
    drawVehicle(ctx, sx, sy, h, len, wid, z, color, { braking: flags & 1, blink });
    if (S.zoom >= 5) { ctx.fillStyle = '#d0d6e0'; ctx.fillText(String(id), sx, sy - (wid / 2 + 0.6) * z); }
  }
  const e = f.ego;
  drawVehicle(ctx, tx(e.x), ty(e.y), e.heading, e.length, e.width, z, '#ffcc33', { outline: '#ffffff', braking: e.accel < -1 });
  ctx.fillStyle = '#ffcc33';
  ctx.fillText('EGO', tx(e.x), ty(e.y) - (e.width / 2 + 0.6) * z);

  // Terminal event markers.
  for (const ev of S.log) {
    if ((ev.type === 'collision' || ev.type === 'road_departure') && ev.data) {
      ctx.strokeStyle = '#ff3b3b'; ctx.lineWidth = 3;
      ctx.beginPath(); ctx.arc(tx(ev.data.x), ty(ev.data.y), Math.max(10, 2.5 * z), 0, 2 * Math.PI); ctx.stroke();
    }
  }

  drawOverview(cx, cy, W / z, H / z);
}

function drawOverview(cx, cy, viewW, viewH) {
  const dpr = fitCanvas(ov);
  const W = ov.width, H = ov.height;
  octx.fillStyle = '#16201a';
  octx.fillRect(0, 0, W, H);
  const x0 = S.info.road.xMin - 20, x1 = S.info.road.xMax + 20, y0 = -56, y1 = 15;
  const tx = (x) => ((x - x0) / (x1 - x0)) * W, ty = (y) => H - ((y - y0) / (y1 - y0)) * H;
  octx.fillStyle = '#3b4048';
  for (const r of S.info.road.regions) { pathPoly(octx, r.polygon, tx, ty); octx.fill(); }
  if (S.session?.destination) { pathPoly(octx, S.session.destination.polygon, tx, ty); octx.fillStyle = '#3ecf8e'; octx.fill(); }
  const f = S.frame;
  if (f) {
    octx.fillStyle = '#8fa3bf';
    for (const n of f.npcs) octx.fillRect(tx(n[1]) - 1, ty(n[2]) - 1, 2 * dpr, 2 * dpr);
    octx.fillStyle = '#ffcc33';
    octx.fillRect(tx(f.ego.x) - 3 * dpr, ty(f.ego.y) - 3 * dpr, 6 * dpr, 6 * dpr);
    octx.strokeStyle = 'rgba(255,255,255,0.6)';
    octx.lineWidth = 1;
    octx.strokeRect(tx(cx - viewW / 2), ty(cy + viewH / 2), tx(cx + viewW / 2) - tx(cx - viewW / 2), ty(cy - viewH / 2) - ty(cy + viewH / 2));
  }
}

// ---------------------------------------------------------------------------------------------
// Init

async function init() {
  S.info = await api('/api/info');
  const sel = $('scenario');
  for (const s of S.info.scenarios) sel.add(new Option(s.name, s.id));
  sel.addEventListener('change', renderParams);
  renderParams();
  for (const sp of S.info.speeds) $('speed').add(new Option(`× ${sp}`, sp, sp === 1, sp === 1));
  $('speed').addEventListener('change', () => api('/api/speed', { speed: Number($('speed').value) }));
  const nc = S.info.nativeController;
  $('native-status').textContent = nc.exists ? `Native controller: ${nc.path}` : `Native controller not built (${nc.path}). Run build_controller.bat, or use manual control.`;
  if (!nc.exists) $('controller').value = 'manual';
  $('seed-random').addEventListener('click', () => { $('seed').value = Math.floor(Math.random() * 1e6); });
  $('load').addEventListener('click', load);
  for (const c of ['start', 'pause', 'step', 'reset']) $(c).addEventListener('click', () => command(c));
  view.addEventListener('wheel', (e) => { e.preventDefault(); S.zoom = Math.min(40, Math.max(1, S.zoom * Math.exp(-e.deltaY * 0.0015))); }, { passive: false });
  window.addEventListener('keydown', (e) => onKey(e, true));
  window.addEventListener('keyup', (e) => onKey(e, false));
  window.addEventListener('blur', () => { for (const k of Object.keys(S.keys)) S.keys[k] = false; if (S.session?.controllerType === 'manual') api('/api/manual', S.keys); });
  connect();
  requestAnimationFrame(draw);
}

init();
