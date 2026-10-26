// CFD lab: virtual tow tank for the vehicle hull. Lattice-Boltzmann (D3Q19,
// regularised collision with Smagorinsky LES, interpolated Bouzidi walls) on
// the GPU, CPU fallback in a worker. Runs a grid-convergence study (Richardson
// extrapolation with GCI uncertainty) and a Reynolds extrapolation in all six
// degrees of freedom, turns the forces into the damping matrices of the
// vehicle model and shows the flow around the hull in 3D.

import { h, fmt, toast, seg, checkbox, confirmDialog, selectField } from '../dom.js';
import { StripChart } from '../charts.js';
import { DOFS, QUALITY, RE_PAIR, RE_SINGLE, V_TRANS, V_ROT, assemble, caseSettings, gridLevels, gridConvergence } from '../../cfd/study.js';
import { gpuSupport } from '../../cfd/gpu.js';
import { runGPU } from '../../cfd/runner.js';
import { dragFromProjections, kinematicViscosity } from '../../hydro/water.js';
import { sphere } from '../../geometry/primitives.js';
import { FlowView } from '../flow3d.js';

const S = {
  quality: 'standard', dofs: { surge: true, sway: true, heave: true, roll: true, pitch: true, yaw: true },
  extrapolate: true, gci: true, both: false, addLinear: true, backendChoice: 'auto',
  running: false, abort: null, cancelCPU: null, status: 'Idle', current: null, progress: 0,
  results: [], study: null, verify: null, history: [], slice: null, mlups: null, field: 'speed',
  canvas: null, backend: null, case: null, startedAt: 0, flows: {}, flowShown: null, flowOpts: { streamlines: true, pressure: true },
};

const AXL = ['Surge', 'Sway', 'Heave', 'Roll', 'Pitch', 'Yaw'];

function backend() {
  if (!S.backend) S.backend = typeof document !== 'undefined' ? gpuSupport() : { ok: false, reason: 'no DOM' };
  return S.backend;
}
function useGPU() { return S.backendChoice === 'cpu' ? false : backend().ok; }
function newCanvas() { return h('canvas', { class: 'cfd-canvas', width: 640, height: 300, 'aria-label': 'Flow field' }); }

// JS versions of the shader colour maps (CPU slices)
function seqMap(t) {
  t = Math.min(1, Math.max(0, t));
  const c0 = [9, 23, 33], c1 = [57, 135, 229], c2 = [235, 247, 252];
  const [a, b, u] = t < 0.6 ? [c0, c1, t / 0.6] : [c1, c2, (t - 0.6) / 0.4];
  return a.map((v, i) => v + (b[i] - v) * u);
}
function divMap(t) {
  t = Math.min(1, Math.max(-1, t));
  const mid = [28, 38, 46], neg = [57, 135, 229], pos = [217, 89, 38];
  const [b, u] = t < 0 ? [neg, -t] : [pos, t];
  return mid.map((v, i) => v + (b[i] - v) * u);
}
function drawSlice(canvas, sl, field, uref) {
  const r = canvas.getBoundingClientRect();
  if (r.width > 10) { const w = Math.round(r.width), hh = Math.round(r.height); if (canvas.width !== w || canvas.height !== hh) { canvas.width = w; canvas.height = hh; } }
  const g = canvas.getContext('2d');
  const img = new ImageData(sl.w, sl.h);
  for (let b = 0; b < sl.h; b++) for (let a = 0; a < sl.w; a++) {
    const k = a + sl.w * b, o = 4 * k;
    let c;
    if (sl.solid[k]) c = [201, 211, 216];
    else if (field === 'speed') c = seqMap(sl.speed[k] / (1.6 * uref));
    else if (field === 'pressure') c = divMap(sl.pres[k] / (0.5 * uref * uref) / 1.2);
    else c = divMap(sl.vort[k] / (0.25 * uref));
    img.data[o] = c[0]; img.data[o + 1] = c[1]; img.data[o + 2] = c[2]; img.data[o + 3] = 255;
  }
  const tmp = document.createElement('canvas');
  tmp.width = sl.w; tmp.height = sl.h;
  tmp.getContext('2d').putImageData(img, 0, 0);
  const s = Math.min(canvas.width / sl.w, canvas.height / sl.h);
  g.fillStyle = '#06131b'; g.fillRect(0, 0, canvas.width, canvas.height);
  g.imageSmoothingEnabled = false;
  g.drawImage(tmp, (canvas.width - sl.w * s) / 2, (canvas.height - sl.h * s) / 2, sl.w * s, sl.h * s);
}

// lattice size of a case (for estimates before the hull is voxelised)
function caseDims(app, N, dofKey) {
  const L = Math.max(...app.geo.bbox.size), sz = app.geo.bbox.size, cell = L / N;
  const d = DOFS.find((x) => x.key === dofKey);
  if (d.kind === 't') {
    const along = sz[d.axis], others = sz.filter((_, i) => i !== d.axis);
    return [(along + 3.5 * L) / cell, (others[0] + 1.6 * L) / cell, (others[1] + 1.6 * L) / cell];
  }
  return sz.map((v) => (v + 1.5 * L) / cell);
}
function planWork(app, N, dofKeys, Res, signs, gci) {
  let total = 0, maxCells = 0;
  for (const k of dofKeys) for (const n of gridLevels(N, gci)) {
    const d = DOFS.find((x) => x.key === k);
    const cells = caseDims(app, n, k).reduce((a, b) => a * b, 1);
    maxCells = Math.max(maxCells, cells);
    const Tc = n / (d.kind === 't' ? V_TRANS : V_ROT);
    total += cells * Tc * (d.kind === 't' ? 16 : 11) * Res.length * signs.length; // typical: about 2/3 of tMax
  }
  return { total, maxCells };
}

function drawGPU(solver, cs, st) {
  const cv = S.canvas, r = cv.getBoundingClientRect();
  if (r.width > 10) { const dpr = Math.min(2, devicePixelRatio || 1); const w = Math.round(r.width * dpr), hh = Math.round(r.height * dpr); if (cv.width !== w || cv.height !== hh) { cv.width = w; cv.height = hh; } }
  const axis = cs.kind === 't' ? 'y' : 'z';
  solver.render({ axis, index: cs.ref[axis === 'y' ? 1 : 2], field: S.field, uref: st.V * (cs.kind === 'r' ? 0.5 : 1) });
}

// one lattice run on whichever backend is in use
async function runOne(app, cs, Re, signal, onProg, snapshot) {
  if (useGPU()) {
    if (!S.canvas) S.canvas = newCanvas();
    const st = caseSettings(cs, Re);
    return runGPU(S.canvas, cs, Re, { signal, onProgress: onProg, onFrame: (solver) => drawGPU(solver, cs, st), snapshot });
  }
  const d = app.jobs.runDedicated('cfdrun', { cs, Re, snapshot }, (f, t, m) => m && onProg(m));
  S.cancelCPU = d.cancel;
  try { return await d.promise; } finally { S.cancelCPU = null; }
}

async function runStudy(app) {
  const cfg = app.cfg;
  if (!app.vehiclePositions || !app.geo) { toast('The hull is not ready yet', 'bad'); return; }
  const dofKeys = DOFS.filter((d) => S.dofs[d.key]).map((d) => d.key);
  if (!dofKeys.length) { toast('Pick at least one motion to test'); return; }
  const N = QUALITY[S.quality].N;
  const Res = S.extrapolate ? RE_PAIR : RE_SINGLE;
  const signs = S.both ? [1, -1] : [1];
  const grids = gridLevels(N, S.gci);
  const ellRef = Math.max(...app.geo.bbox.size);
  const cg = app.sim.mixer.cg.slice();
  // coarse grids first: quick feedback, and the fine run (with the 3D snapshot) comes last per case
  const plan = [];
  for (const dof of dofKeys) for (const sign of signs) for (const Re of Res) for (const n of grids.slice().reverse()) plan.push({ dof, sign, Re, N: n });
  S.running = true; S.results = []; S.study = null; S.history = []; S.slice = null;
  S.startedAt = performance.now();
  const ac = new AbortController();
  S.abort = ac;
  const cases = {};
  const positions = app.vehiclePositions;
  const gpu = useGPU();
  try {
    for (let k = 0; k < plan.length; k++) {
      const jb = plan[k];
      const dofDef = DOFS.find((d) => d.key === jb.dof);
      const key = `${jb.dof}:${jb.sign}:${jb.N}`;
      S.current = { ...jb, index: k, total: plan.length, label: `${dofDef.label}${jb.sign < 0 ? ' (reverse)' : ''} · Re ${jb.Re} · ${jb.N} cells` };
      if (!cases[key]) {
        S.status = `Building the ${jb.N}-cell lattice for ${dofDef.label.toLowerCase()}…`;
        cases[key] = await app.jobs.run('cfdcase', { positions, opts: { dof: jb.dof, sign: jb.sign, N: jb.N, ellRef, cg } });
      }
      if (ac.signal.aborted) throw new Error('cancelled');
      const cs = cases[key];
      S.case = cs; S.history = [];
      const st = caseSettings(cs, jb.Re);
      const pScale = cs.kind === 't' ? st.V * st.V * cs.N * cs.N : st.V * st.V * cs.N ** 3;
      const onProg = (m) => {
        S.progress = m.fraction;
        S.history.push([m.t, -m.value / pScale]);
        if (S.history.length > 600) S.history.splice(0, S.history.length - 600);
        if (m.mlups) S.mlups = m.mlups;
        if (m.slice) S.slice = { ...m.slice, uref: st.V * (cs.kind === 'r' ? 0.5 : 1) };
        S.status = `${S.current.label}: ${fmt(m.t, 1)} of at most ${st.tMax} flow times${Number.isFinite(m.rel) ? ` · change ${fmt(m.rel * 100, 1)}%` : ''}`;
      };
      const snapshot = jb.N === grids[0] && jb.Re === Res[Res.length - 1] && jb.sign === 1;
      const res = await runOne(app, cs, jb.Re, ac.signal, onProg, snapshot);
      if (res.field) S.flows[jb.dof] = { field: res.field, cs: { R: cs.R, origin: cs.origin, cell: cs.cell, kind: cs.kind, axis: cs.axis, sign: cs.sign, N: cs.N }, V: st.V, cg, Re: jb.Re };
      S.results.push({ ...jb, C: res.C, converged: res.converged, rel: res.rel, steps: res.steps, seconds: res.seconds, mlups: res.mlups, nuT: res.nuT ?? null,
        lattice: [cs.nx, cs.ny, cs.nz], blockage: cs.blockage, wall: cs.wall, links: cs.linkStats });
    }
    const rho = app.sim.env.rho, nu = kinematicViscosity(cfg.environment.temperature, cfg.environment.salinity);
    const est = dragFromProjections(app.geo.projections, rho, cfg.hydro.damping.Cd ?? 1);
    const out = assemble(S.results, { rho, nu, ellRef, linearExtra: S.addLinear ? est.linear : [0, 0, 0, 0, 0, 0] });
    S.study = { ...out, ref: cg, estimate: est, meta: { date: new Date().toISOString(), quality: S.quality, N, grids, Re: Res, both: S.both, backend: gpu ? 'gpu' : 'cpu', seconds: (performance.now() - S.startedAt) / 1000, ellRef, dofs: dofKeys, linearFromEstimate: S.addLinear, wall: 'interpolated (Bouzidi)', uncertainty: Object.fromEntries(Object.entries(out.details).map(([k, d]) => [k, d.uncertainty])), approximations: Object.fromEntries(Object.entries(out.details).filter(([, d]) => d.approximation).map(([k, d]) => [k, d.approximation])), couplings: S.both ? 'both tow directions' : 'not measured (one tow direction)', warnings: out.warnings, runs: S.results.map((r) => ({ dof: r.dof, sign: r.sign, Re: r.Re, N: r.N, converged: r.converged, steps: r.steps, lattice: r.lattice, nuT: r.nuT })) } };
    S.status = `Finished in ${fmt(S.study.meta.seconds / 60, 1)} min`;
    toast('CFD study finished: review and apply the damping', 'good', 6000);
  } catch (e) {
    S.status = e.message === 'cancelled' ? 'Cancelled' : `Stopped: ${e.message}`;
    if (e.message !== 'cancelled') toast(`CFD study stopped: ${e.message}`, 'bad', 9000);
  } finally {
    S.running = false; S.abort = null; S.cancelCPU = null; S.current = null;
    app.refreshPanelIf?.('cfd');
  }
}

// Solver check: sphere at Re 100 on three grids, Richardson-extrapolated,
// against the standard drag curve (Clift, Grace & Weber 1978)
async function runVerify(app) {
  const pos = Float32Array.from(sphere([], [0, 0, 0], 0.05, 32, 48));
  const Re = 100, ref = (24 / Re) * (1 + 0.1935 * Re ** 0.6305);
  const grids = [16, 11, 8];
  S.running = true; S.verify = { status: 'running' }; S.history = [];
  const ac = new AbortController(); S.abort = ac;
  app.refreshPanelIf?.('cfd');
  try {
    const pts = [];
    for (let k = grids.length - 1; k >= 0; k--) {
      const N = grids[k];
      S.current = { label: `Solver check: sphere, Re 100, ${N} cells per diameter`, index: grids.length - 1 - k, total: grids.length };
      const cs = await app.jobs.run('cfdcase', { positions: pos, opts: { dof: 'surge', sign: 1, N, ellRef: 0.1, cg: [0, 0, 0], margins: { up: 1.5, down: 3, side: 1.25 } } });
      if (ac.signal.aborted) throw new Error('cancelled');
      S.case = cs; S.history = [];
      const st = caseSettings(cs, Re);
      const onProg = (m) => { S.progress = m.fraction; S.history.push([m.t, -m.value / (st.V * st.V * N * N)]); if (m.slice) S.slice = { ...m.slice, uref: st.V }; if (m.mlups) S.mlups = m.mlups; S.status = `${S.current.label}: ${fmt(m.t, 1)} flow times`; };
      const res = await runOne(app, cs, Re, ac.signal, onProg, false);
      pts.push({ h: 1 / N, v: (-res.C[0] * 8) / Math.PI, N, mlups: res.mlups });
    }
    const g = gridConvergence(pts);
    const fine = pts.find((p) => p.N === grids[0]);
    S.verify = { Cd: g.value, fine: fine.v, gci: g.gci, p: g.p, status: g.status, ref, err: g.value / ref - 1, grids: pts.map((p) => [p.N, p.v]).sort((a, b) => b[0] - a[0]), mlups: fine.mlups };
    S.status = 'Solver check finished';
  } catch (e) {
    S.verify = null;
    S.status = e.message === 'cancelled' ? 'Cancelled' : `Solver check failed: ${e.message}`;
    if (e.message !== 'cancelled') toast(S.status, 'bad', 8000);
  } finally {
    S.running = false; S.abort = null; S.cancelCPU = null; S.current = null;
    app.refreshPanelIf?.('cfd');
  }
}

// ---- used by the one-click AUTO-TUNE ----
export async function runCfdStudy(app, { quality, gci = true, extrapolate = true, both = false, dofs } = {}) {
  if (S.running) throw new Error('The CFD lab is already running');
  Object.assign(S, { quality: quality ?? S.quality, gci, extrapolate, both });
  if (dofs) S.dofs = Object.fromEntries(DOFS.map((d) => [d.key, dofs.includes(d.key)]));
  await runStudy(app);
  if (!S.study) throw new Error(S.status === 'Cancelled' ? 'cancelled' : S.status || 'The CFD study did not finish');
  return S.study;
}
export function cfdAbort() { S.abort?.abort(); S.cancelCPU?.(); }
export const cfdState = () => S;
export function cfdStatus() { return { status: S.status, progress: S.progress, current: S.current, mlups: S.mlups, running: S.running }; }
export function cfdUsesGPU() { return useGPU(); }
export function estimateCfdMinutes(app, quality, gci, extrapolate = true, both = false) {
  const keys = DOFS.map((d) => d.key);
  const { total } = planWork(app, QUALITY[quality].N, keys, extrapolate ? RE_PAIR : RE_SINGLE, both ? [1, -1] : [1], gci);
  const rate = S.mlups ? [S.mlups * 0.8, S.mlups * 1.2] : useGPU() ? [60, 400] : [3, 20];
  return [total / rate[1] / 1e6 / 60, total / rate[0] / 1e6 / 60];
}

export function showFlow(app, dof) {
  const flow = S.flows[dof];
  if (!flow) { app.scene.clearFlowOverlay(); S.flowShown = null; return; }
  try {
    const view = new FlowView(flow);
    app.scene.setFlowOverlay(view, S.flowOpts);
    S.flowShown = dof;
  } catch (e) { toast(`Cannot draw the flow: ${e.message}`, 'bad'); }
}

export default {
  id: 'cfd', title: 'CFD lab', short: 'CFD', icon: 'flask', sub: 'Virtual tow tank: flow, drag and damping from your hull',
  create(app) {
    const cfg = app.cfg;
    app.cfdState = S; // debugging and automated checks
    const el = h('div', { class: 'section', style: { gap: '18px' } });
    const be = backend();
    const gpu = useGPU();

    el.append(h('div', { class: 'card' },
      h('p', { class: 'note' }, 'Your hull is placed in a virtual tow tank and the water around it is solved with the lattice-Boltzmann method (D3Q19, Smagorinsky LES), with the exact wall position on every boundary link (Bouzidi interpolation). It is towed forward, sideways and up, and spun about each axis; the forces and moments give the damping matrices, including couplings such as heave-to-pitch.'),
      h('div', { class: 'row', style: { justifyContent: 'space-between' } },
        h('span', { class: 'note' }, gpu ? `GPU: ${be.renderer || 'WebGL2'}` : `CPU solver in the background${be.ok ? '' : ` (GPU not usable: ${be.reason})`}`),
        be.ok ? seg([['auto', 'GPU'], ['cpu', 'CPU']], S.backendChoice === 'cpu' ? 'cpu' : 'auto', (v) => { S.backendChoice = v; app.refreshPanel(); }) : null)));

    // ---- setup ----
    const N = QUALITY[S.quality].N;
    const dofKeys = DOFS.filter((d) => S.dofs[d.key]).map((d) => d.key);
    const Res = S.extrapolate ? RE_PAIR : RE_SINGLE;
    const grids = gridLevels(N, S.gci);
    const work = app.geo ? planWork(app, N, dofKeys, Res, S.both ? [1, -1] : [1], S.gci) : { total: 0, maxCells: 0 };
    const rate = S.mlups ? [S.mlups * 0.8, S.mlups * 1.2] : gpu ? [60, 400] : [3, 20];
    const runs = dofKeys.length * Res.length * (S.both ? 2 : 1) * grids.length;
    const mem = (work.maxCells * (gpu ? 181 : 160)) / 1e6;
    const est = `${runs} runs, about ${fmt(work.total / 1e9, 1)} billion cell updates: roughly ${fmt(work.total / rate[1] / 1e6 / 60, 0)}–${fmt(work.total / rate[0] / 1e6 / 60, 0)} min${S.mlups ? ` at the measured ${fmt(S.mlups, 0)} MLUPS` : gpu ? ' on a laptop or desktop GPU' : ' on the CPU'}. Largest lattice ${fmt(work.maxCells / 1e6, 2)} million cells (about ${fmt(mem, 0)} MB of ${gpu ? 'GPU ' : ''}memory).`;
    const setup = h('div', { class: 'section' }, h('h3', {}, 'Study'),
      h('div', { class: 'field' }, h('span', {}, 'Resolution of the finest grid'), seg(Object.entries(QUALITY).filter(([k, q]) => !q.hidden || k === S.quality).map(([k, q]) => [k, q.label]), S.quality, (v) => { S.quality = v; app.refreshPanel(); })),
      h('span', { class: 'note' }, QUALITY[S.quality].note),
      h('div', { class: 'row' }, DOFS.map((d) => checkbox(d.label, () => S.dofs[d.key], (v) => { S.dofs[d.key] = v; app.refreshPanel(); }))),
      checkbox(`Grid convergence: ${gridLevels(N, true).join(', ')} cells, Richardson extrapolation and an uncertainty (GCI) for every result (about +35% time)`, () => S.gci, (v) => { S.gci = v; app.refreshPanel(); }),
      checkbox(`Reynolds extrapolation (runs at Re ${RE_PAIR.join(' and ')}, extrapolated to your vehicle)`, () => S.extrapolate, (v) => { S.extrapolate = v; app.refreshPanel(); }),
      checkbox('Tow both directions (needed for the couplings between axes and for front/back asymmetric hulls, twice the time)', () => S.both, (v) => { S.both = v; app.refreshPanel(); }),
      checkbox('Keep a small low-speed linear damping from the estimate', () => S.addLinear, (v) => { S.addLinear = v; }),
      h('p', { class: 'note' }, est),
      h('div', { class: 'row' },
        S.running ? h('button', { class: 'btn danger', onclick: () => { S.abort?.abort(); S.cancelCPU?.(); } }, 'Cancel')
          : h('button', { class: 'btn primary', disabled: !app.geo, onclick: () => { runStudy(app); app.refreshPanel(); } }, 'Run CFD study'),
        S.running ? null : h('button', { class: 'btn', onclick: () => { runVerify(app); app.refreshPanel(); } }, 'Check the solver (sphere)')));
    el.append(setup);

    // ---- live view ----
    const status = h('span', { class: 'note' }, S.status);
    const bar = h('div', { class: 'progress' }, h('i', { style: { width: `${Math.round((S.progress || 0) * 100)}%` } }));
    const overall = h('span', { class: 'note mono' });
    let view;
    if (gpu) { if (!S.canvas) S.canvas = newCanvas(); view = S.canvas; } else view = newCanvas();
    Object.assign(view.style, { width: '100%', height: '210px', display: 'block', borderRadius: '8px', background: '#06131b' });
    const chart = new StripChart({ title: 'Force coefficient', unit: '', series: [{ name: 'this run', key: 'v', color: 'var(--series-1)' }], window: 1e9, height: 150, static: true });
    const legend = h('div', { class: 'row', style: { gap: '8px', fontSize: '0.72rem', color: 'var(--ink-3)' } });
    const drawLegend = () => {
      const grad = S.field === 'speed' ? 'linear-gradient(90deg,#091721,#3987e5 60%,#ebf7fc)' : 'linear-gradient(90deg,#3987e5,#1c262e 50%,#d95926)';
      const ends = S.field === 'speed' ? ['still', 'speed 1.6 × tow'] : S.field === 'pressure' ? ['low pressure', 'high pressure'] : ['clockwise', 'counter-clockwise'];
      legend.replaceChildren(h('span', {}, ends[0]), h('i', { style: { display: 'inline-block', width: '120px', height: '8px', borderRadius: '4px', background: grad } }), h('span', {}, ends[1]), h('span', { style: { marginLeft: 'auto' } }, 'grey: hull'));
    };
    drawLegend();
    const fieldSeg = seg([['speed', 'Speed'], ['vorticity', 'Vorticity'], ['pressure', 'Pressure']], S.field, (v) => { S.field = v; drawLegend(); if (!gpu && S.slice) drawSlice(view, S.slice, S.field, S.slice.uref); });
    const viewWrap = h('div', { style: { position: 'relative' } }, view,
      !S.running && !S.case ? h('div', { class: 'note', style: { position: 'absolute', inset: '0', display: 'grid', placeItems: 'center', textAlign: 'center', padding: '16px', pointerEvents: 'none' } }, 'The flow around your hull appears here while a run is in progress.') : null);
    el.append(h('div', { class: 'section' }, h('h3', {}, 'Tow tank'), viewWrap, h('div', { class: 'row', style: { justifyContent: 'space-between' } }, fieldSeg, overall), legend, bar, status, chart.el,
      h('p', { class: 'note' }, 'Side view through the centre of gravity for tows, top view for spins; water flows left to right. Keep this tab visible while the study runs.')));

    const tick = () => {
      status.textContent = S.status;
      bar.firstChild.style.width = `${Math.round((S.progress || 0) * 100)}%`;
      if (S.current) overall.textContent = `run ${S.current.index + 1}/${S.current.total}${S.mlups ? ` · ${fmt(S.mlups, 0)} MLUPS` : ''}`;
      else overall.textContent = S.mlups ? `${fmt(S.mlups, 0)} MLUPS` : '';
      if (S.history.length > 1) chart.draw({ t: Float32Array.from(S.history, (p) => p[0]), v: Float32Array.from(S.history, (p) => p[1]), fixedStart: S.history[0][0] });
      if (!gpu && S.slice) drawSlice(view, S.slice, S.field, S.slice.uref);
    };

    // ---- solver check ----
    if (S.verify && Number.isFinite(S.verify.Cd)) {
      const v = S.verify;
      el.append(h('div', { class: 'card' }, h('b', {}, 'Solver check: sphere at Re 100'),
        h('dl', { class: 'kv' },
          ...v.grids.flatMap(([n, cd]) => [h('dt', {}, `${n} cells per diameter`), h('dd', {}, `Cd ${fmt(cd, 3)}`)]),
          h('dt', {}, 'Grid-converged (Richardson)'), h('dd', {}, `Cd ${fmt(v.Cd, 3)} ± ${fmt(v.gci * 100, 1)}%`),
          h('dt', {}, 'Standard drag curve'), h('dd', {}, fmt(v.ref, 3)),
          h('dt', {}, 'Difference'), h('dd', {}, `${v.err >= 0 ? '+' : ''}${fmt(v.err * 100, 1)} %`),
          h('dt', {}, 'Observed order'), h('dd', {}, `${fmt(v.p, 2)} (${v.status})`)),
        h('p', { class: 'note' }, Math.abs(v.err) < 0.05 ? 'Within 5% of the experimental value: the solver, the wall treatment and the grid extrapolation work on this computer.' : 'More than 5% from the experimental value: treat the study results with extra care on this computer.')));
    }

    // ---- results ----
    if (S.study) el.append(results(app, S.study));
    const dm = cfg.hydro.damping;
    if (dm.mode === 'cfd' && dm.meta) el.append(h('p', { class: 'note' }, `In use: CFD damping from ${new Date(dm.meta.date).toLocaleString()} (${QUALITY[dm.meta.quality]?.label ?? dm.meta.quality}${dm.meta.grids?.length > 1 ? ', grid-converged' : ''}, ${dm.meta.backend?.toUpperCase()}).`));

    // ---- 3D flow ----
    const flows = Object.keys(S.flows);
    if (flows.length) {
      const pick = selectField({ label: 'Show the flow of', options: flows.map((k) => [k, `${DOFS.find((d) => d.key === k).label} (Re ${S.flows[k].Re})`]), get: () => S.flowShown || flows[0], set: (v) => showFlow(app, v) });
      el.append(h('div', { class: 'section' }, h('h3', {}, 'Flow around the hull in 3D'),
        h('p', { class: 'note' }, 'Streamlines show where the water goes as the hull is towed (or spun); the hull is coloured by pressure coefficient Cp. Use the Orbit camera to look around.'),
        pick,
        h('div', { class: 'row' },
          checkbox('Streamlines', () => S.flowOpts.streamlines, (v) => { S.flowOpts.streamlines = v; if (S.flowShown) showFlow(app, S.flowShown); }),
          checkbox('Hull pressure', () => S.flowOpts.pressure, (v) => { S.flowOpts.pressure = v; if (S.flowShown) showFlow(app, S.flowShown); })),
        h('div', { class: 'row', style: { gap: '8px', fontSize: '0.72rem', color: 'var(--ink-3)' } },
          h('span', {}, 'slow'), h('i', { style: { display: 'inline-block', width: '90px', height: '8px', borderRadius: '4px', background: 'linear-gradient(90deg,#21125c,#8c1f8c,#ed6138,#fdd161)' } }), h('span', {}, 'fast'),
          h('span', { style: { marginLeft: '12px' } }, 'Cp −1'), h('i', { style: { display: 'inline-block', width: '90px', height: '8px', borderRadius: '4px', background: 'linear-gradient(90deg,#296bd9,#c7ccd1,#e05c1f)' } }), h('span', {}, '+1')),
        h('div', { class: 'row' },
          h('button', { class: 'btn primary', onclick: () => { showFlow(app, S.flowShown || flows[0]); app.scene.setCameraMode('orbit'); app.camSelect.value = 'orbit'; } }, 'Show in 3D'),
          h('button', { class: 'btn', onclick: () => { app.scene.clearFlowOverlay(); S.flowShown = null; } }, 'Hide'))));
    }

    el.append(h('div', { class: 'section' }, h('h3', {}, 'How far to trust it'),
      h('p', { class: 'note' }, 'Each result carries its grid uncertainty (GCI, 95% confidence). The real vehicle runs at Reynolds numbers near 100 000; the lattice resolves the flow at a few thousand and extrapolates, which is reliable for open frames and boxy hulls whose flow separates at sharp edges, less so for smooth streamlined hulls where the separation point moves with Reynolds number. The sphere check reproduces the standard drag curve within a few percent. Parts thinner than a cell are thickened to one cell.')));

    requestAnimationFrame(tick);
    return { el, tick };
  },
};

function results(app, R) {
  const cfg = app.cfg;
  const info = app.sim.info;
  const est = R.estimate;
  const proj = app.geo.projections;
  const rho = app.sim.env.rho;
  const rows = DOFS.map((d, j) => {
    const has = R.meta.dofs.includes(d.key);
    if (!has) return h('tr', {}, h('td', {}, d.label), h('td', { class: 'num note', colspan: 4 }, 'not tested'));
    const q = R.DQ[j][j];
    const det = R.details[d.key] || {};
    const Cd = j < 3 && proj.area[j] > 0 ? q / (0.5 * rho * proj.area[j]) : NaN;
    const u = det.uncertainty;
    return h('tr', {}, h('td', {}, d.label, det.approximation ? h('span', { class: 'note', title: det.approximation }, ' (approx.)') : null), h('td', { class: 'num' }, sig(est.quadratic[j])), h('td', { class: 'num' }, h('b', {}, sig(q))),
      h('td', { class: 'num' }, Number.isFinite(u) ? `± ${fmt(u * 100, 1)}%` : '—'),
      h('td', { class: 'num' }, Number.isFinite(Cd) ? fmt(Cd, 2) : '—'));
  });
  // drag at a few speeds, with the linear part
  const speeds = [0.25, 0.5, 1.0], rates = [0.5, 1, 2];
  const dragRows = DOFS.map((d, j) => {
    if (!R.meta.dofs.includes(d.key)) return null;
    const vals = (j < 3 ? speeds : rates).map((v) => R.DL[j][j] * v + R.DQ[j][j] * v * v);
    return h('tr', {}, h('td', {}, d.label), ...vals.map((f) => h('td', { class: 'num' }, `${sig(f)} ${j < 3 ? 'N' : 'N·m'}`)));
  }).filter(Boolean);
  // strongest couplings, relative to the diagonal of the moving axis
  const couplings = [];
  for (let j = 0; j < 6; j++) for (let i = 0; i < 6; i++) {
    if (i === j || !R.DQ[j][j]) continue;
    const L = Math.max(1e-6, R.meta.ellRef);
    const rel = Math.abs(R.DQ[i][j]) * (i < 3 && j >= 3 ? L : i >= 3 && j < 3 ? 1 / L : 1);
    if (rel / Math.abs(R.DQ[j][j]) > 0.08) couplings.push({ i, j, v: R.DQ[i][j], rel: rel / Math.abs(R.DQ[j][j]) });
  }
  couplings.sort((a, b) => b.rel - a.rel);
  const notConverged = R.meta.runs.filter((r) => !r.converged).length;
  return h('div', { class: 'section' }, h('h3', {}, 'Result'),
    h('div', { class: 'table-wrap' }, h('table', { class: 'data' },
      h('thead', {}, h('tr', {}, h('th', {}, 'Motion'), h('th', { class: 'num' }, 'Estimate'), h('th', { class: 'num' }, 'CFD'), h('th', { class: 'num' }, 'Grid unc.'), h('th', { class: 'num' }, 'Cd'))),
      h('tbody', {}, rows))),
    h('p', { class: 'note' }, `Quadratic damping (N·s²/m² for tows, N·m·s² for spins). Cd is based on the projected area of your hull. Current model: ${info.dSource}.${notConverged ? ` ${notConverged} run(s) had an unsteady wake; their forces are time averages.` : ''}`),
    R.meta.dofs.some((k) => DOFS.find((d) => d.key === k).kind === 'r') ? h('p', { class: 'note' }, 'Spin damping (marked approx.) is computed on a hull that does not actually turn: its walls move but the geometry stays put, so the wake is that of a quasi-steady spin. Exact for bodies of revolution; for other hulls its uncertainty includes a 30% model term.') : null,
    ...(R.warnings || []).map((w) => h('p', { class: 'note warn' }, w)),
    h('div', { class: 'table-wrap' }, h('table', { class: 'data' },
      h('thead', {}, h('tr', {}, h('th', {}, 'Drag at'), ...speeds.map((v, k) => h('th', { class: 'num' }, `${v} m/s · ${rates[k]} rad/s`)))),
      h('tbody', {}, dragRows))),
    couplings.length ? h('div', { class: 'card' }, h('b', {}, 'Couplings found'), ...couplings.slice(0, 5).map((c) => h('p', { class: 'note' }, `${AXL[c.j]} motion also produces ${AXL[c.i].toLowerCase()} ${c.i < 3 ? 'force' : 'moment'}: ${sig(-c.v)} ${c.i < 3 ? 'N' : 'N·m'} per (${c.j < 3 ? 'm/s' : 'rad/s'})².`)))
      : h('p', { class: 'note' }, R.meta.both ? 'No significant coupling between the axes.' : 'Couplings between the axes were not measured: a force that does not reverse with the motion (lift) cannot be told from drag with one tow direction. Tick "Tow both directions" to get them.'),
    h('div', { class: 'row' },
      h('button', { class: 'btn primary', onclick: () => {
        cfg.hydro.damping = { ...cfg.hydro.damping, mode: 'cfd', DL: R.DL, DQ: R.DQ, ref: R.ref, meta: R.meta };
        app.changed({ immediate: true });
        toast('CFD damping applied to the vehicle', 'good');
        app.refreshPanel();
      } }, 'Use these in the simulation'),
      h('button', { class: 'btn', onclick: async () => { if (await confirmDialog('Discard this CFD result?', 'The current damping stays as it is.', { ok: 'Discard' })) { S.study = null; app.refreshPanel(); } } }, 'Discard')));
}

function sig(v) {
  if (!Number.isFinite(v)) return '—';
  const a = Math.abs(v);
  return a >= 100 ? v.toFixed(0) : a >= 10 ? v.toFixed(1) : a >= 1 ? v.toFixed(2) : v.toFixed(3);
}
