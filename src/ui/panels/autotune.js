// One-click AUTO-TUNE: for the design that is loaded, compute the added mass
// (panel method, panel-converged), the damping (CFD lab, grid-converged,
// Reynolds-extrapolated, six degrees of freedom) and then tune the ArduSub
// controllers on that hydrodynamic model. Everything ends in an editable
// report: nothing is applied until the user says so.

import { h, fmt, toast, seg, checkbox, saveFile } from '../dom.js';
import { StripChart } from '../charts.js';
import { STEP_AXES } from '../../sim/tuning.js';
import { DOFS } from '../../cfd/study.js';
import { runCfdStudy, cfdAbort, cfdStatus, cfdUsesGPU, estimateCfdMinutes, cfdState } from './cfd.js';
import { slug } from './params.js';

const PRECISION = {
  max: { label: 'Maximum', bem: [1800, 3600, 7200], cfd: 'fine', gci: true, both: true, note: 'Finest grids, three-level convergence for both the panel method and the CFD, tows in both directions (so the couplings between axes are measured).' },
  high: { label: 'High', bem: [900, 1800, 3600], cfd: 'standard', gci: true, note: 'Standard CFD grid with convergence study: about a quarter of the time of Maximum.' },
  fast: { label: 'Fast', bem: [1800], cfd: 'quick', gci: false, note: 'One grid each: a first look in a few minutes, no uncertainty estimate.' },
  test: { label: 'Test', bem: [400], cfd: 'test', gci: false, note: 'Smoke test only.', hidden: true },
};
const STAGES = [
  ['inputs', 'Inputs you entered'],
  ['bem', 'Added mass (panel method)'],
  ['cfd', 'Drag and damping (CFD, 6 DOF)'],
  ['tune', 'ArduSub controller gains'],
];
const AT = { precision: 'max', running: false, cancelled: false, stages: {}, error: null, result: null, edits: {}, include: {}, useHydro: true, startedAt: 0, finishedAt: 0 };

function inputsCheck(app) {
  const cfg = app.cfg;
  const items = [];
  items.push(cfg.mass.source === 'user' ? { ok: true, text: `Mass ${fmt(cfg.mass.mass, 2)} kg (entered)` } : { ok: false, text: `Mass ${fmt(cfg.mass.mass, 2)} kg is an estimate (neutral buoyancy). Enter the real mass in Vehicle.` });
  items.push(cfg.mass.cgMode === 'manual' ? { ok: true, text: 'Centre of gravity from CAD' } : { ok: false, text: `Centre of gravity = centre of buoyancy + offset (${cfg.mass.cgOffset.map((v) => fmt(v * 100, 1)).join(', ')} cm). The CAD value is better.` });
  items.push(cfg.mass.inertia.mode === 'manual' ? { ok: true, text: 'Inertia from CAD' } : { ok: false, text: 'Inertia estimated from the hull at uniform density. CAD values make the roll/pitch tuning more exact.' });
  const tabled = cfg.thrusters.filter((t) => Array.isArray(t.table) && t.table.length >= 2).length;
  items.push(tabled === cfg.thrusters.length ? { ok: true, text: 'Measured thrust curves for every thruster' } : { ok: false, text: `${cfg.thrusters.length - tabled} of ${cfg.thrusters.length} thrusters use a catalogue curve (${[...new Set(cfg.thrusters.map((t) => t.preset))].join(', ')}). A measured PWM-to-thrust table is more exact.` });
  if (cfg.model.source !== 'stl') items.push({ ok: false, text: 'This is the sample vehicle: upload your design first to tune for it.' });
  return items;
}

async function runPipeline(app) {
  const P = PRECISION[AT.precision];
  AT.running = true; AT.cancelled = false; AT.error = null; AT.result = null; AT.edits = {}; AT.include = {};
  AT.startedAt = performance.now();
  AT.stages = Object.fromEntries(STAGES.map(([k]) => [k, { status: 'pending', progress: 0, text: '' }]));
  const st = (k, o) => Object.assign(AT.stages[k], o);
  const cfg = app.cfg;
  try {
    // 1 inputs
    const inputs = inputsCheck(app);
    st('inputs', { status: 'done', progress: 1, text: `${inputs.filter((i) => i.ok).length}/${inputs.length} measured` });
    // 2 added mass
    st('bem', { status: 'running', text: 'Meshing the hull' });
    const bem = await app.jobs.run('bemconv', { positions: app.vehiclePositions, rho: app.sim.env.rho, ref: [0, 0, 0], levels: P.bem }, (f, t) => st('bem', { progress: f, text: t || '' }));
    if (AT.cancelled) throw new Error('cancelled');
    st('bem', { status: 'done', progress: 1, text: `${bem.levels.join(' / ')} panels` });
    // 3 damping
    st('cfd', { status: 'running', text: 'Starting the tow tank' });
    const quality = cfdUsesGPU() || P.cfd === 'test' ? P.cfd : 'quick';
    const study = await runCfdStudy(app, { quality, gci: P.gci, extrapolate: true, both: !!P.both, dofs: DOFS.map((d) => d.key) });
    if (AT.cancelled) throw new Error('cancelled');
    st('cfd', { status: 'done', progress: 1, text: `${study.meta.runs.length} runs in ${fmt(study.meta.seconds / 60, 1)} min` });
    // 4 controllers on the new hydrodynamics
    st('tune', { status: 'running', text: 'Step tests' });
    const hydro = {
      addedMass: { ...cfg.hydro.addedMass, mode: 'bem', matrix: bem.M, ref: bem.ref, meta: { panels: bem.panels, levels: bem.levels, uncertainty: bem.uncertainty, converged: true, date: new Date().toISOString(), rho: app.sim.env.rho } },
      damping: { ...cfg.hydro.damping, mode: 'cfd', DL: study.DL, DQ: study.DQ, ref: study.ref, meta: study.meta },
    };
    const cfgT = JSON.parse(JSON.stringify({ ...cfg, hydro: { ...cfg.hydro, ...hydro } }));
    // suggestions only: nothing is written to the vehicle until "Apply selected"
    const axes = ['roll', 'pitch', 'yaw', 'depth', ...(cfg.sensors?.positioning ? ['poshold'] : [])];
    const tune = await app.jobs.run('autotune', { cfg: cfgT, geo: app.geo, opts: { axes, quick: false } }, (f, t) => st('tune', { progress: f, text: t || '' }));
    st('tune', { status: 'done', progress: 1, text: 'Done' });
    AT.result = { inputs, bem, study, tune, hydro, before: { addedMass: app.sim.info.MA.map((r, i) => r[i]), damping: app.sim.info.DQ.map((r, i) => r[i]), dSource: app.sim.info.dSource, maSource: app.sim.info.maSource }, precision: AT.precision, quality };
    // editable suggestions (axes that keep their gains start unticked)
    for (const [axis, rep] of Object.entries(tune.report)) {
      if (rep.skipped) continue;
      for (const [k, v] of Object.entries(rep.suggested || rep.params || {})) { AT.edits[k] = +(+v).toPrecision(4); AT.include[k] = !rep.keep; }
    }
    AT.finishedAt = performance.now();
    toast('AUTO-TUNE finished: check the report and apply', 'good', 6000);
  } catch (e) {
    const cancelled = e.message === 'cancelled' || AT.cancelled;
    AT.error = cancelled ? 'Cancelled' : e.message;
    for (const [k, s] of Object.entries(AT.stages)) if (s.status === 'running') s.status = cancelled ? 'cancelled' : 'failed';
    if (!cancelled) toast(`AUTO-TUNE stopped: ${e.message}`, 'bad', 9000);
  } finally {
    AT.running = false;
    app.refreshPanelIf('autotune');
  }
}

function apply(app) {
  const R = AT.result;
  const cfg = app.cfg;
  let n = 0;
  if (AT.useHydro) {
    cfg.hydro.addedMass = R.hydro.addedMass;
    cfg.hydro.damping = R.hydro.damping;
    app.changed({ immediate: true });
  }
  for (const [k, v] of Object.entries(AT.edits)) {
    if (!AT.include[k] || !Number.isFinite(v)) continue;
    if (cfg.params[k] !== v) { app.setParam(k, v); n++; }
  }
  toast(`Applied ${AT.useHydro ? 'the computed hydrodynamics and ' : ''}${n} parameter change${n === 1 ? '' : 's'}`, 'good', 6000);
  app.refreshPanel();
}

function exportReport(app) {
  const R = AT.result;
  const doc = {
    schema: 'duburi.autotune', date: new Date().toISOString(), design: app.cfg.model.fileName || app.cfg.name, precision: R.precision,
    inputs: R.inputs,
    addedMass: { matrix: R.bem.M, panels: R.bem.levels, uncertainty: R.bem.uncertainty, reference: 'vehicle origin' },
    damping: { DL: R.study.DL, DQ: R.study.DQ, reference: 'centre of gravity', uncertainty: R.study.meta.uncertainty, runs: R.study.meta.runs },
    gains: Object.fromEntries(Object.keys(AT.edits).map((k) => [k, { current: app.cfg.params[k], tuned: AT.edits[k], use: !!AT.include[k] }])),
    stepTests: Object.fromEntries(Object.entries(R.tune.report).filter(([, r]) => !r.skipped).map(([a, r]) => [a, { before: r.before, after: r.after, kept: !!r.keep, method: r.method, atBound: r.atBound, notes: r.notes }])),
    tuneNotes: R.tune.notes, cfdWarnings: R.study.warnings,
  };
  saveFile(`${slug(app.cfg.name)}-autotune.json`, JSON.stringify(doc, null, 1), 'application/json');
}

export default {
  id: 'autotune', title: 'AUTO-TUNE', short: 'Auto-tune', icon: 'target', sub: 'Hydrodynamics and ArduSub gains for this design, at maximum precision', hidden: true,
  create(app) {
    app.autoTuneState = AT; // debugging and automated checks
    app.cfdState = cfdState();
    const el = h('div', { class: 'section', style: { gap: '18px' } });
    const cfg = app.cfg;
    const inputs = inputsCheck(app);
    const P = PRECISION[AT.precision];
    const gpu = cfdUsesGPU();
    const quality = gpu ? P.cfd : 'quick';
    const est = app.geo ? estimateCfdMinutes(app, quality, P.gci, true, !!P.both) : [0, 0];
    const bemMin = P.bem.reduce((a, n) => a + (n / 3600) ** 2 * 0.25, 0); // minutes, rough (WebAssembly)
    el.append(h('div', { class: 'card' },
      h('b', {}, `Design: ${cfg.model.fileName || 'sample ROV'}`),
      h('p', { class: 'note' }, 'AUTO-TUNE computes this design\'s added mass with the panel method and its drag and damping in the CFD lab, each on several resolutions with Richardson extrapolation and an uncertainty, then tunes the controllers on that model the way ArduPilot\'s AutoTune does: rate-loop twitches (D up until bounce-back, then P up until overshoot), angle P and acceleration limits from angle twitches, then input shaping, depth hold and position hold on top, all inside the ArduSub parameter ranges, and a final check of every axis with the complete set. You see every value before anything is applied, and you can change them.')));

    // inputs check
    el.append(h('div', { class: 'section' }, h('h3', {}, 'Your inputs'),
      ...inputs.map((i) => h('div', { class: 'row', style: { flexWrap: 'nowrap', alignItems: 'flex-start' } }, h('span', { class: `pill ${i.ok ? 'good' : 'warn'}` }, i.ok ? 'entered' : 'estimate'), h('span', { class: 'note' }, i.text))),
      h('div', { class: 'row' }, h('button', { class: 'btn', onclick: () => app.openPanel('vehicle') }, 'Mass, CG, inertia'), h('button', { class: 'btn', onclick: () => app.openPanel('thrusters') }, 'Thrust curves'))));

    // controls
    if (!AT.running) {
      el.append(h('div', { class: 'section' }, h('h3', {}, 'Precision'),
        seg(Object.entries(PRECISION).filter(([k, v]) => !v.hidden || k === AT.precision).map(([k, v]) => [k, v.label]), AT.precision, (v) => { AT.precision = v; app.refreshPanel(); }),
        h('p', { class: 'note' }, `${P.note} Estimated time: ${fmt(bemMin + est[0], 0)}–${fmt(bemMin + est[1] + 0.5, 0)} min${gpu ? '' : ' (no usable GPU here: the CFD runs on the CPU at Quick resolution)'}. Keep this tab open and visible.`),
        h('button', { class: 'btn primary', style: { alignSelf: 'flex-start', height: '40px', fontSize: '1rem' }, disabled: !app.geo, onclick: () => { runPipeline(app); app.refreshPanel(); } }, AT.result ? 'Run AUTO-TUNE again' : 'Start AUTO-TUNE')));
    }

    // progress
    const stageEls = STAGES.map(([k, label]) => {
      const row = h('div', { class: 'card', style: { padding: '10px 12px', gap: '6px' } });
      row.draw = () => {
        const s = AT.stages[k] || { status: 'pending', progress: 0, text: '' };
        const pill = { pending: ['muted', 'waiting'], running: ['warn', 'running'], done: ['good', 'done'], failed: ['bad', 'failed'], cancelled: ['muted', 'cancelled'] }[s.status] || ['muted', s.status];
        let text = s.text;
        if (k === 'cfd' && s.status === 'running') { const c = cfdStatus(); text = c.status; s.progress = c.current ? (c.current.index + (c.progress || 0)) / c.current.total : 0; }
        row.replaceChildren(...[
          h('div', { class: 'row', style: { justifyContent: 'space-between' } }, h('b', {}, label), h('span', { class: `pill ${pill[0]}` }, pill[1])),
          s.status === 'running' ? h('div', { class: 'progress' }, h('i', { style: { width: `${Math.round((s.progress || 0) * 100)}%` } })) : null,
          text ? h('span', { class: 'note' }, text) : null].filter(Boolean));
      };
      row.draw();
      return row;
    });
    if (AT.running || Object.keys(AT.stages).length) {
      el.append(h('div', { class: 'section' }, h('h3', {}, 'Progress'), ...stageEls,
        AT.running ? h('button', { class: 'btn danger', style: { alignSelf: 'flex-start' }, onclick: () => { AT.cancelled = true; cfdAbort(); } }, 'Cancel') : null,
        AT.error ? h('p', { class: `note ${AT.error === 'Cancelled' ? '' : 'bad'}` }, AT.error) : null));
    }

    // report
    const draws = [];
    if (AT.result && !AT.running) el.append(report(app, draws));
    requestAnimationFrame(() => draws.forEach((d) => d()));
    return { el, tick: () => stageEls.forEach((r) => r.draw()) };
  },
};

function report(app, draws) {
  const R = AT.result;
  const cfg = app.cfg;
  const wrap = h('div', { class: 'section', style: { gap: '16px' } }, h('h3', {}, 'Report'),
    h('p', { class: 'note' }, `${PRECISION[R.precision].label} precision · CFD ${R.quality} grid${R.study.meta.grids.length > 1 ? 's ' + R.study.meta.grids.join('/') : ''} · ${fmt((AT.finishedAt - AT.startedAt) / 60000, 1)} min. Uncertainties are 95% grid-convergence indices.`));

  // hydrodynamics
  const AX = ['Surge', 'Sway', 'Heave', 'Roll', 'Pitch', 'Yaw'];
  const UM = ['kg', 'kg', 'kg', 'kg·m²', 'kg·m²', 'kg·m²'], UD = ['N·s²/m²', 'N·s²/m²', 'N·s²/m²', 'N·m·s²', 'N·m·s²', 'N·m·s²'];
  const maNew = R.bem.M.map((r, i) => r[i]);
  const dqNew = R.study.DQ.map((r, i) => r[i]);
  const unc = (u) => (Number.isFinite(u) ? `± ${fmt(u * 100, 1)}%` : '');
  wrap.append(h('div', { class: 'section' }, h('b', {}, 'Hydrodynamics of this design'),
    h('div', { class: 'table-wrap' }, h('table', { class: 'data' },
      h('thead', {}, h('tr', {}, h('th', {}, ''), h('th', { class: 'num' }, 'Added mass now'), h('th', { class: 'num' }, 'Computed'), h('th', { class: 'num' }, 'Damping now'), h('th', { class: 'num' }, 'Computed'))),
      h('tbody', {}, AX.map((a, i) => h('tr', {}, h('td', {}, a),
        h('td', { class: 'num note' }, sig(R.before.addedMass[i])), h('td', { class: 'num' }, h('b', {}, sig(maNew[i])), h('div', { class: 'note' }, `${UM[i]} ${unc(R.bem.uncertainty[i])}`)),
        h('td', { class: 'num note' }, sig(R.before.damping[i])), h('td', { class: 'num' }, h('b', {}, sig(dqNew[i])), h('div', { class: 'note' }, `${UD[i]} ${unc(R.study.meta.uncertainty?.[DOFS[i].key])}`))))))),
    checkbox('Use the computed added mass and damping (full 6 × 6 matrices with couplings)', () => AT.useHydro, (v) => { AT.useHydro = v; }),
    h('p', { class: 'note' }, 'The full matrices can be inspected and edited afterwards in Hydrodynamics (6 × 6 matrix mode). The gains below were tuned on these hydrodynamics.')));

  // gains
  const rows = [];
  for (const [axis, rep] of Object.entries(R.tune.report)) {
    const label = STEP_AXES[axis]?.label ?? axis;
    if (rep.skipped) { rows.push(h('tr', {}, h('td', { colspan: 4, class: 'note' }, `${label}: ${rep.skipped}`))); continue; }
    rows.push(h('tr', {}, h('td', { colspan: 4, style: { color: 'var(--ink-3)', fontSize: '0.72rem', letterSpacing: '0.08em', textTransform: 'uppercase' } }, `${label}${rep.keep ? ' · current gains already best' : ` · score ${fmt(rep.beforeCost, 2)} → ${fmt(rep.afterCost, 2)}`}`)));
    for (const n of rep.notes || []) rows.push(h('tr', {}, h('td', { colspan: 4, class: 'note' }, n)));
    if (rep.atBound?.length) rows.push(h('tr', {}, h('td', { colspan: 4, class: 'note warn' }, `At the edge of the ArduSub range: ${rep.atBound.join(', ')}. The best value may lie beyond it; check this axis in the pool.`)));
    for (const k of Object.keys(rep.suggested || rep.params || {})) {
      const inp = h('input', { type: 'number', step: 'any', value: String(AT.edits[k]), 'aria-label': `${k} tuned value`, style: { width: '10ch' }, onchange: () => { const v = parseFloat(inp.value); if (Number.isFinite(v)) { AT.edits[k] = v; inp.classList.remove('invalid'); } else inp.classList.add('invalid'); } });
      const cb = h('input', { type: 'checkbox', 'aria-label': `Use ${k}`, onchange: () => { AT.include[k] = cb.checked; } });
      cb.checked = !!AT.include[k];
      rows.push(h('tr', {}, h('td', { class: 'mono' }, k), h('td', { class: 'num note' }, fmt(cfg.params[k], 4)), h('td', {}, inp), h('td', {}, cb)));
    }
  }
  for (const n of R.tune.notes || []) rows.push(h('tr', {}, h('td', { colspan: 4, class: 'note' }, n)));
  wrap.append(h('div', { class: 'section' }, h('b', {}, 'ArduSub gains'),
    h('div', { class: 'table-wrap' }, h('table', { class: 'data' }, h('thead', {}, h('tr', {}, h('th', {}, 'Parameter'), h('th', { class: 'num' }, 'Now'), h('th', {}, 'Tuned (edit)'), h('th', {}, 'Use'))), h('tbody', {}, rows))),
    h('p', { class: 'note' }, 'Change any tuned value before applying; untick a row to keep the current value. Every value stays editable later in Parameters and Tuning, and exports to your Pixhawk with Save .param.')));

  // step responses
  for (const [axis, rep] of Object.entries(R.tune.report)) {
    if (rep.skipped) continue;
    const meta = STEP_AXES[axis], c = rep.curves;
    const same = c.before.t.length === c.after.t.length;
    const data = { t: c.after.t, after: rep.keep ? undefined : c.after.y, before: same ? c.before.y : undefined, target: Float32Array.from(c.after.t, () => c.target), fixedStart: 0 };
    const series = rep.keep ? [{ name: 'current gains', key: 'before', color: 'var(--series-1)' }, { name: 'target', key: 'target', color: 'var(--series-2)', dash: true }]
      : [{ name: 'tuned', key: 'after', color: 'var(--series-1)' }, { name: 'target', key: 'target', color: 'var(--series-2)', dash: true }, { name: 'current gains', key: 'before', color: 'var(--series-3)' }];
    const ch = new StripChart({ title: `${meta.label} step on the new model`, unit: meta.unit, series, window: 1e9, height: 180, static: true });
    draws.push(() => ch.draw(data));
    wrap.append(ch.el);
  }

  wrap.append(h('div', { class: 'row' },
    h('button', { class: 'btn primary', onclick: () => apply(app) }, 'Apply selected'),
    h('button', { class: 'btn', onclick: () => exportReport(app) }, 'Save report (JSON)')));
  return wrap;
}

function sig(v) {
  if (!Number.isFinite(v)) return '—';
  const a = Math.abs(v);
  return a >= 100 ? v.toFixed(0) : a >= 10 ? v.toFixed(1) : a >= 1 ? v.toFixed(2) : v.toFixed(3);
}

export function openAutoTune(app) { app.openPanel('autotune'); }
