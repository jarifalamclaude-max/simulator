// Tuning panel: live PID gains, repeatable step tests (target vs actual with
// rise/overshoot/settling metrics) and simulation-based auto-tune.

import { h, numberField, seg, checkbox, fmt, toast, confirmDialog } from '../dom.js';
import { StripChart } from '../charts.js';
import { STEP_AXES } from '../../sim/tuning.js';

const memo = { axis: 'yaw', size: null, noise: true, last: null, prev: null, tune: null, tuneAxes: { roll: true, pitch: true, yaw: true, depth: true }, quick: false, busy: false };

const GAIN_GROUPS = [
  ['Angle loop (how hard it chases the target angle)', ['ATC_ANG_RLL_P', 'ATC_ANG_PIT_P', 'ATC_ANG_YAW_P']],
  ['Roll rate loop', ['ATC_RAT_RLL_P', 'ATC_RAT_RLL_I', 'ATC_RAT_RLL_D', 'ATC_RAT_RLL_IMAX']],
  ['Pitch rate loop', ['ATC_RAT_PIT_P', 'ATC_RAT_PIT_I', 'ATC_RAT_PIT_D', 'ATC_RAT_PIT_IMAX']],
  ['Yaw rate loop', ['ATC_RAT_YAW_P', 'ATC_RAT_YAW_I', 'ATC_RAT_YAW_D', 'ATC_RAT_YAW_IMAX']],
  ['Depth hold', ['PSC_POSZ_P', 'PSC_VELZ_P', 'PSC_ACCZ_P', 'PSC_ACCZ_I']],
  ['Feel', ['ATC_INPUT_TC', 'ACRO_YAW_P', 'PILOT_SPEED_UP', 'PILOT_ACCEL_Z']],
];

function metricRows(m, unit) {
  if (!m) return [];
  return [
    ['Rise time (10–90%)', Number.isFinite(m.riseTime) ? `${fmt(m.riseTime, 2)} s` : 'did not reach'],
    ['Overshoot', `${fmt(m.overshoot, 1)} %`],
    ['Settling time', `${fmt(m.settlingTime, 2)} s`],
    ['Steady-state error', `${fmt(m.steadyError, unit === 'm' ? 3 : 2)} ${unit}`],
    ['Oscillations', String(m.oscillations)],
    ['Time saturated', `${fmt((m.saturation ?? 0) * 100, 0)} %`],
    ['Residual wobble (RMS)', `${fmt(m.tailRms, unit === 'm' ? 3 : 2)} ${unit}`],
  ];
}

export default {
  id: 'tuning', title: 'Tuning', short: 'Tuning', icon: 'target', sub: 'PID gains, step tests and auto-tune',
  create(app) {
    const cfg = app.cfg;
    const el = h('div', { class: 'section', style: { gap: '18px' } });
    const draws = [];

    // ---- live gains ----
    const gains = h('div', { class: 'section' }, h('h3', {}, 'Gains (live)'));
    for (const [title, names] of GAIN_GROUPS) {
      gains.append(h('span', { class: 'note' }, title), h('div', { class: names.length === 3 ? 'grid3' : 'grid2' },
        names.map((n) => numberField({ label: n.replace(/^(ATC_|PSC_)/, ''), step: 'any', get: () => cfg.params[n], set: (v) => app.setParam(n, v), hint: n }))));
    }
    gains.append(h('p', { class: 'note' }, 'Changes apply instantly to the running vehicle. Use the step test below to compare before and after under identical conditions.'));

    // ---- step test ----
    const chartHost = h('div');
    const metricsHost = h('div');
    const runBtn = h('button', { class: 'btn primary', onclick: () => runStep() }, 'Run step test');
    const sizeField = numberField({ label: 'Step size', unit: STEP_AXES[memo.axis].unit, step: STEP_AXES[memo.axis].unit === 'm' ? 0.1 : 5, get: () => memo.size ?? STEP_AXES[memo.axis].size, set: (v) => { memo.size = v; } });
    const stepSec = h('div', { class: 'section' }, h('h3', {}, 'Step test'),
      h('p', { class: 'note' }, 'A fresh copy of your vehicle is held at mid-depth in Depth Hold, then the target jumps by the step size. The live session is not disturbed.'),
      seg(Object.entries(STEP_AXES).map(([k, v]) => [k, v.label]), memo.axis, (v) => { memo.axis = v; memo.size = null; app.refreshPanel(); }),
      h('div', { class: 'grid2', style: { alignItems: 'end' } }, sizeField, checkbox('Sensor noise', () => memo.noise, (v) => { memo.noise = v; })),
      h('div', { class: 'row' }, runBtn), chartHost, metricsHost);

    function showStep() {
      const r = memo.last;
      if (!r) { chartHost.replaceChildren(h('p', { class: 'note' }, 'No test run yet.')); metricsHost.replaceChildren(); return; }
      const series = [{ name: 'actual', key: 'y', color: 'var(--series-1)' }, { name: 'target', key: 'target', color: 'var(--series-2)', dash: true }];
      const data = { t: r.t, y: r.y, target: r.target, fixedStart: 0 };
      const p = memo.prev;
      if (p && p.axis === r.axis && p.t.length === r.t.length) { series.push({ name: 'previous', key: 'prev', color: 'var(--series-3)' }); data.prev = p.y; }
      const chart = new StripChart({ title: `${STEP_AXES[r.axis].label} step ${fmt(r.size, r.unit === 'm' ? 2 : 0)} ${r.unit}`, unit: '', series, window: 1e9, height: 220, static: true });
      chartHost.replaceChildren(chart.el);
      draws.push(() => chart.draw(data));
      const rows = metricRows(r.metrics, r.unit);
      const prevRows = p && p.axis === r.axis ? metricRows(p.metrics, p.unit) : null;
      metricsHost.replaceChildren(h('div', { class: 'table-wrap' }, h('table', { class: 'data' },
        h('thead', {}, h('tr', {}, h('th', {}, 'Metric'), h('th', { class: 'num' }, 'This run'), prevRows ? h('th', { class: 'num' }, 'Previous') : null)),
        h('tbody', {}, rows.map(([k, v], i) => h('tr', {}, h('td', {}, k), h('td', { class: 'num' }, v), prevRows ? h('td', { class: 'num' }, prevRows[i][1]) : null))))),
      ...[r.metrics.diverged ? h('p', { class: 'note bad' }, 'The response diverged: these gains are unstable on this vehicle.') : !r.metrics.reached ? h('p', { class: 'note warn' }, 'The vehicle never reached 90% of the step: too little authority or gains too low.') : null].filter(Boolean));
    }
    async function runStep() {
      if (memo.busy) return;
      memo.busy = true; runBtn.disabled = true; runBtn.textContent = 'Running…';
      try {
        const res = await app.jobs.run('steptest', { cfg: app.cfg, geo: app.geo, opts: { axis: memo.axis, size: memo.size ?? STEP_AXES[memo.axis].size, noise: memo.noise } });
        memo.prev = memo.last; memo.last = res;
        app.refreshPanel();
      } catch (e) { toast(`Step test failed: ${e.message}`, 'bad', 7000); }
      finally { memo.busy = false; runBtn.disabled = false; runBtn.textContent = 'Run step test'; }
    }
    showStep();

    // ---- auto-tune ----
    const prog = h('div', { class: 'progress', hidden: true }, h('i'));
    const progText = h('span', { class: 'note' });
    const tuneBtn = h('button', { class: 'btn primary', onclick: () => runTune() }, 'Auto-tune');
    const tuneHost = h('div', { class: 'section' });
    const tuneSec = h('div', { class: 'section' }, h('h3', {}, 'Auto-tune'),
      h('p', { class: 'note' }, 'Estimates each axis from your vehicle model (inertia, added mass, damping and thrust authority), then searches the controller bandwidth with closed-loop step tests and backs off 15% for robustness. Results are a starting point for pool testing, not a replacement for it.'),
      h('div', { class: 'row' }, ...Object.keys(STEP_AXES).map((a) => checkbox(STEP_AXES[a].label, () => memo.tuneAxes[a], (v) => { memo.tuneAxes[a] = v; }))),
      checkbox('Quick search (fewer trials)', () => memo.quick, (v) => { memo.quick = v; }),
      h('div', { class: 'row' }, tuneBtn, progText), prog, tuneHost);

    async function runTune() {
      const axes = Object.keys(memo.tuneAxes).filter((a) => memo.tuneAxes[a]);
      if (!axes.length) { toast('Pick at least one axis'); return; }
      tuneBtn.disabled = true; prog.hidden = false;
      try {
        const res = await app.jobs.run('autotune', { cfg: app.cfg, geo: app.geo, opts: { axes, quick: memo.quick } }, (f, t) => { prog.firstChild.style.width = `${Math.round(f * 100)}%`; progText.textContent = `${t || 'Working'} · ${Math.round(f * 100)}%`; });
        memo.tune = res;
        app.refreshPanel();
      } catch (e) { toast(`Auto-tune failed: ${e.message}`, 'bad', 7000); tuneBtn.disabled = false; prog.hidden = true; }
    }
    function showTune() {
      const T = memo.tune;
      if (!T) return;
      const changes = Object.entries(T.params).filter(([k, v]) => Math.abs((cfg.params[k] ?? 0) - v) > 1e-9);
      tuneHost.append(h('div', { class: 'row' },
        h('button', { class: 'btn primary', disabled: !changes.length, onclick: async () => {
          for (const [k, v] of Object.entries(T.params)) app.setParam(k, v);
          toast(`Applied ${changes.length} tuned parameters`, 'good');
          app.refreshPanel();
        } }, changes.length ? `Apply ${changes.length} new gains` : 'Gains already applied'),
        h('button', { class: 'btn', onclick: async () => { if (await confirmDialog('Discard the auto-tune result?', 'Your current gains stay as they are.', { ok: 'Discard' })) { memo.tune = null; app.refreshPanel(); } } }, 'Discard')));
      for (const [axis, rep] of Object.entries(T.report)) {
        const meta = STEP_AXES[axis];
        if (rep.skipped) { tuneHost.append(h('p', { class: 'note' }, `${meta.label}: skipped (${rep.skipped}).`)); continue; }
        const c = rep.curves;
        const same = c.before.t.length === c.after.t.length;
        const data = { t: c.after.t, after: rep.keep ? undefined : c.after.y, before: same ? c.before.y : undefined, target: Float32Array.from(c.after.t, () => c.target), fixedStart: 0 };
        const series = rep.keep ? [{ name: 'current gains', key: 'before', color: 'var(--series-1)' }, { name: 'target', key: 'target', color: 'var(--series-2)', dash: true }]
          : [{ name: 'tuned', key: 'after', color: 'var(--series-1)' }, { name: 'target', key: 'target', color: 'var(--series-2)', dash: true }, { name: 'current gains', key: 'before', color: 'var(--series-3)' }];
        const chart = new StripChart({ title: `${meta.label}`, unit: meta.unit, series, window: 1e9, height: 200, static: true });
        draws.push(() => chart.draw(data));
        const b = metricRows(rep.before, meta.unit), a = metricRows(rep.after, meta.unit);
        if (rep.keep) {
          tuneHost.append(h('div', { class: 'card' }, chart.el,
            h('p', { class: 'note' }, `${meta.label}: your current gains already do as well as the best set the tuner found (score ${fmt(rep.beforeCost, 2)}, lower is better), so nothing changes on this axis.`)));
          continue;
        }
        tuneHost.append(h('div', { class: 'card' }, chart.el,
          h('p', { class: 'note' }, `Score ${fmt(rep.beforeCost, 2)} → ${fmt(rep.afterCost, 2)} (lower is better; full step plus a half step back).`),
          h('div', { class: 'table-wrap' }, h('table', { class: 'data' },
            h('thead', {}, h('tr', {}, h('th', {}, ''), h('th', { class: 'num' }, 'Current'), h('th', { class: 'num' }, 'Tuned'))),
            h('tbody', {}, a.map(([k, v], i) => h('tr', {}, h('td', {}, k), h('td', { class: 'num' }, b[i][1]), h('td', { class: 'num' }, v)))))),
          h('div', { class: 'kv' }, ...Object.entries(rep.params).flatMap(([k, v]) => [h('dt', {}, k), h('dd', {}, `${fmt(cfg.params[k], 4)} → ${fmt(v, 4)}`)]))));
      }
    }
    showTune();

    el.append(stepSec, tuneSec, gains);
    requestAnimationFrame(() => draws.forEach((d) => d()));
    return { el };
  },
};
