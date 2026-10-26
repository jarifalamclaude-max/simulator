// Frame & mixer panel: which axes the thruster layout can control, how much
// force/torque each axis gets, cross-coupling, and the mixer actually used
// (automatic maximum-DOF table, ArduSub FRAME_CONFIG table, or custom).

import { h, seg, selectField, fmt, toast, checkbox, numberField } from '../dom.js';
import { AXES, AXIS_LABEL, AXIS_UNIT, tableQuality } from '../../alloc/allocation.js';
import { FRAMES, frameTable } from '../../alloc/frames.js';
import { thrusterLimits } from '../../vehicle/config.js';

const SAT = [['priority', 'Attitude first'], ['scale', 'Scale everything'], ['clamp', 'Clamp each motor (ArduSub generic)']];

export default {
  id: 'mixer', title: 'Frame & mixer', short: 'Mixer', icon: 'mixer', sub: 'Controllable axes, authority and motor mixing',
  create(app) {
    const cfg = app.cfg;
    const sim = app.sim;
    const mix = sim.mixer;
    const an = mix.analysis;
    const el = h('div', { class: 'section', style: { gap: '18px' } });

    // ---- DOF summary ----
    const maxAuth = { force: 1e-9, moment: 1e-9 };
    for (const a of AXES) {
      const k = ['roll', 'pitch', 'yaw'].includes(a) ? 'moment' : 'force';
      maxAuth[k] = Math.max(maxAuth[k], an.authority[a].pos, an.authority[a].neg);
    }
    const rows = AXES.map((a) => {
      const st = an.status[a];
      const pill = st === 'full' ? h('span', { class: 'pill good' }, 'Controllable') : st === 'weak' ? h('span', { class: 'pill warn' }, 'Coupled only') : st === 'disabled' ? h('span', { class: 'pill muted' }, 'Disabled') : h('span', { class: 'pill bad' }, 'Not possible');
      const k = ['roll', 'pitch', 'yaw'].includes(a) ? 'moment' : 'force';
      const au = an.authority[a];
      const wP = (au.pos / maxAuth[k]) * 50, wN = (au.neg / maxAuth[k]) * 50;
      return h('div', { class: 'bar-row', title: `${AXIS_LABEL[a]}: +${fmt(au.pos, 1)} / −${fmt(au.neg, 1)} ${AXIS_UNIT[a]}` },
        h('span', {}, AXIS_LABEL[a]),
        h('div', { class: 'bar-track' },
          h('div', { class: 'bar-fill', style: { left: `${50 - wN}%`, width: `${wN}%`, background: 'var(--reverse)' } }),
          h('div', { class: 'bar-fill', style: { left: '50%', width: `${wP}%`, background: 'var(--foam)' } })),
        h('div', { class: 'row', style: { justifyContent: 'flex-end', gap: '6px' } }, st === 'full' ? h('span', { class: 'mono note' }, `±${fmt(Math.min(au.pos, au.neg), 0)} ${AXIS_UNIT[a]}`) : pill));
    });
    el.append(h('div', { class: 'section' }, h('h3', {}, 'What your thruster layout can do'),
      h('div', { class: 'row', style: { justifyContent: 'space-between' } }, h('b', { style: { fontSize: '1.05rem' } }, an.summary), h('span', { class: 'note mono' }, `rank ${an.rank} / 6`)),
      h('div', { class: 'bars' }, rows),
      h('p', { class: 'note' }, 'Bars show the largest pure force or torque on each axis with every thruster inside its forward/reverse limit (orange: positive direction, blue: negative). Roll is right side down, pitch is nose up, yaw is nose right, heave is up.'),
      couplingNote(an)));

    // ---- mixer choice ----
    const src = cfg.mixer.source;
    const choose = seg([['auto', 'Automatic (max DOF)'], ['frame', 'ArduSub FRAME_CONFIG'], ['custom', 'Custom table']], src, (v) => {
      if (v === 'custom' && !cfg.mixer.custom) cfg.mixer.custom = mix.factors.map((r) => r.slice());
      cfg.mixer.source = v; app.changed({ immediate: true }); app.refreshPanel();
    });
    const mixSec = h('div', { class: 'section' }, h('h3', {}, 'Mixer used by the simulator'), choose);
    if (src === 'auto') {
      mixSec.append(h('p', { class: 'note' }, 'The simulator builds the factor table from your thruster positions and directions so that every controllable axis is independent (no coupling into the others) and uses the full thrust available.'),
        h('div', { class: 'grid2' },
          selectField({ label: 'When thrusters saturate', options: SAT, get: () => cfg.mixer.saturation, set: (v) => { cfg.mixer.saturation = v; app.changed({ immediate: true }); } }),
          numberField({ label: 'Minimum axis strength', step: 0.005, min: 0.001, max: 0.5, get: () => cfg.mixer.threshold, set: (v) => { cfg.mixer.threshold = v; app.changed({ immediate: true }); app.refreshPanel(); }, hint: 'Relative singular value below which an axis counts as not controllable' })),
        h('div', { class: 'row' }, ...AXES.map((a) => checkbox(AXIS_LABEL[a], () => cfg.mixer.axisEnable[a] !== false, (v) => { cfg.mixer.axisEnable[a] = v; app.changed({ immediate: true }); app.refreshPanel(); }))),
        h('p', { class: 'note warn' }, 'ArduSub 4.5 cannot load a custom table at run time (FRAME_CONFIG 7 behaves like SimpleROV-3). To fly this mixer on the real vehicle, compile it into AP_Motors6DOF, or choose the closest built-in frame below and compare.'));
    } else if (src === 'frame') {
      const fc = cfg.params.FRAME_CONFIG;
      mixSec.append(selectField({ label: 'FRAME_CONFIG', options: Object.entries(FRAMES).map(([k, f]) => [k, `${k} · ${f.name}`]), get: () => fc, set: (v) => { app.setParam('FRAME_CONFIG', +v); app.refreshPanel(); } }),
        h('p', { class: 'note' }, `Exactly the table and saturation handling compiled into ArduSub 4.5.7 (${FRAMES[fc]?.mixer} mixing).`),
        ...mix.warnings.map((w) => h('p', { class: 'note warn' }, w)));
    } else {
      mixSec.append(selectField({ label: 'When thrusters saturate', options: SAT, get: () => cfg.mixer.saturation, set: (v) => { cfg.mixer.saturation = v; app.changed({ immediate: true }); } }),
        h('div', { class: 'row' }, h('button', { class: 'btn', onclick: () => { cfg.mixer.custom = an.factors.map((r) => r.slice()); app.changed({ immediate: true }); app.refreshPanel(); toast('Copied the automatic table'); } }, 'Start from automatic table'),
          h('button', { class: 'btn', onclick: () => { cfg.mixer.custom = frameTable(cfg.params.FRAME_CONFIG).slice(0, cfg.thrusters.length); app.changed({ immediate: true }); app.refreshPanel(); } }, 'Start from FRAME_CONFIG table')));
    }
    el.append(mixSec);

    // ---- factor table ----
    const editable = src === 'custom';
    const factors = editable ? cfg.mixer.custom : mix.factors;
    const table = h('table', { class: 'data' },
      h('thead', {}, h('tr', {}, h('th', {}, 'Motor'), ...AXES.map((a) => h('th', { class: 'num' }, AXIS_LABEL[a])))),
      h('tbody', {}, factors.map((row, i) => h('tr', {}, h('td', { class: 'mono' }, `${i + 1} ${cfg.thrusters[i]?.name ?? '(no thruster)'}`),
        ...AXES.map((a, k) => h('td', { class: 'num' }, editable ? h('input', { type: 'number', step: 0.05, value: row[k], style: { width: '6.5ch' }, 'aria-label': `Motor ${i + 1} ${a} factor`, onchange: (e) => { const v = parseFloat(e.target.value); if (Number.isFinite(v)) { row[k] = v; app.changed(); } } }) : fmt(row[k], 3)))))));
    el.append(h('div', { class: 'section' }, h('h3', {}, 'Motor factor table'), h('div', { class: 'table-wrap' }, table),
      h('p', { class: 'note' }, 'Same columns as ArduSub add_motor_raw_6dof(): motor output = Σ factor × controller input, then MOT_n_DIRECTION.')));

    // ---- check of the active table on this geometry ----
    const signs = cfg.thrusters.map((t, i) => (t.polarity ?? 1) * ((cfg.params[`MOT_${i + 1}_DIRECTION`] ?? 1) < 0 ? -1 : 1));
    const th = cfg.thrusters.map((t) => ({ ...t, ...thrusterLimits(t) }));
    const q = tableQuality(th, mix.factors.slice(0, th.length), mix.cg, signs);
    const qRows = q.rows.map((r) => {
      const main = r.main;
      const offs = r.off.filter((o) => Math.abs(o.value) > 0.02 * Math.max(1, Math.abs(main))).sort((a, b) => Math.abs(b.value) - Math.abs(a.value));
      const status = r.reversed ? h('span', { class: 'pill bad' }, 'Reversed') : Math.abs(main) < 1e-6 ? h('span', { class: 'pill muted' }, 'No effect') : offs.length ? h('span', { class: 'pill warn' }, 'Couples') : h('span', { class: 'pill good' }, 'Clean');
      return h('tr', {}, h('td', {}, AXIS_LABEL[r.axis]), h('td', { class: 'num' }, `${fmt(main, 1)} ${AXIS_UNIT[r.axis]}`), h('td', {}, status),
        h('td', { style: { whiteSpace: 'normal' } }, offs.slice(0, 3).map((o) => `${AXIS_LABEL[o.axis]} ${fmt(o.value, 2)} ${AXIS_UNIT[o.axis]}`).join(', ') || '—'));
    });
    el.append(h('div', { class: 'section' }, h('h3', {}, 'Full-stick command check'),
      h('p', { class: 'note' }, 'What one full controller command on each axis really produces on this hull, including prop polarity and MOT_n_DIRECTION. "Couples" lists the unwanted forces and torques that come with it.'),
      h('div', { class: 'table-wrap' }, h('table', { class: 'data' }, h('thead', {}, h('tr', {}, h('th', {}, 'Command'), h('th', { class: 'num' }, 'Produces'), h('th', {}, 'Result'), h('th', {}, 'Unwanted side effects'))), h('tbody', {}, qRows)))));
    return { el };
  },
};

function couplingNote(an) {
  const notes = [];
  for (const [ax, c] of Object.entries(an.coupling)) {
    for (const [other, v] of Object.entries(c)) if (an.status[other] !== 'full' && Math.abs(v) > 1e-3) notes.push(`${AXIS_LABEL[ax]} also produces ${fmt(v, 3)} ${AXIS_UNIT[other]} of ${AXIS_LABEL[other].toLowerCase()} per unit (cannot be cancelled).`);
  }
  if (!notes.length) return null;
  return h('div', { class: 'card' }, h('b', {}, 'Unavoidable coupling'), ...notes.slice(0, 6).map((n) => h('p', { class: 'note' }, n)));
}
