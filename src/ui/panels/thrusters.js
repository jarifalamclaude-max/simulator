// Thrusters panel: position, direction, polarity, thrust curve, dynamics and
// hull interaction for each thruster; layout presets; pick-on-model placement.

import { h, numberField, selectField, vec3Field, seg, checkbox, fmt, toast, confirmDialog, infoDialog } from '../dom.js';
import { THRUSTER_PRESETS, parametricCurve, parseCurveText } from '../../physics/thruster.js';
import { thrusterFromPreset } from '../../vehicle/config.js';
import { frameLayout, FRAMES } from '../../alloc/frames.js';
import { vnorm } from '../../core/math.js';

const S = Math.SQRT1_2;
const DIRS = [
  ['+X', 'Forward (+X)', [1, 0, 0]], ['-X', 'Aft (−X)', [-1, 0, 0]], ['+Y', 'Starboard (+Y)', [0, 1, 0]], ['-Y', 'Port (−Y)', [0, -1, 0]],
  ['+Z', 'Down (+Z)', [0, 0, 1]], ['-Z', 'Up (−Z)', [0, 0, -1]],
  ['FS', 'Fwd-starboard 45°', [S, S, 0]], ['FP', 'Fwd-port 45°', [S, -S, 0]], ['AS', 'Aft-starboard 45°', [-S, S, 0]], ['AP', 'Aft-port 45°', [-S, -S, 0]],
];

export default {
  id: 'thrusters', title: 'Thrusters', short: 'Thrusters', icon: 'thruster', sub: 'Order = ArduSub motor output number',
  create(app) {
    const cfg = app.cfg;
    const el = h('div', { class: 'section', style: { gap: '14px' } });
    const live = [];

    const head = h('div', { class: 'section' },
      h('p', { class: 'note' }, 'Positions are in the vehicle frame (X forward, Y starboard, Z down, metres). Direction is where the thruster pushes the vehicle for a forward command. Thruster 1 is motor output 1, and so on.'),
      h('div', { class: 'row' },
        h('button', { class: 'btn primary', onclick: () => { cfg.thrusters.push(thrusterFromPreset({ name: `T${cfg.thrusters.length + 1}`, pos: [0, 0, 0], axis: [1, 0, 0] })); app.changed({ immediate: true }); app.refreshPanel(); } }, '+ Add thruster'),
        selectField({ label: 'Load layout of an ArduSub frame', options: [['', 'Choose…'], ...Object.entries(FRAMES).filter(([k]) => k !== '7').map(([k, f]) => [k, f.name])], get: () => '', set: async (v) => {
          if (v === '') return;
          if (!(await confirmDialog('Replace all thrusters?', `This loads the ${FRAMES[v].name} layout (${FRAMES[v].table.length} thrusters) with typical positions. You can then move them to match your vehicle.`, { ok: 'Replace' }))) { app.refreshPanel(); return; }
          cfg.thrusters = frameLayout(+v).map((t) => thrusterFromPreset(t, cfg.thrusters[0]?.preset || 't200-16v'));
          app.changed({ immediate: true }); app.refreshPanel();
        } })));
    el.append(head);

    cfg.thrusters.forEach((t, i) => {
      const dirParam = cfg.params[`MOT_${i + 1}_DIRECTION`] ?? 1;
      const net = (t.polarity ?? 1) * (dirParam < 0 ? -1 : 1);
      const netPill = net > 0 ? h('span', { class: 'pill good', title: 'Prop polarity × MOT_DIRECTION' }, 'Pushes correctly') : h('span', { class: 'pill bad', title: 'Prop polarity × MOT_DIRECTION' }, 'Pushes backwards');
      const liveSpan = h('span', { class: 'mono note' }, '—');
      live.push(liveSpan);
      const dirPreset = DIRS.find(([, , v]) => vnorm(t.axis).every((x, k) => Math.abs(x - v[k]) < 1e-3));
      const card = h('div', { class: 'card' },
        h('div', { class: 'row', style: { justifyContent: 'space-between' } },
          h('div', { class: 'row' }, h('b', { class: 'mono' }, `#${i + 1}`), h('input', { type: 'text', value: t.name, 'aria-label': `Thruster ${i + 1} name`, style: { width: '13ch' }, onchange: (e) => { t.name = e.target.value; app.scheduleSave(); } })),
          h('div', { class: 'row' }, netPill, liveSpan)),
        vec3Field({ label: 'Position', get: () => t.pos, set: (v) => { t.pos = v; app.changed(); } }),
        h('div', { class: 'row' },
          h('button', { class: 'btn', onclick: () => {
            toast(`Click on the hull to place thruster ${i + 1}`, '', 4000);
            app.scene.setPickHandler((p) => { t.pos = p.map((v) => +v.toFixed(4)); app.scene.setPickHandler(null); app.changed({ immediate: true }); app.refreshPanel(); toast(`Thruster ${i + 1} placed`, 'good'); });
          } }, 'Place on model'),
          h('button', { class: 'btn', title: 'Mirror to the other side (Y → −Y)', onclick: () => { const c = JSON.parse(JSON.stringify(t)); c.pos[1] = -c.pos[1]; c.axis[1] = -c.axis[1]; c.name = `${t.name} (mirror)`; cfg.thrusters.splice(i + 1, 0, c); app.changed({ immediate: true }); app.refreshPanel(); } }, 'Mirror'),
          h('button', { class: 'btn', disabled: i === 0, title: 'Move up (changes motor number)', onclick: () => { [cfg.thrusters[i - 1], cfg.thrusters[i]] = [cfg.thrusters[i], cfg.thrusters[i - 1]]; app.changed({ immediate: true }); app.refreshPanel(); } }, '↑'),
          h('button', { class: 'btn', disabled: i === cfg.thrusters.length - 1, title: 'Move down (changes motor number)', onclick: () => { [cfg.thrusters[i + 1], cfg.thrusters[i]] = [cfg.thrusters[i], cfg.thrusters[i + 1]]; app.changed({ immediate: true }); app.refreshPanel(); } }, '↓'),
          h('button', { class: 'btn danger', onclick: async () => { if (await confirmDialog(`Remove thruster ${i + 1}?`, 'Motor numbers after it shift down by one.', { ok: 'Remove', danger: true })) { cfg.thrusters.splice(i, 1); app.changed({ immediate: true }); app.refreshPanel(); } } }, 'Remove')),
        h('div', { class: 'grid2' },
          selectField({ label: 'Direction preset', options: [['', dirPreset ? dirPreset[1] : 'Custom'], ...DIRS.map(([k, l]) => [k, l])], get: () => '', set: (k) => { const d = DIRS.find((x) => x[0] === k); if (d) { t.axis = d[2].slice(); app.changed({ immediate: true }); app.refreshPanel(); } } }),
          h('div', { class: 'field' }, h('span', {}, 'Prop / ESC polarity'), seg([[1, 'Normal'], [-1, 'Reversed']], t.polarity ?? 1, (v) => { t.polarity = v; app.changed({ immediate: true }); app.refreshPanel(); }))),
        vec3Field({ label: 'Direction (normalised automatically)', names: ['X', 'Y', 'Z'], unit: '', step: 0.01, get: () => t.axis, set: (v) => { if (Math.hypot(...v) < 1e-6) { toast('Direction cannot be zero', 'bad'); return; } t.axis = v; app.changed(); } }),
        h('details', { class: 'more' }, h('summary', {}, 'Thrust curve and dynamics'),
          h('div', { class: 'section', style: { marginTop: '10px' } },
            selectField({ label: 'Thruster model', options: [...Object.entries(THRUSTER_PRESETS).map(([k, p]) => [k, p.label]), ['custom', 'Custom curve']], get: () => (t.table ? 'custom' : t.preset), set: (k) => {
              if (k === 'custom') { pasteCurve(app, t); return; }
              const p = THRUSTER_PRESETS[k];
              Object.assign(t, { preset: k, curve: { ...p.curve }, table: null, propDiameter: p.propDiameter, maxRpm: p.maxRpm, J0: p.J0, tauUp: p.tauUp, tauDown: p.tauDown });
              app.changed({ immediate: true }); app.refreshPanel();
            } }),
            t.table ? h('p', { class: 'note' }, `Measured curve: ${t.table.length} points, ${fmt(Math.max(...t.table.map((r) => r[1])), 1)} N forward, ${fmt(-Math.min(...t.table.map((r) => r[1])), 1)} N reverse.`) : h('div', { class: 'grid2' },
              numberField({ label: 'Max forward', unit: 'N', step: 0.1, min: 0.1, get: () => t.curve.fwdMax, set: (v) => { t.curve.fwdMax = v; app.changed(); } }),
              numberField({ label: 'Max reverse', unit: 'N', step: 0.1, min: 0.1, get: () => t.curve.revMax, set: (v) => { t.curve.revMax = v; app.changed(); } }),
              numberField({ label: 'ESC deadband (±)', unit: 'µs', step: 1, min: 0, max: 150, get: () => t.curve.deadband, set: (v) => { t.curve.deadband = v; app.changed(); } }),
              numberField({ label: 'Curve shape exponent', step: 0.05, min: 0.5, max: 3, get: () => t.curve.expo, set: (v) => { t.curve.expo = v; app.changed(); } })),
            h('div', { class: 'row' }, h('button', { class: 'btn', onclick: () => pasteCurve(app, t) }, 'Paste measured curve'), t.table ? h('button', { class: 'btn', onclick: () => { t.table = null; app.changed({ immediate: true }); app.refreshPanel(); } }, 'Use parametric curve') : null),
            h('div', { class: 'grid2' },
              numberField({ label: 'Spin-up time constant', unit: 's', step: 0.01, min: 0, get: () => t.tauUp, set: (v) => { t.tauUp = v; app.changed(); } }),
              numberField({ label: 'Spin-down time constant', unit: 's', step: 0.01, min: 0, get: () => t.tauDown, set: (v) => { t.tauDown = v; app.changed(); } }),
              numberField({ label: 'Propeller diameter', unit: 'm', step: 0.001, min: 0.01, get: () => t.propDiameter, set: (v) => { t.propDiameter = v; app.changed(); } }),
              numberField({ label: 'Max RPM', step: 50, min: 100, get: () => t.maxRpm, set: (v) => { t.maxRpm = v; app.changed(); } })),
            checkbox('Thrust drops with in-flow speed (advance ratio J₀ ≈ 0.9)', () => t.inflowModel !== false, (v) => { t.inflowModel = v; app.changed(); }),
            h('div', { class: 'field' }, h('span', {}, 'Hull interaction (thrust deduction)'), seg([['auto', 'From jet blockage'], ['manual', 'Manual']], t.deductionMode, (v) => { t.deductionMode = v; app.changed({ immediate: true }); app.refreshPanel(); })),
            t.deductionMode === 'manual' ? h('div', { class: 'grid2' },
              numberField({ label: 'Forward thrust loss', step: 0.01, min: 0, max: 0.9, get: () => t.deduction, set: (v) => { t.deduction = v; app.changed(); } }),
              numberField({ label: 'Reverse thrust loss', step: 0.01, min: 0, max: 0.9, get: () => t.deductionRev, set: (v) => { t.deductionRev = v; app.changed(); } }))
              : h('p', { class: 'note' }, deductionText(app, i)),
            checkbox('Enabled (untick to simulate a failed thruster)', () => t.enabled !== false, (v) => { t.enabled = v; app.changed({ immediate: true }); }))));
      el.append(card);
    });

    return {
      el,
      tick: () => {
        const th = app.sim.plant.thrusters;
        live.forEach((s, i) => { const t = th[i]; s.textContent = t ? `${t.pwm} µs · ${fmt(t.Teff, 1)} N` : '—'; });
      },
    };
  },
};

function deductionText(app, i) {
  const d = app.sim?.info?.deductions?.[i];
  if (!d) return 'Computed from the hull when the model is built.';
  return `Jet blockage by the hull costs ${fmt(d[0] * 100, 0)}% of forward thrust and ${fmt(d[1] * 100, 0)}% of reverse thrust.`;
}

function pasteCurve(app, t) {
  const ta = h('textarea', { rows: 10, placeholder: 'PWM, thrust\n1100, -4.07\n1200, -2.8\n…\n1900, 5.25' });
  let unit = 'kgf';
  const unitSeg = seg([['N', 'N'], ['kgf', 'kgf'], ['lbf', 'lbf']], unit, (v) => (unit = v));
  const back = infoDialog('Measured thrust curve', h('div', { class: 'section' },
    h('p', { class: 'note' }, 'Paste two columns: ESC pulse width in µs and thrust. Negative thrust for reverse. Blue Robotics publishes this table for the T200 and T500 at several voltages.'),
    h('div', { class: 'row' }, h('span', { class: 'note' }, 'Thrust unit'), unitSeg), ta),
    [h('button', { class: 'btn', onclick: () => {
      try {
        const rows = parseCurveText(ta.value, unit);
        t.table = rows; t.preset = 'custom';
        app.changed({ immediate: true }); back.remove(); app.refreshPanel();
        toast(`Curve with ${rows.length} points applied`, 'good');
      } catch (e) { toast(e.message, 'bad'); }
    } }, 'Apply curve')]);
}
