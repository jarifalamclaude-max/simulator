// Frame panel: choose how ArduSub mixes the thrusters (the automatic
// maximum-DOF mixer or a built-in FRAME_CONFIG) on picture cards, build a
// custom frame (body shape, thruster models, positions and directions, with
// drag-and-drop on a top and a side drawing, or found in the uploaded design
// by app.detectDesignThrusters), and try the mixer by hand:
// move a command slider and see which thrusters fire, which way they push and
// what force or torque comes out - or fly the simulated vehicle with it.

import { h, fmt, fmtSigned, toast, confirmDialog, checkbox, selectField, vec3Field, seg, showTextForCopy } from '../dom.js';
import { AXIS_LABEL, AXIS_ROW, AXIS_SIGN, AXIS_UNIT, effectiveWrench, tableQuality } from '../../alloc/allocation.js';
import { FRAMES, FACTOR_AXES, frameTable, frameLayout } from '../../alloc/frames.js';
import { thrusterFromPreset, thrusterLimits } from '../../vehicle/config.js';
import { THRUSTER_PRESETS } from '../../physics/thruster.js';
import { MODE } from '../../ardusub/sub.js';
import { shapeEditor } from '../shapeEditor.js';

const R2 = Math.SQRT1_2;
const MAX_MOTORS = 12;
const SNAP = 0.005; // m
const DIRS = [
  ['+X', 'Forward', [1, 0, 0]], ['-X', 'Backward', [-1, 0, 0]], ['+Y', 'Right', [0, 1, 0]], ['-Y', 'Left', [0, -1, 0]],
  ['-Z', 'Up', [0, 0, -1]], ['+Z', 'Down', [0, 0, 1]],
  ['FS', '45° forward-right', [R2, R2, 0]], ['FP', '45° forward-left', [R2, -R2, 0]], ['AS', '45° back-right', [-R2, R2, 0]], ['AP', '45° back-left', [-R2, -R2, 0]],
];
const ORDER = ['forward', 'lateral', 'throttle', 'yaw', 'roll', 'pitch'];
const WORDS = {
  forward: ['Surge', 'back', 'forward'], lateral: ['Sway', 'left', 'right'], throttle: ['Heave', 'down', 'up'],
  yaw: ['Yaw', 'nose left', 'nose right'], roll: ['Roll', 'left side down', 'right side down'], pitch: ['Pitch', 'nose down', 'nose up'],
};
const BLURB = {
  0: 'Two forward, three vertical and one lateral thruster.',
  1: 'BlueROV2: four vectored horizontal and two vertical thrusters.',
  2: 'BlueROV2 Heavy: four vectored horizontal and four vertical thrusters.',
  3: 'Four vertical corners, two forward and two lateral, all at right angles.',
  4: 'Two forward thrusters (yaw by difference) and one vertical.',
  5: 'ArduSub runs its default 5-motor table for this frame.',
  6: 'Default 5-motor table: two forward, two vertical, one lateral.',
  7: 'No runtime custom frame in ArduSub 4.5: behaves like SimpleROV-3.',
};
const SHORT = { 0: 'BlueROV1', 1: 'Vectored', 2: 'Vectored 6DOF', 3: 'Vectored 6DOF 90°', 4: 'SimpleROV-3', 5: 'SimpleROV-4', 6: 'SimpleROV-5', 7: 'Custom' };
const S = { u: Object.fromEntries(FACTOR_AXES.map((a) => [a, 0])), selected: null, fly: false, addModel: null, addDir: '+X' };

const withLimits = (list) => list.map((t) => ({ ...t, ...thrusterLimits(t) }));
const norm3 = (v) => { const n = Math.hypot(v[0], v[1], v[2]) || 1; return [v[0] / n, v[1] / n, v[2] / n]; };

// axes a factor table really commands on a given geometry (clean, right sign)
function commandedAxes(th, table, cg, signs) {
  const q = tableQuality(th, table, cg, signs);
  const big = Math.max(1e-9, ...q.rows.map((r) => Math.abs(r.main)));
  return q.rows.filter((r) => r.main > 0.05 * big).map((r) => r.axis);
}

// ---------------------------------------------------------------- drawing
// view 'top': looking down, nose up (u = y right, v = -x); view 'side': from the right, nose right (u = x, v = z down)
function fitView(th, bbox, view, W, H, big) {
  const iu = view === 'top' ? 1 : 0, iv = view === 'top' ? 0 : 2, sv = view === 'top' ? -1 : 1;
  const pts = th.map((t) => t.pos);
  if (bbox) pts.push(bbox.min, bbox.max);
  if (!pts.length) pts.push([-0.2, -0.2, -0.1], [0.2, 0.2, 0.1]);
  let u0 = Infinity, u1 = -Infinity, v0 = Infinity, v1 = -Infinity;
  for (const p of pts) { u0 = Math.min(u0, p[iu]); u1 = Math.max(u1, p[iu]); v0 = Math.min(v0, sv * p[iv]); v1 = Math.max(v1, sv * p[iv]); }
  const mu = big ? Math.max(0.04, (u1 - u0) * 0.14) : 0.02, mv = big ? Math.max(0.04, (v1 - v0) * 0.14) : 0.02;
  u0 -= mu; u1 += mu; v0 -= mv; v1 += mv;
  const pad = big ? 18 : 12;
  const sc = Math.min((W - 2 * pad) / Math.max(1e-3, u1 - u0), (H - 2 * pad) / Math.max(1e-3, v1 - v0));
  const cu = (u0 + u1) / 2, cv = (v0 + v1) / 2;
  return {
    view, W, H, big,
    P: (p) => [W / 2 + (p[iu] - cu) * sc, H / 2 + (sv * p[iv] - cv) * sc],
    // screen point -> the two model coordinates this view controls
    inv: (x, y) => ({ [iu]: cu + (x - W / 2) / sc, [iv]: sv * (cv + (y - H / 2) / sc) }),
  };
}

function drawVehicle({ th, signs, out, cg, bbox, T, selected = null }) {
  const { P, W, H, big, view } = T;
  const parts = [];
  if (bbox) {
    const a = P(bbox.min), b = P(bbox.max);
    const xa = Math.min(a[0], b[0]), xb = Math.max(a[0], b[0]), ya = Math.min(a[1], b[1]), yb = Math.max(a[1], b[1]);
    parts.push(`<rect x="${xa.toFixed(1)}" y="${ya.toFixed(1)}" width="${(xb - xa).toFixed(1)}" height="${(yb - ya).toFixed(1)}" rx="${big ? 10 : 6}" fill="var(--glass-hi)" stroke="var(--line-2)" />`);
    const n = view === 'top' ? [(xa + xb) / 2, ya] : [xb, (ya + yb) / 2];
    parts.push(view === 'top'
      ? `<path d="M${n[0] - 7},${n[1] + 2} L${n[0]},${n[1] - 7} L${n[0] + 7},${n[1] + 2}" fill="none" stroke="var(--foam)" stroke-width="2" />`
      : `<path d="M${n[0] - 2},${n[1] - 7} L${n[0] + 7},${n[1]} L${n[0] - 2},${n[1] + 7}" fill="none" stroke="var(--foam)" stroke-width="2" />`);
    if (big) parts.push(view === 'top' ? `<text x="${n[0]}" y="${n[1] - 11}" text-anchor="middle" fill="var(--ink-3)">FRONT</text>` : `<text x="${n[0] + 11}" y="${n[1] + 4}" fill="var(--ink-3)">FRONT</text>`);
  }
  if (cg) {
    const c = P(cg);
    parts.push(`<circle cx="${c[0]}" cy="${c[1]}" r="${big ? 4 : 3}" fill="#f2d14b" stroke="var(--abyss)" />`);
    if (big) parts.push(`<text x="${c[0] + 7}" y="${c[1] - 6}" fill="var(--ink-3)">CG</text>`);
  }
  const r = big ? 12 : 6.5;
  // far from the viewer first (top view: viewer above; side view: viewer on the right)
  const near = (t) => (view === 'top' ? -t.pos[2] : t.pos[1]);
  const idx = th.map((_, i) => i).sort((a, b) => near(th[a]) - near(th[b]));
  for (const i of idx) {
    const t = th[i];
    const o = out ? out[i] : 0;
    const active = Math.abs(o) > 0.01;
    const sgn = active ? Math.sign(o) : 1;
    const ax = norm3(t.axis);
    const d = [ax[0] * signs[i] * sgn, ax[1] * signs[i] * sgn, ax[2] * signs[i] * sgn];
    const [px, py] = P(t.pos);
    const du = view === 'top' ? d[1] : d[0], dv = view === 'top' ? -d[0] : d[2];
    const toward = view === 'top' ? -d[2] : d[1]; // + = toward the viewer
    const col = t.enabled === false ? 'var(--line-2)' : !active ? 'var(--ink-3)' : o > 0 ? 'var(--foam)' : 'var(--reverse)';
    const k = (0.25 + 0.55 * Math.min(1, Math.abs(o))).toFixed(2);
    const fill = !active ? 'var(--glass-2)' : o > 0 ? `rgba(255,181,71,${k})` : `rgba(79,141,255,${k})`;
    const g = [];
    const inPlane = Math.hypot(du, dv);
    if (inPlane > 0.3) {
      const L = (big ? (active ? 20 + 26 * Math.min(1, Math.abs(o)) : 18) : (active ? 10 + 8 * Math.min(1, Math.abs(o)) : 9)) * inPlane;
      const hx = du / inPlane, hy = dv / inPlane, hs = big ? 7 : 4;
      const sx = px + hx * r, sy = py + hy * r, ex = px + hx * (r + L), ey = py + hy * (r + L);
      g.push(`<line x1="${sx.toFixed(1)}" y1="${sy.toFixed(1)}" x2="${ex.toFixed(1)}" y2="${ey.toFixed(1)}" stroke="${col}" stroke-width="${big ? (active ? 3 : 2) : 1.6}" stroke-linecap="round" />`);
      g.push(`<path d="M${(ex - hx * hs - hy * hs * 0.7).toFixed(1)},${(ey - hy * hs + hx * hs * 0.7).toFixed(1)} L${ex.toFixed(1)},${ey.toFixed(1)} L${(ex - hx * hs + hy * hs * 0.7).toFixed(1)},${(ey - hy * hs - hx * hs * 0.7).toFixed(1)}" fill="none" stroke="${col}" stroke-width="${big ? 2.4 : 1.6}" stroke-linecap="round" stroke-linejoin="round" />`);
    }
    g.push(`<circle class="body" cx="${px.toFixed(1)}" cy="${py.toFixed(1)}" r="${r}" fill="${fill}" stroke="${selected === i ? 'var(--accent)' : col}" stroke-width="${selected === i ? 2.5 : 1.5}" />`);
    const glyph = Math.abs(toward) > 0.55;
    if (glyph) {
      const q = r * 0.45;
      g.push(toward > 0 ? `<circle cx="${px.toFixed(1)}" cy="${py.toFixed(1)}" r="${big ? 2.6 : 1.6}" fill="${col}" />`
        : `<path d="M${px - q},${py - q} L${px + q},${py + q} M${px + q},${py - q} L${px - q},${py + q}" stroke="${col}" stroke-width="${big ? 2 : 1.3}" />`);
    }
    if (!glyph) g.push(`<text x="${px.toFixed(1)}" y="${(py + (big ? 4.5 : 3)).toFixed(1)}" text-anchor="middle" fill="var(--ink)" font-weight="600" style="pointer-events:none">${i + 1}</text>`);
    else if (big) {
      const side = px >= W / 2 ? 1 : -1; // number on the outer side
      g.push(`<text x="${(px + side * (r + 3)).toFixed(1)}" y="${(py - r + 2).toFixed(1)}" text-anchor="${side > 0 ? 'start' : 'end'}" fill="var(--ink)" font-weight="600" style="pointer-events:none">${i + 1}</text>`);
    }
    parts.push(big ? `<g class="thr" data-i="${i}" tabindex="0" role="button" aria-label="Thruster ${i + 1}">${g.join('')}</g>` : g.join(''));
  }
  return `<svg viewBox="0 0 ${W} ${H}" width="100%" role="img" aria-label="${view === 'top' ? 'Top view' : 'Side view'} of the thrusters" style="display:block;font:500 ${big ? 11 : 9}px var(--font-data)">${parts.join('')}</svg>`;
}
const miniView = (th, signs, cg, bbox) => drawVehicle({ th, signs, cg, bbox, T: fitView(th, bbox, 'top', 200, 120, false) });

export default {
  id: 'frame', title: 'Frame', short: 'Frame', icon: 'frame', sub: 'Pick or build the frame, then try every axis by hand',
  create(app) {
    const cfg = app.cfg, P = cfg.params, m = cfg.model;
    const sim = app.sim;
    const mix = sim.mixer;
    const th = withLimits(cfg.thrusters);
    const signs = () => cfg.thrusters.map((t, i) => (t.polarity ?? 1) * ((P[`MOT_${i + 1}_DIRECTION`] ?? 1) < 0 ? -1 : 1));
    const el = h('div', { class: 'section', style: { gap: '18px' } });
    const src = cfg.mixer.source;
    const fcNow = P.FRAME_CONFIG;
    const bbox = app.geo?.bbox;
    const commit = () => { stopFly(); app.changed({ immediate: true }); app.refreshPanel(); };

    // ---------------- frame cards ----------------
    const ownAxes = mix.analysis.controllable || [];
    const card = (on, title, sub, svg, badge, onclick) => h('button', { type: 'button', class: `frame-card${on ? ' on' : ''}`, 'aria-pressed': on ? 'true' : 'false', onclick },
      h('div', { html: svg }),
      h('div', { class: 'row', style: { justifyContent: 'space-between', gap: '6px', flexWrap: 'nowrap' } }, h('span', { class: 't' }, title), on ? h('span', { class: 'pill good' }, 'In use') : h('span', { class: 'pill muted' }, badge)),
      h('span', { class: 's' }, sub));
    const cards = [card(src === 'auto', 'Automatic · custom frame', `Built from your ${cfg.thrusters.length} thrusters: the most axes they can control (${ownAxes.length} of 6). Build it below.`,
      miniView(th, signs(), mix.cg, bbox), `${ownAxes.length} axes`,
      () => { stopFly(); cfg.mixer.source = 'auto'; app.changed({ immediate: true }); app.refreshPanel(); toast(`Automatic mixer: ${ownAxes.length} axes`, 'good'); })];
    for (const [k, f] of Object.entries(FRAMES)) {
      const fc = +k;
      const lay = withLimits(frameLayout(fc));
      const axes = commandedAxes(lay, frameTable(fc), [0, 0, 0], lay.map(() => 1));
      cards.push(card(src === 'frame' && fcNow === fc, `${fc} · ${SHORT[fc]}`, `${f.table.length} thrusters. ${BLURB[fc]}`, miniView(lay, lay.map(() => 1), [0, 0, 0], null), `${axes.length} axes`,
        () => { stopFly(); cfg.mixer.source = 'frame'; app.setParam('FRAME_CONFIG', fc); app.refreshPanel(); toast(`FRAME_CONFIG ${fc}: ${SHORT[fc]}`, 'good'); }));
    }
    el.append(h('div', { class: 'section' }, h('h3', {}, 'Frame'),
      h('p', { class: 'note' }, 'Pick how ArduSub mixes your thrusters. A built-in frame sets FRAME_CONFIG (the table the Pixhawk runs). Automatic works for any custom frame: it builds the table from where your thrusters really are. Arrows show which way each thruster pushes for a positive output; ● pushes toward you, ✕ away.'),
      h('div', { class: 'frame-grid' }, cards)));

    if (src === 'frame') {
      const nF = FRAMES[fcNow]?.table.length ?? 0, nT = cfg.thrusters.length;
      if (nF !== nT) {
        el.append(h('div', { class: 'card', style: { borderColor: 'rgba(250,178,25,0.5)' } },
          h('b', {}, `FRAME_CONFIG ${fcNow} drives ${nF} motors, your vehicle has ${nT} thrusters`),
          h('p', { class: 'note' }, nF > nT ? `Motors ${nT + 1}–${nF} have no thruster, so some axes will be weak or missing.` : `Thrusters ${nF + 1}–${nT} get no command from this frame.`),
          h('div', { class: 'row' },
            h('button', { class: 'btn primary', onclick: () => useLayout(fcNow) }, `Use the standard ${nF}-thruster layout`),
            h('button', { class: 'btn', onclick: () => { cfg.mixer.source = 'auto'; commit(); } }, 'Back to Automatic'))));
      }
    }

    // ---------------- build your own ----------------
    async function useLayout(fc) {
      if (!await confirmDialog(`Use the standard ${SHORT[fc]} thruster layout?`, `Replaces your ${cfg.thrusters.length} thrusters with the ${FRAMES[fc].table.length} of this frame, at typical BlueROV-size positions. Spin directions follow your MOT_n_DIRECTION values. Drag them where you want afterwards.`, { ok: 'Replace thrusters' })) return;
      const preset = cfg.thrusters[0]?.preset || 't200-16v';
      cfg.thrusters = frameLayout(fc).map((t, i) => ({ ...thrusterFromPreset(t, preset), polarity: (P[`MOT_${i + 1}_DIRECTION`] ?? 1) < 0 ? -1 : 1 }));
      S.selected = null;
      commit();
      toast(`${cfg.thrusters.length} thrusters placed`, 'good');
    }
    const freeSpot = () => {
      const bb = bbox || { min: [-0.2, -0.15, -0.1], max: [0.2, 0.15, 0.1] };
      let best = [0, 0, 0], bestD = -1;
      for (const fx of [0.4, 0, -0.4]) for (const fy of [0.45, -0.45, 0]) {
        const p = [fx * (bb.max[0] - bb.min[0]) + (bb.max[0] + bb.min[0]) / 2, fy * (bb.max[1] - bb.min[1]) + (bb.max[1] + bb.min[1]) / 2, (bb.max[2] + bb.min[2]) / 2];
        const d = Math.min(Infinity, ...cfg.thrusters.map((t) => Math.hypot(t.pos[0] - p[0], t.pos[1] - p[1])));
        if (d > bestD) { bestD = d; best = p; }
      }
      return best.map((v) => +(Math.round(v / SNAP) * SNAP).toFixed(4));
    };
    const addThruster = () => {
      if (cfg.thrusters.length >= MAX_MOTORS) { toast(`ArduSub drives at most ${MAX_MOTORS} thrusters`, 'bad'); return; }
      const preset = S.addModel || cfg.thrusters[0]?.preset || 't200-16v';
      const d = DIRS.find((x) => x[0] === S.addDir) ?? DIRS[0];
      const n = cfg.thrusters.length;
      cfg.thrusters.push({ ...thrusterFromPreset({ name: `T${n + 1}`, pos: freeSpot(), axis: d[2].slice() }, preset), polarity: (P[`MOT_${n + 1}_DIRECTION`] ?? 1) < 0 ? -1 : 1 });
      S.selected = n;
      if (cfg.mixer.source === 'frame') cfg.mixer.source = 'auto';
      commit();
      toast(`Thruster ${n + 1} added: drag it into place`, 'good');
    };
    const cppExport = () => {
      const rows = mix.factors.slice(0, cfg.thrusters.length);
      const f = (v) => `${(Math.abs(v) < 5e-5 ? 0 : v).toFixed(4)}f`;
      const text = [
        `// ${cfg.name}: ${rows.length}-thruster mixer from the DUBURI simulator (${src === 'auto' ? 'automatic, maximum DOF' : src === 'frame' ? `FRAME_CONFIG ${fcNow}` : 'custom table'})`,
        '// Paste into AP_Motors6DOF::setup_motors() (libraries/AP_Motors/AP_Motors6DOF.cpp) as a frame case and build ArduSub.',
        '// Factor order: roll, pitch, yaw, throttle, forward, lateral, testing order.',
        'case SUB_FRAME_CUSTOM:',
        ...rows.map((r, i) => `    add_motor_raw_6dof(AP_MOTORS_MOT_${i + 1}, ${r.map(f).join(', ')}, ${i + 1}); // ${cfg.thrusters[i]?.name ?? ''}`),
        '    break;',
      ].join('\n');
      showTextForCopy('AP_Motors6DOF custom frame', text);
    };

    // views (draggable) and the command sliders
    const factors = mix.factors.slice(0, th.length);
    const u = S.u;
    const topBox = h('div', { class: 'frame-view' }), sideBox = h('div', { class: 'frame-view' });
    const Ttop = fitView(th, bbox, 'top', 400, 250, true), Tside = fitView(th, bbox, 'side', 400, 170, true);
    const result = h('div', { class: 'row', style: { gap: '6px', flexWrap: 'wrap', minHeight: '28px' } });
    const inspector = h('div', {});
    const mixOut = () => {
      const o = factors.map((row) => FACTOR_AXES.reduce((s, a, k) => s + (row[k] || 0) * u[a], 0));
      const mx = Math.max(1, ...o.map((v) => Math.abs(v)));
      return { o: o.map((v) => v / mx), k: 1 / mx, scaled: mx > 1 + 1e-9 };
    };
    const draw = () => {
      const sg = signs();
      const { o, k, scaled } = mixOut();
      const any = FACTOR_AXES.some((a) => Math.abs(u[a]) > 0.005);
      topBox.innerHTML = drawVehicle({ th, signs: sg, out: any ? o : null, cg: mix.cg, bbox, T: Ttop, selected: S.selected });
      sideBox.innerHTML = drawVehicle({ th, signs: sg, out: any ? o : null, cg: mix.cg, bbox, T: Tside, selected: S.selected });
      if (!any) { result.replaceChildren(h('span', { class: 'note' }, 'Move a slider: the thrusters that fire light up (orange forward, blue reverse) and the force or torque they make shows here.')); return; }
      // force / torque from nominal thrust, before ArduSub's saturation handling
      const W = effectiveWrench(th, factors, mix.cg, sg);
      const wr = [0, 1, 2, 3, 4, 5].map((r) => FACTOR_AXES.reduce((s, a, j) => s + u[a] * W[j][r], 0) * k);
      const vals = ORDER.map((a) => ({ a, v: wr[AXIS_ROW[a]] * AXIS_SIGN[a] }));
      const mag = (x) => Math.abs(x.v) / (AXIS_UNIT[x.a] === 'N' ? 1 : 0.15);
      const big = Math.max(1e-9, ...vals.map(mag));
      const chips = vals.filter((x) => mag(x) > 0.03 * big).map((x) => {
        const wanted = Math.abs(u[x.a]) > 0.005;
        const wrong = wanted && Math.sign(x.v) !== Math.sign(u[x.a]);
        return h('span', { class: `pill ${wrong ? 'bad' : wanted ? 'good' : 'warn'}`, title: wrong ? 'Opposite to the command: check MOT_n_DIRECTION and prop polarity' : wanted ? 'Commanded' : 'Not commanded: a side effect of this layout or table' },
          `${AXIS_LABEL[x.a]} ${fmtSigned(x.v, 1)} ${AXIS_UNIT[x.a]}${wrong ? ' (wrong way)' : wanted ? '' : ' (side effect)'}`);
      });
      result.replaceChildren(...chips, ...(scaled ? [h('span', { class: 'pill muted', title: 'At least one thruster would exceed 100%: all outputs scaled down together' }, 'thrusters at limit')] : []));
    };

    // drag thrusters in either view; a click without moving selects
    const attachDrag = (box, T) => {
      let drag = null;
      const toSvg = (e) => { const r = box.querySelector('svg').getBoundingClientRect(); return { x: ((e.clientX - r.left) * T.W) / r.width, y: ((e.clientY - r.top) * T.H) / r.height }; };
      box.addEventListener('pointerdown', (e) => {
        const g = e.target.closest?.('[data-i]');
        if (!g) return;
        drag = { i: +g.getAttribute('data-i'), start: toSvg(e), moved: false };
        box.setPointerCapture(e.pointerId);
        e.preventDefault();
      });
      box.addEventListener('pointermove', (e) => {
        if (!drag) return;
        const p = toSvg(e);
        if (!drag.moved && Math.hypot(p.x - drag.start.x, p.y - drag.start.y) < 3) return;
        drag.moved = true;
        const c = T.inv(p.x, p.y), t = cfg.thrusters[drag.i];
        for (const k of Object.keys(c)) t.pos[+k] = +(Math.round(c[k] / SNAP) * SNAP).toFixed(4);
        S.selected = drag.i;
        draw();
      });
      const end = (e) => {
        if (!drag) return;
        const d = drag;
        drag = null;
        try { box.releasePointerCapture(e.pointerId); } catch { /* already released */ }
        S.selected = d.i;
        if (d.moved) commit();
        else { draw(); showInspector(); }
      };
      box.addEventListener('pointerup', end);
      box.addEventListener('pointercancel', end);
      box.addEventListener('keydown', (e) => {
        const g = e.target.closest?.('[data-i]');
        if (g && (e.key === 'Enter' || e.key === ' ')) { S.selected = +g.getAttribute('data-i'); draw(); showInspector(); e.preventDefault(); }
      });
    };
    attachDrag(topBox, Ttop);
    attachDrag(sideBox, Tside);

    // flying with the sliders
    const flyMsg = () => ({ x: Math.round(u.forward * 1000), y: Math.round(u.lateral * 1000), z: Math.round(500 + u.throttle * 500), r: Math.round(u.yaw * 1000), s: Math.round(u.pitch * 400), t: Math.round(u.roll * 400), buttons: 0 });
    let armedByUs = false, prevMode = null;
    function startFly() {
      const fw = app.sim.fw;
      if (!fw.armed) {
        prevMode = fw.mode;
        if (fw.mode !== MODE.MANUAL) app.setMode(MODE.MANUAL);
        if (!fw.arm('frame test')) { const msg = fw.messages[fw.messages.length - 1]; toast(msg ? msg.text : 'Arming failed', 'bad'); return false; }
        armedByUs = true;
      }
      S.fly = true;
      app.pilotOverride = { msg: flyMsg() };
      return true;
    }
    function stopFly() {
      if (!S.fly && !app.pilotOverride) return;
      S.fly = false;
      app.pilotOverride = null;
      const fw = app.sim.fw;
      if (armedByUs) { fw.disarm('frame test'); if (prevMode !== null && prevMode !== fw.mode) app.setMode(prevMode); }
      armedByUs = false;
    }
    const sliders = ORDER.map((a) => {
      const [name, neg, pos] = WORDS[a];
      const val = h('span', { class: 'mono note', style: { textAlign: 'right' } });
      const show = () => { val.textContent = `${Math.round(u[a] * 100)}%`; };
      const inp = h('input', { type: 'range', min: -100, max: 100, step: 1, value: String(Math.round(u[a] * 100)), 'aria-label': `${name} command`, title: `${name}: − ${neg}, + ${pos}` });
      const set = (v) => { u[a] = v; inp.value = String(Math.round(v * 100)); show(); draw(); if (S.fly && app.pilotOverride) app.pilotOverride.msg = flyMsg(); };
      inp.addEventListener('input', () => set(+inp.value / 100));
      inp.addEventListener('dblclick', () => set(0));
      show();
      return { el: [h('span', { title: `− ${neg} · + ${pos}` }, name, h('div', { class: 'note', style: { fontSize: '0.68rem' } }, `− ${neg} / + ${pos}`)), inp, val], set };
    });
    const zero = () => sliders.forEach((s) => s.set(0));

    // the selected thruster: everything about it, editable
    function showInspector() {
      const i = S.selected;
      if (i === null || !cfg.thrusters[i]) { inspector.replaceChildren(h('p', { class: 'note' }, 'Click a thruster to edit it: model, direction, exact position, motor direction. Drag it to move it (5 mm steps).')); return; }
      const t = cfg.thrusters[i], dir = P[`MOT_${i + 1}_DIRECTION`] ?? 1;
      const row = factors[i] || [];
      const a = norm3(t.axis);
      const dirNow = DIRS.find(([, , v]) => v.every((x, k) => Math.abs(x - a[k]) < 1e-3));
      const rotate = (deg) => { const c = Math.cos((deg * Math.PI) / 180), s = Math.sin((deg * Math.PI) / 180); t.axis = norm3([a[0] * c - a[1] * s, a[0] * s + a[1] * c, a[2]]).map((v) => +v.toFixed(6)); commit(); };
      inspector.replaceChildren(h('div', { class: 'card' },
        h('div', { class: 'row', style: { justifyContent: 'space-between' } },
          h('div', { class: 'row', style: { gap: '6px' } }, h('b', { class: 'mono' }, `Motor ${i + 1}`), h('input', { type: 'text', value: t.name || '', 'aria-label': `Thruster ${i + 1} name`, style: { width: '14ch' }, onchange: (e) => { t.name = e.target.value; app.scheduleSave(); } })),
          h('span', { class: `pill ${signs()[i] > 0 ? 'good' : 'warn'}`, title: 'Prop polarity × MOT_n_DIRECTION' }, signs()[i] > 0 ? 'pushes as drawn' : 'reversed')),
        h('div', { class: 'grid2' },
          selectField({ label: 'Thruster model', options: [...Object.entries(THRUSTER_PRESETS).map(([k, p]) => [k, p.label]), ...(t.table ? [['custom', 'Measured table (Thrusters panel)']] : [])], get: () => (t.table ? 'custom' : t.preset), set: (k) => {
            const p = THRUSTER_PRESETS[k];
            if (!p) return;
            Object.assign(t, { preset: k, curve: { ...p.curve }, table: null, propDiameter: p.propDiameter, maxRpm: p.maxRpm, J0: p.J0, tauUp: p.tauUp, tauDown: p.tauDown });
            commit();
          } }),
          selectField({ label: 'Pushes (positive output)', options: [['', dirNow ? dirNow[1] : `Custom (${a.map((v) => fmt(v, 2)).join(', ')})`], ...DIRS.map(([k, l]) => [k, l])], get: () => '', set: (k) => { const d = DIRS.find((x) => x[0] === k); if (d) { t.axis = d[2].slice(); commit(); } } })),
        h('div', { class: 'row' },
          h('button', { class: 'btn', title: 'Rotate the direction 45° to the left (about the vertical axis)', onclick: () => rotate(-45) }, '⟲ 45°'),
          h('button', { class: 'btn', title: 'Rotate the direction 45° to the right', onclick: () => rotate(45) }, '⟳ 45°'),
          h('button', { class: 'btn', title: 'Point it the other way', onclick: () => { t.axis = a.map((v) => -v); commit(); } }, 'Flip'),
          h('button', { class: 'btn', title: 'Add a copy on the other side (Y → −Y)', onclick: () => {
            if (cfg.thrusters.length >= MAX_MOTORS) { toast(`ArduSub drives at most ${MAX_MOTORS} thrusters`, 'bad'); return; }
            const c = JSON.parse(JSON.stringify(t)); c.pos[1] = -c.pos[1]; c.axis[1] = -c.axis[1]; c.name = `${t.name || 'T'} (mirror)`;
            cfg.thrusters.push(c); S.selected = cfg.thrusters.length - 1; commit();
          } }, 'Mirror'),
          h('button', { class: 'btn danger', onclick: async () => {
            if (!await confirmDialog(`Remove thruster ${i + 1}?`, 'Motor numbers after it shift down by one.', { ok: 'Remove', danger: true })) return;
            cfg.thrusters.splice(i, 1); S.selected = null; commit();
          } }, 'Remove')),
        vec3Field({ label: 'Position (m from the model origin)', get: () => t.pos, set: (v) => { t.pos = v.map((x) => +x.toFixed(4)); commit(); }, step: 0.005 }),
        h('div', { class: 'row', style: { justifyContent: 'space-between' } },
          h('div', { class: 'field' }, h('span', {}, 'Prop / ESC polarity'), seg([[1, 'Normal'], [-1, 'Reversed']], t.polarity ?? 1, (v) => { t.polarity = v; commit(); })),
          h('button', { class: 'btn', onclick: () => { app.setParam(`MOT_${i + 1}_DIRECTION`, dir < 0 ? 1 : -1); commit(); toast(`MOT_${i + 1}_DIRECTION = ${dir < 0 ? 1 : -1}`); } }, `MOT_${i + 1}_DIRECTION: ${dir < 0 ? 'reverse' : 'normal'}`)),
        h('p', { class: 'note' }, `Mixer row: ${FACTOR_AXES.map((ax, k) => `${AXIS_LABEL[ax]} ${fmt(row[k] ?? 0, 2)}`).join(' · ')}`)));
    }

    // what the active mixer does on each axis, at a glance
    const q = tableQuality(th, factors, mix.cg, signs());
    const qbig = Math.max(1e-9, ...q.rows.map((x) => Math.abs(x.main) / (AXIS_UNIT[x.axis] === 'N' ? 1 : 0.15)));
    const glance = h('div', { class: 'row', style: { gap: '6px', flexWrap: 'wrap' } }, ORDER.map((ax) => {
      const r = q.rows.find((x) => x.axis === ax);
      const none = Math.abs(r.main) / (AXIS_UNIT[ax] === 'N' ? 1 : 0.15) < 0.03 * qbig;
      const coupled = r.off.some((o) => Math.abs(o.value) > 0.05 * Math.abs(r.main));
      const cls = r.reversed ? 'bad' : none ? 'muted' : coupled ? 'warn' : 'good';
      return h('span', { class: `pill ${cls}`, title: r.reversed ? 'Moves the wrong way' : none ? 'This mixer does not command this axis' : coupled ? 'Works, with side effects on other axes' : 'Clean' }, `${AXIS_LABEL[ax]}${r.reversed ? ' reversed' : none ? ' —' : ''}`);
    }));

    // body
    const bodySeg = seg([['stl', 'My design (STL)'], ['shape', 'Simple shape'], ['sample', 'Sample ROV']], m.source === 'stl' || m.source === 'shape' ? m.source : 'sample', async (v) => {
      if (v === m.source) return;
      stopFly();
      await app.setModelSource(v);
      app.refreshPanel();
    });
    const bodyInfo = m.source === 'shape' ? shapeEditor(app)
      : m.source === 'stl' ? h('div', { class: 'section', style: { gap: '6px' } },
        h('p', { class: 'note' }, `Your design${m.fileName ? ` ${m.fileName}` : ''} (units, axes and origin in Vehicle). Thrusters are drawn over it.`),
        h('div', { class: 'row' }, h('button', { class: 'btn', title: 'Find the thrusters in the uploaded design (part names T200_1…, or duct shapes) and build the frame from them', onclick: () => { stopFly(); app.detectDesignThrusters(); } }, 'Find thrusters in my design')))
      : h('p', { class: 'note' }, 'The sample BlueROV2-style vehicle.');

    const addModelSel = selectField({ label: 'New thruster model', options: Object.entries(THRUSTER_PRESETS).map(([k, p]) => [k, p.label]), get: () => S.addModel || cfg.thrusters[0]?.preset || 't200-16v', set: (v) => { S.addModel = v; } });
    const addDirSel = selectField({ label: 'Pushes', options: DIRS.map(([k, l]) => [k, l]), get: () => S.addDir, set: (v) => { S.addDir = v; } });
    const startSel = selectField({ label: 'Start from a frame layout', options: [['', 'Choose…'], ...Object.keys(FRAMES).map((k) => [k, `${k} · ${SHORT[k]} (${FRAMES[k].table.length} thrusters)`])], get: () => '', set: (v) => { if (v !== '') useLayout(+v); } });

    el.append(h('div', { class: 'section' }, h('h3', {}, 'Your frame'),
      h('div', { class: 'field' }, h('span', {}, 'Body'), bodySeg),
      bodyInfo,
      h('div', { class: 'grid2' }, addModelSel, addDirSel),
      h('div', { class: 'row' },
        h('button', { class: 'btn primary', onclick: addThruster }, `Add thruster (${cfg.thrusters.length}/${MAX_MOTORS})`),
        h('button', { class: 'btn', onclick: cppExport, title: 'add_motor_raw_6dof() lines for a custom ArduSub build' }, 'Copy as ArduSub code')),
      startSel,
      h('p', { class: 'note' }, `Mixer in use: ${src === 'auto' ? 'Automatic (custom frame)' : src === 'frame' ? `FRAME_CONFIG ${fcNow} · ${SHORT[fcNow] ?? ''}` : 'custom table'}. Drag a thruster to move it: from above you move it forward/sideways, from the side forward/up-down.`),
      glance,
      h('div', { class: 'frame-views' }, h('div', { class: 'frame-view-wrap' }, h('span', { class: 'note' }, 'From above'), topBox), h('div', { class: 'frame-view-wrap' }, h('span', { class: 'note' }, 'From the right side'), sideBox)),
      inspector));

    el.append(h('div', { class: 'section' }, h('h3', {}, 'Try it'),
      h('p', { class: 'note' }, 'Each slider is one ArduSub command. Watch which thrusters fire in the drawings above and what comes out. Double-click a slider to centre it.'),
      result,
      h('div', { class: 'axis-sliders' }, sliders.map((s) => s.el)),
      h('div', { class: 'row' }, h('button', { class: 'btn', onclick: zero }, 'Centre all'),
        h('button', { class: 'btn', onclick: () => { stopFly(); zero(); app.sim.reset(); app.refreshPanel(); } }, 'Put the vehicle back at the start')),
      checkbox('Also fly the vehicle in the pool with these sliders (Manual mode, arms if needed, pilot gain applies)', () => S.fly, (v) => { if (v) { if (!startFly()) app.refreshPanel(); } else stopFly(); })));

    draw(); showInspector();
    return { el, dispose: () => stopFly() };
  },
};
