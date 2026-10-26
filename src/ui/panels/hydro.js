// Hydrodynamics panel: added mass (estimate / BEM panel method / manual) and
// damping (estimate / CFD / manual), with the matrices the plant uses.

import { h, seg, numberField, fmt, toast, selectField } from '../dom.js';
import { parseMatrix6 } from '../../vehicle/model.js';

const AX = ['Surge', 'Sway', 'Heave', 'Roll', 'Pitch', 'Yaw'];
const UNITS_M = ['kg', 'kg', 'kg', 'kg·m²', 'kg·m²', 'kg·m²'];

export default {
  id: 'hydro', title: 'Hydrodynamics', short: 'Hydro', icon: 'wave', sub: 'Added mass, damping and thruster–hull interaction',
  create(app) {
    const cfg = app.cfg;
    const info = app.sim.info;
    const el = h('div', { class: 'section', style: { gap: '18px' } });
    const am = cfg.hydro.addedMass, dm = cfg.hydro.damping;

    // ---- added mass ----
    const prog = h('div', { class: 'progress', hidden: true }, h('i'));
    const progText = h('span', { class: 'note' });
    const runBtn = h('button', { class: 'btn primary', onclick: async () => {
      runBtn.disabled = true; prog.hidden = false;
      const t0 = performance.now();
      try {
        const res = await app.jobs.run('bemconv', { positions: app.vehiclePositions, rho: app.sim.env.rho, ref: [0, 0, 0], levels: [900, 1800, 3600] }, (f, t) => { prog.firstChild.style.width = `${Math.round(f * 100)}%`; progText.textContent = `${t || 'Working'} · ${Math.round(f * 100)}%`; });
        am.mode = 'bem';
        am.matrix = res.M;
        am.ref = res.ref;
        am.meta = { panels: res.panels, levels: res.levels, uncertainty: res.uncertainty, status: res.status, converged: true, seconds: (performance.now() - t0) / 1000, asymmetry: res.asymmetry, date: new Date().toISOString(), rho: app.sim.env.rho };
        app.changed({ immediate: true });
        toast(`Added mass from ${res.levels.join(' / ')} panels, Richardson-extrapolated, in ${am.meta.seconds.toFixed(1)} s`, 'good', 5000);
        app.refreshPanel();
      } catch (e) { toast(`Panel method failed: ${e.message}`, 'bad', 7000); runBtn.disabled = false; prog.hidden = true; }
    } }, am.mode === 'bem' && am.matrix ? 'Run panel method again' : 'Run panel method (3 resolutions)');
    const amSec = h('div', { class: 'section' }, h('h3', {}, 'Added mass'),
      h('p', { class: 'note' }, 'Water that has to be accelerated with the vehicle. The panel method (verified against Capytaine within 1–2% on the same meshes, and against the exact ellipsoid solution) solves potential flow around your hull on three resolutions and extrapolates, returning the full 6 × 6 matrix with couplings such as surge–pitch.'),
      seg([['estimate', 'Ellipsoid estimate'], ['bem', 'Panel method'], ['manual', 'Diagonal'], ['matrix', '6×6 matrix']], am.mode, (v) => {
        if (v === 'bem' && !am.matrix) { toast('Run the panel method first'); app.refreshPanel(); return; }
        if (v === 'manual') am.diag = info.MA.map((r, i) => +r[i].toFixed(4));
        if (v === 'matrix' && !am.userMatrix) { am.userMatrix = info.MA.map((r) => r.map((x) => +x.toPrecision(6))); am.userRef = 'cg'; }
        am.mode = v; app.changed({ immediate: true }); app.refreshPanel();
      }),
      h('div', { class: 'row' }, runBtn, progText), prog);
    if (am.mode === 'manual') amSec.append(h('div', { class: 'grid3' }, AX.map((a, i) => numberField({ label: a, unit: UNITS_M[i], step: 0.01, min: 0, get: () => am.diag[i], set: (v) => { am.diag[i] = v; app.changed(); } }))));
    if (am.mode === 'matrix') amSec.append(matrixEditor('Added mass, rows/columns surge, sway, heave, roll, pitch, yaw (kg, kg·m, kg·m²)', () => am.userMatrix, (M) => {
      const asym = Math.max(...M.flatMap((r, i) => r.map((v, j) => Math.abs(v - M[j][i])))) / Math.max(1e-12, ...M.flat().map(Math.abs));
      if (asym > 0.05) toast('This matrix is not symmetric; added mass from potential flow should be.', 'bad', 6000);
      am.userMatrix = M; app.changed({ immediate: true }); app.refreshPanel();
    }), selectField({ label: 'Matrix is given about', options: [['cg', 'Centre of gravity'], ['origin', 'Vehicle origin (it is moved to the CG)']], get: () => am.userRef || 'cg', set: (v) => { am.userRef = v; app.changed({ immediate: true }); app.refreshPanel(); } }));
    if (am.meta && am.mode === 'bem') {
      amSec.append(h('p', { class: 'note' }, am.meta.levels ? `Panel method on ${am.meta.levels.join(' / ')} panels, Richardson-extrapolated to zero panel size (${am.meta.seconds.toFixed(1)} s). Reference point: vehicle origin, moved to the CG automatically.` : `Panel method: ${am.meta.panels} panels, ${am.meta.seconds.toFixed(1)} s, symmetry error ${(am.meta.asymmetry * 100).toFixed(2)}%.`));
      if (am.meta.uncertainty) amSec.append(h('div', { class: 'kv' }, ...AX.flatMap((a, i) => [h('dt', {}, `${a} uncertainty`), h('dd', {}, Number.isFinite(am.meta.uncertainty[i]) ? `± ${(am.meta.uncertainty[i] * 100).toFixed(1)}% (${am.meta.status?.[i] ?? ''})` : '—')])));
    }
    amSec.append(matrixTable(info.MA, `In use (${info.maSource}), about the CG`, UNITS_M));
    el.append(amSec);

    // ---- damping ----
    const dSec = h('div', { class: 'section' }, h('h3', {}, 'Damping (drag)'),
      h('p', { class: 'note' }, 'Force = −(linear × v + quadratic × v|v|) on each axis. The CFD lab measures these for your hull in a virtual tow tank, including cross-coupling.'),
      seg([['estimate', 'Estimate'], ['cfd', 'CFD lab'], ['manual', 'Diagonal'], ['matrix', '6×6 matrices']], dm.mode, (v) => {
        if (v === 'cfd' && !dm.DQ) { toast('Run the CFD lab first'); app.openPanel('cfd'); return; }
        if (v === 'manual') { dm.linear = info.DL.map((r, i) => +r[i].toFixed(4)); dm.quadratic = info.DQ.map((r, i) => +r[i].toFixed(4)); }
        if (v === 'matrix' && !dm.userDQ) { dm.userDL = info.DL.map((r) => r.map((x) => +x.toPrecision(6))); dm.userDQ = info.DQ.map((r) => r.map((x) => +x.toPrecision(6))); }
        dm.mode = v; app.changed({ immediate: true }); app.refreshPanel();
      }));
    if (dm.mode === 'matrix') dSec.append(
      h('p', { class: 'note' }, 'Paste matrices from ANSYS, STAR-CCM+, OpenFOAM or a system-identification fit, about the centre of gravity, body axes X forward, Y right, Z down. Column j is the motion, row i the force or moment.'),
      matrixEditor('Linear damping (N·s/m, N·m·s/rad)', () => dm.userDL, (M) => { dm.userDL = M; app.changed({ immediate: true }); app.refreshPanel(); }),
      matrixEditor('Quadratic damping (N·s²/m², N·m·s²/rad²)', () => dm.userDQ, (M) => { dm.userDQ = M; app.changed({ immediate: true }); app.refreshPanel(); }));
    if (dm.mode === 'estimate') dSec.append(numberField({ label: 'Drag coefficient Cd', step: 0.05, min: 0.1, max: 3, get: () => dm.Cd ?? 1, set: (v) => { dm.Cd = v; app.changed({ immediate: true }); app.refreshPanel(); } }));
    if (dm.mode === 'manual') {
      dSec.append(h('b', { class: 'note' }, 'Linear (N·s/m, N·m·s/rad)'), h('div', { class: 'grid3' }, AX.map((a, i) => numberField({ label: a, step: 0.01, min: 0, get: () => dm.linear[i], set: (v) => { dm.linear[i] = v; app.changed(); } }))),
        h('b', { class: 'note' }, 'Quadratic (N·s²/m², N·m·s²/rad²)'), h('div', { class: 'grid3' }, AX.map((a, i) => numberField({ label: a, step: 0.01, min: 0, get: () => dm.quadratic[i], set: (v) => { dm.quadratic[i] = v; app.changed(); } }))));
    }
    dSec.append(h('button', { class: 'btn', onclick: () => app.openPanel('cfd') }, 'Open CFD lab'),
      matrixTable(info.DQ, `Quadratic damping in use (${info.dSource})`, null, true),
      matrixTable(info.DL, 'Linear damping in use', null, true));
    el.append(dSec);

    // ---- thruster interaction + water ----
    el.append(h('div', { class: 'section' }, h('h3', {}, 'Water and thrusters'),
      h('dl', { class: 'kv' },
        h('dt', {}, 'Water density'), h('dd', {}, `${fmt(app.sim.env.rho, 2)} kg/m³`),
        h('dt', {}, 'Kinematic viscosity'), h('dd', {}, `${(app.sim.env.nu * 1e6).toFixed(3)} mm²/s`),
        ...info.deductions.flatMap((d, i) => [h('dt', {}, `Thruster ${i + 1} hull blockage`), h('dd', {}, `${fmt(d[0] * 100, 0)}% fwd · ${fmt(d[1] * 100, 0)}% rev`)])),
      h('p', { class: 'note' }, 'Also simulated: thrust loss with in-flow speed, ventilation near the surface, partial buoyancy and extra drag while breaking the surface, current with turbulence, waves, and added-mass Munk moments.')));
    return { el };
  },
};

function matrixTable(M, caption, units, compact) {
  const max = Math.max(...M.flat().map(Math.abs), 1e-12);
  return h('div', { class: 'section', style: { gap: '6px' } }, h('span', { class: 'note' }, caption),
    h('div', { class: 'table-wrap' }, h('table', { class: 'data' },
      h('thead', {}, h('tr', {}, h('th', {}, ''), ...AX.map((a) => h('th', { class: 'num' }, a.slice(0, compact ? 2 : 5))))),
      h('tbody', {}, M.map((row, i) => h('tr', {}, h('th', {}, AX[i]), ...row.map((v, j) => h('td', { class: 'num', style: { opacity: Math.abs(v) < max * 1e-3 && i !== j ? 0.35 : 1, color: i === j ? 'var(--ink)' : 'var(--ink-2)' } }, fmtSci(v)))))))));
}
function fmtSci(v) {
  const a = Math.abs(v);
  if (a === 0) return '0';
  if (a >= 100) return v.toFixed(0);
  if (a >= 1) return v.toFixed(2);
  if (a >= 0.01) return v.toFixed(3);
  return v.toExponential(1);
}

// textarea editor for a 6 x 6 matrix (paste from a spreadsheet or solver output)
function matrixEditor(label, get, set) {
  const M = get();
  const ta = h('textarea', { rows: 7, spellcheck: false, 'aria-label': label, style: { fontFamily: 'var(--font-data)', fontSize: '0.75rem', whiteSpace: 'pre' } });
  ta.value = (M || []).map((r) => r.map((v) => String(+(+v).toPrecision(6)).padStart(11)).join(' ')).join('\n');
  const msg = h('span', { class: 'note' });
  return h('div', { class: 'field' }, h('span', {}, label), ta,
    h('div', { class: 'row' }, h('button', { class: 'btn', onclick: () => {
      try { set(parseMatrix6(ta.value)); toast('Matrix applied', 'good'); } catch (e) { msg.textContent = e.message; ta.classList.add('invalid'); }
    } }, 'Apply matrix'), msg));
}
