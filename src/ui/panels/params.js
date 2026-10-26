// Parameters panel: every ArduSub 4.5.7 parameter, searchable and grouped by
// prefix like QGroundControl's editor, with units, range, description, value
// lists and bitmasks from ArduPilot's metadata. Marks which values change the
// simulated flight and which are only stored and written back on export.

import { h, toast, confirmDialog, pickFile, saveFile, checkbox, seg } from '../dom.js';
import { PARAM_DEFS, PARAM_INDEX, PARAM_GROUPS, groupLabel, defaultParams, parseParamFile, formatParamFile, paramRange, crossCheck, STRUCTURAL } from '../../ardusub/params.js';
import { applyParamFile, allParamsForExport, setParamValue } from '../../vehicle/config.js';
import DUBURI_PARAMS from '../../../profiles/duburi_pixhawk_param.param';

const KEPT = '__kept';
const PAGE = 150;
const state = { q: '', group: '', show: 'all', open: new Set(), limit: PAGE };
const COUNTS = PARAM_DEFS.reduce((c, p) => { c[p.group] = (c[p.group] || 0) + 1; return c; }, {});
const SIM_COUNT = PARAM_DEFS.filter((p) => p.effect === 'sim').length;

export async function importParamFile(app, file) {
  const text = await file.text();
  const { params, errors, types, format } = parseParamFile(text);
  const n = Object.keys(params).length;
  if (!n) { toast(`No parameters found in ${file.name}`, 'bad'); return; }
  const dirs = Object.keys(params).filter((k) => /^MOT_\d+_DIRECTION$/.test(k)).length;
  let sync = false;
  if (dirs) {
    sync = await confirmDialog('Match thruster spin directions?',
      'This file sets MOT_n_DIRECTION. If your real vehicle already flies correctly with these values, the props of the reversed motors must push the other way on the hardware. Match the simulated thrusters to that (recommended), or keep the thruster directions you set in the simulator?',
      { ok: 'Match my vehicle', cancel: 'Keep simulator thrusters' });
  }
  const rep = applyParamFile(app.cfg, params, { syncPolarity: sync, types });
  app.changed({ immediate: true });
  app.refreshPanel();
  const parts = [`${n} parameters (${format === 'qgc' ? 'QGC' : 'Mission Planner'} format): ${rep.simulated} simulated, ${rep.stored} stored`];
  if (rep.unknown.length) parts.push(`${rep.unknown.length} not in ArduSub 4.5.7, kept for export`);
  if (rep.converted.length) parts.push(`${rep.converted.length} old names converted (${rep.converted.map(([a, b]) => `${a}→${b}`).slice(0, 3).join(', ')})`);
  if (errors.length) parts.push(`${errors.length} lines skipped`);
  if (rep.polarityChanged.length) parts.push(`thrusters ${rep.polarityChanged.join(', ')} reversed`);
  toast(`${file.name}: ${parts.join('; ')}`, 'good', 8000);
}

const fmtVal = (v) => (Number.isInteger(v) ? String(v) : String(+(+v).toPrecision(7)));
export function slug(s) { return String(s || 'vehicle').toLowerCase().replace(/[^a-z0-9]+/g, '-').replace(/^-|-$/g, '') || 'vehicle'; }

export default {
  id: 'params', title: 'Parameters', short: 'Params', icon: 'params', sub: 'Every ArduSub 4.5.7 parameter, live like QGroundControl',
  create(app) {
    const cfg = app.cfg;
    const defaults = defaultParams();
    const el = h('div', { class: 'section', style: { gap: '14px' } });
    const val = (name) => cfg.params[name] ?? defaults[name];
    const isMod = (name) => Math.abs(val(name) - defaults[name]) > 1e-9;
    const kept = () => Object.keys(cfg.extraParams || {});
    const problems = () => new Map(crossCheck(cfg.params).map(([k, m]) => [k, m]));

    // ---- toolbar ----
    const exportAs = (style) => {
      const all = allParamsForExport(cfg);
      saveFile(`${slug(cfg.name)}.${style === 'qgc' ? 'params' : 'param'}`, formatParamFile(all, { style }), 'text/plain');
    };
    const search = h('input', { type: 'search', placeholder: 'Search name, title or description', value: state.q, 'aria-label': 'Search parameters', oninput: () => { state.q = search.value; state.limit = PAGE; render(); } });
    const groupSel = h('select', { 'aria-label': 'Parameter group', onchange: () => { state.group = groupSel.value; state.limit = PAGE; render(); } },
      h('option', { value: '' }, `All groups (${PARAM_DEFS.length})`),
      PARAM_GROUPS.map((g) => h('option', { value: g }, `${g}_ · ${groupLabel(g)} (${COUNTS[g]})`)),
      kept().length ? h('option', { value: KEPT }, `Not in ArduSub 4.5.7, kept from your file (${kept().length})`) : null);
    groupSel.value = state.group;
    const show = seg([['all', 'All'], ['sim', 'Affect simulation'], ['inert', 'Stored only'], ['mod', 'Changed']], state.show, (v) => { state.show = v; state.limit = PAGE; render(); });
    const modCount = PARAM_DEFS.filter((p) => isMod(p.name)).length;
    el.append(h('div', { class: 'section' },
      h('div', { class: 'row' },
        h('button', { class: 'btn primary', onclick: async () => { const f = await pickFile('.param,.parm,.params,.txt'); if (f) await importParamFile(app, f); } }, 'Load .param file'),
        h('button', { class: 'btn', onclick: () => exportAs('mp') }, 'Save (Mission Planner)'),
        h('button', { class: 'btn', onclick: () => exportAs('qgc') }, 'Save (QGC)')),
      h('div', { class: 'grid2' }, search, groupSel),
      show,
      h('p', { class: 'note' },
        `${PARAM_DEFS.length} ArduSub 4.5.7 parameters with ArduPilot's descriptions, ranges and defaults. `,
        h('span', { class: 'pill good' }, `${SIM_COUNT} affect the simulation`), ' ',
        h('span', { class: 'pill muted' }, `${PARAM_DEFS.length - SIM_COUNT} stored only`), ' ',
        modCount ? h('span', { class: 'pill warn' }, `${modCount} changed from default`) : null,
        ' Stored-only values (calibration, EKF tuning, serial ports, logging…) don\'t change the simulated flight; they are kept exactly and written back when you save.',
        cfg.paramNames?.length ? ` Export writes the ${cfg.paramNames.length} parameters of your file plus any you change.` : '')));

    const list = h('div', { class: 'prm-list' });
    el.append(list);

    // ---- editors ----
    function editor(p, row) {
      const v = val(p.name);
      if (p.values && !p.bitmask) {
        const opts = Object.entries(p.values);
        const sel = h('select', { 'aria-label': p.name, onchange: () => commit(p, Number(sel.value), sel, row) }, opts.map(([k, l]) => h('option', { value: String(k) }, `${k} · ${l}`)));
        if (!opts.some(([k]) => Number(k) === v)) sel.append(h('option', { value: String(v) }, `${v} · (not a listed value)`));
        sel.value = String(v);
        return sel;
      }
      const r = paramRange(p.name) || {};
      const inp = h('input', { type: 'number', step: p.increment ?? (p.type === 'REAL32' ? 'any' : 1), min: r.min, max: r.max, value: fmtVal(v), 'aria-label': p.name, class: 'prm-num' });
      inp.addEventListener('change', () => commit(p, inp.value.trim() === '' ? NaN : Number(inp.value), inp, row));
      inp.addEventListener('keydown', (e) => { if (e.key === 'Enter') { e.preventDefault(); inp.blur(); } });
      if (!p.bitmask) return inp;
      // bitmask: number + one checkbox per documented bit
      const bits = h('details', { class: 'prm-bits' }, h('summary', {}, 'bits'),
        Object.entries(p.bitmask).map(([b, l]) => {
          const cb = h('input', { type: 'checkbox' });
          cb.checked = ((val(p.name) >>> 0) & (2 ** +b)) !== 0;
          cb.addEventListener('change', () => { const cur = val(p.name) >>> 0; const nv = cb.checked ? cur | (2 ** +b) : cur & ~(2 ** +b); commit(p, nv >>> 0 > 2147483647 ? (nv >>> 0) - 4294967296 : nv >>> 0, inp, row); inp.value = fmtVal(val(p.name)); });
          return h('label', { class: 'row check' }, cb, h('span', { class: 'note' }, `${b}: ${l}`));
        }));
      return h('div', { class: 'prm-edit-bits' }, inp, bits);
    }
    function commit(p, value, input, row) {
      const r = setParamValue(cfg, p.name, value);
      const msg = row.querySelector('.prm-msg');
      input.classList.toggle('invalid', !!r.error);
      if (r.error) { msg.textContent = r.error; msg.className = 'prm-msg note bad'; toast(`${p.name}: ${r.error}`, 'bad'); return; }
      app.setParam(p.name, r.value);
      if ('value' in input && input.tagName === 'INPUT') input.value = fmtVal(r.value);
      const cross = problems().get(p.name);
      msg.textContent = [r.warning, cross].filter(Boolean).join(' · ');
      msg.className = `prm-msg note ${r.warning || cross ? 'warn' : ''}`;
      if (STRUCTURAL.test(p.name)) toast(`${p.name} changed: the vehicle model was rebuilt (a real autopilot needs a reboot).`);
      row.classList.toggle('mod', isMod(p.name));
      const badge = row.querySelector('.prm-modpill');
      if (badge) badge.hidden = !isMod(p.name);
    }

    function rowFor(p) {
      const row = h('div', { class: `prm-row${isMod(p.name) ? ' mod' : ''}` });
      const meta = [];
      if (p.units) meta.push(p.units);
      if (p.min !== undefined || p.max !== undefined) meta.push(`range ${p.min ?? '…'} – ${p.max ?? '…'}`);
      if (p.increment !== undefined) meta.push(`step ${p.increment}`);
      meta.push(`default ${fmtVal(p.default)}${p.defaultUnknown ? ' (board-specific)' : ''}`);
      meta.push(p.type);
      const cross = problems().get(p.name);
      row.append(
        h('div', { class: 'prm-head' },
          h('span', { class: 'mono prm-name' }, p.name),
          h('span', { class: 'prm-title' }, p.displayName || ''),
          h('span', { class: 'prm-pills' },
            p.effect === 'sim' ? h('span', { class: 'pill good', title: 'The simulator uses this value' }, 'simulated') : h('span', { class: 'pill muted', title: 'Stored and exported unchanged; does not change the simulated flight' }, 'stored'),
            p.forced !== undefined ? h('span', { class: 'pill warn', title: p.note }, `forced ${p.forced} at boot`) : null,
            p.reboot ? h('span', { class: 'pill warn', title: 'Needs a reboot on the real autopilot' }, 'reboot') : null,
            p.readOnly ? h('span', { class: 'pill muted' }, 'read-only') : null,
            h('span', { class: 'pill warn prm-modpill', hidden: !isMod(p.name) }, 'changed'))),
        h('div', { class: 'prm-body' },
          h('div', { class: 'prm-value' }, editor(p, row), p.units ? h('span', { class: 'note' }, p.units) : null),
          h('div', { class: 'note prm-desc' }, p.desc || '', p.note ? h('div', { class: 'note warn' }, p.note) : null),
          h('div', { class: 'note prm-meta' }, meta.join(' · ')),
          h('div', { class: `prm-msg note ${cross ? 'warn' : ''}` }, cross || '')));
      return row;
    }

    // parameters from the user's file that ArduSub 4.5.7 does not define: editable, exported as they are
    function keptRows(q) {
      return kept().filter((k) => !q || k.toLowerCase().includes(q)).sort().map((k) => {
        const inp = h('input', { type: 'number', step: 'any', value: String(cfg.extraParams[k]), 'aria-label': k, class: 'prm-num' });
        inp.addEventListener('change', () => { const x = Number(inp.value); const bad = !Number.isFinite(x) || inp.value.trim() === ''; inp.classList.toggle('invalid', bad); if (bad) return; cfg.extraParams[k] = x; app.scheduleSave(); });
        return h('div', { class: 'prm-row' },
          h('div', { class: 'prm-head' }, h('span', { class: 'mono prm-name' }, k), h('span', { class: 'prm-pills' }, h('span', { class: 'pill muted', title: 'Not an ArduSub 4.5.7 parameter (board-specific or another firmware version): kept from your file and written back on export' }, 'kept from file'))),
          h('div', { class: 'prm-body' }, h('div', { class: 'prm-value' }, inp)));
      });
    }

    const matches = (p, q) => !q || p.name.toLowerCase().includes(q) || (p.displayName || '').toLowerCase().includes(q) || (p.desc || '').toLowerCase().includes(q);
    const shown = (p) => state.show === 'all' || (state.show === 'mod' ? isMod(p.name) : p.effect === state.show);
    function more(n, total) {
      return n < total ? h('button', { class: 'btn small', onclick: () => { state.limit += PAGE * 2; render(); } }, `Show more (${total - n} left)`) : null;
    }
    function render() {
      const q = state.q.trim().toLowerCase();
      if (state.group === KEPT) { const rows = keptRows(q); list.replaceChildren(...(rows.length ? rows : [h('p', { class: 'note' }, 'No parameters match.')])); return; }
      const sel = PARAM_DEFS.filter((p) => (!state.group || p.group === state.group) && matches(p, q) && shown(p));
      if (q || state.group || state.show !== 'all') { // flat list
        const rows = sel.slice(0, state.limit).map(rowFor);
        const extra = q && !state.group && state.show === 'all' ? keptRows(q) : [];
        list.replaceChildren(...rows, ...extra, ...(rows.length + extra.length ? [] : [h('p', { class: 'note' }, 'No parameters match.')]), more(rows.length, sel.length) || '');
        return;
      }
      // grouped by prefix, rows built when a group is opened
      list.replaceChildren(...PARAM_GROUPS.map((g) => {
        const ps = sel.filter((p) => p.group === g);
        const nSim = ps.filter((p) => p.effect === 'sim').length, nMod = ps.filter((p) => isMod(p.name)).length;
        const body = h('div', { class: 'prm-group-body' });
        const d = h('details', { class: 'prm-group', open: state.open.has(g) },
          h('summary', {}, h('span', { class: 'mono' }, `${g}_`), ' ', h('span', {}, groupLabel(g)), h('span', { class: 'note' }, ` ${ps.length}`),
            nSim ? h('span', { class: 'pill good' }, `${nSim} simulated`) : null, nMod ? h('span', { class: 'pill warn' }, `${nMod} changed`) : null),
          body);
        const fill = () => { if (!body.childElementCount) body.append(...ps.map(rowFor)); };
        if (d.open) fill();
        d.addEventListener('toggle', () => { if (d.open) { state.open.add(g); fill(); } else state.open.delete(g); });
        return d;
      }), kept().length ? h('p', { class: 'note' }, `${kept().length} names from your file are not ArduSub 4.5.7 parameters; pick "Not in ArduSub 4.5.7" above to see them.`) : '');
    }
    render();

    // ---- defaults / profile ----
    el.append(h('div', { class: 'section' }, h('h3', {}, 'Reset'),
      h('div', { class: 'row' },
        h('button', { class: 'btn', onclick: async () => {
          if (!await confirmDialog('Load the DUBURI Pixhawk profile?', 'Replaces all parameters with the values from duburi_pixhawk_param.param (ArduSub 4.5.7, FRAME_CONFIG 2) and matches thruster directions to its MOT_n_DIRECTION.', { ok: 'Load profile' })) return;
          cfg.params = defaultParams(); cfg.extraParams = {}; cfg.paramNames = []; cfg.paramTypes = {};
          applyParamFile(cfg, parseParamFile(DUBURI_PARAMS).params, { syncPolarity: true });
          app.changed({ immediate: true }); app.refreshPanel(); toast('DUBURI profile loaded', 'good');
        } }, 'Load DUBURI profile'),
        h('button', { class: 'btn danger', onclick: async () => {
          if (!await confirmDialog('Reset every parameter to the ArduSub default?', 'Your changes and imported values will be lost (names that ArduSub 4.5.7 does not define are kept for export).', { ok: 'Reset all', danger: true })) return;
          cfg.params = defaultParams();
          app.changed({ immediate: true }); app.refreshPanel(); toast('Parameters reset to ArduSub defaults');
        } }, 'Reset to ArduSub defaults')),
      checkbox('Expand groups that have changed values', () => state.autoOpen, (v) => { state.autoOpen = v; if (v) PARAM_GROUPS.forEach((g) => { if (PARAM_DEFS.some((p) => p.group === g && isMod(p.name))) state.open.add(g); }); else state.open.clear(); render(); }),
      h('p', { class: 'note' }, 'Values take effect immediately, exactly as when you write a parameter from QGroundControl while the vehicle is in the water. Out-of-range values are clamped to the range ArduPilot documents; integer parameters are rounded.')));
    return { el };
  },
};
