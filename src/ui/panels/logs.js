// Logs & files panel: autopilot messages, telemetry CSV export, vehicle file
// export/import (optionally with the STL inside) and the stand-alone page.

import { h, fmt, toast, saveFile, pickFile, idbGet, confirmDialog, checkbox } from '../dom.js';
import { formatParamFile } from '../../ardusub/params.js';
import { allParamsForExport, normalizeConfig } from '../../vehicle/config.js';
import { slug } from './params.js';

const SEV = ['EMERGENCY', 'ALERT', 'CRITICAL', 'ERROR', 'WARNING', 'NOTICE', 'INFO', 'DEBUG'];
const memo = { withStl: true };

export function toBase64(buf) {
  const u8 = new Uint8Array(buf);
  let s = '';
  for (let i = 0; i < u8.length; i += 0x8000) s += String.fromCharCode.apply(null, u8.subarray(i, i + 0x8000));
  return btoa(s);
}
export function fromBase64(b64) {
  const s = atob(b64);
  const u8 = new Uint8Array(s.length);
  for (let i = 0; i < s.length; i++) u8[i] = s.charCodeAt(i);
  return u8.buffer;
}

export async function exportVehicle(app, withStl) {
  const doc = { ...app.cfg, exported: new Date().toISOString(), team: { name: app.team.name } };
  if (withStl && app.cfg.model.source === 'stl') {
    const buf = await idbGet(`stl:${app.team.id}`);
    if (buf && buf.byteLength < 60e6) doc.stl = { name: app.cfg.model.fileName, base64: toBase64(buf) };
    else if (buf) toast('STL is larger than 60 MB: exported the vehicle without it.', 'bad', 6000);
  }
  return saveFile(`${slug(app.cfg.name)}.duburi.json`, JSON.stringify(doc), 'application/json');
}
export async function importVehicle(app, file) {
  let doc;
  try { doc = JSON.parse(await file.text()); } catch { toast(`${file.name} is not a DUBURI vehicle file`, 'bad'); return; }
  let cfg;
  try { cfg = normalizeConfig(doc); } catch (e) { toast(e.message, 'bad'); return; }
  const stl = doc.stl?.base64 ? fromBase64(doc.stl.base64) : null;
  delete cfg.stl; delete cfg.exported; delete cfg.team;
  if (cfg.model.source === 'stl' && !stl) toast('This vehicle file has no STL inside: load the STL in Vehicle.', '', 6000);
  await app.loadVehicle(cfg, stl);
  toast(`Loaded ${cfg.name}`, 'good');
}

export default {
  id: 'logs', title: 'Logs & files', short: 'Files', icon: 'log', sub: 'Messages, telemetry and vehicle files',
  create(app) {
    const el = h('div', { class: 'section', style: { gap: '18px' } });
    const msgBox = h('div', { class: 'card', style: { maxHeight: '260px', overflowY: 'auto', fontFamily: 'var(--font-data)', fontSize: '0.75rem', gap: '3px' } });
    const telNote = h('p', { class: 'note' });
    let lastMsg = null, lastLen = -1;
    const drawMsgs = () => {
      const tel = app.sim.telemetry;
      telNote.textContent = `${tel.count.toLocaleString()} samples at 50 Hz (${fmt(tel.count / 50, 1)} s of the last ${fmt(tel.capacity / 50 / 60, 0)} minutes are kept).`;
      const m = app.sim.fw.messages;
      // the list is capped (push + shift), so its length stops changing: compare the newest entry
      const newest = m[m.length - 1] ?? null;
      if (newest === lastMsg && m.length === lastLen && msgBox.childElementCount) return;
      lastMsg = newest; lastLen = m.length;
      msgBox.replaceChildren(...(m.length ? m.slice().reverse().map((x) => h('div', { style: { color: x.severity <= 3 ? '#ff9a9a' : x.severity === 4 ? 'var(--warning)' : 'var(--ink-2)' } }, `${fmt(x.t, 1).padStart(7)}  ${SEV[x.severity] ?? x.severity}  ${x.text}`)) : [h('span', { class: 'note' }, 'No messages yet.')]));
    };
    drawMsgs();
    el.append(h('div', { class: 'section' }, h('h3', {}, 'Autopilot messages'), msgBox));

    el.append(h('div', { class: 'section' }, h('h3', {}, 'Telemetry'), telNote,
      h('div', { class: 'row' },
        h('button', { class: 'btn primary', onclick: () => saveFile(`duburi-telemetry-${Date.now()}.csv`, app.sim.telemetry.toCSV(), 'text/csv') }, 'Export all channels (CSV)'),
        h('button', { class: 'btn', onclick: () => saveFile(`duburi-attitude-${Date.now()}.csv`, app.sim.telemetry.toCSV(['t', 'mode', 'armed', 'roll', 'roll_t', 'pitch', 'pitch_t', 'yaw', 'yaw_t', 'depth', 'depth_t', 'depth_meas', 'climb', 'saturated']), 'text/csv') }, 'Attitude & depth only'),
        h('button', { class: 'btn', onclick: () => { app.sim.telemetry.clear(); drawMsgs(); toast('Telemetry cleared'); } }, 'Clear')),
      h('p', { class: 'note' }, 'Columns include targets, PID terms, every motor output, PWM and thrust, so you can compare with a dataflash log in your own tools.')));

    el.append(h('div', { class: 'section' }, h('h3', {}, 'Vehicle file'),
      h('p', { class: 'note' }, 'Everything about the vehicle in one file: geometry settings, mass, hydrodynamics (including panel-method and CFD results), thrusters, mixer, sensors, environment and all parameters. Share it with your team or keep versions of your setup.'),
      checkbox('Include the STL inside the file', () => memo.withStl, (v) => { memo.withStl = v; }),
      h('div', { class: 'row' },
        h('button', { class: 'btn primary', onclick: () => exportVehicle(app, memo.withStl) }, 'Save vehicle file'),
        h('button', { class: 'btn', onclick: async () => {
          const f = await pickFile('.json,application/json'); if (!f) return;
          if (!await confirmDialog('Replace the current vehicle?', `Loads ${f.name} into ${app.team.name}. Export the current vehicle first if you want to keep it.`, { ok: 'Load vehicle' })) return;
          await importVehicle(app, f);
        } }, 'Load vehicle file'))));

    el.append(h('div', { class: 'section' }, h('h3', {}, 'Parameters'),
      h('div', { class: 'row' },
        h('button', { class: 'btn', onclick: () => saveFile(`${slug(app.cfg.name)}.param`, formatParamFile(allParamsForExport(app.cfg), { style: 'mp' }), 'text/plain') }, 'Save .param (Mission Planner)'),
        h('button', { class: 'btn', onclick: () => saveFile(`${slug(app.cfg.name)}.params`, formatParamFile(allParamsForExport(app.cfg), { style: 'qgc' }), 'text/plain') }, 'Save .params (QGC)'))));

    el.append(h('div', { class: 'section' }, h('h3', {}, 'Simulator page'),
      h('p', { class: 'note' }, 'The whole simulator is one HTML file. Save it and open it in Chrome or Edge to use a gamepad, run offline at the pool, or give it to another team.'),
      h('div', { class: 'row' }, h('button', { class: 'btn', onclick: () => {
        const src = window.__DUBURI_SOURCE__;
        if (!src) { toast('Page source not available in this view', 'bad'); return; }
        saveFile('duburi-simulator.html', src, 'text/html');
      } }, 'Save simulator page'))));
    return { el, tick: drawMsgs };
  },
};
