// Mission panel: a simple AUTO waypoint list (pool frame, NED metres) sent to the
// firmware with fw.setMission(list), and GUIDED "go to" (also: click the water in
// the Top view while in GUIDED).

import { h, numberField, toast, fmt } from '../dom.js';
import { MODE, MODE_LABEL } from '../../ardusub/sub.js';

export default {
  id: 'mission', title: 'Mission', short: 'Mission', icon: 'target', sub: 'AUTO waypoints and GUIDED go-to',
  create(app) {
    const cfg = app.cfg;
    if (!Array.isArray(cfg.mission)) cfg.mission = [];
    const el = h('div', { class: 'section', style: { gap: '18px' } });
    const fw = () => app.sim.fw;
    const save = () => { app.scheduleSave(); app.wpSig = null; };
    const here = () => app.sim.plant.position.map((v) => +v.toFixed(2));
    const has = (k) => typeof fw()[k] === 'function';

    const list = h('div', { class: 'section' });
    const draw = () => {
      const wps = cfg.mission;
      list.replaceChildren(...(wps.length ? wps.map((w, i) => h('div', { class: 'card', style: { gap: '6px' } },
        h('div', { class: 'row', style: { justifyContent: 'space-between' } }, h('b', {}, `WP ${i + 1}`),
          h('div', { class: 'row', style: { gap: '4px' } },
            h('button', { class: 'btn', title: 'Move up', disabled: i === 0, onclick: () => { [wps[i - 1], wps[i]] = [wps[i], wps[i - 1]]; save(); draw(); } }, '↑'),
            h('button', { class: 'btn', title: 'Move down', disabled: i === wps.length - 1, onclick: () => { [wps[i + 1], wps[i]] = [wps[i], wps[i + 1]]; save(); draw(); } }, '↓'),
            h('button', { class: 'btn', title: 'Set to the vehicle position', onclick: () => { w.pos = here(); save(); draw(); } }, 'Here'),
            h('button', { class: 'btn', title: 'Remove', onclick: () => { wps.splice(i, 1); save(); draw(); } }, '×'))),
        h('div', { class: 'grid2' },
          numberField({ label: 'North', unit: 'm', step: 0.1, get: () => w.pos[0], set: (v) => { w.pos[0] = v; save(); } }),
          numberField({ label: 'East', unit: 'm', step: 0.1, get: () => w.pos[1], set: (v) => { w.pos[1] = v; save(); } }),
          numberField({ label: 'Depth', unit: 'm', step: 0.1, min: 0, get: () => w.pos[2], set: (v) => { w.pos[2] = v; save(); } }),
          numberField({ label: 'Hold', unit: 's', step: 1, min: 0, get: () => w.hold ?? 0, set: (v) => { w.hold = v; save(); } }))))
        : [h('p', { class: 'note' }, 'No waypoints yet. Add one at the vehicle position, or type the coordinates.')]));
    };
    draw();

    const status = h('p', { class: 'note' });
    const drawStatus = () => {
      const f = fw();
      const m = f.mission ?? f.auto?.mission;
      const idx = f.missionIndex ?? f.auto?.index ?? f.wpIndex;
      status.textContent = `Mode ${MODE_LABEL[f.mode] ?? f.mode}${f.armed ? ', armed' : ', disarmed'}.${Array.isArray(m) ? ` Vehicle holds ${m.length} waypoint${m.length === 1 ? '' : 's'}${Number.isFinite(idx) ? `, active WP ${idx + 1}` : ''}.` : ''} Position N ${fmt(app.sim.plant.position[0])} E ${fmt(app.sim.plant.position[1])} depth ${fmt(app.sim.plant.position[2])} m.`;
    };
    drawStatus();

    el.append(h('div', { class: 'section' }, h('h3', {}, 'AUTO mission'),
      h('p', { class: 'note' }, 'Waypoints in the pool frame (north/east from the pool centre, depth below the surface). Upload sends them to the vehicle; AUTO flies them in order at WPNAV_SPEED and holds each for its hold time. Waypoints show as dots in the 3D view.'),
      has('setMission') ? null : h('p', { class: 'note warn' }, 'This firmware build has no AUTO mission support yet: the list is kept with the vehicle and uploaded when it has.'),
      status, list,
      h('div', { class: 'row' },
        h('button', { class: 'btn', onclick: () => { cfg.mission.push({ pos: here(), hold: 0 }); save(); draw(); } }, 'Add at vehicle'),
        h('button', { class: 'btn', onclick: () => { const p = here(); p[0] += 1; cfg.mission.push({ pos: p, hold: 0 }); save(); draw(); } }, 'Add 1 m ahead (north)'),
        h('button', { class: 'btn', onclick: () => { cfg.mission.length = 0; save(); draw(); } }, 'Clear'),
        h('button', { class: 'btn primary', onclick: () => app.uploadMission(cfg.mission) }, 'Upload to vehicle'),
        h('button', { class: 'btn', onclick: () => { if (MODE.AUTO === undefined) return; if (app.uploadMission(cfg.mission)) app.setMode(MODE.AUTO); } }, 'Upload and start AUTO'))));

    const g = { pos: here() };
    el.append(h('div', { class: 'section' }, h('h3', {}, 'GUIDED go to'),
      h('p', { class: 'note' }, 'Sends a position target (switching to GUIDED first). Or choose the Top camera, enter GUIDED and click the water: the vehicle goes there at its current depth.'),
      h('div', { class: 'grid3' },
        numberField({ label: 'North', unit: 'm', step: 0.1, get: () => g.pos[0], set: (v) => { g.pos[0] = v; } }),
        numberField({ label: 'East', unit: 'm', step: 0.1, get: () => g.pos[1], set: (v) => { g.pos[1] = v; } }),
        numberField({ label: 'Depth', unit: 'm', step: 0.1, min: 0, get: () => g.pos[2], set: (v) => { g.pos[2] = v; } })),
      h('div', { class: 'row' },
        h('button', { class: 'btn primary', onclick: () => app.guidedGoto(g.pos) }, 'Go'),
        h('button', { class: 'btn', onclick: () => { app.camSelect.value = 'top'; app.scene.setCameraMode('top'); toast('Top view: switch to GUIDED and click the water'); } }, 'Top view'))));
    return { el, tick: drawStatus };
  },
};
