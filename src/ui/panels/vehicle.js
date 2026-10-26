// Vehicle panel: hull geometry (STL import, units, axes, origin), mass and
// centre of gravity, inertia, buoyancy and the resulting static behaviour.

import { h, numberField, selectField, vec3Field, seg, checkbox, fmt, toast, confirmDialog } from '../dom.js';
import { AXIS_NAMES, UNIT_SCALES } from '../../geometry/mesh.js';
import { inertiaTensor, cgOf } from '../../vehicle/model.js';
import { DESIGN_ACCEPT } from '../../geometry/formats.js';
import { shapeEditor } from '../shapeEditor.js';

export default {
  id: 'vehicle', title: 'Vehicle', short: 'Vehicle', icon: 'hull', sub: 'Hull geometry, mass, centre of gravity and buoyancy',
  create(app) {
    const cfg = app.cfg;
    const el = h('div', { class: 'section', style: { gap: '18px' } });
    const geo = app.geo;
    const info = app.sim?.info;

    // ---- model ----
    const drop = h('div', { class: 'dropzone', tabindex: '0', role: 'button', 'aria-label': 'Load an STL file' },
      h('b', {}, 'Drop your design here'), h('div', {}, 'or click to choose a file: STL (binary or ASCII), OBJ or PLY'));
    const fileIn = h('input', { type: 'file', accept: DESIGN_ACCEPT, hidden: true });
    const load = async (f) => { if (!f) return; drop.textContent = `Reading ${f.name}…`; await app.loadDesignFile(f); app.refreshPanel(); };
    drop.addEventListener('click', () => fileIn.click());
    drop.addEventListener('keydown', (e) => { if (e.key === 'Enter' || e.key === ' ') { e.preventDefault(); fileIn.click(); } });
    fileIn.addEventListener('change', () => load(fileIn.files[0]));
    drop.addEventListener('dragover', (e) => { e.preventDefault(); drop.classList.add('over'); });
    drop.addEventListener('dragleave', () => drop.classList.remove('over'));
    drop.addEventListener('drop', (e) => { e.preventDefault(); e.stopPropagation(); drop.classList.remove('over'); load(e.dataTransfer.files[0]); });

    const m = cfg.model;
    const isSTL = m.source === 'stl';
    const topo = geo.topology;
    const modelSec = h('div', { class: 'section' }, h('h3', {}, 'Hull model'),
      h('div', { class: 'row', style: { justifyContent: 'space-between' } },
        h('div', {}, h('b', {}, isSTL ? m.fileName || 'STL file' : m.source === 'shape' ? 'Simple shape (built in the simulator)' : 'Sample ROV (8 thrusters, vectored 6-DOF)'), h('div', { class: 'note' }, `${topo.triangles.toLocaleString()} triangles · ${fmt(geo.bbox.size[0], 3)} × ${fmt(geo.bbox.size[1], 3)} × ${fmt(geo.bbox.size[2], 3)} m (L × W × H)`))),
      h('div', { class: 'field' }, h('span', {}, 'Body'), seg([['stl', 'My design (STL)'], ['shape', 'Simple shape'], ['sample', 'Sample ROV']], isSTL || m.source === 'shape' ? m.source : 'sample', async (v) => { if (v === m.source) return; await app.setModelSource(v); app.refreshPanel(); })),
      m.source === 'shape' ? shapeEditor(app) : null,
      drop, fileIn);
    if (isSTL) {
      const g = h('div', { class: 'grid2' },
        selectField({ label: 'File units', options: Object.keys(UNIT_SCALES).map((u) => [u, u]), get: () => m.units, set: (v) => { m.units = v; app.changed({ geometry: true }); app.refreshPanel(); } }),
        numberField({ label: 'Extra scale', step: 0.01, min: 0.0001, get: () => m.scale ?? 1, set: (v) => { m.scale = v; app.changed({ geometry: true }); } }),
        selectField({ label: 'Forward axis in the file', options: AXIS_NAMES.map((a) => [a, a]), get: () => m.forward, set: (v) => { if (v.slice(1) === m.up.slice(1)) { toast('Forward and up must be different axes', 'bad'); app.refreshPanel(); return; } m.forward = v; app.changed({ geometry: true }); } }),
        selectField({ label: 'Up axis in the file', options: AXIS_NAMES.map((a) => [a, a]), get: () => m.up, set: (v) => { if (v.slice(1) === m.forward.slice(1)) { toast('Forward and up must be different axes', 'bad'); app.refreshPanel(); return; } m.up = v; app.changed({ geometry: true }); } }),
        selectField({ label: 'Origin', options: [['bbox', 'Centre of bounding box'], ['file', 'Origin of the STL file']], get: () => m.origin, set: (v) => { m.origin = v; app.changed({ geometry: true }); } }),
        h('label', { class: 'field' }, h('span', {}, 'Hull colour'), h('input', { type: 'color', class: 'swatch', value: m.color || '#dfe6ea', oninput: (e) => { m.color = e.target.value; app.scene.setColor(m.color); app.scheduleSave(); } })));
      modelSec.append(g,
        vec3Field({ label: 'Origin offset (moves the hull in the vehicle frame)', get: () => m.offset || [0, 0, 0], set: (v) => { m.offset = v; app.changed({ geometry: true }); } }),
        checkbox('Sealed hull: count enclosed cavities as displaced water (for hollow shell exports)', () => m.sealedHull, (v) => { m.sealedHull = v; app.changed({ geometry: true }); }),
        h('p', { class: 'note' }, 'Vehicle frame: X forward, Y to starboard (right), Z down, metres. Check the red X arrow in the 3D view points to the bow.'),
        h('div', { class: 'row' },
          h('button', { class: 'btn', onclick: () => app.detectDesignThrusters() }, 'Find thrusters in my design'),
          h('span', { class: 'note' }, 'Parts named T200_1, Thruster 2… (OBJ, ASCII STL) or duct shapes. Runs after every upload.')));
    }
    const meshPill = topo.watertight ? h('span', { class: 'pill good' }, 'Closed mesh') : h('span', { class: 'pill warn' }, `${topo.boundaryEdges.toLocaleString()} open edges`);
    modelSec.append(h('div', { class: 'card' },
      h('div', { class: 'row', style: { justifyContent: 'space-between' } }, h('b', {}, 'Mesh check'), meshPill),
      h('dl', { class: 'kv' },
        h('dt', {}, `Displaced volume (${geo.volumeSource === 'ray' ? 'integrated along rays' : geo.volumeSource === 'sealed-hull' ? 'sealed hull' : 'voxels'})`), h('dd', {}, `${fmt(geo.volume * 1000, 2)} L`),
        h('dt', {}, 'Volume from mesh surface'), h('dd', {}, Number.isFinite(geo.meshVolume) && Math.abs(geo.meshVolume) > 1e-9 ? `${fmt(Math.abs(geo.meshVolume) * 1000, 2)} L` : '—'),
        h('dt', {}, 'Voxel size'), h('dd', {}, `${fmt(geo.cell * 1000, 1)} mm`),
        h('dt', {}, 'Centre of buoyancy (x, y, z)'), h('dd', {}, geo.centroid.map((v) => fmt(v, 3)).join(', ') + ' m')),
      topo.watertight ? null : h('p', { class: 'note warn' }, 'The mesh is not closed. The simulator still voxelises it robustly, but check the displaced volume and enter it manually under Buoyancy if it looks wrong.')));
    el.append(modelSec);

    // ---- mass & CG ----
    const massSec = h('div', { class: 'section' }, h('h3', {}, 'Mass and centre of gravity'));
    const cg = cgOf(cfg, geo);
    massSec.append(h('div', { class: 'grid2' },
      numberField({ label: cfg.mass.source === 'user' ? 'Mass in air' : 'Mass in air (estimated: enter yours)', unit: 'kg', step: 0.01, min: 0.01, get: () => cfg.mass.mass, set: (v) => { cfg.mass.mass = v; cfg.mass.source = 'user'; app.changed(); refreshDerived(); } }),
      h('div', { class: 'field' }, h('span', {}, 'CG from'), seg([['cb-offset', 'CB + offset'], ['manual', 'CAD value']], cfg.mass.cgMode, (v) => {
        if (v === 'manual' && cfg.mass.cgMode !== 'manual') cfg.mass.cg = cgOf(cfg, geo).map((x) => +x.toFixed(4));
        cfg.mass.cgMode = v; app.changed(); app.refreshPanel();
      }))));
    if (cfg.mass.cgMode === 'manual') massSec.append(vec3Field({ label: 'Centre of gravity (vehicle frame)', get: () => cfg.mass.cg, set: (v) => { cfg.mass.cg = v; app.changed(); } }));
    else massSec.append(vec3Field({ label: 'CG offset from the centre of buoyancy (Z down: positive = CG below CB)', get: () => cfg.mass.cgOffset, set: (v) => { cfg.mass.cgOffset = v; app.changed(); } }),
      h('p', { class: 'note' }, `CG is at ${cg.map((v) => fmt(v, 3)).join(', ')} m. For accuracy enter the CG from your CAD mass properties.`));
    const I = cfg.mass.inertia;
    const It = inertiaTensor(cfg, geo, cg);
    massSec.append(h('div', { class: 'field' }, h('span', {}, 'Inertia about the CG'), seg([['auto', 'Estimate from hull'], ['manual', 'CAD value']], I.mode, (v) => {
      if (v === 'manual' && I.mode !== 'manual') { I.Ixx = +It[0].toFixed(4); I.Iyy = +It[4].toFixed(4); I.Izz = +It[8].toFixed(4); I.Pxy = +(-It[1]).toFixed(5); I.Pxz = +(-It[2]).toFixed(5); I.Pyz = +(-It[5]).toFixed(5); }
      I.mode = v; app.changed(); app.refreshPanel();
    })));
    if (I.mode === 'manual') {
      massSec.append(h('div', { class: 'grid3' },
        ...['Ixx', 'Iyy', 'Izz'].map((k) => numberField({ label: k, unit: 'kg·m²', step: 0.001, min: 1e-6, get: () => I[k], set: (v) => { I[k] = v; app.changed(); } })),
        ...['Pxy', 'Pxz', 'Pyz'].map((k) => numberField({ label: k, unit: 'kg·m²', step: 0.0001, get: () => I[k], set: (v) => { I[k] = v; app.changed(); } }))),
      h('p', { class: 'note' }, 'Products of inertia as CAD tools report them (P = ∫xy dm, about the CG, in the vehicle frame).'));
    } else massSec.append(h('p', { class: 'note' }, `Uniform-density estimate: Ixx ${fmt(It[0], 3)}, Iyy ${fmt(It[4], 3)}, Izz ${fmt(It[8], 3)} kg·m². Batteries and ballast make the real values smaller; enter CAD values when you have them.`));
    el.append(massSec);

    // ---- buoyancy ----
    const B = cfg.buoyancy;
    const buoySec = h('div', { class: 'section' }, h('h3', {}, 'Buoyancy'),
      h('div', { class: 'field' }, h('span', {}, 'Displaced volume and centre of buoyancy'), seg([['auto', 'From hull'], ['manual', 'Manual']], B.mode, (v) => {
        if (v === 'manual' && B.mode !== 'manual') { B.volume = +geo.volume.toFixed(6); B.cb = geo.centroid.map((x) => +x.toFixed(4)); }
        B.mode = v; app.changed(); app.refreshPanel();
      })));
    if (B.mode === 'manual') buoySec.append(numberField({ label: 'Displaced volume', unit: 'L', step: 0.01, min: 0.001, get: () => B.volume * 1000, set: (v) => { B.volume = v / 1000; app.changed(); } }),
      vec3Field({ label: 'Centre of buoyancy (vehicle frame)', get: () => B.cb, set: (v) => { B.cb = v; app.changed(); } }));
    const derived = h('div', { class: 'card' });
    const refreshDerived = () => {
      const inf = app.sim?.info;
      if (!inf) return;
      derived.replaceChildren(
        h('div', { class: 'row', style: { justifyContent: 'space-between' } }, h('b', {}, 'In the water'), buoyPill(inf)),
        h('dl', { class: 'kv' },
          h('dt', {}, 'Weight'), h('dd', {}, `${fmt(inf.weightN, 1)} N`),
          h('dt', {}, `Buoyancy (ρ ${fmt(inf.rho, 1)} kg/m³)`), h('dd', {}, `${fmt(inf.buoyancyN, 1)} N`),
          h('dt', {}, 'Net (up positive)'), h('dd', {}, `${fmt(inf.netBuoyancyN, 2)} N · ${fmt(inf.netBuoyancyKg * 1000, 0)} g`),
          h('dt', {}, 'CB above CG (GM)'), h('dd', {}, `${fmt(inf.GM * 100, 1)} cm`),
          h('dt', {}, 'Resting trim roll / pitch'), h('dd', {}, `${fmt(inf.staticRollDeg, 1)}° / ${fmt(inf.staticPitchDeg, 1)}°`),
          h('dt', {}, 'Natural roll / pitch period'), h('dd', {}, `${fmt(inf.rollPeriod, 2)} s / ${fmt(inf.pitchPeriod, 2)} s`)),
        h('div', { class: 'row' },
          h('button', { class: 'btn', onclick: () => { cfg.mass.mass = +(inf.rho * inf.volume).toFixed(3); cfg.mass.source = 'user'; app.changed({ immediate: true }); app.refreshPanel(); } }, 'Make neutral'),
          h('button', { class: 'btn', onclick: () => { cfg.mass.mass = +(inf.rho * inf.volume * 0.985).toFixed(3); cfg.mass.source = 'user'; app.changed({ immediate: true }); app.refreshPanel(); } }, 'Slightly positive (1.5%)')),
        ...(inf.warnings || []).map((w) => h('p', { class: 'note warn' }, w)));
    };
    refreshDerived();
    buoySec.append(derived);
    el.append(buoySec);

    // ---- start pose ----
    el.append(h('div', { class: 'section' }, h('h3', {}, 'Start position'),
      h('div', { class: 'grid3' },
        numberField({ label: 'North', unit: 'm', step: 0.1, get: () => cfg.start.position[0], set: (v) => { cfg.start.position[0] = v; app.scheduleSave(); } }),
        numberField({ label: 'East', unit: 'm', step: 0.1, get: () => cfg.start.position[1], set: (v) => { cfg.start.position[1] = v; app.scheduleSave(); } }),
        numberField({ label: 'Depth', unit: 'm', step: 0.1, min: -1, get: () => cfg.start.position[2], set: (v) => { cfg.start.position[2] = v; app.scheduleSave(); } })),
      numberField({ label: 'Heading', unit: '°', step: 1, get: () => cfg.start.yaw ?? 0, set: (v) => { cfg.start.yaw = v; app.scheduleSave(); } }),
      h('button', { class: 'btn', onclick: () => app.resetVehicle() }, 'Reset vehicle to start')));

    return { el, onModelChanged: refreshDerived, tick: () => {} };
  },
};

function buoyPill(inf) {
  const g = inf.netBuoyancyKg * 1000;
  if (Math.abs(g) < 30) return h('span', { class: 'pill good' }, 'Neutral');
  if (g > 0) return h('span', { class: Math.abs(inf.netBuoyancyKg) > 0.05 * inf.mass ? 'pill warn' : 'pill good' }, `Floats +${fmt(g, 0)} g`);
  return h('span', { class: 'pill warn' }, `Sinks ${fmt(-g, 0)} g`);
}
