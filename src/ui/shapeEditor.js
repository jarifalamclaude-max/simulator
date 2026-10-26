// Editor for the simple parametric body (Vehicle and Frame panels).

import { h, numberField, selectField, checkbox } from './dom.js';
import { DEFAULT_SHAPE, SHAPES, DIM_LABEL } from '../vehicle/shapes.js';

export function shapeEditor(app) {
  const m = app.cfg.model;
  const s = (m.shape = { ...DEFAULT_SHAPE, ...(m.shape || {}) });
  const apply = () => {
    if (app.cfg.mass.source !== 'user') app.cfg.mass.mass = 0; // neutral mass re-estimated for the new body
    app.changed({ geometry: true });
  };
  const def = SHAPES[s.type] ?? SHAPES.openframe;
  return h('div', { class: 'section' },
    selectField({ label: 'Body shape', options: Object.entries(SHAPES).map(([k, v]) => [k, v.label]), get: () => s.type, set: (v) => { s.type = v; apply(); app.refreshPanel(); } }),
    h('div', { class: 'grid2' }, def.dims.map((d) => numberField({ label: DIM_LABEL[d], unit: 'm', step: 0.005, min: 0.02, max: 20, get: () => s[d], set: (v) => { s[d] = v; apply(); } }))),
    s.type === 'openframe' ? checkbox('Buoyancy foam blocks on top', () => s.foam !== false, (v) => { s.foam = v; apply(); }) : null,
    checkbox('A thruster duct at every thruster (moves with them)', () => s.ducts !== false, (v) => { s.ducts = v; apply(); }),
    h('p', { class: 'note' }, 'A quick body for trying a frame idea before CAD. Buoyancy, added mass and CFD run on it like on an STL; the mass is estimated for neutral buoyancy until you enter the real one in Vehicle.'));
}
