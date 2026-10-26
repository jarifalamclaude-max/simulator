// Simple parametric bodies for trying a frame before (or without) CAD: an
// open-frame ROV, a closed box and a torpedo-shaped AUV. Like the sample
// vehicle they are built from closed primitives, with a thruster duct at
// every thruster of the configuration, so buoyancy, BEM and CFD run on them
// exactly as on an uploaded STL. Vehicle frame: x forward, y right, z down.

import { box, cylinder, tube, ellipsoid } from '../geometry/primitives.js';
import { vnorm } from '../core/math.js';

export const DEFAULT_SHAPE = { type: 'openframe', length: 0.46, width: 0.34, height: 0.26, tube: 0.114, diameter: 0.2, foam: true, ducts: true };

export const SHAPES = {
  openframe: { label: 'Open frame (two side plates, tubes, foam)', dims: ['length', 'width', 'height', 'tube'] },
  box: { label: 'Closed box', dims: ['length', 'width', 'height'] },
  torpedo: { label: 'Torpedo (AUV)', dims: ['length', 'diameter'] },
};
export const DIM_LABEL = { length: 'Length (X)', width: 'Width (Y)', height: 'Height (Z)', tube: 'Main tube diameter', diameter: 'Diameter' };

export function shapeGeometry(shape = DEFAULT_SHAPE, thrusters = []) {
  const s = { ...DEFAULT_SHAPE, ...shape };
  const L = Math.max(0.05, +s.length), W = Math.max(0.05, +s.width), H = Math.max(0.03, +s.height);
  const parts = [];
  const part = (name, color, build) => { const arr = []; build(arr); if (arr.length) parts.push({ name, color, positions: new Float32Array(arr) }); };

  if (s.type === 'box') {
    part('Hull', '#cfd8dc', (o) => box(o, [0, 0, 0], [L, W, H]));
  } else if (s.type === 'torpedo') {
    // nose and tail caps protrude 0.75 D and 0.55 D beyond the cylinder, so the
    // overall length is exactly L (caps shortened when L < 1.3 D)
    const D = Math.max(0.03, +s.diameter), body = Math.max(0.01, L - 1.3 * D);
    const kc = Math.min(1, (L - body) / (1.3 * D));
    part('Hull', '#cfd8dc', (o) => {
      cylinder(o, [0, 0, 0], [1, 0, 0], D / 2, body, 40);
      ellipsoid(o, [body / 2, 0, 0], [D * 0.75 * kc, D / 2, D / 2], 16, 32); // nose
      ellipsoid(o, [-body / 2, 0, 0], [D * 0.55 * kc, D / 2, D / 2], 16, 32); // tail
    });
  } else {
    // open frame in the style of a BlueROV2, scaled to L x W x H
    const t = 0.0127, Dt = Math.min(Math.max(0.03, +s.tube), H * 0.6, W * 0.6);
    part('Frame', '#1f2a33', (o) => {
      box(o, [0, W / 2 - t / 2, 0], [L, t, H]);
      box(o, [0, -(W / 2 - t / 2), 0], [L, t, H]);
      box(o, [L / 2 - 0.03, 0, H / 2 - 0.025], [0.012, W - 2 * t, 0.05]);
      box(o, [-(L / 2 - 0.03), 0, H / 2 - 0.025], [0.012, W - 2 * t, 0.05]);
      box(o, [0, 0, H / 2 - 0.003], [L * 0.88, W - 2 * t, 0.006]);
    });
    part('Electronics tube', '#cfd8dc', (o) => cylinder(o, [0, 0, -H * 0.12], [1, 0, 0], Dt / 2, L * 0.65, 40));
    part('Battery tube', '#b0bec5', (o) => cylinder(o, [0, 0, H * 0.22], [1, 0, 0], Dt * 0.39, L * 0.55, 32));
    if (s.foam !== false) part('Buoyancy foam', '#ffb300', (o) => {
      box(o, [0, W * 0.3, -H / 2 + H * 0.1], [L * 0.74, W * 0.24, H * 0.2]);
      box(o, [0, -W * 0.3, -H / 2 + H * 0.1], [L * 0.74, W * 0.24, H * 0.2]);
    });
  }
  // thruster ducts (T200-sized unless the thruster says otherwise)
  if (s.ducts !== false && thrusters.length) part('Thrusters', '#263238', (o) => {
    for (const th of thrusters) {
      if (th.enabled === false) continue;
      const a = vnorm(th.axis);
      const r = Math.max(0.02, (th.propDiameter ?? 0.076) / 2 + 0.012);
      tube(o, th.pos, a, r, r - 0.006, 0.09, 24);
      cylinder(o, th.pos, a, Math.max(0.012, r * 0.42), 0.06, 16);
    }
  });
  const total = parts.reduce((n, p) => n + p.positions.length, 0);
  const positions = new Float32Array(total);
  let off = 0;
  for (const p of parts) { positions.set(p.positions, off); off += p.positions.length; }
  return { positions, parts };
}
