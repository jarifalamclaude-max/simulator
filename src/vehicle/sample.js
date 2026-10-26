// Procedural sample vehicle: an 8-thruster vectored-6DOF ROV in the style of
// a BlueROV2 Heavy, built from closed primitives so every analysis step
// (buoyancy, BEM, CFD) runs on it exactly as it would on a user STL.

import { box, cylinder, tube, sphere } from '../geometry/primitives.js';
import { heavyLayout } from '../alloc/frames.js';
import { vnorm } from '../core/math.js';

export function sampleGeometry() {
  const parts = [];
  const part = (name, color, build) => { const arr = []; build(arr); parts.push({ name, color, positions: new Float32Array(arr) }); };

  part('Electronics enclosure', '#cfd8dc', (o) => {
    cylinder(o, [0, 0, -0.03], [1, 0, 0], 0.057, 0.3, 40);
  });
  part('Camera dome', '#9fd3ff', (o) => sphere(o, [0.15, 0, -0.03], 0.056, 14, 28));
  part('Battery enclosure', '#b0bec5', (o) => cylinder(o, [-0.01, 0, 0.09], [1, 0, 0], 0.0445, 0.25, 32));
  part('Frame', '#1f2a33', (o) => {
    box(o, [0, 0.165, 0.02], [0.457, 0.0127, 0.254]);
    box(o, [0, -0.165, 0.02], [0.457, 0.0127, 0.254]);
    box(o, [0.21, 0, 0.11], [0.012, 0.318, 0.05]);
    box(o, [-0.21, 0, 0.11], [0.012, 0.318, 0.05]);
    box(o, [0, 0, 0.145], [0.4, 0.318, 0.006]);
    for (const sx of [1, -1]) for (const sy of [1, -1]) box(o, [sx * 0.12, sy * 0.192, -0.07], [0.07, 0.055, 0.012]);
  });
  part('Buoyancy foam', '#ffb300', (o) => {
    box(o, [0, 0.1, -0.125], [0.34, 0.08, 0.05]);
    box(o, [0, -0.1, -0.125], [0.34, 0.08, 0.05]);
  });
  part('Thrusters', '#263238', (o) => {
    for (const t of heavyLayout()) {
      const a = vnorm(t.axis);
      tube(o, t.pos, a, 0.05, 0.044, 0.09, 24);
      cylinder(o, t.pos, a, 0.021, 0.06, 16);
    }
  });
  const total = parts.reduce((s, p) => s + p.positions.length, 0);
  const positions = new Float32Array(total);
  let off = 0;
  for (const p of parts) { positions.set(p.positions, off); off += p.positions.length; }
  return { positions, parts };
}
