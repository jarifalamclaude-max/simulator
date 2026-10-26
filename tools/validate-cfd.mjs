// CFD lab validation: tows a sphere through the full pipeline (mesh ->
// voxel case -> lattice-Boltzmann -> wall correction -> coefficient) and
// compares the drag coefficient with the standard drag curve.
//   node tools/validate-cfd.mjs [cellsPerDiameter=12] [Re=100]
import { sphere } from '../src/geometry/primitives.js';
import { buildCase, runCPU } from '../src/cfd/study.js';

const N = +(process.argv[2] || 12), Re = +(process.argv[3] || 100), wall = process.argv[4] || 'interpolated';
// standard drag curve (Clift, Grace & Weber 1978), valid 20 < Re < 260 and beyond with the second form
const cdRef = Re <= 260 ? (24 / Re) * (1 + 0.1935 * Re ** 0.6305) : (24 / Re) * (1 + 0.15 * Re ** 0.687) + 0.42 / (1 + 42500 * Re ** -1.16);
const pos = Float32Array.from(sphere([], [0, 0, 0], 0.05, 32, 48));
const cs = buildCase(pos, { dof: 'surge', N, ellRef: 0.1, wall, margins: { up: 1.5, down: 3, side: 1.25 } });
console.log(`lattice ${cs.nx} x ${cs.ny} x ${cs.nz}, ${cs.wall} walls${cs.linkStats ? ` (${cs.linkStats.hits}/${cs.linkStats.links} links hit the mesh)` : ''}, frontal ${cs.proj[0].area} cells, wall factor ${cs.kappa[0].toFixed(3)}, blockage ${(cs.blockage * 100).toFixed(1)}%`);
const t0 = Date.now();
let last = 0;
const res = runCPU(cs, Re, { onProgress: (m) => { if (Date.now() - last > 5000) { last = Date.now(); console.log(`  t = ${m.t.toFixed(1)} flow times, change ${(m.rel * 100).toFixed(2)}%`); } } });
const Cd = (-res.C[0] * 8) / Math.PI;
console.log(`Re ${Re}, ${N} cells per diameter: Cd = ${Cd.toFixed(3)} (standard curve ${cdRef.toFixed(3)}, ${((Cd / cdRef - 1) * 100).toFixed(1)}%), ${res.steps} steps, converged ${res.converged}, ${((Date.now() - t0) / 1000).toFixed(0)} s, ${res.mlups.toFixed(1)} MLUPS`);
