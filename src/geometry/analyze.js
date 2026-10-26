// One-shot geometry analysis of a hull (already in the vehicle frame, metres).
// Heavy for big STLs, so the browser runs it in a worker.

import { bbox, topology, signedVolume, makeGrid, voxelizeSolid, voxelizeSurface, massProps, projections, buoyancyElements, contactPoints, union, fillEnclosed, rayVolume, projectionColumns } from './mesh.js';

export function chooseCell(bb, targetCells = 56, minCell = 0.003) {
  const L = Math.max(...bb.size);
  return Math.max(minCell, L / targetCells);
}

export function analyzeGeometry(positions, opts = {}) {
  const onProgress = opts.onProgress || (() => {});
  const bb = bbox(positions);
  if (!Number.isFinite(bb.size[0]) || Math.max(...bb.size) <= 0) throw new Error('Empty or invalid geometry');
  onProgress(0.05, 'Checking mesh topology');
  const topo = topology(positions);
  const sv = signedVolume(positions);
  const cell = opts.cell ?? chooseCell(bb, opts.targetCells ?? 56);
  onProgress(0.15, 'Voxelising hull volume');
  const solid = makeGrid(bb, cell, 2);
  const vs = voxelizeSolid(positions, solid);
  onProgress(0.5, 'Marking thin features');
  const surf = makeGrid(bb, cell, 2);
  voxelizeSurface(positions, surf);
  // "wet" solid used for CFD walls and jet blockage: solid + surface cells
  const wet = union(solid, surf);
  onProgress(0.65, 'Mass properties');
  let mp = massProps(solid);
  let volumeSource = 'voxel';
  if (mp.volume <= 0) { mp = massProps(wet); volumeSource = 'surface-voxel'; }
  // Sealed cavities (e.g. hollow shells exported without internal parts) count
  // as displaced volume only if the user says the hull is sealed.
  const sealedGrid = opts.sealedHull ? fillEnclosed(wet) : null;
  const sealed = sealedGrid ? massProps(sealedGrid) : null;
  onProgress(0.75, 'Projected areas');
  // about the solid centroid for display; the plant re-evaluates them about the CG from the columns
  const proj = projections(wet, mp.centroid);
  const projCols = projectionColumns(wet);
  onProgress(0.8, 'Integrating the displaced volume');
  // exact-along-ray volume: no voxel staircase on thin plates and small parts
  let ray = null;
  if (!sealed && volumeSource === 'voxel' && opts.rayVolume !== false) {
    const rv = rayVolume(positions, bb, cell, { sub: opts.raySub ?? 6 });
    if (rv.volume > 0 && rv.openFraction < 0.02 && Math.abs(rv.volume / mp.volume - 1) < 0.6) ray = rv;
  }
  onProgress(0.85, 'Buoyancy elements');
  // sealed hull: the elements must displace the filled cavity too, or the
  // partly- and fully-submerged buoyancy paths disagree
  let els = buoyancyElements(sealedGrid ?? (volumeSource === 'voxel' ? solid : wet), opts.buoyancyResolution ?? 14);
  if (ray) {
    // keep the voxel distribution, match the integrated volume and centre of buoyancy
    const sv0 = els.reduce((t, e) => t + e.v, 0);
    const c0 = [0, 1, 2].map((k) => els.reduce((t, e) => t + e.p[k] * e.v, 0) / sv0);
    const k = ray.volume / sv0, d = [0, 1, 2].map((q) => ray.centroid[q] - c0[q]);
    els = els.map((e) => ({ p: [e.p[0] + d[0], e.p[1] + d[1], e.p[2] + d[2]], v: e.v * k, h: e.h }));
  }
  const contacts = contactPoints(wet);
  onProgress(1, 'Done');
  return {
    bbox: bb,
    topology: topo,
    meshVolume: sv.volume,
    meshCentroid: sv.centroid,
    cell,
    gridDims: [solid.nx, solid.ny, solid.nz],
    openRayFraction: vs.openRayFraction,
    volume: sealed ? sealed.volume : ray ? ray.volume : mp.volume,
    centroid: sealed ? sealed.centroid : ray ? ray.centroid : mp.centroid,
    voxelVolume: mp.volume,
    inertiaUnit: mp.inertiaUnit,
    solidCentroid: mp.centroid,
    volumeSource: sealed ? 'sealed-hull' : ray ? 'ray' : volumeSource,
    projections: proj,
    projectionRef: mp.centroid,
    projCols,
    buoyEls: els,
    contacts,
    wet: { nx: wet.nx, ny: wet.ny, nz: wet.nz, cell: wet.cell, origin: wet.origin, data: wet.data },
  };
}
