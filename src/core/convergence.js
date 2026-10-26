// Grid / panel convergence: Richardson extrapolation with Roache's grid
// convergence index, used by the CFD lab (lattice resolution) and the panel
// method (panel count).

/**
 * Grid convergence (Roache's GCI as set out by Celik et al. 2008, J. Fluids
 * Eng. 130). points: [{ h, v }] from the finest grid to the coarsest, h = 1/N.
 * Returns the Richardson-extrapolated value, the fine-grid convergence index
 * (relative uncertainty, 95% confidence), the observed order p and whether the
 * grids converge monotonically.
 * opts.pFormal: formal order of the scheme (default 2: Bouzidi-wall LBM, the
 * panel method; about 1 for a plain voxel staircase). The extrapolation and the
 * GCI use min(p_observed, pFormal): an observed order above the formal one is
 * not trusted (Roache 1998; Celik 2008 sec. 2.1).
 * Safety factor Fs = 1.25 with three grids and an observed order, Fs = 3 for a
 * two-grid study with an assumed order (Roache 1998).
 */
export function gridConvergence(points, { pFormal = 2 } = {}) {
  const P = points.slice().sort((a, b) => a.h - b.h);
  const f1 = P[0];
  if (P.length < 2) return { value: f1.v, gci: NaN, p: NaN, status: 'single grid', fine: f1.v };
  if (P.length === 2) {
    const r = P[1].h / f1.h, p = pFormal;
    const ea = Math.abs((f1.v - P[1].v) / f1.v);
    return { value: (r ** p * f1.v - P[1].v) / (r ** p - 1), gci: (3 * ea) / (r ** p - 1), p, status: 'two grids', fine: f1.v };
  }
  const [a, b, c] = P;
  const r21 = b.h / a.h, r32 = c.h / b.h;
  const e21 = b.v - a.v, e32 = c.v - b.v;
  if (Math.abs(e21) <= 1e-12 * Math.max(1e-30, Math.abs(a.v))) return { value: a.v, gci: 0, p: Infinity, status: 'converged', fine: a.v };
  const s = Math.sign(e32 / e21) || 1;
  let p = 2;
  for (let it = 0; it < 60; it++) {
    const qp = Math.log((r21 ** p - s) / (r32 ** p - s));
    const pn = Math.abs(Math.log(Math.abs(e32 / e21)) + qp) / Math.log(r21);
    if (!Number.isFinite(pn)) break;
    const done = Math.abs(pn - p) < 1e-7;
    p = pn;
    if (done) break;
  }
  const ea = Math.abs((a.v - b.v) / a.v);
  if (s < 0) {
    // oscillatory: no extrapolation, uncertainty = spread of the grids
    const spread = Math.max(Math.abs(b.v - a.v), Math.abs(c.v - a.v)) / Math.abs(a.v);
    return { value: a.v, gci: spread, p, status: 'oscillatory', fine: a.v };
  }
  const pu = Math.min(Math.max(p, 1), pFormal);
  const value = (r21 ** pu * a.v - b.v) / (r21 ** pu - 1);
  const gci = (1.25 * ea) / (r21 ** pu - 1);
  return { value, gci, p, pUsed: pu, status: p < 1 || p > 3 ? 'outside asymptotic range' : 'monotonic', fine: a.v };
}
