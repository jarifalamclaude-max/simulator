// D3Q19 lattice (lattice units: dx = dt = 1, cs^2 = 1/3).
// Directions are ordered so that opposite pairs are (1,2), (3,4), ... (17,18).

export const Q = 19;
export const C = [
  [0, 0, 0],
  [1, 0, 0], [-1, 0, 0], [0, 1, 0], [0, -1, 0], [0, 0, 1], [0, 0, -1],
  [1, 1, 0], [-1, -1, 0], [1, -1, 0], [-1, 1, 0],
  [1, 0, 1], [-1, 0, -1], [1, 0, -1], [-1, 0, 1],
  [0, 1, 1], [0, -1, -1], [0, 1, -1], [0, -1, 1],
];
export const W = C.map((c) => {
  const n = Math.abs(c[0]) + Math.abs(c[1]) + Math.abs(c[2]);
  return n === 0 ? 1 / 3 : n === 1 ? 1 / 18 : 1 / 36;
});
export const OPP = C.map((_, i) => (i === 0 ? 0 : i % 2 === 1 ? i + 1 : i - 1));

// Equilibrium of the shifted distribution g = f - w (density rho, velocity u).
export function geq(i, rho, ux, uy, uz) {
  const c = C[i];
  const cu = c[0] * ux + c[1] * uy + c[2] * uz;
  const uu = ux * ux + uy * uy + uz * uz;
  return W[i] * ((rho - 1) + rho * (3 * cu + 4.5 * cu * cu - 1.5 * uu));
}

// Smagorinsky-augmented relaxation time from the non-equilibrium momentum flux
// magnitude (Hou et al. 1996): tau = (tau0 + sqrt(tau0^2 + 18 sqrt2 Cs^2 |Pi| / rho)) / 2
export const SMAG_K = 18 * Math.SQRT2;
