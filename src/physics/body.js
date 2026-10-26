// 6-DOF underwater vehicle plant (Fossen 2011 formulation, body FRD frame,
// origin at the CG):
//
//   M_RB nu_dot + M_A nu_r_dot + C_RB(nu) nu + C_A(nu_r) nu_r + D(nu_r) nu_r + g(eta) = tau + tau_FK
//
// with full 6x6 added mass (from BEM or estimates) scaled by submergence and
// by the image effect of nearby walls / floor / surface, coupled linear +
// quadratic damping (from CFD or estimates), surface-aware buoyancy from
// volume elements, Froude-Krylov and added-mass inertial forces from the
// fluid acceleration (waves, turbulence, current seen in rotating axes),
// thruster forces with in-flow/ventilation effects and propeller reaction
// torque, a battery whose sag scales the thrust, and mass-scaled penalty
// contacts with the pool walls and floor.
// Pressure effects: buoyancy uses the in-situ water density (EOS-80) and the
// hull's own compressibility; a pressure housing pushed past its depth rating
// (or given a leak on purpose) floods through an orifice, compressing the air
// inside (Boyle) - that internal pressure and the flooded water (weight, mass
// and inertia) are what the leak detector and ArduSub's FS_LEAK / FS_PRESS
// failsafes see.

import { qToMat, qnormalize, clamp } from '../core/math.js';
import { ThrusterState } from './thruster.js';
import { Battery } from './battery.js';

export function inv6(M) {
  const n = 6;
  const A = M.map((r, i) => [...r, ...Array.from({ length: n }, (_, j) => (i === j ? 1 : 0))]);
  for (let c = 0; c < n; c++) {
    let p = c;
    for (let r = c + 1; r < n; r++) if (Math.abs(A[r][c]) > Math.abs(A[p][c])) p = r;
    if (Math.abs(A[p][c]) < 1e-12) throw new Error('Mass matrix is singular');
    [A[c], A[p]] = [A[p], A[c]];
    const d = A[c][c];
    for (let k = 0; k < 2 * n; k++) A[c][k] /= d;
    for (let r = 0; r < n; r++) {
      if (r === c) continue;
      const f = A[r][c];
      if (f === 0) continue;
      for (let k = 0; k < 2 * n; k++) A[r][k] -= f * A[c][k];
    }
  }
  return A.map((r) => r.slice(n));
}

// skew-symmetric S(r): S(r) v = r x v
const skew = (r) => [[0, -r[2], r[1]], [r[2], 0, -r[0]], [-r[1], r[0], 0]];

export class VehiclePlant {
  constructor(model, env) {
    this.env = env;
    this.t = 0;
    this.y = new Float64Array(13);
    this.k = [new Float64Array(13), new Float64Array(13), new Float64Array(13), new Float64Array(13)];
    this.tmp = new Float64Array(13);
    this.diag = {};
    this.flood = { active: false, volume: 0, pIn: env.pAtm ?? 101325, p0: env.pAtm ?? 101325, cause: '' };
    this.setModel(model);
    this.reset();
  }

  setModel(m) {
    this.m = m;
    // rigid-body mass matrix about the (dry) CG
    const MRB = Array.from({ length: 6 }, () => new Array(6).fill(0));
    for (let i = 0; i < 3; i++) MRB[i][i] = m.mass;
    for (let i = 0; i < 3; i++) for (let j = 0; j < 3; j++) MRB[3 + i][3 + j] = m.Ig[i * 3 + j];
    this.MRB = MRB;
    this.thrusters = m.thrusters.map((d, i) => {
      const prev = this.thrusters && this.thrusters[i];
      const t = new ThrusterState(d);
      if (prev) t.n = prev.n;
      return t;
    });
    // bounding radius for the fully-submerged shortcut
    let R = 0;
    for (const e of m.buoyEls) R = Math.max(R, Math.hypot(e.p[0], e.p[1], e.p[2]) + Math.hypot(e.h[0], e.h[1], e.h[2]));
    this.boundR = R;
    // radius of the sphere with the displaced volume (image-effect length scale)
    this.aEq = Math.cbrt((3 * Math.max(m.volume, 1e-9)) / (4 * Math.PI));
    this.extraForce = [0, 0, 0, 0, 0, 0]; // body-frame disturbance hook (tether, payload...)
    const prevBat = this.battery?.snapshot?.();
    this.battery = new Battery(m.battery);
    if (prevBat) this.battery.restore(prevBat);
    this.hotelLoadA = Number.isFinite(+m.hotelLoadA) ? Math.max(0, +m.hotelLoadA) : 2;
    this.massKey = null;
    this.ctKey = null;
    this.contactGains(1 / 800);
    this.updateMass(1, [1, 1, 1], 0);
  }

  // Effective generalised mass: M_RB + flooded water (point mass at the CB,
  // with its parallel-axis inertia) + M_A scaled by the submerged fraction sA
  // and, per translational axis, by the wall/floor/surface image factor f
  // (applied as S M_A S with S = diag(sqrt f), which keeps it symmetric).
  updateMass(sA, f, mf, force = false) {
    const k = this.massKey;
    if (!force && k && Math.abs(k.sA - sA) < 2e-3 && Math.abs(k.f[0] - f[0]) < 2e-3 && Math.abs(k.f[1] - f[1]) < 2e-3 && Math.abs(k.f[2] - f[2]) < 2e-3 && Math.abs(k.mf - mf) <= 1e-3 * this.m.mass) return false;
    const A = this.m.MA;
    const s = [Math.sqrt(f[0]), Math.sqrt(f[1]), Math.sqrt(f[2]), 1, 1, 1];
    const MAe = A.map((r, i) => r.map((v, j) => v * s[i] * s[j] * sA));
    const M = this.MRB.map((r, i) => r.map((v, j) => v + MAe[i][j]));
    if (mf > 0) {
      const S = skew(this.m.cb);
      for (let i = 0; i < 3; i++) {
        M[i][i] += mf;
        for (let j = 0; j < 3; j++) {
          M[i][3 + j] -= mf * S[i][j];
          M[3 + i][j] += mf * S[i][j];
          let ss = 0;
          for (let q = 0; q < 3; q++) ss += S[i][q] * S[q][j];
          M[3 + i][3 + j] -= mf * ss;
        }
      }
    }
    this.MAe = MAe;
    this.M = M;
    this.Minv = inv6(M);
    this.massKey = { sA, f: f.slice(), mf };
    return true;
  }

  // translational added-mass factors from the image effect of a nearby plane
  // (sphere of radius a at distance h: 1 + 3/8 (a/h)^3 normal, 1 + 3/16 (a/h)^3 parallel;
  // the free surface acts as a rigid lid at manoeuvring frequencies)
  proximityFactors(y) {
    const env = this.env, a = this.aEq, pf = env.floor;
    const f = [1, 1, 1];
    const add = (h, axis) => {
      const r3 = Math.pow(a / Math.max(h, a), 3);
      if (r3 < 1e-6) return;
      for (let i = 0; i < 3; i++) f[i] *= 1 + (i === axis ? 0.375 : 0.1875) * r3;
    };
    add(pf.depthAt(y[0]) - y[2], 2);
    add(y[2] + env.surfaceElevation(y[0], y[1]), 2);
    add(pf.L / 2 - Math.abs(y[0]), 0);
    add(pf.W / 2 - Math.abs(y[1]), 1);
    return f;
  }

  // Penalty-contact gains scaled to this vehicle and step: per point
  //   k <= m_pt wc^2 (wc = 0.5 / dt),  c <= 0.9 m_pt / dt,
  // with m_pt the smaller of the translational and rotational effective mass
  // per supporting point (N_ref points carry the vehicle), so a 1 kg and a
  // 30 kg vehicle both stay inside the RK4 stability region.
  contactGains(dt) {
    const ct = this.env.cfg.contact, m = this.m;
    if (this.ctKey && this.ctKey.dt === dt && this.ctKey.ct === ct) return this.ctGains;
    const A = m.MA;
    const nRef = Math.max(4, Math.ceil(m.contacts.length / 3));
    const mEff = m.mass + (A[0][0] + A[1][1] + A[2][2]) / 3;
    const Imin = Math.min(m.Ig[0] + A[3][3], m.Ig[4] + A[4][4], m.Ig[8] + A[5][5]);
    let r2 = 0;
    for (const c of m.contacts) r2 = Math.max(r2, c[0] * c[0] + c[1] * c[1] + c[2] * c[2]);
    const mPt = Math.min(mEff / nRef, r2 > 0 ? Imin / (nRef * r2) : Infinity);
    const wc = 0.5 / dt;
    const k = Math.min(Math.max(0, ct.stiffness), mPt * wc * wc);
    // explicit-integration bound on the per-point damping rate: c dt / m_pt <= 0.9
    const cMax = Math.max(1.4 * Math.sqrt(k * mPt), (0.9 * mPt) / dt);
    const c = Math.min(Math.max(0, ct.damping), cMax);
    this.ctGains = { k, c, cf: cMax, mPt, nRef };
    this.ctKey = { dt, ct };
    return this.ctGains;
  }

  reset(pose = {}) {
    const y = this.y;
    y.fill(0);
    const p = pose.position ?? [0, 0, 1.0];
    y[0] = p[0]; y[1] = p[1]; y[2] = p[2];
    const q = pose.quaternion ?? [1, 0, 0, 0];
    y[3] = q[0]; y[4] = q[1]; y[5] = q[2]; y[6] = q[3];
    if (pose.velocity) { y[7] = pose.velocity[0]; y[8] = pose.velocity[1]; y[9] = pose.velocity[2]; }
    if (pose.rates) { y[10] = pose.rates[0]; y[11] = pose.rates[1]; y[12] = pose.rates[2]; }
    for (const t of this.thrusters) { t.n = 0; t.dwell = 0; t.Tcmd = 0; t.Teff = 0; t.Q = 0; t.pwm = 1500; t.current = 0; }
    this.clearFlood();
    if (pose.keepBattery !== true) this.battery.reset();
    this.accelNed = [0, 0, 0];
    this.nuDot = [0, 0, 0, 0, 0, 0];
    this.diag = {};
    this.massKey = null;
    this.t = 0;
  }

  // ---- pressure housing: flooding ----
  clearFlood() {
    const p0 = this.env.pAtm ?? 101325; // sealed at the surface
    this.flood = { active: false, volume: 0, pIn: p0, p0, cause: '', startDepth: NaN };
  }
  startLeak(cause = 'manual') {
    if (this.flood.active) return;
    this.flood.active = true;
    this.flood.cause = cause;
    this.flood.startDepth = this.y[2];
  }
  // orifice inflow Q = Cd A sqrt(2 dp / rho) while outside pressure exceeds the
  // (Boyle-compressed) air pressure inside the housing
  updateFlood(dt) {
    const h = this.m.hull;
    const f = this.flood;
    if (!h) return;
    const env = this.env, y = this.y;
    const depth = y[2] + env.surfaceElevation(y[0], y[1]);
    if (!f.active && h.floodOverRating && h.rating > 0 && depth > h.rating) this.startLeak('rating');
    const Va = h.airVolume;
    if (!(Va > 0)) { f.pIn = f.p0; return; }
    if (f.active && f.volume < Va * 0.999) {
      const pOut = env.pAtm + env.pressureGauge(Math.max(0, depth));
      const pIn = (f.p0 * Va) / (Va - f.volume);
      const dp = pOut - pIn;
      if (dp > 0) f.volume = Math.min(Va * 0.999, f.volume + 0.62 * h.leakArea * Math.sqrt((2 * dp) / env.rho) * dt);
    }
    f.pIn = (f.p0 * Va) / (Va - f.volume);
  }

  get position() { return [this.y[0], this.y[1], this.y[2]]; }
  get quaternion() { return [this.y[3], this.y[4], this.y[5], this.y[6]]; }
  get velocityBody() { return [this.y[7], this.y[8], this.y[9]]; }
  get rates() { return [this.y[10], this.y[11], this.y[12]]; }
  get busVoltage() { return this.battery.voltage; }
  get totalCurrent() { return this.battery.current; }

  // Generalised forces and state derivative.
  deriv(y, out, record = false) {
    const m = this.m, env = this.env;
    const q = [y[3], y[4], y[5], y[6]];
    const R = qToMat(q);
    const u = y[7], v = y[8], w = y[9], p = y[10], qq = y[11], r = y[12];
    const pz = y[2];
    const eta = env.surfaceElevation(y[0], y[1]);

    // fluid velocity and acceleration in body axes
    const fl = env.fluidVelocity(y[0], y[1], pz);
    const ub = R[0] * fl[0] + R[3] * fl[1] + R[6] * fl[2];
    const vb = R[1] * fl[0] + R[4] * fl[1] + R[7] * fl[2];
    const wb = R[2] * fl[0] + R[5] * fl[1] + R[8] * fl[2];
    const ur = u - ub, vr = v - vb, wr = w - wb;
    const fa = env.fluidAccel ? env.fluidAccel(y[0], y[1], pz) : [0, 0, 0];

    const tau = [0, 0, 0, 0, 0, 0];
    const rho = env.rho, g = env.g;

    // --- gravity at CG (NED down) ---
    const W = m.mass * g;
    tau[0] += R[6] * W; tau[1] += R[7] * W; tau[2] += R[8] * W;

    // --- buoyancy: in-situ water density and hull compression at this depth ---
    let subFrac = 1;
    const fb = [0, 0, 0], mb = [0, 0, 0];
    const depthCG = pz + eta; // depth below the local free surface (eta positive up)
    const pg = env.pressureGauge(Math.max(0, depthCG));
    const hull = m.hull;
    const bScale = (env.densityAtGauge(pg) / rho) * Math.max(0.5, 1 - (hull ? hull.beta : 0) * pg);
    const full = depthCG > this.boundR + 1e-3;
    if (full) {
      const B = rho * g * m.volume * bScale;
      const fx = -R[6] * B, fy = -R[7] * B, fz = -R[8] * B;
      fb[0] = fx; fb[1] = fy; fb[2] = fz;
      const c = m.cb;
      mb[0] = c[1] * fz - c[2] * fy; mb[1] = c[2] * fx - c[0] * fz; mb[2] = c[0] * fy - c[1] * fx;
    } else {
      // Each element's force is vertical, so the submerged part's centroid
      // (shifted vertically from the element centre) gives the same moment.
      let vsub = 0;
      for (const e of m.buoyEls) {
        const ez = pz + R[6] * e.p[0] + R[7] * e.p[1] + R[8] * e.p[2];
        const ex = y[0] + R[0] * e.p[0] + R[1] * e.p[1] + R[2] * e.p[2];
        const ey = y[1] + R[3] * e.p[0] + R[4] * e.p[1] + R[5] * e.p[2];
        const d = ez + env.surfaceElevation(ex, ey);
        const hz = Math.abs(R[6]) * e.h[0] + Math.abs(R[7]) * e.h[1] + Math.abs(R[8]) * e.h[2];
        const frac = clamp((d + hz) / (2 * hz + 1e-9), 0, 1);
        if (frac <= 0) continue;
        const dv = e.v * frac;
        vsub += dv;
        const B = rho * g * dv * bScale;
        const fx = -R[6] * B, fy = -R[7] * B, fz = -R[8] * B;
        fb[0] += fx; fb[1] += fy; fb[2] += fz;
        mb[0] += e.p[1] * fz - e.p[2] * fy; mb[1] += e.p[2] * fx - e.p[0] * fz; mb[2] += e.p[0] * fy - e.p[1] * fx;
      }
      subFrac = m.volume > 0 ? vsub / m.volume : 0;
    }
    // water that flooded the housing is extra weight (taken at the centre of buoyancy)
    const mf = rho * this.flood.volume;
    if (mf > 0) {
      const Wf = mf * g, c = m.cb;
      const fx = R[6] * Wf, fy = R[7] * Wf, fz = R[8] * Wf;
      fb[0] += fx; fb[1] += fy; fb[2] += fz;
      mb[0] += c[1] * fz - c[2] * fy; mb[1] += c[2] * fx - c[0] * fz; mb[2] += c[0] * fy - c[1] * fx;
    }
    for (let i = 0; i < 3; i++) { tau[i] += fb[i]; tau[3 + i] += mb[i]; }

    // --- fluid inertia: Froude-Krylov rho V a_f (at the CB) + M_A nu_c_dot ---
    // nu_c_dot (body) = R^T a_f - omega x v_c: Fossen's M_A nu_r_dot with the
    // current term moved to the right-hand side.
    const afb = [R[0] * fa[0] + R[3] * fa[1] + R[6] * fa[2], R[1] * fa[0] + R[4] * fa[1] + R[7] * fa[2], R[2] * fa[0] + R[5] * fa[1] + R[8] * fa[2]];
    const ncd = [afb[0] - (qq * wb - r * vb), afb[1] - (r * ub - p * wb), afb[2] - (p * vb - qq * ub)];
    const fk = [0, 0, 0, 0, 0, 0];
    {
      let F;
      if (full) { const k = rho * m.volume * bScale; F = [k * afb[0], k * afb[1], k * afb[2]]; }
      else {
        // near the surface the vertical part is already in the element buoyancy (local eta)
        const k = rho * m.volume * subFrac * bScale;
        F = [k * (R[0] * fa[0] + R[3] * fa[1]), k * (R[1] * fa[0] + R[4] * fa[1]), k * (R[2] * fa[0] + R[5] * fa[1])];
      }
      const c = m.cb, MA = this.MAe;
      for (let i = 0; i < 6; i++) fk[i] = MA[i][0] * ncd[0] + MA[i][1] * ncd[1] + MA[i][2] * ncd[2];
      fk[0] += F[0]; fk[1] += F[1]; fk[2] += F[2];
      fk[3] += c[1] * F[2] - c[2] * F[1]; fk[4] += c[2] * F[0] - c[0] * F[2]; fk[5] += c[0] * F[1] - c[1] * F[0];
      for (let i = 0; i < 6; i++) tau[i] += fk[i];
    }

    // --- damping (scaled by submerged fraction, + surface wave-making drag) ---
    const nur = [ur, vr, wr, p, qq, r];
    const sd = Math.max(subFrac, 0.02);
    const surfBoost = 1 + (env.cfg.surfaceDrag ?? 0) * 4 * subFrac * (1 - subFrac);
    const DL = m.DL, DQ = m.DQ;
    const absnu = nur.map((x) => Math.abs(x) * x);
    const fd = [0, 0, 0, 0, 0, 0];
    for (let i = 0; i < 6; i++) {
      let s = 0;
      const dl = DL[i], dq = DQ[i];
      for (let j = 0; j < 6; j++) s += dl[j] * nur[j] + dq[j] * absnu[j];
      fd[i] = -s * sd * (i < 3 ? surfBoost : 1);
      tau[i] += fd[i];
    }

    // --- Coriolis: rigid body (origin at CG) ---
    const Ig = m.Ig;
    const Iw = [Ig[0] * p + Ig[1] * qq + Ig[2] * r, Ig[3] * p + Ig[4] * qq + Ig[5] * r, Ig[6] * p + Ig[7] * qq + Ig[8] * r];
    const ms = m.mass;
    // m * (w x v)
    tau[0] -= ms * (qq * w - r * v);
    tau[1] -= ms * (r * u - p * w);
    tau[2] -= ms * (p * v - qq * u);
    // w x (I w)
    tau[3] -= qq * Iw[2] - r * Iw[1];
    tau[4] -= r * Iw[0] - p * Iw[2];
    tau[5] -= p * Iw[1] - qq * Iw[0];
    // flooded water as a point mass mf at the CB: mf (w x v + w x (w x r)) and its moment
    if (mf > 0) {
      const c = m.cb;
      const wr0 = qq * c[2] - r * c[1], wr1 = r * c[0] - p * c[2], wr2 = p * c[1] - qq * c[0];
      const a0 = mf * ((qq * w - r * v) + (qq * wr2 - r * wr1));
      const a1 = mf * ((r * u - p * w) + (r * wr0 - p * wr2));
      const a2 = mf * ((p * v - qq * u) + (p * wr1 - qq * wr0));
      tau[0] -= a0; tau[1] -= a1; tau[2] -= a2;
      tau[3] -= c[1] * a2 - c[2] * a1; tau[4] -= c[2] * a0 - c[0] * a2; tau[5] -= c[0] * a1 - c[1] * a0;
    }

    // --- Coriolis: added mass (relative velocity), full 6x6 ---
    const A = this.MAe;
    const pt = [0, 0, 0], pr = [0, 0, 0];
    for (let i = 0; i < 3; i++) {
      pt[i] = A[i][0] * ur + A[i][1] * vr + A[i][2] * wr + A[i][3] * p + A[i][4] * qq + A[i][5] * r;
      pr[i] = A[3 + i][0] * ur + A[3 + i][1] * vr + A[3 + i][2] * wr + A[3 + i][3] * p + A[3 + i][4] * qq + A[3 + i][5] * r;
    }
    // [w x pt ; vr x pt + w x pr]
    tau[0] -= qq * pt[2] - r * pt[1];
    tau[1] -= r * pt[0] - p * pt[2];
    tau[2] -= p * pt[1] - qq * pt[0];
    tau[3] -= (vr * pt[2] - wr * pt[1]) + (qq * pr[2] - r * pr[1]);
    tau[4] -= (wr * pt[0] - ur * pt[2]) + (r * pr[0] - p * pr[2]);
    tau[5] -= (ur * pt[1] - vr * pt[0]) + (p * pr[1] - qq * pr[0]);

    // --- thrusters (force along the axis + prop reaction torque about it) ---
    const ft = [0, 0, 0, 0, 0, 0];
    for (let i = 0; i < this.thrusters.length; i++) {
      const th = this.thrusters[i];
      const d = th.def;
      if (th.n === 0) { if (record) { th.Teff = 0; th.Q = 0; } continue; }
      const a = d.axis, rp = d.pos;
      // local relative water velocity at the thruster
      const lx = ur + qq * rp[2] - r * rp[1];
      const ly = vr + r * rp[0] - p * rp[2];
      const lz = wr + p * rp[1] - qq * rp[0];
      const sgn = th.n > 0 ? 1 : -1;
      const inflow = sgn * (lx * a[0] + ly * a[1] + lz * a[2]);
      const tzw = pz + R[6] * rp[0] + R[7] * rp[1] + R[8] * rp[2];
      const txw = y[0] + R[0] * rp[0] + R[1] * rp[1] + R[2] * rp[2];
      const tyw = y[1] + R[3] * rp[0] + R[4] * rp[1] + R[5] * rp[2];
      const Te = th.effective(inflow, tzw + env.surfaceElevation(txw, tyw));
      const Q = th.reactionTorque();
      if (record) { th.Teff = Te; th.Q = Q; }
      const fx = Te * a[0], fy = Te * a[1], fz = Te * a[2];
      ft[0] += fx; ft[1] += fy; ft[2] += fz;
      ft[3] += rp[1] * fz - rp[2] * fy + Q * a[0]; ft[4] += rp[2] * fx - rp[0] * fz + Q * a[1]; ft[5] += rp[0] * fy - rp[1] * fx + Q * a[2];
    }
    for (let i = 0; i < 6; i++) tau[i] += ft[i];

    // --- contacts with pool floor and walls (each surface independently) ---
    const fc = [0, 0, 0, 0, 0, 0];
    const ct = env.cfg.contact, pf = env.floor, cg = this.ctGains;
    const hx = pf.L / 2, hy = pf.W / 2;
    for (const c of m.contacts) {
      const wx = y[0] + R[0] * c[0] + R[1] * c[1] + R[2] * c[2];
      const wy = y[1] + R[3] * c[0] + R[4] * c[1] + R[5] * c[2];
      const wz = pz + R[6] * c[0] + R[7] * c[1] + R[8] * c[2];
      const floorZ = pf.depthAt(wx);
      if (!(wz > floorZ || wx > hx || wx < -hx || wy > hy || wy < -hy)) continue;
      // contact point velocity (NED)
      const bvx = u + qq * c[2] - r * c[1], bvy = v + r * c[0] - p * c[2], bvz = w + p * c[1] - qq * c[0];
      const Vx = R[0] * bvx + R[1] * bvy + R[2] * bvz, Vy = R[3] * bvx + R[4] * bvy + R[5] * bvz, Vz = R[6] * bvx + R[7] * bvy + R[8] * bvz;
      let Fx = 0, Fy = 0, Fz = 0;
      const surface = (pen, nX, nY, nZ) => {
        if (pen <= 0) return;
        const vn = Vx * nX + Vy * nY + Vz * nZ;
        const Fn = cg.k * pen - cg.c * vn;
        if (Fn <= 0) return;
        Fx += Fn * nX; Fy += Fn * nY; Fz += Fn * nZ;
        const tx = Vx - vn * nX, ty = Vy - vn * nY, tz = Vz - vn * nZ;
        const tl = Math.hypot(tx, ty, tz);
        if (tl < 1e-9) return;
        // regularised Coulomb friction; the viscous slope is bounded like the damping
        const fmax = ct.friction * Fn;
        const vreg = Math.max(0.02, fmax / Math.max(cg.cf, 1e-9));
        const mu = fmax * Math.min(1, tl / vreg);
        Fx -= (mu * tx) / tl; Fy -= (mu * ty) / tl; Fz -= (mu * tz) / tl;
      };
      if (wz > floorZ) {
        // floor (possibly sloped): push out along the local floor normal
        const k = pf.slopeAt(wx), nn = Math.sqrt(1 + k * k);
        surface((wz - floorZ) / nn, k / nn, 0, -1 / nn);
      }
      if (wx > hx) surface(wx - hx, -1, 0, 0);
      if (wx < -hx) surface(-hx - wx, 1, 0, 0);
      if (wy > hy) surface(wy - hy, 0, -1, 0);
      if (wy < -hy) surface(-hy - wy, 0, 1, 0);
      if (!Fx && !Fy && !Fz) continue;
      // to body
      const bx = R[0] * Fx + R[3] * Fy + R[6] * Fz, by = R[1] * Fx + R[4] * Fy + R[7] * Fz, bz = R[2] * Fx + R[5] * Fy + R[8] * Fz;
      fc[0] += bx; fc[1] += by; fc[2] += bz;
      fc[3] += c[1] * bz - c[2] * by; fc[4] += c[2] * bx - c[0] * bz; fc[5] += c[0] * by - c[1] * bx;
    }
    for (let i = 0; i < 6; i++) tau[i] += fc[i] + this.extraForce[i];

    // --- solve M nu_dot = tau ---
    const Mi = this.Minv;
    for (let i = 0; i < 6; i++) {
      let s = 0;
      for (let j = 0; j < 6; j++) s += Mi[i][j] * tau[j];
      out[7 + i] = s;
    }
    // kinematics
    out[0] = R[0] * u + R[1] * v + R[2] * w;
    out[1] = R[3] * u + R[4] * v + R[5] * w;
    out[2] = R[6] * u + R[7] * v + R[8] * w;
    out[3] = 0.5 * (-q[1] * p - q[2] * qq - q[3] * r);
    out[4] = 0.5 * (q[0] * p + q[2] * r - q[3] * qq);
    out[5] = 0.5 * (q[0] * qq + q[3] * p - q[1] * r);
    out[6] = 0.5 * (q[0] * r + q[1] * qq - q[2] * p);

    if (record) {
      this.diag = { thrust: ft, buoyancy: [...fb, ...mb], damping: fd, contact: fc, fluidInertia: fk, submerged: subFrac, fluidBody: [ub, vb, wb], fluidAccel: fa, weight: W, buoyancyMag: Math.hypot(fb[0], fb[1], fb[2]), pressureGauge: pg, buoyancyScale: bScale, floodMass: mf, addedMassScale: this.massKey?.sA ?? 1, busVoltage: this.battery.voltage, current: this.battery.current };
      // inertial acceleration of the CG (NED) and specific force (body)
      const vd = [out[7] + qq * w - r * v, out[8] + r * u - p * w, out[9] + p * v - qq * u];
      this.accelNed = [R[0] * vd[0] + R[1] * vd[1] + R[2] * vd[2], R[3] * vd[0] + R[4] * vd[1] + R[5] * vd[2], R[6] * vd[0] + R[7] * vd[1] + R[8] * vd[2]];
      this.nuDot = [out[7], out[8], out[9], out[10], out[11], out[12]];
    }
    return subFrac;
  }

  // Advance by dt with ESC PWM per thruster (held constant over the step).
  // Thruster rotors, battery and flooding are advanced once per step (zero-order
  // hold); the rigid-body state with classical RK4 and a renormalised quaternion.
  step(dt, pwm) {
    const vbus = this.battery.voltage;
    let amps = this.hotelLoadA;
    for (let i = 0; i < this.thrusters.length; i++) {
      const th = this.thrusters[i];
      th.update(pwm ? pwm[i] ?? 1500 : 1500, dt, vbus);
      amps += th.current;
    }
    this.battery.step(dt, amps);
    this.updateFlood(dt);
    this.contactGains(dt);
    const y = this.y, k1 = this.k[0];
    const sub = this.deriv(y, k1, true);
    if (this.updateMass(sub, this.proximityFactors(y), this.env.rho * this.flood.volume)) this.deriv(y, k1, true);
    // Damping stiffness: the local decay rate (D_L + 2 D_Q |nu|) / M_ii must stay
    // inside RK4's real-axis stability limit (2.78); split the step when it does not.
    const DL = this.m.DL, DQ = this.m.DQ, M = this.M, sd = Math.max(sub, 0.02);
    let lam = 0;
    for (let i = 0; i < 6; i++) lam = Math.max(lam, (sd * (Math.abs(DL[i][i]) + 2 * Math.abs(DQ[i][i] * y[7 + i]))) / M[i][i]);
    const n = Math.min(64, Math.max(1, Math.ceil((lam * dt) / 2)));
    this.rkSubsteps = n;
    const h = dt / n;
    for (let s = 0; s < n; s++) this.rk4(h, s > 0);
    for (let i = 0; i < 13; i++) if (!Number.isFinite(y[i])) throw new Error('Physics diverged (non-finite state). Check mass/inertia/damping values.');
    this.t += dt;
  }
  // one classical RK4 step of the rigid-body state (k1 already evaluated unless fresh)
  rk4(dt, fresh = false) {
    const y = this.y, [k1, k2, k3, k4] = this.k, t = this.tmp;
    if (fresh) this.deriv(y, k1);
    for (let i = 0; i < 13; i++) t[i] = y[i] + 0.5 * dt * k1[i];
    this.deriv(t, k2);
    for (let i = 0; i < 13; i++) t[i] = y[i] + 0.5 * dt * k2[i];
    this.deriv(t, k3);
    for (let i = 0; i < 13; i++) t[i] = y[i] + dt * k3[i];
    this.deriv(t, k4);
    for (let i = 0; i < 13; i++) y[i] += (dt / 6) * (k1[i] + 2 * k2[i] + 2 * k3[i] + k4[i]);
    const qn = qnormalize([y[3], y[4], y[5], y[6]]);
    y[3] = qn[0]; y[4] = qn[1]; y[5] = qn[2]; y[6] = qn[3];
  }

  // Mechanical energy (J): kinetic 1/2 nu^T M nu (with the current added mass)
  // plus gravity and buoyancy potential when fully submerged. For tests and diagnostics.
  energy() {
    const y = this.y, nu = [y[7], y[8], y[9], y[10], y[11], y[12]], M = this.M;
    let K = 0;
    for (let i = 0; i < 6; i++) for (let j = 0; j < 6; j++) K += 0.5 * nu[i] * M[i][j] * nu[j];
    const R = qToMat(this.quaternion), m = this.m, env = this.env, c = m.cb;
    const zcb = y[2] + R[6] * c[0] + R[7] * c[1] + R[8] * c[2];
    const B = env.rho * env.g * m.volume;
    return K - m.mass * env.g * y[2] + B * zcb;
  }

  specificForceBody() {
    // accelerometer: R^T (a - g)
    const R = qToMat(this.quaternion);
    const g = this.env.g;
    const a = [this.accelNed[0], this.accelNed[1], this.accelNed[2] - g];
    return [R[0] * a[0] + R[3] * a[1] + R[6] * a[2], R[1] * a[0] + R[4] * a[1] + R[7] * a[2], R[2] * a[0] + R[5] * a[1] + R[8] * a[2]];
  }
}
