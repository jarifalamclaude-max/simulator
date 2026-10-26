// Battery pack: open-circuit voltage vs state of charge, ohmic resistance and
// one RC polarisation branch (Thevenin model), coulomb counting. Temperature
// is not modelled. Default: Blue Robotics 14.8 V 18 Ah Li-ion (4S, ~30 mOhm).
//
//   V_term = n_cells * OCV(SoC) - I * R0(SoC) - V1,   dV1/dt = I / C1 - V1 / (R1 C1)
//
// Current is positive when discharging. The polarisation branch gives the
// slow extra sag under sustained load and the recovery once the load drops.

// per-cell OCV [V] at SoC = 0, 0.05, 0.1, 0.2 ... 1.0
const OCV_SOC = [0, 0.05, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0];
export const OCV_TABLES = {
  // NMC/NCA 18650 / 21700 (Li-ion), 0.05C rest curve
  liion: [3.0, 3.3, 3.45, 3.55, 3.62, 3.68, 3.74, 3.81, 3.89, 3.98, 4.07, 4.2],
  // LiCoO2 pouch (LiPo)
  lipo: [3.27, 3.61, 3.69, 3.73, 3.77, 3.8, 3.84, 3.87, 3.95, 4.02, 4.11, 4.2],
};

export const DEFAULT_BATTERY = { cells: 4, capacityMah: 18000, internalR_ohm: 0.03, chemistry: 'liion', soc0: 1, polarizationR_ohm: null, polarizationTau_s: 30 };

export function cellOcv(soc, chemistry = 'liion') {
  const T = OCV_TABLES[chemistry] ?? OCV_TABLES.liion;
  const s = Math.min(1, Math.max(0, soc));
  let i = 0;
  while (i < OCV_SOC.length - 2 && s > OCV_SOC[i + 1]) i++;
  const f = (s - OCV_SOC[i]) / (OCV_SOC[i + 1] - OCV_SOC[i]);
  return T[i] + f * (T[i + 1] - T[i]);
}

export class Battery {
  constructor(spec = {}) {
    const s = { ...DEFAULT_BATTERY, ...(spec || {}) };
    this.spec = s;
    this.cells = Math.max(1, Math.round(+s.cells || 4));
    this.capacityMah = Math.max(1, +s.capacityMah || 18000);
    this.R0 = Math.max(0, Number.isFinite(+s.internalR_ohm) ? +s.internalR_ohm : 0.03);
    this.R1 = Number.isFinite(+s.polarizationR_ohm) && s.polarizationR_ohm !== null ? Math.max(0, +s.polarizationR_ohm) : 0.5 * this.R0;
    this.tau1 = Math.max(0.1, +s.polarizationTau_s || 30);
    this.chemistry = OCV_TABLES[s.chemistry] ? s.chemistry : 'liion';
    this.reset();
  }
  reset() {
    const soc0 = Math.min(1, Math.max(0, Number.isFinite(+this.spec.soc0) ? +this.spec.soc0 : 1));
    this.consumedAs = (1 - soc0) * this.capacityMah * 3.6; // ampere-seconds drawn so far
    this.V1 = 0;
    this.I = 0;
    this.energyWh = 0;
    this._update();
  }
  // carry the charge state across a rebuild (e.g. live parameter edits)
  snapshot() { return { consumedAs: this.consumedAs, V1: this.V1, I: this.I, energyWh: this.energyWh }; }
  restore(st) {
    if (!st) return;
    this.consumedAs = Math.min(this.capacityMah * 3.6, Math.max(0, +st.consumedAs || 0));
    this.V1 = +st.V1 || 0; this.I = +st.I || 0; this.energyWh = +st.energyWh || 0;
    this._update();
  }
  get soc() { return Math.max(0, 1 - this.consumedAs / (this.capacityMah * 3.6)); }
  // resistance grows as the pack empties (typical Li-ion DCIR curve, ~2x at 5 % SoC)
  get resistance() { const s = this.soc; return this.R0 * (1 + 0.6 * Math.max(0, 0.3 - s) / 0.3 + 2.5 * Math.max(0, 0.05 - s) / 0.05); }
  get restingVoltage() { return this.cells * cellOcv(this.soc, this.chemistry); }
  get voltage() { return this.Vt; }
  get current() { return this.I; }
  get consumedMah() { return this.consumedAs / 3.6; }
  get remainingPct() { return 100 * this.soc; }
  _update() {
    // a flat pack collapses: the terminal voltage never goes below zero
    this.Vt = Math.max(0, this.restingVoltage - this.I * this.resistance - this.V1);
  }
  // advance by dt [s] with a discharge current [A] (negative = charging)
  step(dt, currentA) {
    const I = Number.isFinite(currentA) ? currentA : 0;
    this.I = I;
    if (dt > 0) {
      this.consumedAs = Math.max(0, this.consumedAs + I * dt);
      // exact discretisation of the RC branch for a held current
      const a = Math.exp(-dt / this.tau1);
      this.V1 = this.V1 * a + I * this.R1 * (1 - a);
    }
    this._update();
    if (dt > 0) this.energyWh += (this.Vt * I * dt) / 3600;
    return this.Vt;
  }
}
