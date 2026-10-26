// Fixed-capacity ring-buffer telemetry with named channels and CSV export.

export const CHANNELS = [
  ['t', 's'], ['armed', ''], ['mode', ''],
  ['roll', 'deg'], ['pitch', 'deg'], ['yaw', 'deg'],
  ['roll_t', 'deg'], ['pitch_t', 'deg'], ['yaw_t', 'deg'],
  ['ahrs_roll', 'deg'], ['ahrs_pitch', 'deg'], ['ahrs_yaw', 'deg'],
  ['p', 'deg/s'], ['q', 'deg/s'], ['r', 'deg/s'],
  ['p_t', 'deg/s'], ['q_t', 'deg/s'], ['r_t', 'deg/s'],
  ['depth', 'm'], ['depth_meas', 'm'], ['depth_t', 'm'], ['climb', 'm/s'],
  ['x', 'm'], ['y', 'm'], ['z', 'm'],
  ['u', 'm/s'], ['v', 'm/s'], ['w', 'm/s'],
  ['in_roll', ''], ['in_pitch', ''], ['in_yaw', ''], ['in_thr', ''], ['in_fwd', ''], ['in_lat', ''],
  ['rll_P', ''], ['rll_I', ''], ['rll_D', ''], ['pit_P', ''], ['pit_I', ''], ['pit_D', ''], ['yaw_P', ''], ['yaw_I', ''], ['yaw_D', ''],
  ['saturated', ''], ['submerged', ''], ['gain', ''],
  ['thrust_total', 'N'],
  ['press_abs', 'mbar'], ['press_int', 'mbar'], ['flood', 'mL'], ['floor_dist', 'm'],
  // state estimate, battery monitor, rangefinder, temperatures, failsafe flags (contract §6)
  ['depth_est', 'm'], ['climb_est', 'm/s'],
  ['batt_v', 'V'], ['batt_a', 'A'], ['batt_mah', 'mAh'], ['batt_pct', '%'],
  ['rf_dist', 'm'], ['rf_ok', ''], ['temp_int', '°C'], ['temp_water', '°C'], ['compass_hdg', 'deg'], ['turns', 'rev'],
  ['fs_pilot', ''], ['fs_gcs', ''], ['fs_ekf', ''], ['fs_leak', ''], ['fs_press', ''], ['fs_temp', ''], ['fs_crash', ''], ['fs_terrain', ''], ['fs_sensor', ''], ['fs_batt', ''], ['gcs_link', ''],
];
export const MAX_MOTORS = 12;
for (let i = 0; i < MAX_MOTORS; i++) CHANNELS.push([`m${i + 1}_out`, ''], [`m${i + 1}_pwm`, 'us'], [`m${i + 1}_N`, 'N']);

export class Telemetry {
  constructor(capacity = 36000) {
    this.capacity = capacity;
    this.names = CHANNELS.map((c) => c[0]);
    this.units = Object.fromEntries(CHANNELS);
    this.index = Object.fromEntries(this.names.map((n, i) => [n, i]));
    this.data = this.names.map(() => new Float32Array(capacity));
    this.head = 0;
    this.count = 0;
    this.version = 0;
  }
  clear() { this.head = 0; this.count = 0; this.version++; }
  push(rec) {
    const h = this.head;
    for (let i = 0; i < this.names.length; i++) {
      const v = rec[this.names[i]];
      this.data[i][h] = v === undefined ? NaN : v;
    }
    this.head = (h + 1) % this.capacity;
    if (this.count < this.capacity) this.count++;
    this.version++;
  }
  // iterate the last `n` samples (oldest first)
  series(name, n = this.count) {
    const ch = this.data[this.index[name]];
    if (!ch) return new Float32Array(0);
    n = Math.min(n, this.count);
    const out = new Float32Array(n);
    let k = (this.head - n + this.capacity) % this.capacity;
    for (let i = 0; i < n; i++) { out[i] = ch[k]; k = (k + 1) % this.capacity; }
    return out;
  }
  latest(name) {
    if (!this.count) return NaN;
    return this.data[this.index[name]][(this.head - 1 + this.capacity) % this.capacity];
  }
  toCSV(channels = this.names) {
    const cols = channels.filter((c) => c in this.index);
    const lines = [cols.map((c) => (this.units[c] ? `${c} [${this.units[c]}]` : c)).join(',')];
    const series = cols.map((c) => this.series(c));
    for (let i = 0; i < this.count; i++) lines.push(series.map((s) => (Number.isFinite(s[i]) ? +s[i].toPrecision(6) : '')).join(','));
    return lines.join('\n');
  }
}
