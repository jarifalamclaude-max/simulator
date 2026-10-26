// ArduSub 4.5.7 parameter table: every parameter the firmware exposes, with
// ArduPilot's own metadata, MAVLink storage type and default (generated into
// paramdefs.js by tools/gen-params.mjs from the ArduSub-4.5.7 sources). Each
// entry also says whether the simulator uses it (effect 'sim') or only stores
// and exports it ('inert'). Mission Planner / QGroundControl .param import and
// export with types, legacy-name conversion and range checks live here too.

import { ROWS, ENUMS } from './paramdefs.js';

export const MAV_PARAM_TYPE = { INT8: 2, INT16: 4, INT32: 6, REAL32: 9 };
const TYPE_NAME = { 2: 'INT8', 4: 'INT16', 6: 'INT32', 9: 'REAL32' };
const INT_RANGE = { 2: [-128, 127], 4: [-32768, 32767], 6: [-2147483648, 2147483647] };

// Parameters the simulator reads (the rest are stored, shown and exported unchanged).
const SIM = [
  /^FRAME_CONFIG$/, /^MOT_/, /^ATC_/, /^PSC_/, /^WPNAV_/, /^CIRCLE_/, /^PILOT_/, /^ACRO_/, /^THR_DZ$/, /^ANGLE_MAX$/,
  /^SURFACE_DEPTH$/, /^SURFTRAK_DEPTH$/, /^JS_/, /^BTN\d+_S?FUNCTION$/, /^RC\d+_(MIN|MAX|TRIM|DZ|REVERSED)$/,
  /^RC_(OPTIONS|OVERRIDE_TIME|PROTOCOLS|SPEED|FS_TIMEOUT)$/, /^FS_/, /^SCHED_LOOP_RATE$/, /^SYSID_(THISMAV|MYGCS)$/,
  /^BATT_(MONITOR|CAPACITY|LOW_VOLT|CRT_VOLT|LOW_MAH|CRT_MAH|FS_LOW_ACT|FS_CRT_ACT|LOW_TIMER|FS_VOLTSRC|ARM_VOLT|ARM_MAH)$/,
  /^BARO_(SPEC_GRAV|ALT_OFFSET|EXT_BUS|FLTR_RNG|PROBE_EXT)$/, /^BARO\d_GND_PRESS$/, /^INS_(GYRO_FILTER|ACCEL_FILTER|FAST_SAMPLE|GYRO_RATE)$/, /^INS_HNTC[H2]_(ENABLE|FREQ|BW|ATT|HMNCS|REF|MODE|FM_RAT)$/,
  /^AHRS_TRIM_[XYZ]$/, /^EK3_(HGT_DELAY|ABIAS_P_NSE|ACC_BIAS_LIM|ACC_P_NSE|HGT_I_GATE|MAG_M_NSE|POSNE_M_NSE|TAU_OUTPUT|SRC1_\w+)$/,
  /^COMPASS_(ENABLE|USE\d?|DEC|MOTCT|MOT_[XYZ])$/, /^RNGFND1_(TYPE|MIN_CM|MAX_CM|ORIENT|POS_[XYZ])$/, /^RNGFND_SQ_MIN$/,
  /^TEMP1_TYPE$/, /^LEAK\d_(PIN|TYPE|LOGIC)$/, /^VISO_TYPE$/, /^FLOW_TYPE$/, /^ARMING_(CHECK|OPTIONS|MIS_ITEMS)$/, /^GPS_TYPE$/, /^ORIGIN_(LAT|LON|ALT)$/, /^WP_YAW_BEHAVIOR$/, /^XTRACK_ANG_LIM$/, /^MIS_RESTART$/,
  /^MNT1_(TYPE|DEFLT_MODE|RC_RATE|PITCH_MIN|PITCH_MAX|ROLL_MIN|ROLL_MAX|YAW_MIN|YAW_MAX)$/, /^CAM1_TYPE$/, /^ACTUATOR\d_INC$/,
  /^FILT\d_(TYPE|NOTCH_FREQ|NOTCH_Q|NOTCH_ATT)$/, /^RELAY\d+_(FUNCTION|PIN|DEFAULT|INVERTED)$/, /^SERVO\d+_(FUNCTION|MIN|MAX|TRIM|REVERSED)$/,
];
// Values ArduSub overwrites at boot whatever is stored.
export const FORCED = {
  MOT_THST_HOVER: { value: 0.5, why: 'ArduSub sets MOT_THST_HOVER to 0.5 at every boot (ROVs are neutrally buoyant; ArduSub/Parameters.cpp:801)' },
};
// Parameters whose change rebuilds the vehicle model / needs a reboot on the autopilot.
export const STRUCTURAL = /^(FRAME_CONFIG|SCHED_LOOP_RATE|INS_|AHRS_TRIM|BARO_SPEC|EK3_)|^MOT_\d+_DIRECTION$/;

const GROUP_LABEL = {
  ACRO: 'Acro mode', ACTUATOR: 'Actuators', AHRS: 'AHRS', ANGLE: 'Pilot', ARMING: 'Arming', ARSPD: 'Airspeed', ATC: 'Attitude control',
  AVOID: 'Avoidance', BARO: 'Barometer / depth sensor', BATT: 'Battery monitor', BRD: 'Board', BTN: 'Joystick buttons', CAM: 'Camera',
  CAN: 'CAN bus', CIRCLE: 'Circle mode', COMPASS: 'Compass', EK2: 'EKF2', EK3: 'EKF3', FENCE: 'Fence', FILT: 'Filters (notch)',
  FLOW: 'Optical flow', FRAME: 'Frame', FS: 'Failsafes', GCS: 'Ground station', GPS: 'GPS', GRIP: 'Gripper', INS: 'IMU (INS)',
  JS: 'Joystick', LEAK: 'Leak detector', LOG: 'Logging', LOIT: 'Loiter', MIS: 'Mission', MNT: 'Camera mount', MOT: 'Motors',
  NTF: 'Notify (LEDs / buzzer)', PILOT: 'Pilot', PSC: 'Position / depth control', RALLY: 'Rally points', RC: 'RC input', RCMAP: 'RC mapping',
  RELAY: 'Relays', RNGFND: 'Rangefinder', RPM: 'RPM sensors', SCHED: 'Scheduler', SCR: 'Scripting', SERIAL: 'Serial ports',
  SERVO: 'Servo / motor outputs', SR: 'MAVLink stream rates', STAT: 'Statistics', SURFACE: 'Depth', SURFTRAK: 'Depth', SYSID: 'MAVLink ids',
  TEMP: 'Temperature sensors', TERRAIN: 'Terrain', THR: 'Pilot', VISO: 'Visual odometry', WPNAV: 'Waypoint navigation', XTRACK: 'Waypoint navigation',
  WP: 'Waypoint navigation', ORIGIN: 'EKF origin', FORMAT: 'System', NET: 'Networking', DDS: 'DDS / ROS 2', PRX: 'Proximity', EAHRS: 'External AHRS',
};
export function paramGroup(name) {
  const tok = name.split('_')[0];
  if (/^EK[23]$/.test(tok)) return tok;
  return tok.replace(/^BATT[2-9A-G]$/, 'BATT').replace(/^([A-Z]+?)\d+$/, '$1');
}
export const groupLabel = (g) => GROUP_LABEL[g] || g;

const table = (i) => (i >= 0 ? Object.fromEntries(ENUMS[i].map(([k, v]) => [k, v])) : null);
const enumCache = new Map();
const cachedTable = (i) => { if (i < 0) return null; if (!enumCache.has(i)) enumCache.set(i, table(i)); return enumCache.get(i); };

export const PARAM_DEFS = ROWS.map(([name, def, type, units, min, max, inc, displayName, desc, vi, bi, flags]) => {
  const effect = FORCED[name] ? 'inert' : SIM.some((r) => r.test(name)) ? 'sim' : 'inert';
  const p = {
    name, default: def, type: TYPE_NAME[type], mavType: type, units, unit: units, displayName, desc,
    group: paramGroup(name), effect, values: cachedTable(vi), bitmask: cachedTable(bi),
    reboot: !!(flags & 1), readOnly: !!(flags & 2), advanced: !!(flags & 4), volatile: !!(flags & 16), calibration: !!(flags & 32),
    subDefault: !!(flags & 64), reference: !!(flags & 128),
  };
  if (min !== null) p.min = min;
  if (max !== null) p.max = max;
  if (inc !== null) p.increment = inc;
  if (flags & 8) p.defaultUnknown = true;
  if (FORCED[name]) { p.forced = FORCED[name].value; p.note = FORCED[name].why; }
  if (effect === 'inert') p.inert = true;
  return p;
});
export const PARAM_INDEX = Object.fromEntries(PARAM_DEFS.map((p) => [p.name, p]));
export const PARAM_GROUPS = [...new Set(PARAM_DEFS.map((p) => p.group))].sort();

// ArduSub's default joystick button table (joystick.cpp default_js_buttons) as [FUNCTION, SFUNCTION].
export const DEFAULT_BUTTONS = Array.from({ length: 16 }, (_, i) => [PARAM_INDEX[`BTN${i}_FUNCTION`].default, PARAM_INDEX[`BTN${i}_SFUNCTION`].default]);

let defaultsCache = null;
export function defaultParams() {
  if (!defaultsCache) defaultsCache = Object.fromEntries(PARAM_DEFS.map((p) => [p.name, p.default]));
  return { ...defaultsCache };
}
export const isSimParam = (name) => PARAM_INDEX[name]?.effect === 'sim';
export function mavTypeOf(name, fallback) { return PARAM_INDEX[name]?.mavType ?? fallback ?? null; }

// ---------------------------------------------------------------- legacy names
// Old names that ArduSub converts on boot (AP_Param::convert_old_parameters and
// the library conversions), so older files land on the current parameter.
const LEGACY = {
  // ArduSub/Parameters.cpp:822-834 Sub::convert_old_parameters: RAT_*_FILT -> RAT_*_FLTE
  ATC_RAT_RLL_FILT: 'ATC_RAT_RLL_FLTE', ATC_RAT_PIT_FILT: 'ATC_RAT_PIT_FLTE', ATC_RAT_YAW_FILT: 'ATC_RAT_YAW_FLTE',
  // ArduSub/Parameters.cpp:764-770 conversion_table (failsafe battery / compass enable moved into the libraries)
  FS_BATT_VOLTAGE: 'BATT_LOW_VOLT', FS_BATT_MAH: 'BATT_LOW_MAH', FS_BATT_ENABLE: 'BATT_FS_LOW_ACT', MAG_ENABLE: 'COMPASS_ENABLE',
  // AP_BattMonitor analog backend rename, AP_Airspeed shortened name
  BATT_AMP_PERVOLT: 'BATT_AMP_PERVLT', ARSPD_TUBE_ORDER: 'ARSPD_TUBE_ORDR',
  // AP_Relay 4.5 conversion (RELAY_PIN.. -> RELAYn_PIN)
  RELAY_PIN: 'RELAY1_PIN', RELAY_PIN2: 'RELAY2_PIN', RELAY_PIN3: 'RELAY3_PIN', RELAY_PIN4: 'RELAY4_PIN', RELAY_PIN5: 'RELAY5_PIN', RELAY_PIN6: 'RELAY6_PIN',
};
export function legacyName(name) {
  if (PARAM_INDEX[name]) return null;
  if (LEGACY[name] && PARAM_INDEX[LEGACY[name]]) return LEGACY[name];
  let m;
  if ((m = /^BATT(\d)_AMP_PERVOLT$/.exec(name)) && PARAM_INDEX[`BATT${m[1]}_AMP_PERVLT`]) return `BATT${m[1]}_AMP_PERVLT`;
  if ((m = /^MNT_(\w+)$/.exec(name)) && PARAM_INDEX[`MNT1_${m[1]}`]) return `MNT1_${m[1]}`; // AP_Mount 4.3: MNT_ -> MNT1_
  if ((m = /^CAM_(\w+)$/.exec(name)) && PARAM_INDEX[`CAM1_${m[1]}`] && !PARAM_INDEX[name]) return `CAM1_${m[1]}`; // AP_Camera 4.4: CAM_ -> CAM1_
  return null;
}
// Rename legacy entries in a parsed {name: value} map. The current name wins when both are present.
export function convertLegacyParams(params) {
  const out = {}, converted = [];
  for (const [k, v] of Object.entries(params)) { const n = legacyName(k); if (!n) out[k] = v; }
  for (const [k, v] of Object.entries(params)) {
    const n = legacyName(k);
    if (!n) continue;
    if (!(n in out)) { out[n] = v; converted.push([k, n]); } else converted.push([k, n, 'ignored: file also has the new name']);
  }
  return { params: out, converted };
}

// ---------------------------------------------------------------- import / export
// Mission Planner (NAME,VALUE / NAME VALUE) or QGroundControl (sysid compid NAME VALUE TYPE) files.
export function parseParamFile(text) {
  const out = {}, types = {}, errors = [];
  let sysid = null, compid = null, qgcLines = 0, mpLines = 0;
  String(text).split(/\r?\n/).forEach((line, n) => {
    const s = line.trim();
    if (!s || s.startsWith('#')) return;
    let name, value;
    const qgc = s.split('\t');
    if (qgc.length >= 4 && /^\d+$/.test(qgc[0]) && /^\d+$/.test(qgc[1])) {
      name = qgc[2].trim(); value = qgc[3].trim();
      sysid ??= +qgc[0]; compid ??= +qgc[1];
      if (qgc[4] !== undefined && /^\d+$/.test(qgc[4].trim())) types[name.toUpperCase()] = +qgc[4].trim();
      qgcLines++;
    } else {
      const m = /^([A-Za-z0-9_]+)\s*[,\s]\s*([-+0-9.eE]+|nan|inf)\s*(?:[,\s].*)?$/i.exec(s);
      if (!m) { errors.push(`line ${n + 1}: ${s.slice(0, 40)}`); return; }
      name = m[1]; value = m[2]; mpLines++;
    }
    const v = Number(value);
    if (!Number.isFinite(v)) { errors.push(`line ${n + 1}: bad value for ${name}`); return; }
    out[name.toUpperCase()] = v;
  });
  return { params: out, errors, types, sysid, compid, format: qgcLines > mpLines ? 'qgc' : 'mp' };
}

// shortest decimal that reads back as the same float32 (what the autopilot stores)
export function formatValue(v, mavType = 9) {
  if (!Number.isFinite(v)) return '0';
  if (mavType !== 9) return String(Math.round(v));
  if (Number.isInteger(v) && Math.abs(v) < 1e15) return String(v);
  const f = Math.fround(v);
  for (let p = 1; p <= 9; p++) { const s = v.toPrecision(p); if (Math.fround(+s) === f) return String(+s); }
  return String(v);
}

export const PARAM_META = Symbol('paramMeta'); // {types, sysid, compid} carried by allParamsForExport()

// style 'mp' (Mission Planner .param) or 'qgc' (QGroundControl .params, with MAV_PARAM_TYPE per row)
export function formatParamFile(params, { style = 'mp', header = true, types, sysid, compid } = {}) {
  const meta = params[PARAM_META] || {};
  types = { ...(meta.types || {}), ...(types || {}) };
  sysid = Math.round(sysid ?? meta.sysid ?? params.SYSID_THISMAV ?? 1);
  compid = Math.round(compid ?? meta.compid ?? 1);
  const names = Object.keys(params).sort();
  const lines = [];
  const typeOf = (n) => mavTypeOf(n, types[n] ?? (Number.isInteger(params[n]) ? null : 9)) ?? guessType(n, params[n]);
  if (style === 'qgc') {
    if (header) lines.push(`# Onboard parameters for Vehicle ${sysid}`, '#', '# Stack: ArduPilot', '# Vehicle: Sub', '# Version: 4.5.7', '#', '# Vehicle-Id Component-Id Name Value Type');
    for (const n of names) { const t = typeOf(n); lines.push(`${sysid}\t${compid}\t${n}\t${formatValue(params[n], t)}\t${t}`); }
  } else {
    if (header) lines.push(`# DUBURI simulator parameter export (ArduSub 4.5.7) ${new Date().toISOString()}`);
    for (const n of names) lines.push(`${n},${formatValue(params[n], typeOf(n))}`);
  }
  return lines.join('\n') + '\n';
}
// unknown parameter without a type from a QGC file: integers by name pattern, else float
function guessType(name, v) {
  if (!Number.isInteger(v)) return 9;
  if (/_(TYPE|ENABLE|ENABLED|PIN|BUS|ADDR|ORIENT|PROTOCOL|FUNCTION|OPTIONS?|MASK|ID|DEVID|REVERSED)\d*$/.test(name)) return /DEVID|_ID\d*$|MASK|OPTIONS/.test(name) ? 6 : 2;
  return 9;
}

// ---------------------------------------------------------------- validation
// The range ArduSub's documentation gives, widened to include the default (some
// @Range lines are narrower than the vehicle default, e.g. ATC_ACCEL_Y_MAX with
// Sub's 110000), with 0 accepted where the default is 0 ("disabled").
export function paramRange(name) {
  const d = PARAM_INDEX[name];
  if (!d) return null;
  let lo = d.min, hi = d.max;
  if (lo !== undefined && d.default < lo) lo = d.default;
  if (hi !== undefined && d.default > hi) hi = d.default;
  const ir = INT_RANGE[d.mavType];
  if (ir) { lo = Math.max(lo ?? ir[0], ir[0]); hi = Math.min(hi ?? ir[1], ir[1]); }
  return { min: lo, max: hi, zeroOk: d.default === 0 };
}
// {value, error, warning}: error = rejected (not a number); value is clamped to
// the range and rounded for integer parameters; warning explains any change.
export function checkParam(name, value, { clamp = true } = {}) {
  const v0 = typeof value === 'string' ? Number(value) : value;
  if (typeof v0 !== 'number' || !Number.isFinite(v0)) return { value: null, error: 'Not a number' };
  const d = PARAM_INDEX[name];
  if (!d) return { value: v0 };
  let v = v0;
  const warn = [];
  if (d.mavType !== 9 && !Number.isInteger(v)) { v = Math.round(v); warn.push(`${d.type} parameter: rounded to ${v}`); }
  const r = paramRange(name);
  if (!(r.zeroOk && v === 0)) {
    if (r.min !== undefined && v < r.min) { warn.push(`below minimum ${r.min}`); if (clamp) v = r.min; }
    if (r.max !== undefined && v > r.max) { warn.push(`above maximum ${r.max}`); if (clamp) v = r.max; }
  }
  if (d.values && !(v in d.values) && !d.bitmask) warn.push('not one of the documented values');
  if (d.bitmask && Number.isInteger(v)) {
    const known = Object.keys(d.bitmask).reduce((m, b) => m + 2 ** +b, 0);
    if (v > 0 && (v & ~known) !== 0 && v < 2 ** 31) warn.push('sets undocumented bits');
  }
  if (d.forced !== undefined && v !== d.forced) warn.push(`ArduSub forces ${d.forced} at boot`);
  const res = { value: v };
  if (warn.length) res.warning = (v !== v0 && clamp ? `clamped to ${v}: ` : '') + warn.join('; ');
  return res;
}
// Back-compatible check: null when acceptable, else a message (out of range or not a number).
export function validateParam(name, value) {
  const r = checkParam(name, value, { clamp: false });
  if (r.error) return r.error;
  const d = PARAM_INDEX[name];
  if (!d || !r.warning || !/minimum|maximum/.test(r.warning)) return null;
  return r.warning.replace(/^./, (c) => c.toUpperCase());
}
// Cross-parameter checks ArduSub would refuse to arm with (ARMING_CHECK_RC) or that break the maths.
export function crossCheck(P) {
  const out = [];
  for (let i = 1; i <= 16; i++) {
    const mn = P[`RC${i}_MIN`], mx = P[`RC${i}_MAX`], tr = P[`RC${i}_TRIM`], dz = P[`RC${i}_DZ`] ?? 0;
    if (mn === undefined || mx === undefined) continue;
    if (!(mn < mx)) out.push([`RC${i}_MIN`, `RC${i}_MIN must be below RC${i}_MAX`]);
    else if (tr !== undefined && (tr < mn || tr > mx)) out.push([`RC${i}_TRIM`, `RC${i}_TRIM outside RC${i}_MIN..MAX`]);
    else if (tr !== undefined && (tr - dz <= mn || tr + dz >= mx) && i !== 3) out.push([`RC${i}_DZ`, `RC${i}_DZ leaves no stick travel around the trim`]);
  }
  if (P.JS_GAIN_MIN > P.JS_GAIN_MAX) out.push(['JS_GAIN_MIN', 'JS_GAIN_MIN is above JS_GAIN_MAX']);
  if (P.MOT_PWM_MIN >= P.MOT_PWM_MAX) out.push(['MOT_PWM_MIN', 'MOT_PWM_MIN must be below MOT_PWM_MAX']);
  return out;
}
