// Vehicle configuration: one JSON document that fully describes a vehicle
// (geometry source, mass properties, hydrodynamics, thrusters, mixer, sensors,
// ArduSub parameters, joystick). Versioned so saved files stay loadable.

import { heavyLayout } from '../alloc/frames.js';
import { defaultParams, PARAM_INDEX, PARAM_DEFS, PARAM_META, checkParam, convertLegacyParams } from '../ardusub/params.js';
import { THRUSTER_PRESETS } from '../physics/thruster.js';
import { DEFAULT_ENVIRONMENT } from '../physics/environment.js';
import { DEFAULT_SHAPE } from './shapes.js';

export const CONFIG_SCHEMA = 'duburi.vehicle';

// Pressure housing (electronics / battery enclosures) of the vehicle.
export const DEFAULT_HULL = {
  depthRating: 100, // m: rated depth of the pressure housings
  compressibility: 0, // % of the displaced volume lost per 10 bar (about 100 m); 0 = rigid
  airVolume: 0, // L of air inside the housings (0 = auto: 20% of the displaced volume)
  leakArea: 2, // mm²: effective leak hole once a housing fails
  floodOverRating: true, // housings start to leak below the rated depth
  leakSensor: true, // a leak detector is fitted (ArduSub LEAK1_PIN set)
};

// External depth/pressure sensors (absolute pressure range, resolution).
export const DEPTH_SENSORS = {
  bar02: { label: 'Bar02 (2 bar · 10 m)', maxBar: 2, resMbar: 0.016 },
  bar30: { label: 'Bar30 (30 bar · 300 m)', maxBar: 30, resMbar: 0.2 },
  custom: { label: 'Custom' },
};
export function depthSensorSpec(s = {}) {
  const p = DEPTH_SENSORS[s.depthSensor] ?? DEPTH_SENSORS.bar30;
  const custom = s.depthSensor === 'custom';
  const maxBar = custom ? Math.max(0.1, +s.depthSensorMax || 30) : p.maxBar;
  const resMbar = custom ? Math.max(0, +s.depthSensorRes || 0) : p.resMbar;
  return { maxPa: maxBar * 1e5, resPa: resMbar * 100, maxBar, resMbar, label: custom ? `Custom (${maxBar} bar)` : p.label };
}
export const CONFIG_VERSION = 1;

export function thrusterFromPreset(base, presetKey = 't200-16v') {
  const p = THRUSTER_PRESETS[presetKey] ?? THRUSTER_PRESETS['t200-16v'];
  return {
    name: base.name ?? 'Thruster',
    pos: base.pos ?? [0, 0, 0],
    axis: base.axis ?? [1, 0, 0],
    polarity: base.polarity ?? 1,
    preset: presetKey,
    curve: { ...p.curve },
    table: null,
    propDiameter: p.propDiameter,
    maxRpm: p.maxRpm,
    J0: p.J0,
    tauUp: p.tauUp,
    tauDown: p.tauDown,
    deductionMode: 'auto',
    deduction: 0,
    deductionRev: 0,
    inflowModel: true,
    enabled: true,
  };
}

// Thrust limits of a thruster definition (measured table wins over the parametric curve).
export function thrusterLimits(t) {
  if (Array.isArray(t.table) && t.table.length >= 2) {
    const T = t.table.map((r) => +r[1]);
    return { fwdMax: Math.max(1e-6, ...T), revMax: Math.max(1e-6, ...T.map((v) => -v)) };
  }
  return { fwdMax: t.curve?.fwdMax ?? 50, revMax: t.curve?.revMax ?? 40 };
}

export function defaultConfig() {
  return {
    schema: CONFIG_SCHEMA,
    version: CONFIG_VERSION,
    name: 'DUBURI sample ROV',
    model: { source: 'sample', units: 'mm', forward: '+X', up: '+Z', origin: 'bbox', offset: [0, 0, 0], color: '#dfe6ea', sealedHull: false, fileName: null, shape: { ...DEFAULT_SHAPE } },
    mass: { mass: 13.5, source: 'estimate', cgMode: 'cb-offset', cgOffset: [0, 0, 0.025], cg: [0, 0, 0], inertia: { mode: 'auto', Ixx: 0.3, Iyy: 0.4, Izz: 0.45, Pxy: 0, Pxz: 0, Pyz: 0 } },
    buoyancy: { mode: 'auto', volume: 0.0135, cb: [0, 0, 0] },
    hydro: {
      addedMass: { mode: 'estimate', diag: [6, 8, 15, 0.15, 0.15, 0.15], matrix: null, ref: [0, 0, 0], meta: null },
      damping: { mode: 'estimate', Cd: 1.0, linear: [4, 6, 6, 0.1, 0.1, 0.1], quadratic: [20, 30, 40, 1.5, 1.5, 1.5], DL: null, DQ: null, ref: [0, 0, 0], meta: null },
    },
    thrusters: heavyLayout().map((t) => thrusterFromPreset(t, 't200-16v')),
    mixer: { source: 'auto', saturation: 'priority', threshold: 0.02, axisEnable: { roll: true, pitch: true, yaw: true, throttle: true, forward: true, lateral: true }, custom: null },
    // positioning: a horizontal position source ArduSub can use (DVL / USBL via VISO or ExternalNav). Off by default:
    // with the default EK3_SRC1_POSXY = 3 (GPS) ArduSub has no position underwater (Sub::position_ok, system.cpp:212).
    sensors: { gyroNoise: 0.0015, accelNoise: 0.04, attitudeNoise: 0.02, depthNoise: 0.002, depthRate: 50, depthSensor: 'bar30', depthSensorMax: 30, depthSensorRes: 0.2, depthSensorPos: [0, 0, 0], positioning: false, positionNoise: 0.01, positionRate: 10 },
    hull: { ...DEFAULT_HULL },
    environment: JSON.parse(JSON.stringify(DEFAULT_ENVIRONMENT)),
    start: { position: [0, 0, 1.0], yaw: 0 },
    sim: { physicsRate: 800, seed: 1 },
    params: defaultParams(), // every ArduSub 4.5.7 parameter (PARAM_DEFS), file values over defaults
    extraParams: {}, // names from a file that this ArduSub version does not define (board-specific, newer firmware): kept verbatim
    paramNames: [], // names present on the vehicle (from imported files); exported with any changed ones
    paramTypes: {}, // MAV_PARAM_TYPE of extraParams read from a QGC file
    joystick: {
      deadzone: 0.06,
      expo: 0,
      sendRate: 50,
      axes: { x: { axis: 3, invert: true }, y: { axis: 2, invert: false }, z: { axis: 1, invert: true }, r: { axis: 0, invert: false } },
      triggers: { left: 'none', right: 'none' },
      // browser "standard gamepad" index -> MAVLink/QGC (SDL) button number
      buttonMap: { 0: 0, 1: 1, 2: 2, 3: 3, 8: 4, 16: 5, 9: 6, 10: 7, 11: 8, 4: 9, 5: 10, 12: 11, 13: 12, 14: 13, 15: 14 },
    },
  };
}

// Merge a (possibly older / partial) config onto defaults.
export function normalizeConfig(cfg) {
  const d = defaultConfig();
  if (!cfg || typeof cfg !== 'object') return d;
  if (cfg.schema && cfg.schema !== CONFIG_SCHEMA) throw new Error('Not a DUBURI vehicle file');
  const merge = (a, b) => {
    if (Array.isArray(a) || Array.isArray(b)) return b ?? a;
    if (a && typeof a === 'object' && b && typeof b === 'object') {
      const o = { ...a };
      for (const k of Object.keys(b)) o[k] = k in a ? merge(a[k], b[k]) : b[k];
      return o;
    }
    return b === undefined ? a : b;
  };
  const out = merge(d, cfg);
  // user mappings round-trip exactly ("Not used" = key removed): replace, don't merge with the defaults
  if (cfg.joystick?.buttonMap) out.joystick.buttonMap = { ...cfg.joystick.buttonMap };
  if (cfg.joystick?.axes) out.joystick.axes = { ...d.joystick.axes, ...cfg.joystick.axes };
  const known = defaultParams();
  const saved = cfg.params || {};
  const extraIn = { ...(cfg.extraParams || {}) };
  const { params: conv } = convertLegacyParams(extraIn);
  const params = { ...known };
  const extra = {};
  for (const [k, v] of Object.entries(saved)) { if (k in known) params[k] = v; else extra[k] = v; }
  // Parameters the simulator started to know after this vehicle was saved:
  // the value from the user's file (kept in extraParams) wins over the new default.
  for (const [k, v] of Object.entries(conv)) {
    if (!(k in known)) { extra[k] = v; continue; }
    if (!(k in saved) || (saved[k] === known[k] && v !== known[k])) params[k] = v;
  }
  out.params = params;
  out.extraParams = extra;
  out.paramTypes = { ...(cfg.paramTypes || {}) };
  // vehicles saved before paramNames existed: everything they carried was on the vehicle
  out.paramNames = Array.isArray(cfg.paramNames) ? [...cfg.paramNames]
    : Object.keys(cfg.extraParams || {}).length ? [...new Set([...Object.keys(saved), ...Object.keys(conv)])].filter((k) => k in known || k in extra).sort() : [];
  out.hull = { ...DEFAULT_HULL, ...(out.hull || {}) };
  if (out.hull.leakSensorSource === 'params' || (!out.hull.leakSensorSource && out.paramNames.length)) {
    const det = leakDetectorFromParams({ ...extra, ...params }, out.paramNames);
    if (det !== null) { out.hull.leakSensor = det; out.hull.leakSensorSource = 'params'; }
  }
  out.thrusters = (cfg.thrusters || d.thrusters).map((t) => ({ ...thrusterFromPreset(t, t.preset || 't200-16v'), ...t }));
  out.version = CONFIG_VERSION;
  out.schema = CONFIG_SCHEMA;
  return out;
}

// Apply an ArduSub .param map (parseParamFile().params) to a config. Every value
// is kept exactly as in the file (no clamping); names this ArduSub version does
// not define go to extraParams. Legacy names are converted as ArduSub does at boot.
// opts.types: MAV_PARAM_TYPE per name from a QGC file (kept for unknown names).
export function applyParamFile(cfg, parsed, { syncPolarity = true, types = {} } = {}) {
  const known = PARAM_INDEX;
  const { params: file, converted } = convertLegacyParams(parsed);
  const report = { applied: 0, simulated: 0, stored: 0, kept: 0, unknown: [], converted, polarityChanged: [] };
  cfg.params = cfg.params || defaultParams();
  cfg.extraParams = cfg.extraParams || {};
  cfg.paramTypes = cfg.paramTypes || {};
  const names = new Set(cfg.paramNames || []);
  for (const [k, v] of Object.entries(file)) {
    names.add(k);
    if (known[k]) {
      cfg.params[k] = v; delete cfg.extraParams[k];
      if (known[k].effect === 'sim') report.simulated++; else report.stored++;
    } else {
      cfg.extraParams[k] = v; report.unknown.push(k);
      if (types[k] !== undefined) cfg.paramTypes[k] = types[k];
    }
  }
  cfg.paramNames = [...names].sort();
  report.applied = report.simulated; // change the simulated flight
  report.kept = report.stored + report.unknown.length; // stored and exported unchanged
  // the leak detector the file configures (LEAKn_PIN / LEAKn_TYPE)
  const det = leakDetectorFromParams({ ...cfg.params, ...file }, Object.keys(file));
  if (det !== null) { cfg.hull = { ...DEFAULT_HULL, ...(cfg.hull || {}), leakSensor: det, leakSensorSource: 'params' }; report.leakDetector = det; }
  // horizontal positioning the parameters can use (Sub::position_ok needs an absolute XY source underwater)
  if (['VISO_TYPE', 'EK3_SRC1_POSXY', 'EK3_SRC1_VELXY', 'FLOW_TYPE'].some((k) => k in file) && cfg.sensors?.positioningSource !== 'user') {
    cfg.sensors = { ...(cfg.sensors || {}), positioning: positioningFromParams(cfg.params), positioningSource: 'params' };
    report.positioning = cfg.sensors.positioning;
  }
  if (syncPolarity) {
    // Assume the real vehicle flies correctly: a reversed MOT_n_DIRECTION means
    // that thruster's prop/ESC pushes the other way on the hardware.
    cfg.thrusters.forEach((t, i) => {
      const dir = cfg.params[`MOT_${i + 1}_DIRECTION`] ?? 1;
      const pol = dir < 0 ? -1 : 1;
      if (t.polarity !== pol) { t.polarity = pol; report.polarityChanged.push(i + 1); }
    });
  }
  return report;
}

// Can ArduSub hold position with these parameters underwater? GPS (EK3_SRCn_POSXY 3)
// gives nothing submerged; ExternalNav (6, a DVL/USBL through VISO) or optical flow does.
export function positioningFromParams(P) {
  return (P.VISO_TYPE ?? 0) > 0 || P.EK3_SRC1_POSXY === 6 || P.EK3_SRC1_VELXY === 6 || ((P.FLOW_TYPE ?? 0) > 0 && P.EK3_SRC1_VELXY === 5);
}

// The single setter for a parameter change from the UI / tuning / joystick panels:
// validates (range from the ArduSub docs, integer types), clamps, stores it and
// keeps derived vehicle settings (leak detector) in sync.
// Returns {value, warning?, error?}. On error nothing is changed.
export function setParamValue(cfg, name, value, { clamp = true } = {}) {
  const r = checkParam(name, value, { clamp });
  if (r.error) return r;
  if (PARAM_INDEX[name]) { cfg.params[name] = r.value; if (cfg.extraParams) delete cfg.extraParams[name]; }
  else { cfg.extraParams = cfg.extraParams || {}; cfg.extraParams[name] = r.value; }
  if (/^LEAK\d_(PIN|TYPE)$/.test(name) && (cfg.hull?.leakSensorSource ?? 'params') === 'params') {
    const det = leakDetectorFromParams({ ...cfg.extraParams, ...cfg.params }, [...(cfg.paramNames || []), name]);
    if (det !== null) cfg.hull = { ...DEFAULT_HULL, ...(cfg.hull || {}), leakSensor: det, leakSensorSource: 'params' };
  }
  if (/^(VISO_TYPE|EK3_SRC1_(POSXY|VELXY)|FLOW_TYPE)$/.test(name) && cfg.sensors && cfg.sensors.positioningSource === 'params') cfg.sensors.positioning = positioningFromParams(cfg.params);
  return r;
}

// ArduSub 4.5 leak detectors (AP_LeakDetector.cpp): LEAKn_TYPE -1 disabled, 0 analog, 1 digital.
// init() gives an unsaved TYPE a default from the pin on FMUv3/v5 boards (13-15 analog, 50-55
// digital), so a pin with no TYPE in the file is a working detector there.
// `present`: names stored on the vehicle (default: every key of `all`).
// Returns true / false when the parameters say, null when they don't mention it.
export function leakDetectorFromParams(all, present = null) {
  const has = present ? new Set(present) : null;
  const inFile = (k) => (has ? has.has(k) : k in all);
  let seen = false;
  for (let i = 1; i <= 3; i++) {
    const pin = all[`LEAK${i}_PIN`];
    if (pin === undefined || !inFile(`LEAK${i}_PIN`)) continue;
    seen = true;
    let type = inFile(`LEAK${i}_TYPE`) ? all[`LEAK${i}_TYPE`] : undefined;
    if (type === undefined) type = pin >= 13 && pin <= 15 ? 0 : pin >= 50 && pin <= 55 ? 1 : pin >= 0 ? 1 : -1;
    if (pin >= 0 && type >= 0) return true;
  }
  return seen ? false : null;
}

// What a .param export writes: every parameter present on the vehicle (the names
// from its files; the reference Pixhawk set when none was loaded), every value
// changed from the ArduSub default, and the unknown names kept from the file.
// Carries {types, sysid} for formatParamFile under PARAM_META.
export function allParamsForExport(cfg) {
  const P = cfg.params || {};
  const names = new Set(cfg.paramNames?.length ? cfg.paramNames : PARAM_DEFS.filter((p) => p.reference || p.effect === 'sim').map((p) => p.name));
  for (const p of PARAM_DEFS) if (p.name in P && P[p.name] !== p.default) names.add(p.name);
  const out = {};
  for (const n of [...names].sort()) {
    if (n in P) out[n] = P[n];
    else if (cfg.extraParams && n in cfg.extraParams) out[n] = cfg.extraParams[n];
    else if (PARAM_INDEX[n]) out[n] = PARAM_INDEX[n].default;
  }
  for (const [k, v] of Object.entries(cfg.extraParams || {})) out[k] = v;
  Object.defineProperty(out, PARAM_META, { value: { types: { ...(cfg.paramTypes || {}) }, sysid: P.SYSID_THISMAV ?? 1, compid: 1 }, enumerable: false });
  return out;
}
