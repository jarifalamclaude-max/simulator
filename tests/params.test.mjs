import test from 'node:test';
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import {
  PARAM_DEFS, PARAM_INDEX, defaultParams, parseParamFile, formatParamFile, validateParam, checkParam, convertLegacyParams,
  formatValue, crossCheck, MAV_PARAM_TYPE,
} from '../src/ardusub/params.js';
import { defaultConfig, applyParamFile, allParamsForExport, normalizeConfig, setParamValue, leakDetectorFromParams, positioningFromParams } from '../src/vehicle/config.js';

const TEXT = readFileSync(new URL('../profiles/duburi_pixhawk_param.param', import.meta.url), 'utf8');
const FILE = parseParamFile(TEXT).params;
const fileLines = TEXT.split(/\r?\n/).filter((l) => l.trim() && !l.startsWith('#'));

test('PARAM_DEFS: every ArduSub 4.5.7 parameter, each name of the DUBURI file resolves', () => {
  assert.ok(PARAM_DEFS.length > 2500, `${PARAM_DEFS.length}`);
  assert.equal(new Set(PARAM_DEFS.map((p) => p.name)).size, PARAM_DEFS.length, 'unique names');
  assert.equal(fileLines.length, 943);
  assert.equal(Object.keys(FILE).length, 943);
  const missing = Object.keys(FILE).filter((k) => !PARAM_INDEX[k]);
  assert.deepEqual(missing, []);
  for (const p of PARAM_DEFS) {
    assert.ok(p.effect === 'sim' || p.effect === 'inert', p.name);
    assert.ok([2, 4, 6, 9].includes(p.mavType), p.name);
    assert.ok(Number.isFinite(p.default), p.name);
  }
  // parameters that do not exist on ArduSub (AP_GROUPINFO_FRAME without AP_PARAM_FRAME_SUB, SITL)
  for (const n of ['RC7_OPTION', 'ARMING_REQUIRE', 'RC_FS_TIMEOUT', 'SIM_SPEEDUP', 'ATC_RAT_RLL_FILT']) assert.ok(!PARAM_INDEX[n], n);
});

test('contract parameter names exist with ArduPilot metadata', () => {
  const names = ['MOT_SPOOL_TIM_DN', 'MOT_SAFE_TIME', 'MOT_BAT_CURR_MAX', 'MOT_BAT_CURR_TC', 'ATC_RAT_RLL_SMAX', 'PSC_VELXY_FF', 'BATT_MONITOR', 'BATT_LOW_VOLT',
    'BATT_CRT_VOLT', 'BATT_LOW_MAH', 'BATT_CRT_MAH', 'BATT_FS_LOW_ACT', 'BATT_FS_CRT_ACT', 'BATT_LOW_TIMER', 'FS_GCS_ENABLE', 'FS_TEMP_ENABLE', 'FS_TEMP_MAX',
    'FS_EKF_ACTION', 'FS_TERRAIN_ENAB', 'SURFTRAK_DEPTH', 'RNGFND1_TYPE', 'RNGFND1_MAX_CM', 'RNGFND_SQ_MIN', 'COMPASS_USE', 'INS_GYRO_FILTER', 'INS_HNTCH_ENABLE',
    'EK3_SRC1_POSXY', 'BARO_FLTR_RNG', 'BARO_EXT_BUS', 'TEMP1_TYPE', 'LEAK1_PIN', 'LEAK3_LOGIC', 'WPNAV_RADIUS', 'CIRCLE_RATE', 'JS_GAIN_STEPS', 'BTN31_SFUNCTION',
    'RC16_DZ', 'RC_OVERRIDE_TIME', 'SERVO16_FUNCTION', 'SERVO32_REVERSED', 'RC_SPEED', 'ARMING_CHECK', 'ARMING_OPTIONS', 'SYSID_THISMAV', 'GCS_PID_MASK',
    'LOG_BITMASK', 'ACTUATOR6_INC', 'ORIGIN_LAT', 'MNT1_RC_RATE', 'RELAY6_FUNCTION', 'AHRS_ORIENTATION', 'MOT_THST_HOVER', 'SR0_EXTRA1'];
  for (const n of names) assert.ok(PARAM_INDEX[n], n);
  const p = PARAM_INDEX.ATC_RAT_RLL_P;
  assert.equal(p.displayName, 'Roll axis rate controller P gain');
  assert.deepEqual([p.min, p.max, p.increment], [0, 0.3, 0.005]);
  assert.equal(PARAM_INDEX.FRAME_CONFIG.values[2], 'Vectored_6DOF');
  assert.equal(PARAM_INDEX.LOG_BITMASK.bitmask[17], 'MOTBATT');
  assert.equal(PARAM_INDEX.PILOT_SPEED_UP.units, 'cm/s');
  assert.equal(PARAM_INDEX.MOT_THST_HOVER.forced, 0.5);
  assert.equal(PARAM_INDEX.MOT_THST_HOVER.effect, 'inert');
  assert.equal(PARAM_INDEX.ATC_RAT_RLL_P.effect, 'sim');
  assert.equal(PARAM_INDEX.COMPASS_OFS_X.effect, 'inert');
});

test('defaults match the ArduSub 4.5.7 sources', () => {
  const expect = {
    FRAME_CONFIG: 1, // Parameters.cpp:277 SUB_FRAME_VECTORED
    SURFACE_DEPTH: -10, PILOT_SPEED_UP: 100, PILOT_ACCEL_Z: 100, THR_DZ: 100, // config.h, defaults_table
    FS_GCS_ENABLE: 2, FS_LEAK_ENABLE: 1, FS_PRESS_MAX: 105000, FS_TEMP_MAX: 62, FS_PILOT_TIMEOUT: 3, FS_EKF_THRESH: 0.8, // defines.h
    JS_GAIN_DEFAULT: 0.5, JS_GAIN_STEPS: 4, RC_SPEED: 200, ACRO_YAW_P: 3.375, ACRO_EXPO: 0.3, SURFTRAK_DEPTH: -50, RNGFND_SQ_MIN: 90,
    ATC_RAT_RLL_P: 0.135, ATC_RAT_RLL_I: 0.09, ATC_RAT_RLL_D: 0.0036, ATC_RAT_RLL_IMAX: 0.444, ATC_RAT_RLL_FLTT: 30, ATC_RAT_YAW_P: 0.18, ATC_RAT_YAW_FLTD: 5, // AC_AttitudeControl_Sub.h
    ATC_ANG_RLL_P: 6, ATC_ACCEL_Y_MAX: 110000, ATC_RATE_Y_MAX: 180, CIRCLE_RATE: 2, RC3_TRIM: 1100, RC4_DZ: 40, RC1_DZ: 30, // defaults_table, radio.cpp
    PSC_POSZ_P: 3, PSC_VELZ_P: 8, PSC_ACCZ_P: 0.5, PSC_ACCZ_I: 0.1, PSC_JERK_Z: 50, PSC_VELXY_P: 6, WPNAV_SPEED: 100, // AC_PosControl, defaults_table
    MOT_PWM_MIN: 1100, MOT_PWM_MAX: 1900, MOT_THST_HOVER: 0.5, MOT_SPIN_MIN: 0.15, MOT_SPOOL_TIME: 0.5,
    ARMING_CHECK: 64 | 128 | 256, MNT1_TYPE: 1, MNT1_DEFLT_MODE: 3, MNT1_RC_RATE: 30, SERVO9_FUNCTION: 59, SERVO10_FUNCTION: 7, BARO_PROBE_EXT: 768, EK3_SRC1_VELZ: 0,
    LEAK1_PIN: -1, LEAK1_TYPE: -1, BATT_CAPACITY: 3300, BATT_LOW_TIMER: 10, SYSID_THISMAV: 1, SYSID_MYGCS: 255, XTRACK_ANG_LIM: 45, WP_YAW_BEHAVIOR: 4,
    LOG_BITMASK: 176126, INS_GYRO_FILTER: 20, INS_ACC1_CALTEMP: -300, FENCE_TYPE: 7, COMPASS_USE: 1, BTN1_FUNCTION: 5, BTN4_FUNCTION: 4, BTN5_FUNCTION: 1, BTN9_SFUNCTION: 27, // joystick.cpp default_js_buttons,
  };
  const d = defaultParams();
  for (const [k, v] of Object.entries(expect)) assert.equal(d[k], v, k);
  assert.ok(Object.keys(expect).length >= 30);
});

test('QGC export: MAV_PARAM_TYPE per parameter and SYSID_THISMAV as vehicle id', () => {
  const cfg = defaultConfig();
  applyParamFile(cfg, FILE);
  cfg.params.SYSID_THISMAV = 7;
  const out = formatParamFile(allParamsForExport(cfg), { style: 'qgc' });
  const rows = Object.fromEntries(out.split('\n').filter((l) => l && !l.startsWith('#')).map((l) => l.split('\t')).map((c) => [c[2], c]));
  assert.equal(rows.FRAME_CONFIG[4], String(MAV_PARAM_TYPE.INT8));
  assert.equal(rows.RC1_MIN[4], String(MAV_PARAM_TYPE.INT16));
  assert.equal(rows.ARMING_CHECK[4], String(MAV_PARAM_TYPE.INT32));
  assert.equal(rows.ATC_RAT_RLL_P[4], String(MAV_PARAM_TYPE.REAL32));
  assert.equal(rows.FRAME_CONFIG[0], '7');
  assert.equal(rows.FRAME_CONFIG[1], '1');
});

for (const style of ['mp', 'qgc']) {
  test(`the 943-line DUBURI file round-trips value for value (${style === 'mp' ? 'Mission Planner' : 'QGC'})`, () => {
    const cfg = defaultConfig();
    applyParamFile(cfg, FILE, { syncPolarity: true });
    const text = formatParamFile(allParamsForExport(cfg), { style });
    const back = parseParamFile(text);
    assert.equal(back.errors.length, 0);
    assert.deepEqual(Object.keys(back.params).sort(), Object.keys(FILE).sort(), 'same 943 names, nothing added');
    for (const [k, v] of Object.entries(FILE)) assert.equal(back.params[k], v, k);
    // and again through a fresh vehicle from the exported file (types preserved for QGC)
    const cfg2 = defaultConfig();
    applyParamFile(cfg2, back.params, { types: back.types });
    const again = formatParamFile(allParamsForExport(cfg2), { style, header: false });
    assert.equal(again, text.split('\n').filter((l) => !l.startsWith('#')).join('\n'));
    // byte-level: every Mission Planner line of the original reappears unchanged
    if (style === 'mp') { const lines = new Set(text.split('\n')); for (const l of fileLines) assert.ok(lines.has(l.trim()), l); }
  });
}

test('float values keep full float32 precision; integers are written as integers', () => {
  assert.equal(formatValue(0.006551251, 9), '0.006551251');
  assert.equal(formatValue(0.1 + 0.2, 9), '0.3');
  assert.equal(Math.fround(+formatValue(Math.PI, 9)), Math.fround(Math.PI));
  assert.equal(formatValue(100825.5, 9), '100825.5');
  assert.equal(formatValue(-1, 2), '-1');
  assert.equal(formatValue(2.0000001, 2), '2');
  const t = formatParamFile({ ATC_RAT_RLL_P: 0.123456789123 }, { style: 'mp', header: false });
  assert.equal(Math.fround(parseParamFile(t).params.ATC_RAT_RLL_P), Math.fround(0.123456789123));
});

test('unknown names from a QGC file keep their type and value on export', () => {
  const txt = '1\t1\tFOO_BAR\t3\t4\n1\t1\tFRAME_CONFIG\t2\t2\n';
  const p = parseParamFile(txt);
  assert.equal(p.format, 'qgc');
  assert.deepEqual(p.types, { FOO_BAR: 4, FRAME_CONFIG: 2 });
  const cfg = defaultConfig();
  const rep = applyParamFile(cfg, p.params, { types: p.types });
  assert.deepEqual(rep.unknown, ['FOO_BAR']);
  const out = formatParamFile(allParamsForExport(cfg), { style: 'qgc' });
  assert.ok(out.includes('1\t1\tFOO_BAR\t3\t4'));
});

test('legacy names are converted as ArduSub does at boot', () => {
  const { params, converted } = convertLegacyParams({ ATC_RAT_RLL_FILT: 12, ATC_RAT_YAW_FILT: 4, FS_BATT_ENABLE: 1, FS_BATT_VOLTAGE: 13.2, MNT_TYPE: 1, ATC_RAT_PIT_FILT: 7, ATC_RAT_PIT_FLTE: 9 });
  assert.equal(params.ATC_RAT_RLL_FLTE, 12); // Sub::convert_old_parameters (Parameters.cpp:826)
  assert.equal(params.ATC_RAT_YAW_FLTE, 4);
  assert.equal(params.ATC_RAT_PIT_FLTE, 9, 'the new name wins');
  assert.equal(params.BATT_FS_LOW_ACT, 1);
  assert.equal(params.BATT_LOW_VOLT, 13.2);
  assert.equal(params.MNT1_TYPE, 1);
  assert.ok(!('ATC_RAT_RLL_FILT' in params));
  assert.equal(converted.length, 6);
  const cfg = defaultConfig();
  const rep = applyParamFile(cfg, { ATC_RAT_RLL_FILT: 12 });
  assert.equal(cfg.params.ATC_RAT_RLL_FLTE, 12);
  assert.deepEqual(rep.unknown, []);
});

test('validation: documented ranges clamp, integers round, NaN is rejected, 0 stays "disabled"', () => {
  assert.deepEqual(checkParam('RC3_DZ', 800).value, 200); // RC_Channel.cpp:103 DZ 0-200 (the NaN depth target)
  assert.equal(checkParam('ATC_RAT_RLL_P', 5).value, 0.3);
  assert.equal(checkParam('ATC_ANG_YAW_P', 9).value, 6); // @Range{Sub} 0-6
  assert.equal(checkParam('ATC_ACCEL_Y_MAX', 110000).value, 110000, 'Sub default above the doc range stays valid');
  assert.equal(checkParam('ATC_RAT_RLL_FLTE', 0).value, 0, '0 = filter off');
  assert.equal(checkParam('PILOT_SPEED_UP', 20).value, 50);
  assert.equal(checkParam('FRAME_CONFIG', 2.4).value, 2);
  assert.equal(checkParam('RC1_MIN', 1e6).value, 2200);
  assert.ok(checkParam('RC3_DZ', NaN).error);
  assert.ok(checkParam('RC3_DZ', 'abc').error);
  assert.ok(checkParam('MOT_THST_HOVER', 0.3).warning.includes('forces 0.5'));
  assert.equal(validateParam('RC3_DZ', 100), null);
  assert.match(validateParam('RC3_DZ', 800), /maximum 200/);
  assert.equal(validateParam('THR_DZ', 400), 'Above maximum 300');
  const cfg = defaultConfig();
  const r = setParamValue(cfg, 'RC3_DZ', 800);
  assert.equal(r.value, 200);
  assert.equal(cfg.params.RC3_DZ, 200);
  assert.ok(setParamValue(cfg, 'JS_GAIN_MIN', NaN).error);
  assert.equal(cfg.params.JS_GAIN_MIN, 0.25, 'unchanged on error');
  cfg.params.JS_GAIN_MIN = 0.8; cfg.params.JS_GAIN_MAX = 0.5;
  assert.ok(crossCheck(cfg.params).some(([k]) => k === 'JS_GAIN_MIN'));
  // a file is loaded exactly, even with out-of-range values
  const c2 = defaultConfig();
  applyParamFile(c2, { RC3_DZ: 800 });
  assert.equal(c2.params.RC3_DZ, 800);
});

test('leak detector follows LEAKn_PIN / LEAKn_TYPE after import and on later edits', () => {
  const cfg = defaultConfig();
  applyParamFile(cfg, FILE);
  assert.equal(cfg.hull.leakSensor, false);
  setParamValue(cfg, 'LEAK1_PIN', 27);
  setParamValue(cfg, 'LEAK1_TYPE', 1);
  assert.equal(cfg.hull.leakSensor, true);
  setParamValue(cfg, 'LEAK1_TYPE', -1);
  assert.equal(cfg.hull.leakSensor, false, 'TYPE -1 saved: disabled');
  // AP_LeakDetector::init: an unsaved TYPE defaults from the pin on FMUv3/v5 boards
  assert.equal(leakDetectorFromParams({ LEAK1_PIN: 14 }), true);
  assert.equal(leakDetectorFromParams({ LEAK1_PIN: 14, LEAK1_TYPE: -1 }), false);
  assert.equal(leakDetectorFromParams({ FOO: 1 }), null);
  // survives save / load
  const back = normalizeConfig(JSON.parse(JSON.stringify(cfg)));
  assert.equal(back.hull.leakSensor, false);
});

test('positioning follows the EKF sources: GPS gives no position underwater, ExternalNav does', () => {
  assert.equal(defaultConfig().sensors.positioning, false);
  const cfg = defaultConfig();
  cfg.sensors.positioning = true;
  applyParamFile(cfg, FILE); // EK3_SRC1_POSXY 3 (GPS), VISO_TYPE 0
  assert.equal(cfg.sensors.positioning, false);
  assert.equal(positioningFromParams({ ...defaultParams(), EK3_SRC1_POSXY: 6, VISO_TYPE: 1 }), true);
  const dvl = defaultConfig();
  applyParamFile(dvl, { VISO_TYPE: 1, EK3_SRC1_POSXY: 6, EK3_SRC1_VELXY: 6 });
  assert.equal(dvl.sensors.positioning, true);
  // a user choice is not overridden by a later import
  const u = defaultConfig();
  u.sensors.positioning = true; u.sensors.positioningSource = 'user';
  applyParamFile(u, FILE);
  assert.equal(u.sensors.positioning, true);
});

test('saved vehicles: joystick "Not used" survives a reload; old configs migrate to the full table', () => {
  const cfg = defaultConfig();
  delete cfg.joystick.buttonMap[16];
  const back = normalizeConfig(JSON.parse(JSON.stringify(cfg)));
  assert.ok(!(16 in back.joystick.buttonMap));
  assert.equal(back.joystick.buttonMap[0], 0);
  // a vehicle saved by the previous version (sim params + extraParams)
  const old = { schema: 'duburi.vehicle', params: { FRAME_CONFIG: 2, ATC_RAT_RLL_P: 0.2 }, extraParams: { COMPASS_OFS_X: 306.1, BOARD_THING: 3, ATC_RAT_YAW_FILT: 3 } };
  const n = normalizeConfig(old);
  assert.equal(n.params.COMPASS_OFS_X, 306.1);
  assert.equal(n.params.ATC_RAT_YAW_FLTE, 3);
  assert.deepEqual(n.extraParams, { BOARD_THING: 3 });
  assert.equal(Object.keys(n.params).length, PARAM_DEFS.length);
  const exp = allParamsForExport(n);
  assert.equal(exp.BOARD_THING, 3);
  assert.equal(exp.ATC_RAT_RLL_P, 0.2);
  assert.ok(n.paramNames.includes('COMPASS_OFS_X'));
});

test('export of a vehicle without a file: the reference Pixhawk set plus changed values', () => {
  const cfg = defaultConfig();
  cfg.params.BATT2_MONITOR = 4;
  const exp = allParamsForExport(cfg);
  for (const k of Object.keys(FILE)) assert.ok(k in exp, k);
  assert.equal(exp.BATT2_MONITOR, 4);
  assert.equal(exp.FRAME_CONFIG, 1);
});
