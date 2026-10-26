#!/usr/bin/env node
// Generates src/ardusub/paramdefs.js: every parameter ArduSub 4.5.7 exposes,
// with ArduPilot's own metadata, storage type and default.
//
//   node tools/gen-params.mjs [--ardupilot ../ardupilot] [--check apm.pdef.json] [--out src/ardusub/paramdefs.js] [-v]
//   (debug: --cond EXPR evaluates an #if, --macro NAME, --pp FILE:FROM:TO prints preprocessed lines)
// Needs the ArduSub-4.5.7 checkout with all of libraries/ (git sparse-checkout add libraries).
//
// 1. Documentation: a port of Tools/autotest/param_metadata/param_parse.py
//    (@Param/@Group/@Path doc comments, {Sub} vehicle tags, Vector3Parameter,
//    CopyFieldsFrom/CopyValuesFrom). --check compares the names with the JSON
//    param_parse.py writes (python3 param_parse.py --vehicle Sub --format json).
// 2. Storage type and default: the AP_GROUPINFO / GSCALAR entry that follows
//    each doc block, the AP_Int8/Int16/Int32/Float/Vector3f member it points to,
//    and its default expression evaluated with the macros, enums and #if
//    branches of a ChibiOS (Pixhawk) ArduSub build. PID defaults come from the
//    constructor arguments (AP_PARAM_FLAG_DEFAULT_POINTER).
// 3. Sub overrides: ArduSub/Parameters.h defaults_table (non-Navigator branch),
//    set_default calls in ArduSub (radio.cpp dead zones, joystick.cpp buttons,
//    AC_AttitudeControl_Sub angle P) and MOT_THST_HOVER forced to 0.5.
// Parameters whose AP_GROUPINFO_FRAME flags exclude AP_PARAM_FRAME_SUB do not
// exist on ArduSub and are left out, as are SITL-only SIM_* parameters.

import { readFileSync, writeFileSync, existsSync, readdirSync, statSync } from 'node:fs';
import { join, dirname, normalize, basename, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';
import { execSync } from 'node:child_process';

const HERE = dirname(fileURLToPath(import.meta.url));
const arg = (k, d) => { const i = process.argv.indexOf(k); return i > 0 ? process.argv[i + 1] : d; };
const AP = resolve(arg('--ardupilot', process.env.ARDUPILOT || join(HERE, '../../ardupilot')));
const OUT = resolve(arg('--out', join(HERE, '../src/ardusub/paramdefs.js')));
const CHECK = arg('--check', null);
const VEHICLE = 'Sub';
const VEHICLES = ['Rover', 'Sub', 'Copter', 'Plane', 'Tracker', 'AP_Periph', 'Blimp'];
const warn = [];
const TYPES = 'u?int(?:8|16|32|64)_t|float|double|int|unsigned|long|bool|uint|char|int8|uint8|int16|uint16|size_t';
const M_CONST = { M_PI: Math.PI, M_PI_2: Math.PI / 2, GRAVITY_MSS: 9.80665, DEG_TO_RAD: Math.PI / 180, RAD_TO_DEG: 180 / Math.PI, INT8_MAX: 127, INT16_MAX: 32767, INT32_MAX: 2147483647, UINT8_MAX: 255, UINT16_MAX: 65535, UINT32_MAX: 4294967295, FLT_MAX: 3.4028234663852886e38, MAV_SYSTEM_ID: 1, MAV_MOUNT_MODE_RC_TARGETING: 3, MAV_MOUNT_MODE_RETRACT: 0, MAV_MOUNT_MODE_NEUTRAL: 1, MAV_MOUNT_MODE_MAVLINK_TARGETING: 2, MAV_MOUNT_MODE_GPS_POINT: 4, ROTATION_NONE: 0, ROTATION_PITCH_180: 12, ROTATION_YAW_90: 2, ROTATION_YAW_270: 6, ROTATION_ROLL_180: 8, MAV_SEVERITY_ALERT: 1 };

const text = new Map();
const read = (f) => { f = normalize(f); if (!text.has(f)) text.set(f, existsSync(f) ? readFileSync(f, 'utf8').replace(/\r\n?/g, '\n') : null); return text.get(f); };

// ---------------------------------------------------------------- 1. docs
const progParam = /@Param(?:{([^}]+)})?: (\w+).*((?:\n[ \t]*\/\/ @(\w+)(?:{([^}]+)})?: ?(.*))+)(?:\n[ \t\r]*\n|\n[ \t]+[A-Z]|\n--\]\])/g;
const progFields = /[ \t]*\/\/ @(\w+): ?([^\r\n]*)/g;
const progTagged = /[ \t]*\/\/ @(\w+){([^}]+)}: ([^\r\n]*)/g;
const progGroups = /@Group: *(\w+).*((?:\n[ \t]*\/\/ @(Path): (\S+))+)/g;
const KNOWN = new Set(['Description', 'DisplayName', 'Values', 'Range', 'Units', 'Increment', 'User', 'RebootRequired', 'Bitmask', 'Volatile', 'ReadOnly', 'Calibration', 'Vector3Parameter', 'SortValues']);
const NV = new Set(['Values', 'Bitmask']);

const libs = []; // top-level (vehicle) libraries
const allLibs = [];
const vehicleParams = [];
const vdir = join(AP, 'ArduSub');

function newLib(name, parent = null) { return { name, group: name, Path: null, params: [], parent, file: null }; }
function fieldsOf(t) { return [...t.matchAll(progFields)].map((m) => [m[1], m[2]]); }

function parseParamBlock(m, prefix, file, lib) {
  const [, only, pname, ftext] = m;
  if (only) { const l = only.split(',').map((s) => s.trim()); if (!l.includes(VEHICLE)) return []; }
  const p = { name: prefix + pname, short: pname, file, lib, pos: m.index };
  for (const [k, v0] of fieldsOf(ftext)) {
    const v = v0.replace(/@PREFIX@/g, prefix).trimEnd();
    if (KNOWN.has(k)) { if (k in p && NV.has(k)) p[k] += ', ' + v; else p[k] = v; }
    else if (k === 'CopyFieldsFrom' || k === 'CopyValuesFrom') p[k] = v0;
  }
  if (lib) { // library params honour {Vehicle} tags
    let seenThis = false, seenOther = false;
    for (const t of ftext.matchAll(progTagged)) {
      const [, k, veh, val] = t;
      const vs = veh.split(',').map((s) => s.trim());
      if (!KNOWN.has(k)) continue;
      if (!vs.includes(VEHICLE)) { if (vs.length && NV.has(k)) seenOther = true; continue; }
      let append = false;
      if (NV.has(k)) { if (seenThis) append = k in p; seenThis = true; }
      const v = val.replace(/@PREFIX@/g, prefix);
      p[k] = append ? p[k] + ',' + v : v;
    }
    if (p.Values === undefined && p.Bitmask === undefined && seenOther) return [];
  }
  if (p.Vector3Parameter !== undefined) {
    return ['X', 'Y', 'Z'].map((a) => ({ ...p, name: p.name + '_' + a, short: p.short + '_' + a, vec: a, Description: p.Description !== undefined ? `${p.Description} (${a}-axis)` : undefined }));
  }
  return [p];
}

function processVehicle() {
  const file = join(vdir, 'Parameters.cpp');
  const t = read(file);
  for (const g of t.matchAll(progGroups)) {
    const lib = newLib(g[1]);
    for (const [k, v] of fieldsOf(g[2])) if (k === 'Path') lib.Path = v;
    if (!libs.some((l) => l.name === lib.name)) libs.push(lib);
  }
  for (const m of t.matchAll(progParam)) vehicleParams.push(...parseParamBlock(m, '', file, null));
}

function processLibrary(lib, pathprefix = null) {
  for (let path of lib.Path.split(',')) {
    path = path.trim();
    let fname;
    if (pathprefix !== null) fname = join(pathprefix, path);
    else if (!path.includes('/')) fname = join(vdir, path);
    else fname = normalize(join(AP, 'libraries', path));
    const t = path ? read(fname) : null;
    if (t == null) { warn.push(`doc path not found: ${path} (${lib.name})`); continue; }
    lib.files = [...(lib.files || []), fname];
    for (const m of t.matchAll(progParam)) lib.params.push(...parseParamBlock(m, lib.name, fname, lib));
    const done = new Map();
    for (const g of t.matchAll(progGroups)) {
      const group = g[1];
      let sub, append = true;
      if (done.has(group)) { sub = done.get(group); append = false; } else { sub = newLib(group, lib); sub.parentFile = fname; done.set(group, sub); }
      for (const [k, v] of fieldsOf(g[2])) if (k === 'Path') sub.Path = v;
      if (!libs.some((l) => l.name === sub.name)) {
        if (append) sub.name = lib.name + sub.name;
        processLibrary(sub, dirname(fname));
        if (append) allLibs.push(sub);
      }
    }
  }
}

processVehicle();
const veh = newLib(''); veh.Path = '../libraries/AP_Vehicle/AP_Vehicle.cpp';
libs.unshift(veh);
allLibs.push(...libs);
for (const l of libs) if (l.Path) processLibrary(l); else warn.push(`no Path for ${l.name}`);

const docList = [...vehicleParams, ...allLibs.flatMap((l) => l.params)];
const byName = new Map();
for (const p of docList) if (!byName.has(p.name)) byName.set(p.name, p);
for (const p of docList) { // CopyFieldsFrom / CopyValuesFrom
  for (const key of ['CopyValuesFrom', 'CopyFieldsFrom']) {
    if (!p[key]) continue;
    const want = p[key].trim() + (p.vec ? '_' + p.vec : '');
    const src = byName.get(want);
    if (!src) { warn.push(`${p.name}: ${key} ${want} not found`); continue; }
    if (key === 'CopyValuesFrom') p.Values = src.Values;
    else for (const k of Object.keys(src)) if (!(k in p) && KNOWN.has(k)) p[k] = src[k];
    delete p[key];
  }
}

// ---------------------------------------------------------------- 2. C++ sources
function walk(d, out = []) {
  for (const e of readdirSync(d)) {
    const f = join(d, e);
    const s = statSync(f);
    if (s.isDirectory()) { if (!/^(examples|tests|utility_tests|benchmarks|hwdef|SITL|AP_HAL_(SITL|Linux|ESP32|Empty|QURT)|AP_Periph)$/.test(e)) walk(f, out); }
    else if (/\.(h|cpp)$/.test(e) && !/AP_HAL\/board\/(?!chibios\.h)/.test(f)) out.push(f);
  }
  return out;
}
const SRC = [...walk(join(AP, 'libraries')), ...walk(vdir)];

// object-like macros, first raw pass (used to evaluate #if)
const rawMacros = new Map();
const allRaw = new Map(); // every definition, any branch (last resort when evaluating defaults)
const defRe = /^[ \t]*#[ \t]*define[ \t]+([A-Za-z_]\w*)(?![\w(])[ \t]*((?:[^\n\\]|\\\n)*)/gm;
// defined(X) is true when X is #defined somewhere other than inside its own "#ifndef X" guard
const unguarded = new Set();
// is the #define at offset `pos` of file f inside an "#ifndef NAME" (or "#if !defined(NAME)") block?
const dirCache = new Map();
function isGuarded(f, pos, name) {
  if (!dirCache.has(f)) { // open-directive stack at every #define
    const t = read(f), st = [], at = new Map();
    const re = /^[ \t]*#[ \t]*(if|ifdef|ifndef|elif|else|endif|define)\b([^\n]*)/gm;
    let m;
    while ((m = re.exec(t))) {
      if (m[1] === 'if' || m[1] === 'ifdef' || m[1] === 'ifndef') st.push(m[1] + ' ' + m[2].trim());
      else if (m[1] === 'endif') st.pop();
      else if (m[1] === 'define') at.set(m.index, st.slice());
    }
    dirCache.set(f, at);
  }
  const stack = dirCache.get(f).get(pos) || [];
  const re = new RegExp(`^(ifndef\\s+${name}\\b|if\\s+!\\s*defined\\s*\\(?\\s*${name}\\b)`);
  return stack.some((d) => re.test(d));
}
for (const f of SRC) {
  const t = read(f);
  for (const m of t.matchAll(defRe)) {
    if (!rawMacros.has(m[1])) rawMacros.set(m[1], clean(m[2]));
    if (!allRaw.has(m[1])) allRaw.set(m[1], []); allRaw.get(m[1]).push({ v: clean(m[2]), f });
    if (!isGuarded(f, m.index, m[1])) unguarded.add(m[1]);
  }
}
const NOT_DEFINED = new Set(['HAL_BUILD_AP_PERIPH', 'STM32H7', 'HAL_BOOTLOADER_BUILD', 'IOMCU_FW', 'HAL_SITL', 'AP_PERIPH']);
function clean(s) { return s.replace(/\\\n/g, ' ').replace(/\/\*.*?\*\//g, ' ').replace(/\/\/.*$/, '').trim(); }

// build configuration: ChibiOS (Pixhawk), ArduSub, not Navigator
const FIXED = {
  CONFIG_HAL_BOARD: 10, HAL_BOARD_SITL: 3, HAL_BOARD_LINUX: 7, HAL_BOARD_CHIBIOS: 10, HAL_BOARD_ESP32: 12, HAL_BOARD_EMPTY: 99,
  CONFIG_HAL_BOARD_SUBTYPE: 5009, HAL_BOARD_SUBTYPE_LINUX_NAVIGATOR: 1023, HAL_BOARD_SUBTYPE_CHIBIOS_GENERIC: 5009,
  APM_BUILD_DIRECTORY: 7, APM_BUILD_ArduSub: 7, APM_BUILD_COPTER_OR_HELI: 0, HAL_MINIMIZE_FEATURES: 0, BOARD_FLASH_SIZE: 2048,
  HAL_PROGRAM_SIZE_LIMIT_KB: 2048, HAL_WITH_DSP: 1, HAL_NUM_CAN_IFACES: 2, HAL_CANMANAGER_ENABLED: 1, HAL_MAX_CAN_PROTOCOL_DRIVERS: 2,
  AP_SIM_ENABLED: 0, HAL_SIM_GPS_ENABLED: 0, HAL_GCS_ENABLED: 1,
  HAL_LOGGING_ENABLED: 1, HAL_OS_FATFS_IO: 1, HAL_HAVE_IMU_HEATER: 1, HAL_INS_TEMPERATURE_CAL_ENABLE: 1, INS_MAX_INSTANCES: 3, AP_FEATURE_BOARD_DETECT: 1,
};
const condCache = new Map();
function macroNum(name, depth = 0) {
  if (name in FIXED) return FIXED[name];
  if (depth > 12 || !rawMacros.has(name)) return 0;
  const v = rawMacros.get(name);
  if (v === '') return 1;
  const r = evalC(condPrep(v), (id) => macroNum(id, depth + 1));
  return r === null ? 1 : r;
}
function condPrep(expr) {
  return clean(expr)
    .replace(/APM_BUILD_TYPE\s*\(\s*(\w+)\s*\)/g, (_, b) => (b === 'APM_BUILD_ArduSub' ? '1' : '0'))
    .replace(/defined\s*\(\s*(\w+)\s*\)|defined\s+(\w+)/g, (_, a, b) => (NOT_DEFINED.has(a || b) ? '0' : (a || b) in FIXED || unguarded.has(a || b) ? '1' : '0'))
    .replace(/__has_include\s*\([^)]*\)/g, '0');
}
function evalCond(expr) {
  if (condCache.has(expr)) return condCache.get(expr);
  const s = condPrep(expr);
  const r = evalC(s, (id) => macroNum(id));
  const v = r === null ? true : !!r;
  condCache.set(expr, v);
  return v;
}

// blank inactive #if branches with spaces (offsets are kept)
const ppCache = new Map();
function preprocessed(f) {
  if (ppCache.has(f)) return ppCache.get(f);
  const lines = read(f).split('\n');
  const st = []; // {active, taken, parent}
  const act = () => st.every((s) => s.active);
  const out = [];
  for (let i = 0; i < lines.length; i++) {
    let L = lines[i];
    let j = i;
    while (/^\s*#/.test(L) && /\\$/.test(lines[j]) && j + 1 < lines.length) { j++; L = L.replace(/\\$/, ' ') + lines[j]; }
    const m = /^\s*#\s*(if|ifdef|ifndef|elif|else|endif)\b(.*)$/.exec(L);
    if (m) {
      const [, d, rest] = m;
      if (d === 'if' || d === 'ifdef' || d === 'ifndef') {
        const parent = act();
        const c = d === 'if' ? evalCond(rest) : d === 'ifdef' ? evalCond(`defined(${rest.trim()})`) : !evalCond(`defined(${rest.trim()})`);
        st.push({ active: parent && c, taken: c, parent });
      } else if (d === 'elif') { const s = st[st.length - 1]; if (s) { const c = !s.taken && evalCond(rest); s.active = s.parent && c; s.taken = s.taken || c; } }
      else if (d === 'else') { const s = st[st.length - 1]; if (s) { s.active = s.parent && !s.taken; s.taken = true; } }
      else st.pop();
      for (let k = i; k <= j; k++) out.push(' '.repeat(lines[k].length));
      i = j; continue;
    }
    const a = act();
    for (let k = i; k <= j; k++) out.push(a ? lines[k] : ' '.repeat(lines[k].length));
    i = j;
  }
  const r = out.join('\n');
  ppCache.set(f, r);
  return r;
}

// second pass: #if conditions evaluated with the macros of the active branches only
// (e.g. AP_TEMPERATURE_SENSOR_ENABLED is defined three ways under #if)
for (let pass = 0; pass < 2; pass++) {
  const next = new Map(), ung = new Set();
  for (const f of SRC) {
    const t = preprocessed(f);
    for (const m of t.matchAll(defRe)) {
      if (!next.has(m[1])) next.set(m[1], clean(m[2]));
      if (!isGuarded(f, m.index, m[1])) ung.add(m[1]);
    }
  }
  rawMacros.clear(); for (const [k, v] of next) rawMacros.set(k, v);
  unguarded.clear(); for (const k of ung) unguarded.add(k);
  ppCache.clear(); condCache.clear();
}

// macros / constants / enums from the active branches
const macros = new Map(); // name -> [{v, f}]
const addM = (n, v, f) => { if (!macros.has(n)) macros.set(n, []); macros.get(n).push({ v, f }); };
for (const f of SRC) {
  const t = preprocessed(f);
  for (const m of t.matchAll(defRe)) addM(m[1], clean(m[2]), f);
  for (const m of t.matchAll(/\b(?:static\s+)?(?:constexpr|const)\s+(?:static\s+)?(?:u?int\d+_t|float|double|int|unsigned|long)\s+([A-Za-z_]\w*)\s*=\s*([^;{}]+);/g)) addM(m[1], clean(m[2]), f);
  // enums: enum [class] Name [: type] { A = 1, B, ... }  inside class X -> also X::A
  for (const m of t.matchAll(/\benum\s+(?:class\s+|struct\s+)?(\w+)?\s*(?::\s*[\w:]+\s*)?\{([^{}]*)\}/g)) {
    const ename = m[1] || '';
    const owner = enclosingClass(t, m.index);
    let next = 0;
    for (let item of m[2].replace(/\/\*[\s\S]*?\*\//g, '').replace(/\/\/[^\n]*/g, '').split(',')) {
      item = item.trim();
      if (!item || item.startsWith('#')) continue;
      const mm = /^([A-Za-z_]\w*)\s*(?:=\s*([\s\S]+))?$/.exec(item);
      if (!mm) continue;
      let v = next;
      if (mm[2] !== undefined) { const r = evalExpr(mm[2], f, 0, ename, owner); v = r === null ? NaN : r; }
      next = v + 1;
      const val = String(v);
      for (const k of [mm[1], ename && `${ename}::${mm[1]}`, owner && `${owner}::${mm[1]}`, owner && ename && `${owner}::${ename}::${mm[1]}`]) if (k) addM(k, Number.isFinite(v) ? val : 'NaN', f);
    }
  }
}
function enclosingClass(t, pos) {
  // nearest preceding "class X" / "struct X" whose brace is still open at pos
  const head = t.slice(Math.max(0, pos - 60000), pos);
  const re = /\b(?:class|struct)\s+(\w+)[^;{(]*\{/g;
  let m, best = null;
  while ((m = re.exec(head))) {
    let depth = 0;
    for (let i = m.index + m[0].length - 1; i < head.length; i++) { if (head[i] === '{') depth++; else if (head[i] === '}') { depth--; if (depth === 0) break; } }
    if (depth > 0) best = m[1];
  }
  return best;
}

// C expression evaluator (numbers, macros, enums, casts, arithmetic)
function evalC(expr, lookup) {
  let s = String(expr).replace(/\/\*.*?\*\//g, '').replace(/\/\/.*$/, '').trim();
  if (!s) return null;
  s = s.replace(/static_cast<[^>]+>\s*\(/g, '(').replace(new RegExp(`\\(\\s*(?:const\\s+)?(?:${TYPES})(?:\\s+(?:${TYPES}))*\\s*\\)`, 'g'), '')
    .replace(new RegExp(`\\b(?:${TYPES})\\s*\\(`, 'g'), '(').replace(/\btrue\b/g, '1').replace(/\bfalse\b/g, '0');
  const toks = s.match(/0[xX][0-9a-fA-F]+[uUlL]*|(?:\d+\.?\d*|\.\d+)(?:[eE][-+]?\d+)?[fFuUlL]*|[A-Za-z_]\w*(?:\s*::\s*\w+)*|<<|>>|<=|>=|==|!=|&&|\|\||\S/g) || [];
  let js = '';
  for (let i = 0; i < toks.length; i++) {
    const t = toks[i];
    if (/^0[xX]/.test(t)) js += t.replace(/[uUlL]+$/, '');
    else if (/^[\d.]/.test(t)) js += t.replace(/[fFuUlL]+$/, '');
    else if (/^[A-Za-z_]/.test(t)) {
      if (toks[i + 1] === '(') {
        const fn = { MIN: 'Math.min', MAX: 'Math.max', fabsf: 'Math.abs', radians: '((x)=>x*Math.PI/180)', degrees: '((x)=>x*180/Math.PI)', sq: '((x)=>x*x)' }[t];
        if (!fn) return null;
        js += fn; continue;
      }
      const v = lookup(t.replace(/\s+/g, ''));
      if (v === null || v === undefined || Number.isNaN(v)) return null;
      js += `(${v})`;
    } else if ('+-*/%()<>|&^~!?:,'.includes(t) || ['<<', '>>', '<=', '>=', '==', '!=', '&&', '||'].includes(t)) js += t;
    else return null;
  }
  try { const r = Function(`"use strict";return (${js});`)(); return typeof r === 'boolean' ? +r : Number.isFinite(r) ? r : null; } catch { return null; }
}
const fallbackUsed = new Set();
function evalExpr(expr, file, depth = 0, scope = '', owner = '') {
  if (depth > 15) return null;
  return evalC(expr, (id) => {
    if (id in FIXED) return FIXED[id];
    if (id in M_CONST) return M_CONST[id];
    const parts = id.split('::');
    const cands = [id, parts.slice(-2).join('::'), scope && `${scope}::${parts.at(-1)}`, owner && `${owner}::${parts.at(-1)}`, parts.at(-1)].filter(Boolean);
    for (const c of cands) {
      const L = macros.get(c);
      if (!L) continue;
      const pick = L.find((x) => x.f === file) || L.find((x) => dirname(x.f) === dirname(file)) || L.find((x) => x.f.includes('/ArduSub/')) || L[0];
      const r = evalExpr(pick.v, pick.f, depth + 1);
      if (r !== null) return r;
      for (const x of L) { if (x === pick) continue; const r2 = evalExpr(x.v, x.f, depth + 1); if (r2 !== null) return r2; }
    }
    for (const c of cands) { // defined only in a branch this build does not take
      const L = allRaw.get(c);
      if (!L) continue;
      const pick = L.find((x) => x.f === file) || L.find((x) => dirname(x.f) === dirname(file)) || L[0];
      const r = evalExpr(pick.v, pick.f, depth + 1);
      if (r !== null) { fallbackUsed.add(c); return r; }
    }
    return null;
  });
}

// split "a, f(b, c), d" at top-level commas
function splitArgs(s) {
  const out = []; let d = 0, cur = '';
  for (const ch of s) {
    if ('([{<'.includes(ch) && !(ch === '<' && !/static_cast|AP_Enum/.test(cur))) d++;
    if (')]}'.includes(ch) || (ch === '>' && d > 0 && /static_cast|AP_Enum/.test(cur))) d--;
    if (ch === ',' && d === 0) { out.push(cur.trim()); cur = ''; } else cur += ch;
  }
  if (cur.trim()) out.push(cur.trim());
  return out;
}
function callArgs(t, open) { // t[open] === '('
  let d = 0;
  for (let i = open; i < t.length; i++) { if (t[i] === '(' || t[i] === '{') d++; else if (t[i] === ')' || t[i] === '}') { d--; if (d === 0) return { args: splitArgs(t.slice(open + 1, i)), end: i }; } }
  return null;
}

// AP_GROUPINFO / GSCALAR / AP_SUBGROUPINFO entries per file
const entryCache = new Map();
function entries(f, raw = false) {
  const key = raw ? f + '#raw' : f;
  if (entryCache.has(key)) return entryCache.get(key);
  const t = raw ? read(f) : preprocessed(f);
  const out = [];
  const re = /\b(AP_GROUPINFO(?:_FLAGS_DEFAULT_POINTER|_FLAGS_FRAME|_FLAGS|_FRAME)?|GSCALAR|ASCALAR|AP_SUBGROUPINFO|AP_SUBGROUPPTR|AP_SUBGROUPVARPTR|AP_SUBGROUPEXTENSION|GOBJECT|GOBJECTN|GGROUP|GOBJECTPTR)\s*\(/g;
  for (const m of t.matchAll(re)) {
    if (/#\s*define\s*$/.test(t.slice(Math.max(0, m.index - 20), m.index)) || /define\s+\w*$/.test(t.slice(m.index - 40, m.index))) continue;
    const c = callArgs(t, m.index + m[0].length - 1);
    if (!c) continue;
    const a = c.args, k = m[1];
    const e = { kind: k, pos: m.index, file: f };
    if (k === 'GSCALAR' || k === 'ASCALAR') Object.assign(e, { name: unq(a[1]), element: a[0], cls: k === 'GSCALAR' ? 'Parameters' : null, def: a[2] });
    else if (k.startsWith('AP_GROUPINFO')) {
      Object.assign(e, { name: unq(a[0]), cls: a[2], element: a[3], def: a[4] });
      if (k === 'AP_GROUPINFO_FLAGS' || k === 'AP_GROUPINFO_FLAGS_FRAME') e.flags = a[5];
      if (k === 'AP_GROUPINFO_FRAME') e.frame = a[5];
      if (k === 'AP_GROUPINFO_FLAGS_FRAME') e.frame = a[6];
      if (k === 'AP_GROUPINFO_FLAGS_DEFAULT_POINTER') e.defPtr = true;
    } else if (k === 'AP_SUBGROUPINFO' || k === 'AP_SUBGROUPPTR') Object.assign(e, { sub: true, element: a[0], name: unq(a[1]), cls: a[4] });
    else if (k === 'AP_SUBGROUPVARPTR') Object.assign(e, { sub: true, element: a[0], name: unq(a[1]), cls: null });
    else if (k === 'AP_SUBGROUPEXTENSION') Object.assign(e, { sub: true, element: null, name: unq(a[0]), cls: a[2] });
    else if (k === 'GOBJECTN') Object.assign(e, { sub: true, element: a[0], name: unq(a[2]), cls: a[3] });
    else Object.assign(e, { sub: true, element: a[0], name: unq(a[1]), cls: a[2] });
    if (e.name !== null) out.push(e);
  }
  entryCache.set(key, out);
  return out;
}
function unq(s) { const m = /^"([^"]*)"$/.exec((s || '').trim()); return m ? m[1] : null; }

// member declarations -> storage type
const decl = new Map(); // member -> [{type, f, cls}]
for (const f of SRC) {
  if (!f.endsWith('.h') && !f.endsWith('Parameters.h')) continue;
  const t = read(f);
  for (const m of t.matchAll(/\b(?:RestrictIDTypeArray<\s*)?AP_(Int8|Int16|Int32|Float|Vector3f|Enum16|Enum)\b(?:<[^>;]*>)?(?:\s*,[^>;]*>)?\s+([^;()]+);/g)) {
    const type = { Int8: 'INT8', Enum: 'INT8', Int16: 'INT16', Enum16: 'INT16', Int32: 'INT32', Float: 'REAL32', Vector3f: 'VEC3' }[m[1]];
    const cls = enclosingClass(t, m.index);
    for (let n of m[2].split(',')) {
      n = n.replace(/\[.*$/, '').replace(/=.*$/, '').trim();
      if (!/^\w+$/.test(n)) continue;
      if (!decl.has(n)) decl.set(n, []);
      decl.get(n).push({ type, f, cls });
    }
  }
}
function typeOf(element, cls, file) {
  const el = String(element || '').replace(/\[[^\]]*\]/g, '').replace(/\._priv_instance$/, '').split(/\.|->/).pop().trim();
  const L = decl.get(el);
  if (!L) return null;
  const pick = L.find((d) => d.cls === cls) || L.find((d) => dirname(d.f) === dirname(file)) || L[0];
  return pick.type;
}

// class name -> file holding its var_info
const varInfoFile = new Map();
for (const f of SRC) for (const m of read(f).matchAll(/AP_Param::GroupInfo\s+(\w+)::(\w+)\s*\[\s*\]/g)) if (!varInfoFile.has(`${m[1]}::${m[2]}`)) varInfoFile.set(`${m[1]}::${m[2]}`, f);

// default of an AP_PARAM_FLAG_DEFAULT_POINTER member: the constructor argument
function ctorDefault(cls, defMember, instance, hostFile) {
  const cf = varInfoFile.get(`${cls}::var_info`);
  if (!cf) return null;
  const hdr = cf.replace(/\.cpp$/, '.h');
  const ct = [existsSync(hdr) ? read(hdr) : '', read(cf)].join('\n');
  // which constructor parameter initialises default_xxx
  const ini = new RegExp(`\\b${defMember}\\s*\\(\\s*([\\w.]+)\\s*\\)`).exec(ct);
  if (!ini) return null;
  const pname = ini[1];
  // instantiation: ", _pid_rate_roll(args)" or ": _p_pos_z(args)"
  const el = String(instance).replace(/\[.*$/, '').trim();
  const cands = [hostFile, hostFile.replace(/\.cpp$/, '.h')].filter((f) => existsSync(f));
  for (const f of [...cands, ...readdirSync(dirname(hostFile)).map((x) => join(dirname(hostFile), x))]) {
    if (!/\.(cpp|h)$/.test(f)) continue;
    const t = preprocessed(f);
    const re = new RegExp(`(?:[:,]|\\n)\\s*${el.replace(/[.*+?^${}()|[\]\\]/g, '\\$&')}\\s*([({])`, 'g');
    for (const m of t.matchAll(re)) {
      const c = callArgs(t, m.index + m[0].length - 1);
      if (!c) continue;
      let args = c.args;
      if (pname.startsWith('defaults.')) { // AC_PID(const Defaults &defaults): Defaults{ .p = X, ... }
        const field = pname.slice(9);
        const body = args.join(',');
        const fm = new RegExp(`\\.${field}\\s*=\\s*([^,}]+)`).exec(body);
        return fm ? { expr: fm[1], file: f } : { expr: '0', file: f };
      }
      // positional: find the constructor signature with pname
      const sigs = [...ct.matchAll(new RegExp(`\\b${cls}\\s*\\(([^)]*\\b${pname}\\b[^)]*)\\)`, 'g'))].map((x) => splitArgs(x[1]));
      if (!sigs.length) return null;
      const ps = sigs.find((q) => q.some((a) => new RegExp(`\\b${pname}\\b\\s*=`).test(a))) || sigs[0];
      const idx = ps.findIndex((q) => new RegExp(`\\b${pname}\\b`).test(q));
      if (idx < 0) return null;
      if (args[idx] !== undefined) return { expr: args[idx], file: f };
      const dm = /=\s*(.+)$/.exec(ps[idx]);
      return dm ? { expr: dm[1], file: cf } : null;
    }
  }
  return null;
}

// set_default() calls on members inside a class constructor, e.g. _p_angle_roll.kP().set_default(X)
const ACCESSOR = { kP: 'P', kI: 'I', kD: 'D', ff: 'FF', kIMAX: 'IMAX', filt_T_hz: 'FLTT', filt_E_hz: 'FLTE', filt_D_hz: 'FLTD' };

// ---------------------------------------------------------------- resolve each documented param
function findEntry(files, short, docPos, docFile) {
  return findEntry1(files, short, docPos, docFile, false) || findEntry1(files, short, docPos, docFile, true);
}
function findEntry1(files, short, docPos, docFile, raw) {
  for (const f of files) {
    const E = entries(f, raw).filter((e) => !e.sub && (e.name === short || (e.name + '_X' === short || e.name + '_Y' === short || e.name + '_Z' === short)));
    if (!E.length) continue;
    const after = f === docFile ? E.filter((e) => e.pos >= docPos) : [];
    return (after.length ? after : E).sort((a, b) => a.pos - b.pos)[0];
  }
  return null;
}
function libChain(lib) { const c = []; for (let l = lib; l; l = l.parent) c.unshift(l); return c; }

const defs = [];
const unresolved = [];
for (const p of docList) {
  if (/^SIM_/.test(p.name)) continue;
  const files = [p.file, ...(p.lib?.files || []).filter((f) => f !== p.file)];
  let e = findEntry(files, p.short, p.pos, p.file);
  let viaSub = null;
  if (!e) { // e.g. ATC_RAT_RLL_P documented next to AP_SUBGROUPINFO(_pid_rate_roll, "RAT_RLL_", ...)
    for (const s of [...entries(p.file), ...entries(p.file, true)].filter((x) => x.sub && x.name && p.short.startsWith(x.name) && x.cls)) {
      const vf = varInfoFile.get(`${s.cls}::var_info`);
      if (!vf) continue;
      const ee = findEntry([vf], p.short.slice(s.name.length), 0, null);
      if (ee) { e = ee; viaSub = s; break; }
    }
  }
  if (!e && p.short === 'ENABLE') e = null;
  const d = { name: p.name, doc: p };
  if (e) {
    d.entry = e;
    if (e.frame) {
      const fl = evalExpr(e.frame, e.file);
      if (fl !== null && !(fl & 8)) { d.skipFrame = true; }
    }
    d.type = e.kind === 'GSCALAR' || e.kind === 'ASCALAR' ? typeOf(e.element, 'Parameters', join(vdir, 'Parameters.h')) : typeOf(e.element, e.cls, e.file);
    if (d.type === 'VEC3') { d.type = 'REAL32'; }
    // default
    let dex = e.def, dfile = e.file;
    if (e.defPtr) {
      let host = viaSub, hostFile = viaSub ? p.file : null;
      if (!host && p.lib) { // documented through @Group: find AP_SUBGROUPINFO(element, "<group>") in the parent's file
        const pf = p.lib.parentFile;
        if (pf) { host = entries(pf).find((x) => x.sub && x.name === p.lib.group); hostFile = pf; }
      }
      const r = host ? ctorDefault(e.cls, e.def, host.element, hostFile) : null;
      if (r) { dex = r.expr; dfile = r.file; } else dex = null;
      // instance.kP().set_default(X) in the host constructor
      if (host && hostFile) {
        const leaf = p.short.slice(viaSub ? host.name.length : 0);
        for (const f of [hostFile, ...readdirSync(dirname(hostFile)).map((x) => join(dirname(hostFile), x))].filter((x) => /\.cpp$/.test(x))) {
          const t = preprocessed(f);
          for (const m of t.matchAll(new RegExp(`\\b${host.element}\\.(\\w+)\\(\\)\\.set_default\\(([^;]+)\\);`, 'g'))) if (ACCESSOR[m[1]] === leaf) { dex = m[2].replace(/\)\s*$/, ''); dfile = f; }
        }
      }
    }
    if (dex != null) {
      const vm = /^Vector3f\s*\((.*)\)$/.exec(dex.trim());
      if (vm && p.vec) dex = splitArgs(vm[1])['XYZ'.indexOf(p.vec)] ?? '0';
      d.default = evalExpr(dex, dfile);
      if (d.default === null) d.defExpr = dex;
    }
  }
  if (d.default == null) unresolved.push(`${p.name}${e ? ` = ${d.defExpr ?? '?'}` : ' (no AP_GROUPINFO)'}`);
  defs.push(d);
}

// ---------------------------------------------------------------- 3. Sub overrides
const subH = preprocessed(join(vdir, 'Parameters.h'));
const tbl = /defaults_table\[\]\s*=\s*\{([\s\S]*?)\n\};/.exec(subH)[1];
const SUBDEF = {};
for (const m of tbl.matchAll(/\{\s*"(\w+)"\s*,\s*([^}]+)\}/g)) SUBDEF[m[1]] = evalExpr(m[2].replace(/\/\/.*$/gm, ''), join(vdir, 'Parameters.h'));
// ArduSub/radio.cpp: set_default_dead_zone on roll, pitch, throttle, yaw, forward, lateral (RCMAP 1..6)
const radio = read(join(vdir, 'radio.cpp'));
const chMap = { roll: 1, pitch: 2, throttle: 3, yaw: 4, forward: 5, lateral: 6 };
for (const m of radio.matchAll(/channel_(\w+)->set_default_dead_zone\((\d+)\)/g)) SUBDEF[`RC${chMap[m[1]]}_DZ`] = +m[2];
// ArduSub/joystick.cpp default_js_buttons(): BTN0..15 FUNCTION / SFUNCTION
const js = read(join(vdir, 'joystick.cpp'));
const btnEnum = {};
for (const m of read(join(AP, 'libraries/AP_JSButton/AP_JSButton.h')).matchAll(/\b(k_\w+)\s*=\s*(\d+)/g)) btnEnum[m[1]] = +m[2];
const jsTbl = /default_js_buttons[\s\S]*?=\s*\{([\s\S]*?)\};/.exec(js)[1];
[...jsTbl.matchAll(/\{\s*JSButton::button_function_t::(k_\w+)\s*,\s*JSButton::button_function_t::(k_\w+)\s*\}/g)].forEach((m, i) => { SUBDEF[`BTN${i}_FUNCTION`] = btnEnum[m[1]]; SUBDEF[`BTN${i}_SFUNCTION`] = btnEnum[m[2]]; });
// ArduSub/Parameters.cpp:801 AP_Param::set_by_name("MOT_THST_HOVER", 0.5) (forced, not only a default)
SUBDEF.MOT_THST_HOVER = 0.5;
// ArduSub/system.cpp:44-59: external baro bus default 1 on Pixhawk-class boards
SUBDEF.BARO_EXT_BUS ??= 1;
// AP_BoardConfig.cpp:101 heater{{HAL_IMUHEAT_P_DEFAULT, HAL_IMUHEAT_I_DEFAULT, 70}} (AC_PI brace-init)
const bcf = join(AP, 'libraries/AP_BoardConfig/AP_BoardConfig.cpp');
Object.assign(SUBDEF, { BRD_HEAT_P: evalExpr('HAL_IMUHEAT_P_DEFAULT', bcf), BRD_HEAT_I: evalExpr('HAL_IMUHEAT_I_DEFAULT', bcf), BRD_HEAT_IMAX: 70 });

for (const d of defs) if (d.name in SUBDEF && SUBDEF[d.name] !== null) { d.default = SUBDEF[d.name]; d.subDefault = true; }

// ---------------------------------------------------------------- output
const seen = new Set();
const final = [];
const docOnly = []; // documented, but no AP_GROUPINFO behind it (removed/renamed parameters)
for (const d of defs) {
  if (d.skipFrame || seen.has(d.name)) continue;
  if (!d.entry) { docOnly.push(d.name); continue; }
  seen.add(d.name);
  final.push(d);
}
final.sort((a, b) => (a.name < b.name ? -1 : 1));

const enumTables = []; const enumIdx = new Map();
function table(s, bitmask) {
  if (!s) return -1;
  const o = [];
  for (const part of s.split(',')) {
    const i = part.indexOf(':');
    if (i < 0) continue;
    const k = part.slice(0, i).trim(), v = part.slice(i + 1).trim();
    if (k === '' || !Number.isFinite(+k)) continue;
    o.push([+k, v]);
  }
  if (!o.length) return -1;
  const key = (bitmask ? 'b' : 'v') + JSON.stringify(o);
  if (!enumIdx.has(key)) { enumIdx.set(key, enumTables.length); enumTables.push(o); }
  return enumIdx.get(key);
}
const T = { INT8: 2, INT16: 4, INT32: 6, REAL32: 9 };
const guessType = (d) => {
  const p = d.doc;
  if (p.Values || p.Bitmask) return 'INT8';
  return Number.isInteger(d.default) && p.Increment && Number.isInteger(+p.Increment) ? 'INT16' : 'REAL32';
};
const rows = [];
// names present on the reference vehicle (a Pixhawk running ArduSub 4.5.7): flag 128
const PROFILE = resolve(arg('--profile', join(HERE, '../profiles/duburi_pixhawk_param.param')));
const onRef = new Set(existsSync(PROFILE) ? readFileSync(PROFILE, 'utf8').split(/\r?\n/).map((l) => l.split(/[,\s]+/)[0]).filter((n) => /^[A-Z]/.test(n)) : []);
let noType = 0; const guessed = [];
for (const d of final) {
  const p = d.doc;
  let type = d.type;
  if (!type) { type = guessType(d); noType++; guessed.push(d.name); }
  let def = d.default ?? 0;
  if (type !== 'REAL32') def = Math.trunc(def); else def = +(+def).toPrecision(9);
  const rg = (p.Range || '').trim().split(/\s+/).map(Number);
  const flags = (p.RebootRequired && /true/i.test(p.RebootRequired) ? 1 : 0) | (p.ReadOnly && /true/i.test(p.ReadOnly) ? 2 : 0) | (p.User && /Advanced/i.test(p.User) ? 4 : 0) | (d.default == null ? 8 : 0) | (p.Volatile && /true/i.test(p.Volatile) ? 16 : 0) | (p.Calibration ? 32 : 0) | (d.subDefault ? 64 : 0) | (onRef.has(d.name) ? 128 : 0);
  rows.push([d.name, def, T[type], p.Units || '', rg.length === 2 && rg.every(Number.isFinite) ? rg[0] : null, rg.length === 2 && rg.every(Number.isFinite) ? rg[1] : null,
    p.Increment && Number.isFinite(+p.Increment) ? +p.Increment : null, (p.DisplayName || '').trim(), (p.Description || '').trim(), table(p.Values), table(p.Bitmask, true), flags]);
}

const header = `// GENERATED by tools/gen-params.mjs from ArduPilot ${gitDesc()} (ArduSub, ChibiOS build). Do not edit.
// Every parameter ArduSub 4.5.7 exposes: [name, default, MAV_PARAM_TYPE (2 INT8, 4 INT16, 6 INT32, 9 REAL32),
// units, min, max, increment, display name, description, values table, bitmask table, flags]
// flags: 1 reboot required, 2 read-only, 4 advanced, 8 default not resolved from source, 16 volatile, 32 calibration,
// 64 Sub default override, 128 present on the reference Pixhawk vehicle (profiles/duburi_pixhawk_param.param).
`;
const body = `${header}export const ENUMS = ${JSON.stringify(enumTables)};\nexport const ROWS = [\n${rows.map((r) => JSON.stringify(r)).join(',\n')}\n];\n`;
writeFileSync(OUT, body);

console.log(`documented ${docList.length}, defined ${rows.length} (frame-excluded ${defs.filter((d) => d.skipFrame).length}, SIM_ skipped), type guessed ${noType}, unresolved defaults ${unresolved.length}, enum tables ${enumTables.length}, ${(body.length / 1024).toFixed(0)} KiB -> ${OUT}`);
if (process.argv.includes('-v')) { console.log(unresolved.join('\n')); console.log(warn.join('\n')); console.log('doc-only (dropped): ' + docOnly.join(' ')); console.log('type guessed: ' + guessed.join(' ')); }
if (CHECK) {
  const j = JSON.parse(readFileSync(CHECK, 'utf8'));
  const ref = new Set(); for (const g of Object.keys(j)) if (g !== 'json') for (const k of Object.keys(j[g])) if (!/^SIM_/.test(k)) ref.add(k);
  const mine = new Set(docList.map((p) => p.name).filter((n) => !/^SIM_/.test(n)));
  const a = [...ref].filter((n) => !mine.has(n)), b = [...mine].filter((n) => !ref.has(n));
  console.log(`check vs param_parse.py: ${ref.size} reference, missing ${a.length}: ${a.slice(0, 40).join(' ')}; extra ${b.length}: ${b.slice(0, 40).join(' ')}`);
}
function gitDesc() { try { return 'ArduSub-4.5.7 ' + execSync('git rev-parse --short HEAD', { cwd: AP }).toString().trim(); } catch { return 'ArduSub-4.5.7'; } }
for (let i = 0; i < process.argv.length; i++) if (process.argv[i] === '--cond') console.log('cond', process.argv[i + 1], evalCond(process.argv[i + 1]), macroNum(process.argv[i + 1]));
for (let i = 0; i < process.argv.length; i++) if (process.argv[i] === '--pp') { const [f, a, b] = process.argv[i + 1].split(':'); console.log(preprocessed(join(AP, f)).split('\n').slice(+a - 1, +b).join('\n')); }
for (let i = 0; i < process.argv.length; i++) if (process.argv[i] === '--macro') console.log(macros.get(process.argv[i + 1]));
