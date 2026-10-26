// Pool & water panel: pool/tank size and depth (flat or sloped floor, any
// size), hydrostatic pressure, the vehicle's pressure housings (rating,
// compression, leaks), water properties, current, turbulence, waves, sensors,
// contact and simulation speed.

import { h, numberField, selectField, vec3Field, seg, checkbox, fmt, fmtSigned, toast } from '../dom.js';
import { waterDensity, kinematicViscosity, airPressureAtAltitude } from '../../hydro/water.js';
import { DEFAULT_POOL, floorProfile } from '../../physics/environment.js';
import { DEFAULT_HULL, DEPTH_SENSORS, depthSensorSpec, leakDetectorFromParams } from '../../vehicle/config.js';

export const POOL_PRESETS = [
  ['tank', 'Test tank · 4 × 3 m, 1.5 m deep', { length: 4, width: 3, depth: 1.5, floor: 'flat', freeboard: 0.3, style: 'tiles' }],
  ['home', 'Home pool · 8 × 4 m, 1.0 → 1.8 m', { length: 8, width: 4, depth: 1.8, floor: 'sloped', shallowDepth: 1.0, shallowEnd: 'south', slopeStart: 3, slopeEnd: 6, freeboard: 0.2, style: 'tiles' }],
  ['pool25s', 'Pool · 25 × 12 m, 1.2 → 2.0 m', { length: 25, width: 12, depth: 2.0, floor: 'sloped', shallowDepth: 1.2, shallowEnd: 'south', slopeStart: 8, slopeEnd: 15, freeboard: 0.3, style: 'tiles' }],
  ['pool25', 'Pool · 25 × 12 m, 4 m deep', { length: 25, width: 12, depth: 4, floor: 'flat', freeboard: 0.3, style: 'tiles' }],
  ['pool50', 'Pool · 50 × 25 m, 3 m deep', { length: 50, width: 25, depth: 3, floor: 'flat', freeboard: 0.3, style: 'tiles' }],
  ['diving', 'Diving well · 20 × 20 m, 5 m deep', { length: 20, width: 20, depth: 5, floor: 'flat', freeboard: 0.5, style: 'tiles' }],
  ['lake', 'Lake · 200 × 200 m, 2 → 30 m', { length: 200, width: 200, depth: 30, floor: 'sloped', shallowDepth: 2, shallowEnd: 'south', slopeStart: 0, slopeEnd: 200, freeboard: 0, style: 'natural' }],
  ['sea', 'Open sea · 500 × 500 m, 300 m deep', { length: 500, width: 500, depth: 300, floor: 'flat', freeboard: 0, style: 'natural' }],
];

function matchPreset(pool) {
  for (const [id, , v] of POOL_PRESETS) if (Object.entries(v).every(([k, x]) => pool[k] === x)) return id;
  return 'custom';
}

const bar = (pa) => `${(pa / 1e5).toFixed(pa < 1e6 ? 4 : 3)} bar`;

// side view of the pool along its length (vertical scale exaggerated when shallow)
function sideView(E, sim) {
  const pool = E.pool, prof = floorProfile(pool);
  const L = prof.L, D = prof.maxDepth, F = Math.max(0, +pool.freeboard || 0);
  const W = 400, H = 150, pad = { l: 34, r: 30, t: 22, b: 18 };
  const sx = (W - pad.l - pad.r) / L, sy = (H - pad.t - pad.b) / (D + F);
  const X = (x) => pad.l + (x + L / 2) * sx, Y = (d) => pad.t + (d + F) * sy;
  const xs = [-L / 2, ...prof.breaks, L / 2];
  const floorPts = xs.map((x) => `${X(x).toFixed(1)},${Y(prof.depthAt(x)).toFixed(1)}`).join(' ');
  const water = `${X(-L / 2)},${Y(0)} ${floorPts} ${X(L / 2)},${Y(0)}`;
  const ground = `${X(-L / 2)},${Y(-F)} ${floorPts} ${X(L / 2)},${Y(-F)} ${X(L / 2) + 10},${Y(-F)} ${X(L / 2) + 10},${H - 4} ${X(-L / 2) - 10},${H - 4} ${X(-L / 2) - 10},${Y(-F)}`;
  const ex = sy / sx;
  const p = sim?.plant?.position;
  const rating = sim?.plant?.m?.hull?.rating;
  const parts = [
    `<polygon points="${ground}" fill="var(--glass-hi)" />`,
    `<polygon points="${water}" fill="rgba(92,200,224,0.22)" />`,
    `<polyline points="${X(-L / 2)},${Y(-F)} ${floorPts} ${X(L / 2)},${Y(-F)}" fill="none" stroke="var(--ink-2)" stroke-width="1.5" />`,
    `<line x1="${X(-L / 2)}" y1="${Y(0)}" x2="${X(L / 2)}" y2="${Y(0)}" stroke="var(--accent)" stroke-width="1.5" />`,
    rating > 0 && rating < D ? `<line x1="${X(-L / 2)}" y1="${Y(rating)}" x2="${X(L / 2)}" y2="${Y(rating)}" stroke="#ff8a8a" stroke-dasharray="5 4" /><text x="${X(L / 2) - 4}" y="${Y(rating) - 4}" text-anchor="end" fill="#ff8a8a">rated ${rating} m</text>` : '',
    `<text x="${(X(-L / 2) + X(L / 2)) / 2}" y="12" text-anchor="middle" fill="var(--ink-2)">${fmt(L, L < 10 ? 1 : 0)} m long · ${fmt(pool.width, pool.width < 10 ? 1 : 0)} m wide</text>`,
    `<text x="${X(-L / 2) - 4}" y="${Y(prof.depthAt(-L / 2)) + 4}" text-anchor="end" fill="var(--ink-2)">${fmt(prof.depthAt(-L / 2), 1)}</text>`,
    `<text x="${X(L / 2) + 4}" y="${Y(prof.depthAt(L / 2)) + 4}" fill="var(--ink-2)">${fmt(prof.depthAt(L / 2), 1)}</text>`,
    `<text x="${X(-L / 2)}" y="${H - 6}" fill="var(--ink-3)">S</text><text x="${X(L / 2)}" y="${H - 6}" text-anchor="end" fill="var(--ink-3)">N</text>`,
    ex > 1.5 ? `<text x="${(X(-L / 2) + X(L / 2)) / 2}" y="${H - 6}" text-anchor="middle" fill="var(--ink-3)">depth drawn ×${ex < 10 ? ex.toFixed(1) : Math.round(ex)}</text>` : '',
    p ? `<circle cx="${X(Math.max(-L / 2, Math.min(L / 2, p[0])))}" cy="${Y(Math.max(-F, Math.min(D, p[2])))}" r="4.5" fill="var(--foam)" stroke="var(--abyss)" stroke-width="1.5" />` : '',
  ];
  return `<svg viewBox="0 0 ${W} ${H}" width="100%" role="img" aria-label="Side view of the pool" style="display:block;font:500 10px var(--font-data)">${parts.join('')}</svg>`;
}

export default {
  id: 'env', title: 'Pool & water', short: 'Pool', icon: 'env', sub: 'Pool size and depth, pressure, water, current, waves, sensors',
  create(app) {
    const cfg = app.cfg;
    const E = cfg.environment;
    E.pool = { ...DEFAULT_POOL, ...E.pool };
    cfg.hull = { ...DEFAULT_HULL, ...(cfg.hull || {}) };
    const HU = cfg.hull;
    const el = h('div', { class: 'section', style: { gap: '18px' } });
    const live = () => app.changed({ immediate: true });
    const poolChanged = (refresh = false) => { app.poolChanged(); if (refresh) app.refreshPanel(); else { presetSel.refresh(); drawView(); } };

    // ---------------- pool ----------------
    const P = E.pool;
    const view = h('div', { style: { border: '1px solid var(--line)', borderRadius: '8px', padding: '6px 4px', background: 'var(--chart-bg)' } });
    const poolNote = h('p', { class: 'note' });
    const drawView = () => {
      view.innerHTML = sideView(E, app.sim);
      const prof = floorProfile(P), env = app.sim?.env;
      const vol = prof.W * (prof.sloped ? (prof.a * prof.shallowDepth + (prof.b - prof.a) * (prof.shallowDepth + prof.deepDepth) / 2 + (prof.L - prof.b) * prof.deepDepth) : prof.L * prof.maxDepth);
      poolNote.textContent = `${vol < 1e4 ? fmt(vol, 0) : (vol / 1e6).toFixed(vol < 1e6 ? 3 : 1) + ' million'} m³ of water. Deepest point ${fmt(prof.maxDepth, 2)} m: ${env ? bar(env.pAtm + env.pressureGauge(prof.maxDepth)) : '—'} absolute.`;
    };
    const presetSel = selectField({
      label: 'Preset', options: [['custom', 'Custom (your numbers)'], ...POOL_PRESETS.map(([id, label]) => [id, label])], get: () => matchPreset(P),
      set: (id) => { const pr = POOL_PRESETS.find((x) => x[0] === id); if (!pr) return; Object.assign(P, pr[2]); poolChanged(true); },
    });
    const sloped = P.floor === 'sloped';
    const dimField = (label, key, o = {}) => numberField({ label, unit: 'm', step: o.step ?? 0.1, min: o.min ?? 0.1, max: o.max ?? 20000, get: () => P[key], set: (v) => { P[key] = v; poolChanged(o.refresh); }, hint: o.hint });
    el.append(h('div', { class: 'section' }, h('h3', {}, 'Pool or tank'),
      h('p', { class: 'note' }, 'Any size and depth: a test tank, your pool, a lake or the sea. The floor, walls, buoyancy, pressure and depth sensor all follow these numbers.'),
      presetSel,
      h('div', { class: 'grid3' },
        dimField('Length (N–S)', 'length', { min: 0.5 }),
        dimField('Width (E–W)', 'width', { min: 0.5 }),
        dimField(sloped ? 'Deep-end depth' : 'Water depth', 'depth', { min: 0.2, max: 11000, refresh: false })),
      h('div', { class: 'field' }, h('span', {}, 'Floor'), seg([['flat', 'Flat'], ['sloped', 'Shallow end + slope']], P.floor, (v) => {
        P.floor = v;
        if (v === 'sloped' && !(P.shallowDepth < P.depth)) { P.shallowDepth = +Math.max(0.2, P.depth * 0.4).toFixed(2); P.slopeStart = +(P.length * 0.3).toFixed(1); P.slopeEnd = +(P.length * 0.6).toFixed(1); }
        poolChanged(true);
      })),
      sloped ? h('div', { class: 'grid2' },
        dimField('Shallow-end depth', 'shallowDepth', { min: 0.05, max: 11000 }),
        selectField({ label: 'Shallow end', options: [['south', 'South wall'], ['north', 'North wall']], get: () => P.shallowEnd, set: (v) => { P.shallowEnd = v; poolChanged(); } }),
        dimField('Slope starts at', 'slopeStart', { min: 0, hint: 'Metres from the shallow-end wall' }),
        dimField('Slope ends at', 'slopeEnd', { min: 0, hint: 'Metres from the shallow-end wall (full depth from here on)' })) : null,
      h('div', { class: 'grid2' },
        numberField({ label: 'Wall above water', unit: 'm', step: 0.05, min: 0, max: 20, get: () => P.freeboard, set: (v) => { P.freeboard = v; poolChanged(); } }),
        h('div', { class: 'field' }, h('span', {}, 'Look'), seg([['tiles', 'Pool tiles'], ['natural', 'Natural bottom']], P.style, (v) => { P.style = v; poolChanged(); }))),
      view, poolNote));

    // ---------------- pressure ----------------
    const pTable = h('div', { class: 'table-wrap' });
    const pressureRows = () => {
      const sim = app.sim;
      if (!sim) return;
      const env = sim.env, s = sim.sensors, plant = sim.plant, hull = plant.m.hull;
      const V = plant.m.volume, rho0 = env.rho, SG = cfg.params.BARO_SPEC_GRAV ?? 1;
      const at = (depth) => {
        const pg = env.pressureGauge(depth), pa = env.pAtm + pg, rho = env.densityAtGauge(pg);
        const dB = ((rho / rho0) * (1 - hull.beta * pg) - 1) * rho0 * V * 1000; // grams of extra lift
        const pr = s.quantize(pa);
        return { pa, rho, dB, ardu: (pr - s.groundPressure) / 9800 / SG, over: pa > s.spec.maxPa };
      };
      const pos = plant.position, dv = Math.max(0, pos[2]);
      const a = at(dv), b = at(env.floor.maxDepth);
      const row = (k, f1, f2) => h('tr', {}, h('td', {}, k), h('td', { class: 'num' }, f1), h('td', { class: 'num' }, f2));
      pTable.replaceChildren(h('table', { class: 'data' },
        h('thead', {}, h('tr', {}, h('th', {}, ''), h('th', { class: 'num' }, 'Vehicle now'), h('th', { class: 'num' }, 'Deepest floor'))),
        h('tbody', {},
          row('True depth', `${fmt(dv, 2)} m`, `${fmt(env.floor.maxDepth, 2)} m`),
          row('Absolute pressure', bar(a.pa), bar(b.pa)),
          row('Water density', `${fmt(a.rho, 2)} kg/m³`, `${fmt(b.rho, 2)} kg/m³`),
          row('Buoyancy vs surface', `${fmtSigned(a.dB, 1)} g`, `${fmtSigned(b.dB, 1)} g`),
          row('Depth ArduSub shows', `${fmt(s.depthMeasured, 2)} m`, b.over ? `${fmt(b.ardu, 2)} m (sensor maxed)` : `${fmt(b.ardu, 2)} m`))));
    };
    el.append(h('div', { class: 'section' }, h('h3', {}, 'Pressure'),
      h('div', { class: 'grid2' },
        numberField({ label: 'Air pressure', unit: 'hPa', step: 0.5, min: 300, max: 1100, get: () => E.airPressure ?? 1013.25, set: (v) => { E.airPressure = v; live(); } }),
        numberField({ label: 'or site elevation', unit: 'm', step: 10, min: -400, max: 5000, get: () => NaN, set: (v) => { E.airPressure = +airPressureAtAltitude(v).toFixed(2); live(); app.refreshPanel(); }, hint: 'Sets the air pressure of the standard atmosphere at this elevation' })),
      pTable,
      h('p', { class: 'note' }, 'Absolute pressure = air pressure + the weight of the water above, with water getting denser as it is compressed (UNESCO EOS-80). ArduSub turns the pressure sensor reading into depth with 9800 Pa per metre × BARO_SPEC_GRAV, so its depth drifts from the true depth when the specific gravity or the water density does not match — exactly as on the real vehicle.')));

    // ---------------- pressure housing ----------------
    const hullStatus = h('div', { class: 'row', style: { gap: '8px', flexWrap: 'wrap' } });
    const showHull = () => {
      const sim = app.sim;
      if (!sim) return;
      const f = sim.plant.flood, hull = sim.plant.m.hull, wet = sim.sensors.leak;
      const air = hull.airVolume * 1000;
      hullStatus.replaceChildren(
        h('span', { class: `pill ${f.volume > 0 ? 'bad' : 'good'}` }, f.volume > 0 ? `${fmt(f.volume * 1e6, 0)} mL of water inside (${fmt((100 * f.volume) / hull.airVolume, 1)}%)` : 'Dry'),
        h('span', { class: `pill ${f.active ? 'bad' : 'muted'}` }, f.active ? (f.cause === 'rating' ? 'Leaking: rated depth exceeded' : 'Leaking') : 'Sealed'),
        h('span', { class: `pill ${wet ? 'bad' : 'muted'}` }, wet ? 'Leak detector wet' : hull.leakSensor ? 'Leak detector dry' : 'No leak detector'),
        h('span', { class: 'pill muted' }, `Inside ${fmt(f.pIn / 100, 1)} hPa`),
        h('span', { class: 'pill muted' }, `${fmt(air, 2)} L of air`));
    };
    const allP = { ...(cfg.extraParams || {}), ...cfg.params };
    const fileDet = leakDetectorFromParams(allP);
    const pinTxt = `LEAK1_PIN = ${allP.LEAK1_PIN ?? '—'}, LEAK1_TYPE = ${allP.LEAK1_TYPE ?? '—'}`;
    const leakNote = fileDet === null ? null : h('p', { class: 'note', style: fileDet === HU.leakSensor ? {} : { color: 'var(--warning)' } },
      fileDet ? `Your parameters configure a leak detector (${pinTxt}).`
        : HU.leakSensor ? `Simulating a detector, but your parameters configure none (${pinTxt}): on the real vehicle ArduSub would not see this leak.`
          : `Your parameters configure no leak detector (${pinTxt}), so ArduSub cannot see a leak. Tick to simulate one anyway.`);
    el.append(h('div', { class: 'section' }, h('h3', {}, 'Pressure housings (your vehicle)'),
      h('div', { class: 'grid2' },
        numberField({ label: 'Rated depth', unit: 'm', step: 5, min: 0, max: 11000, get: () => HU.depthRating, set: (v) => { HU.depthRating = v; live(); }, hint: 'Depth rating of the weakest enclosure (0 = no limit)' }),
        numberField({ label: 'Compressibility', unit: '% per 10 bar', step: 0.05, min: 0, max: 20, get: () => HU.compressibility, set: (v) => { HU.compressibility = v; live(); }, hint: 'Volume the vehicle loses per 10 bar (about 100 m) as foam and housings compress. 0 = rigid.' }),
        numberField({ label: 'Air inside housings', unit: 'L', step: 0.1, min: 0, max: 5000, get: () => HU.airVolume, set: (v) => { HU.airVolume = v; live(); }, hint: '0 = estimate as 20% of the displaced volume' }),
        numberField({ label: 'Leak hole size', unit: 'mm²', step: 0.5, min: 0, max: 5000, get: () => HU.leakArea, set: (v) => { HU.leakArea = v; live(); }, hint: 'Effective hole area once a housing leaks; water flows in at 0.62 × area × √(2 Δp / ρ)' })),
      checkbox('Housings start to leak below the rated depth', () => HU.floodOverRating, (v) => { HU.floodOverRating = v; live(); }),
      checkbox(`Leak detector fitted (FS_LEAK_ENABLE = ${cfg.params.FS_LEAK_ENABLE ?? 1}: ${['disabled', 'warn only', 'surface'][cfg.params.FS_LEAK_ENABLE ?? 1] ?? '?'})`, () => HU.leakSensor, (v) => { HU.leakSensor = v; HU.leakSensorSource = 'user'; live(); app.refreshPanel(); }),
      leakNote,
      hullStatus,
      h('div', { class: 'row' },
        h('button', { class: 'btn', onclick: () => { app.sim.plant.startLeak('manual'); toast('Leak started at the current depth', 'bad'); showHull(); } }, 'Start a leak now'),
        h('button', { class: 'btn', onclick: () => { app.sim.plant.clearFlood(); toast('Housing dried and sealed', 'good'); showHull(); } }, 'Seal and dry')),
      h('p', { class: 'note' }, 'Deeper water squeezes the vehicle (less buoyancy) and gets denser (more buoyancy). Past the rated depth the housing leaks: water flows in until the trapped air is compressed to the outside pressure, the vehicle gets heavier, the leak detector trips FS_LEAK and the rising internal pressure trips FS_PRESS (FS_PRESS_MAX).')));

    // ---------------- water ----------------
    const rhoNote = h('p', { class: 'note' });
    const showRho = () => { rhoNote.textContent = `Surface density ${fmt(E.densityOverride > 0 ? E.densityOverride : waterDensity(E.temperature, E.salinity), 2)} kg/m³ · viscosity ${(kinematicViscosity(E.temperature, E.salinity) * 1e6).toFixed(3)} mm²/s. Set BARO_SPEC_GRAV to match (1.0 fresh, about 1.024 sea) or the depth reading will be off.`; };
    showRho();
    el.append(h('div', { class: 'section' }, h('h3', {}, 'Water'),
      h('div', { class: 'grid3' },
        numberField({ label: 'Temperature', unit: '°C', step: 0.5, min: 0, max: 40, get: () => E.temperature, set: (v) => { E.temperature = v; showRho(); live(); } }),
        numberField({ label: 'Salinity', unit: 'g/kg', step: 0.5, min: 0, max: 42, get: () => E.salinity, set: (v) => { E.salinity = v; showRho(); live(); } }),
        numberField({ label: 'Density override', unit: 'kg/m³', step: 0.1, min: 0, get: () => E.densityOverride, set: (v) => { E.densityOverride = v; showRho(); live(); }, hint: '0 = compute from temperature and salinity' })),
      rhoNote,
      h('div', { class: 'row' },
        h('button', { class: 'btn', onclick: () => { E.salinity = 0; app.setParam('BARO_SPEC_GRAV', 1.0); live(); app.refreshPanel(); } }, 'Fresh water (SPEC_GRAV 1.0)'),
        h('button', { class: 'btn', onclick: () => { E.salinity = 35; app.setParam('BARO_SPEC_GRAV', 1.024); live(); app.refreshPanel(); } }, 'Sea water (35 g/kg, 1.024)')),
      numberField({ label: 'Visibility (visual only)', unit: 'm', step: 1, min: 2, max: 80, get: () => E.visibility ?? 30, set: (v) => { E.visibility = v; app.sim.env.cfg.visibility = v; app.scheduleSave(); } })));

    el.append(h('div', { class: 'section' }, h('h3', {}, 'Current and turbulence'),
      h('div', { class: 'grid3' }, ['North', 'East', 'Down'].map((n, k) => numberField({ label: n, unit: 'm/s', step: 0.02, get: () => E.current[k], set: (v) => { E.current[k] = v; live(); } }))),
      h('div', { class: 'grid2' },
        numberField({ label: 'Turbulence strength (σ)', unit: 'm/s', step: 0.01, min: 0, max: 2, get: () => E.turbulence.sigma, set: (v) => { E.turbulence.sigma = v; live(); } }),
        numberField({ label: 'Turbulence time scale', unit: 's', step: 0.1, min: 0.1, max: 30, get: () => E.turbulence.tau, set: (v) => { E.turbulence.tau = v; live(); } }))));

    el.append(h('div', { class: 'section' }, h('h3', {}, 'Surface waves'),
      h('div', { class: 'grid3' },
        numberField({ label: 'Amplitude', unit: 'm', step: 0.01, min: 0, max: 2, get: () => E.waves.amplitude, set: (v) => { E.waves.amplitude = v; live(); } }),
        numberField({ label: 'Period', unit: 's', step: 0.1, min: 0.5, max: 20, get: () => E.waves.period, set: (v) => { E.waves.period = v; live(); } }),
        numberField({ label: 'Direction', unit: '°', step: 5, get: () => E.waves.direction, set: (v) => { E.waves.direction = v; live(); } })),
      h('p', { class: 'note' }, 'Linear (Airy) deep-water waves: the orbital velocity and the wave pressure decay with depth, so the vehicle feels them near the surface and the depth sensor sees the water level move.')));

    const S = cfg.sensors;
    const spec = depthSensorSpec(S);
    el.append(h('div', { class: 'section' }, h('h3', {}, 'Sensors'),
      h('div', { class: 'grid2' },
        selectField({ label: 'Depth sensor', options: Object.entries(DEPTH_SENSORS).map(([k, v]) => [k, v.label]), get: () => S.depthSensor ?? 'bar30', set: (v) => { S.depthSensor = v; live(); app.refreshPanel(); } }),
        S.depthSensor === 'custom'
          ? h('div', { class: 'grid2' },
            numberField({ label: 'Range', unit: 'bar abs', step: 1, min: 0.5, max: 2000, get: () => S.depthSensorMax ?? 30, set: (v) => { S.depthSensorMax = v; live(); } }),
            numberField({ label: 'Resolution', unit: 'mbar', step: 0.01, min: 0, max: 100, get: () => S.depthSensorRes ?? 0.2, set: (v) => { S.depthSensorRes = v; live(); } }))
          : h('p', { class: 'note', style: { alignSelf: 'end' } }, `Reads up to ${spec.maxBar} bar absolute (about ${fmt(((spec.maxBar * 1e5 - 101325) / 9800), 0)} m), resolution ${spec.resMbar} mbar. Deeper than that the reading stays at its maximum.`)),
      vec3Field({ label: 'Depth sensor position from the CG', get: () => S.depthSensorPos || [0, 0, 0], set: (v) => { S.depthSensorPos = v; live(); }, step: 0.01 }),
      h('div', { class: 'grid2' },
        numberField({ label: 'Gyro noise', unit: 'rad/s', step: 0.0005, min: 0, get: () => S.gyroNoise, set: (v) => { S.gyroNoise = v; live(); } }),
        numberField({ label: 'Accelerometer noise', unit: 'm/s²', step: 0.01, min: 0, get: () => S.accelNoise, set: (v) => { S.accelNoise = v; live(); } }),
        numberField({ label: 'Attitude estimate error', unit: '°', step: 0.01, min: 0, get: () => S.attitudeNoise, set: (v) => { S.attitudeNoise = v; live(); } }),
        numberField({ label: 'Depth sensor noise', unit: 'm', step: 0.001, min: 0, get: () => S.depthNoise, set: (v) => { S.depthNoise = v; live(); } }),
        numberField({ label: 'Depth sensor rate', unit: 'Hz', step: 1, min: 1, max: 400, get: () => S.depthRate, set: (v) => { S.depthRate = v; live(); } })),
      checkbox('Positioning system present (DVL / USBL) — needed for Position Hold', () => S.positioning, (v) => { S.positioning = v; S.positioningSource = 'user'; live(); app.refreshPanel(); }),
      S.positioning ? numberField({ label: 'Position noise', unit: 'm', step: 0.005, min: 0, get: () => S.positionNoise, set: (v) => { S.positionNoise = v; live(); } }) : null,
      h('div', { class: 'row' }, h('button', { class: 'btn', onclick: () => { Object.assign(S, { gyroNoise: 0, accelNoise: 0, attitudeNoise: 0, depthNoise: 0 }); live(); app.refreshPanel(); } }, 'Ideal sensors'),
        h('button', { class: 'btn', onclick: () => { Object.assign(S, { gyroNoise: 0.0015, accelNoise: 0.04, attitudeNoise: 0.02, depthNoise: 0.002 }); live(); app.refreshPanel(); } }, 'Typical Pixhawk + Bar30'))));

    el.append(h('div', { class: 'section' }, h('h3', {}, 'Contact with walls and floor'),
      h('div', { class: 'grid3' },
        numberField({ label: 'Stiffness', unit: 'N/m', step: 1000, min: 1000, get: () => E.contact.stiffness, set: (v) => { E.contact.stiffness = v; live(); } }),
        numberField({ label: 'Damping', unit: 'N·s/m', step: 50, min: 0, get: () => E.contact.damping, set: (v) => { E.contact.damping = v; live(); } }),
        numberField({ label: 'Friction', step: 0.05, min: 0, max: 2, get: () => E.contact.friction, set: (v) => { E.contact.friction = v; live(); } }))));

    el.append(h('div', { class: 'section' }, h('h3', {}, 'Simulation'),
      h('div', { class: 'field' }, h('span', {}, 'Speed'), seg([[0.25, '¼×'], [0.5, '½×'], [1, '1×'], [2, '2×'], [4, '4×'], [10, '10×']], app.sim.timeScale, (v) => { app.sim.timeScale = v; })),
      h('div', { class: 'grid2' },
        numberField({ label: 'Physics rate', unit: 'Hz', step: 100, min: 200, max: 4000, get: () => cfg.sim.physicsRate, set: (v) => { cfg.sim.physicsRate = v; live(); } }),
        numberField({ label: 'Random seed', step: 1, min: 1, get: () => cfg.sim.seed, set: (v) => { cfg.sim.seed = Math.round(v); live(); } })),
      h('p', { class: 'note' }, `Firmware loop ${app.sim.loopHz} Hz (SCHED_LOOP_RATE), ${app.sim.substeps} physics steps per loop with RK4. Simulation costs ${fmt(app.simCost ?? 0, 2)} ms per frame. 10× helps with long dives in deep water.`)));

    drawView(); pressureRows(); showHull();
    return { el, tick: () => { drawView(); pressureRows(); showHull(); }, onModelChanged: () => { pressureRows(); showHull(); } };
  },
};
