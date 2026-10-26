// Application shell: layout, team profiles, persistence, geometry pipeline,
// simulation loop and the panel system.

import { h, icon, toast, confirmDialog, storage, idbGet, idbPut, clear, pickFile } from './dom.js';
import { Scene3D } from './scene3d.js';
import { HUD } from './hud.js';
import { InputManager, shortName } from './input.js';
import { Jobs } from './jobs.js';
import { GraphDock } from './dock.js';
import { PANELS } from './panels/index.js';
import { Simulation } from '../sim/simulation.js';
import { MODE, MODE_LABEL, SIM_MODES } from '../ardusub/sub.js';
import { defaultConfig, normalizeConfig, applyParamFile, thrusterFromPreset, setParamValue } from '../vehicle/config.js';
import { parseParamFile } from '../ardusub/params.js';
import { sampleGeometry } from '../vehicle/sample.js';
import { parseSTL, writeBinarySTL } from '../geometry/stl.js';
import { parseMesh, DESIGN_ACCEPT, DESIGN_EXTENSIONS } from '../geometry/formats.js';
import { makeModelTransform, transformPositions, bbox, guessUnits } from '../geometry/mesh.js';
import { detectThrusters } from '../geometry/thrusterDetect.js';
import { thrusterNotice } from './thrusterNotice.js';
import { cgOf } from '../vehicle/model.js';
import { shapeGeometry, DEFAULT_SHAPE } from '../vehicle/shapes.js';
import { waterDensity } from '../hydro/water.js';
import DUBURI_PARAMS from '../../profiles/duburi_pixhawk_param.param';

const KEY = 'duburi.v1';
const CAMERAS = [['chase', 'Chase'], ['orbit', 'Orbit'], ['fpv', 'Camera'], ['top', 'Top'], ['free', 'Free']];

export class App {
  constructor(root) {
    this.root = root;
    this.jobs = new Jobs();
    this.toast = toast;
    this.panelOpen = null;
    this.last = performance.now();
    this.frame = 0;
    this.saveTimer = null;
    this.rebuildTimer = null;
  }

  // ------------------------------------------------------------------ boot
  async start() {
    this.buildLayout();
    this.scene = new Scene3D(this.viewport);
    this.hud = new HUD(this.hudHost);
    this.input = new InputManager(this);
    this.input.bindStick(this.stickL, 'left');
    this.input.bindStick(this.stickR, 'right');
    this.loadTeams();
    await this.loadSession();
    this.applyTeamBranding();
    this.renderRail();
    requestAnimationFrame((t) => this.loop(t));
    this.onResize();
    window.addEventListener('resize', () => this.onResize());
    // drop a design file anywhere on the page
    this.root.addEventListener('dragover', (e) => { if ([...(e.dataTransfer?.items || [])].some((i) => i.kind === 'file')) { e.preventDefault(); this.root.classList.add('dropping'); } });
    this.root.addEventListener('dragleave', (e) => { if (e.target === this.root || !this.root.contains(e.relatedTarget)) this.root.classList.remove('dropping'); });
    this.root.addEventListener('drop', (e) => {
      this.root.classList.remove('dropping');
      const f = e.dataTransfer?.files?.[0];
      if (!f || e.defaultPrevented) return;
      e.preventDefault();
      if (DESIGN_EXTENSIONS.some((x) => f.name.toLowerCase().endsWith(x))) this.loadDesignFile(f).then((ok) => ok && this.refreshPanel());
      else toast('Drop an STL, OBJ or PLY file of your vehicle', 'bad');
    });
    this.welcome();
  }

  // first visit of a team with the sample vehicle: invite them to load their design
  welcome() {
    const key = `${KEY}.welcomed.${this.team.id}`;
    if (storage.get(key, false) || this.cfg.model.source !== 'sample') return;
    storage.set(key, true);
    const back = h('div', { class: 'modal-back' });
    const close = () => back.remove();
    back.append(h('div', { class: 'modal', role: 'dialog', 'aria-modal': 'true' },
      h('h3', {}, 'Welcome to the DUBURI simulator'),
      h('p', { class: 'note' }, 'Load the 3D design of your vehicle (STL, OBJ or PLY, from SolidWorks, Fusion, Onshape, FreeCAD…). The simulator measures its volume, buoyancy, centre of buoyancy and inertia, checks which motions your thrusters can make, and the AUTO-TUNE button computes its hydrodynamics and ArduSub gains for it.'),
      h('p', { class: 'note' }, 'Until then you are flying the sample 8-thruster ROV with your DUBURI Pixhawk parameters.'),
      h('div', { class: 'row', style: { justifyContent: 'flex-end' } },
        h('button', { class: 'btn', onclick: close }, 'Fly the sample first'),
        h('button', { class: 'btn primary', onclick: () => { close(); this.pickDesign(); } }, 'Upload my design'))));
    this.root.append(back);
  }

  async pickDesign() {
    const f = await pickFile(DESIGN_ACCEPT);
    if (!f) return;
    const ok = await this.loadDesignFile(f);
    if (ok) { this.openPanel('vehicle'); }
  }

  openAutoTune() { this.openPanel(this.panelOpen === 'autotune' ? null : 'autotune'); }

  // ------------------------------------------------------------------ layout
  buildLayout() {
    const r = this.root;
    clear(r);
    this.viewport = h('div', { id: 'viewport' });
    this.hudHost = h('div', { id: 'hud' });
    this.logo = h('img', { class: 'logo', alt: '', hidden: true });
    this.logoFallback = h('div', { class: 'logo logo-fallback', 'aria-hidden': 'true' }, 'D');
    this.teamLine = h('span', {}, '');
    // every mode the firmware port offers (SIM_MODES from sub.js); the select gives the keyboard back after a pick
    this.modeSelect = h('select', { class: 'mode-select', id: 'mode-select', 'aria-label': 'Flight mode', onchange: () => { this.setMode(+this.modeSelect.value); this.modeSelect.blur(); } },
      SIM_MODES.map((m) => h('option', { value: String(m) }, MODE_LABEL[m] ?? `Mode ${m}`)));
    this.armBtn = h('button', { class: 'arm-btn', id: 'arm-btn', onclick: () => this.toggleArm() }, 'DISARMED');
    this.gainChip = h('span', { class: 'chip hide-sm', title: 'Pilot gain (JS_GAIN)' }, 'Gain ', h('b', {}, '50%'));
    this.inputChip = h('span', { class: 'chip hide-sm', title: 'Active pilot input' }, 'Keyboard');
    this.gcsBtn = h('button', { class: 'chip hide-sm', id: 'gcs-btn', title: 'Simulate losing the ground-station link: no MANUAL_CONTROL and no heartbeat reach the vehicle (pilot input and GCS failsafes)', 'aria-pressed': 'false', onclick: (e) => { this.toggleGcsLink(); e.currentTarget.blur(); } }, 'GCS link ', h('b', {}, 'OK'));
    this.timeChip = h('span', { class: 'chip mono hide-sm', title: 'Simulation time and real-time factor' }, '0.0 s');
    this.playBtn = h('button', { class: 'icon-btn', id: 'play-btn', title: 'Pause / resume (P)', onclick: () => this.togglePause() }, icon('pause'));
    this.resetBtn = h('button', { class: 'icon-btn', title: 'Reset vehicle (Backspace)', onclick: () => this.resetVehicle() }, icon('reset'));
    this.camSelect = h('select', { class: 'mode-select hide-sm', id: 'camera-select', 'aria-label': 'Camera', onchange: () => { this.scene.setCameraMode(this.camSelect.value); this.camSelect.blur(); } },
      CAMERAS.map(([k, l]) => h('option', { value: k }, l)));
    this.graphBtn = h('button', { class: 'icon-btn', title: 'Live graphs (G)', onclick: () => this.toggleDock() }, icon('chart'));
    this.uploadBtn = h('button', { class: 'btn top-btn', title: 'Upload your vehicle design (STL, OBJ or PLY)', onclick: () => this.pickDesign() }, icon('upload'), h('span', { class: 'hide-sm' }, 'Upload design'));
    this.tuneBtn = h('button', { class: 'btn primary top-btn', title: 'Compute the hydrodynamics of this design and tune ArduSub for it at maximum precision', onclick: () => this.openAutoTune() }, icon('target'), h('span', {}, 'AUTO-TUNE'));
    this.topbar = h('header', { class: 'topbar' },
      h('div', { class: 'brand' }, this.logo, this.logoFallback, h('div', { class: 'wordmark' }, h('b', {}, 'DUBURI'), this.teamLine)),
      h('div', { class: 'bar-group top-actions' }, this.uploadBtn, this.tuneBtn),
      h('div', { class: 'bar-spacer' }),
      h('div', { class: 'bar-group' }, this.modeSelect, this.armBtn, this.gainChip, this.inputChip, this.gcsBtn),
      h('div', { class: 'bar-spacer' }),
      h('div', { class: 'bar-group' }, this.timeChip, this.playBtn, this.resetBtn, this.camSelect, this.graphBtn));
    this.rail = h('nav', { class: 'rail', 'aria-label': 'Vehicle setup' });
    this.panelHost = h('aside', { class: 'panel', hidden: true, 'aria-live': 'polite' });
    this.dockHost = h('section', { class: 'dock', hidden: true, 'aria-label': 'Live graphs' });
    this.stickL = h('div', { class: 'stick', 'aria-label': 'Left stick: up/down and yaw' }, h('i'), h('small', {}, 'HEAVE · YAW'));
    this.stickR = h('div', { class: 'stick', 'aria-label': 'Right stick: forward and lateral' }, h('i'), h('small', {}, 'SURGE · SWAY'));
    const sticks = h('div', { class: 'sticks' }, this.stickL, this.stickR);
    r.append(this.viewport, this.hudHost, this.topbar, this.rail, this.panelHost, this.dockHost, sticks);
  }

  renderRail() {
    clear(this.rail);
    for (const p of PANELS) {
      if (p.hidden) continue;
      const b = h('button', { class: this.panelOpen === p.id ? 'active' : '', title: p.title, 'aria-pressed': this.panelOpen === p.id ? 'true' : 'false', onclick: () => this.openPanel(this.panelOpen === p.id ? null : p.id) }, icon(p.icon), p.short || p.title);
      this.rail.append(b);
    }
  }

  openPanel(id) {
    if (this.activePanel?.dispose) this.activePanel.dispose();
    this.activePanel = null;
    this.panelOpen = id;
    this.scene.setPickHandler(null);
    this.renderRail();
    clear(this.panelHost);
    if (!id) { this.panelHost.hidden = true; this.onResize(); return; }
    const def = PANELS.find((p) => p.id === id);
    const inst = def.create(this);
    this.activePanel = inst;
    this.panelHost.append(
      h('header', {}, h('div', {}, h('h2', {}, def.title), def.sub ? h('div', { class: 'sub' }, def.sub) : null),
        h('button', { class: 'icon-btn close', title: 'Close panel', onclick: () => this.openPanel(null) }, icon('close'))),
      h('div', { class: 'body' }, inst.el));
    this.panelHost.hidden = false;
    this.onResize();
  }
  refreshPanelIf(id) { if (this.panelOpen === id) this.refreshPanel(); }
  refreshPanel() { if (this.panelOpen) { const s = this.panelHost.querySelector('.body')?.scrollTop; this.openPanel(this.panelOpen); const b = this.panelHost.querySelector('.body'); if (b && s) b.scrollTop = s; } }

  toggleDock(force) {
    const show = force ?? this.dockHost.hidden;
    this.dockHost.hidden = !show;
    this.graphBtn.classList.toggle('on', show);
    if (show && !this.dock) this.dock = new GraphDock(this, this.dockHost);
    this.onResize();
  }

  onResize() {
    const narrow = window.innerWidth <= 760;
    const panelW = this.panelHost.hidden ? 0 : narrow ? 0 : this.panelHost.getBoundingClientRect().width;
    const dockH = this.dockHost.hidden ? 0 : this.dockHost.getBoundingClientRect().height;
    const bar = this.topbar.getBoundingClientRect().height;
    const insets = { left: narrow ? 0 : 66 + panelW, top: bar, bottom: dockH + (narrow ? 58 : 0), right: 0 };
    this.hud.setInsets(insets);
    this.scene.setInsets(insets);
  }

  // ------------------------------------------------------------------ teams & persistence
  loadTeams() {
    this.teams = storage.get(`${KEY}.teams`, null);
    if (!Array.isArray(this.teams) || !this.teams.length) this.teams = [{ id: 'duburi', name: 'Team DUBURI', logo: null, color: '#ffb547' }];
    const active = storage.get(`${KEY}.activeTeam`, this.teams[0].id);
    this.team = this.teams.find((t) => t.id === active) || this.teams[0];
  }
  saveTeams() {
    storage.set(`${KEY}.teams`, this.teams);
    storage.set(`${KEY}.activeTeam`, this.team.id);
  }
  applyTeamBranding() {
    const t = this.team;
    this.teamLine.textContent = t.name || '';
    if (t.logo) { this.logo.src = t.logo; this.logo.hidden = false; this.logoFallback.hidden = true; }
    else { this.logo.hidden = true; this.logoFallback.hidden = false; this.logoFallback.textContent = (t.name || 'D').replace(/^team\s+/i, '').slice(0, 1).toUpperCase(); }
    document.documentElement.style.setProperty('--foam', t.color || '#ffb547');
    this.logoFallback.style.color = t.color || '#ffb547';
  }
  async switchTeam(id) {
    this.saveNow();
    this.team = this.teams.find((t) => t.id === id) || this.teams[0];
    this.saveTeams();
    this.applyTeamBranding();
    await this.loadSession();
    this.refreshPanel();
    toast(`Switched to ${this.team.name}`);
  }

  scheduleSave() {
    clearTimeout(this.saveTimer);
    this.saveTimer = setTimeout(() => this.saveNow(), 800);
  }
  saveNow() {
    if (!this.cfg) return;
    storage.set(`${KEY}.team.${this.team.id}.cfg`, this.cfg);
  }

  async loadSession() {
    this.closeDetection();
    const saved = storage.get(`${KEY}.team.${this.team.id}.cfg`, null);
    let cfg;
    if (saved) {
      try { cfg = normalizeConfig(saved); } catch (e) { cfg = null; toast('Saved vehicle could not be loaded; starting from the sample.', 'bad'); }
    }
    this.firstRun = !cfg;
    if (!cfg) {
      cfg = defaultConfig();
      if (this.team.id === 'duburi') applyParamFile(cfg, parseParamFile(DUBURI_PARAMS).params, { syncPolarity: true });
      else { cfg.params.FRAME_CONFIG = 2; cfg.name = `${this.team.name} ROV`; }
      cfg.mixer.source = 'auto';
    }
    this.cfg = cfg;
    this.stlRaw = null;
    if (cfg.model.source === 'stl') {
      const buf = await idbGet(`stl:${this.team.id}`);
      if (buf) { try { this.stlRaw = this.withParts(parseSTL(buf)); } catch { this.stlRaw = null; } }
      if (!this.stlRaw) { toast('Saved STL not found in this browser: showing the sample vehicle. Load your STL again in Vehicle.', 'bad', 6000); cfg.model.source = 'sample'; }
    }
    await this.updateGeometry({ resetVehicle: true });
  }

  // ------------------------------------------------------------------ geometry
  vehicleParts() {
    const m = this.cfg.model;
    if (m.source === 'stl' && this.stlRaw) {
      const T = makeModelTransform({ units: m.units, scale: m.scale ?? 1, forward: m.forward, up: m.up });
      let pos = transformPositions(this.stlRaw.positions, T);
      const bb = bbox(pos);
      const t = m.origin === 'file' ? [0, 0, 0] : bb.center.map((v) => -v);
      const off = m.offset || [0, 0, 0];
      pos = transformPositions(pos, { R: [1, 0, 0, 0, 1, 0, 0, 0, 1], s: 1 }, [t[0] + off[0], t[1] + off[1], t[2] + off[2]]);
      return { positions: pos, parts: [{ name: 'Hull', color: m.color, positions: pos }] };
    }
    if (m.source === 'shape') {
      const g = shapeGeometry(m.shape || DEFAULT_SHAPE, this.cfg.thrusters);
      return { positions: g.positions, parts: g.parts };
    }
    const s = sampleGeometry();
    return { positions: s.positions, parts: s.parts };
  }

  // thruster layout fingerprint: a simple-shape body has a duct at every thruster
  thrusterSig() { return JSON.stringify(this.cfg.thrusters.map((t) => [t.pos, t.axis, t.enabled, t.propDiameter])); }

  // switch the body between the uploaded design, a simple shape and the sample
  async setModelSource(source) {
    const m = this.cfg.model;
    if (source === 'stl' && !this.stlRaw) {
      const buf = await idbGet(`stl:${this.team.id}`);
      if (buf) { try { this.stlRaw = this.withParts(parseSTL(buf)); } catch { this.stlRaw = null; } }
      if (!this.stlRaw) { toast('No design is stored in this browser yet: upload your STL, OBJ or PLY.', 'bad', 5000); return false; }
    }
    if (source === 'shape') m.shape = { ...DEFAULT_SHAPE, ...(m.shape || {}) };
    m.source = source;
    if (this.cfg.mass.source !== 'user') this.cfg.mass.mass = 0; // neutral mass is re-estimated for the new body
    await this.updateGeometry({ resetVehicle: true });
    return true;
  }

  async updateGeometry({ resetVehicle = false } = {}) {
    const { positions, parts } = this.vehicleParts();
    this.vehiclePositions = positions;
    this.busy('Analysing hull…');
    let geo;
    try {
      geo = await this.jobs.run('analyze', { positions, opts: { sealedHull: !!this.cfg.model.sealedHull } }, (f, t) => this.busy(`${t}… ${Math.round(f * 100)}%`));
    } catch (e) {
      this.busy(null);
      toast(`Hull analysis failed: ${e.message}`, 'bad', 8000);
      if (this.cfg.model.source === 'stl') { this.cfg.model.source = 'sample'; return this.updateGeometry({ resetVehicle: true }); }
      throw e;
    }
    this.busy(null);
    this.geo = geo;
    // an estimated mass stays neutral for whatever body is loaded; a mass you entered is never changed
    if (this.firstRun || !(this.cfg.mass.mass > 0) || this.cfg.mass.source === 'estimate') {
      this.cfg.mass.mass = +(waterDensity(this.cfg.environment.temperature, this.cfg.environment.salinity) * geo.volume * 0.985).toFixed(3);
      this.cfg.mass.source = 'estimate';
      this.firstRun = false;
    }
    this.parts = parts;
    this.shapeSig = this.cfg.model.source === 'shape' ? this.thrusterSig() : null;
    this.scene.buildPool(this.cfg.environment.pool);
    this.scene.setVehicleGeometry(parts, this.cfg.model.color);
    this.buildSim(resetVehicle);
    this.scheduleSave();
  }

  buildSim(reset) {
    try {
      if (!this.sim || reset) {
        const prev = this.sim;
        this.sim = new Simulation(this.cfg, this.geo);
        // a new simulation keeps the pause / speed the controls show
        if (prev) { this.sim.paused = prev.paused; this.sim.timeScale = prev.timeScale; }
        this.sim.on((ev) => { if (ev === 'reset' || ev === 'rebuild') { this.syncModeOptions(); this.applyGcsLink(); } });
        this.playBtn?.replaceChildren(icon(this.sim.paused ? 'play' : 'pause'));
      } else this.sim.rebuild(this.cfg, this.geo, { keepState: true });
      this.applyGcsLink();
      this.simError = null;
    } catch (e) {
      this.simError = e.message;
      toast(`Cannot build the vehicle model: ${e.message}`, 'bad', 8000);
      return;
    }
    this.refreshOverlays();
    this.syncModeOptions();
  }

  // the select lists SIM_MODES; a mode the vehicle is in but not listed (e.g. MOTOR_DETECT entered by the firmware) is added
  syncModeOptions() {
    const m = this.sim?.fw?.mode;
    if (m === undefined) return;
    if (![...this.modeSelect.options].some((o) => o.value === String(m))) this.modeSelect.append(h('option', { value: String(m) }, MODE_LABEL[m] ?? `Mode ${m}`));
    if (document.activeElement !== this.modeSelect) this.modeSelect.value = String(m);
  }

  // ---- simulated GCS link (MANUAL_CONTROL + heartbeat) ----
  toggleGcsLink(force) {
    this.gcsLinkDown = force ?? !this.gcsLinkDown;
    this.applyGcsLink();
    toast(this.gcsLinkDown ? 'GCS link dropped: the vehicle gets no MANUAL_CONTROL or heartbeat' : 'GCS link restored', this.gcsLinkDown ? 'bad' : 'good');
  }
  applyGcsLink() {
    const sim = this.sim;
    if (!sim) return;
    const down = !!this.gcsLinkDown;
    sim.opts.noPilot = down;
    sim.gcsLinkDown = down;
    if (sim.fw) { sim.fw.gcsLinkDown = down; sim.fw.setGcsLink?.(!down); }
    this.gcsBtn?.classList.toggle('bad', down);
    this.gcsBtn?.setAttribute('aria-pressed', down ? 'true' : 'false');
    this.gcsBtn?.replaceChildren('GCS link ', h('b', {}, down ? 'DOWN' : 'OK'));
  }
  gcsBlocked() { if (this.gcsLinkDown) { toast('The GCS link is down: restore it first (GCS link button)', 'bad'); return true; } return false; }

  // AUTO mission (cfg.mission: [{pos:[n,e,d], hold}]) sent to the firmware when it offers setMission
  uploadMission(list = this.cfg.mission || []) {
    const wps = list.map((w) => ({ ...w, pos: w.pos.slice(), n: w.pos[0], e: w.pos[1], d: w.pos[2], hold: w.hold ?? 0 }));
    const fw = this.sim?.fw;
    if (!fw?.setMission) { toast('This firmware build has no AUTO mission support yet', 'bad'); return false; }
    const r = fw.setMission(wps);
    if (r === false) { const m = fw.messages[fw.messages.length - 1]; toast(m ? m.text : 'Mission rejected', 'bad'); return false; }
    toast(`Mission with ${wps.length} waypoint${wps.length === 1 ? '' : 's'} sent to the vehicle`, 'good');
    return true;
  }
  // GUIDED "go to" (NED metres)
  guidedGoto(pos) {
    const fw = this.sim?.fw;
    if (!fw || this.gcsBlocked()) return false;
    const fn = fw.setGuidedTarget ?? fw.setGuidedPosition ?? fw.guidedGoto ?? fw.guided?.setTarget;
    if (!fn) { toast('This firmware build has no GUIDED target support yet', 'bad'); return false; }
    if (fw.mode !== MODE.GUIDED && MODE.GUIDED !== undefined && !fw.setMode(MODE.GUIDED, 'gcs')) { const m = fw.messages[fw.messages.length - 1]; toast(m ? m.text : 'GUIDED not available', 'bad'); return false; }
    const r = fn.call(fw.guided?.setTarget === fn ? fw.guided : fw, pos.slice());
    if (r === false) { toast('Guided target rejected', 'bad'); return false; }
    this.guidedTarget = pos.slice();
    toast(`Going to N ${pos[0].toFixed(2)} m, E ${pos[1].toFixed(2)} m, depth ${pos[2].toFixed(2)} m`, 'good');
    return true;
  }

  refreshOverlays() {
    const cg = cgOf(this.cfg, this.geo);
    const info = this.sim?.info;
    this.scene.setOverlays({ thrusters: this.cfg.thrusters, cg, cb: info ? info.cb : this.geo.centroid, bbox: this.geo.bbox });
  }

  // config changed: rebuild the model (keeps the vehicle where it is)
  changed({ geometry = false, immediate = false } = {}) {
    this.scheduleSave();
    // a simple-shape body follows its thrusters (ducts are part of the body)
    if (!geometry && this.cfg.model.source === 'shape' && this.thrusterSig() !== this.shapeSig) {
      clearTimeout(this.shapeTimer);
      this.shapeTimer = setTimeout(() => this.updateGeometry({ resetVehicle: false }), 450);
    }
    if (geometry) {
      // units, axes or origin changed while detected thrusters wait for Accept: find them again
      this.updateGeometry({ resetVehicle: true }).then(() => { if (this.pendingDetection && this.cfg.model.source === 'stl') this.detectDesignThrusters({ auto: true, rerun: true }); });
      return;
    }
    clearTimeout(this.rebuildTimer);
    const run = () => { this.buildSim(false); this.activePanel?.onModelChanged?.(); this.dock?.reset?.(); };
    if (immediate) run(); else this.rebuildTimer = setTimeout(run, 120);
  }

  // pool size / depth / floor changed: redraw it, rebuild the water and keep
  // the vehicle (and its start position) inside the new pool
  poolChanged() {
    this.scene.buildPool(this.cfg.environment.pool);
    this.changed({ immediate: true });
    const k = this.sim?.keepInPool?.(this.cfg.start.position);
    if (k?.moved) { this.cfg.start.position = k.p.map((v) => +v.toFixed(3)); toast('Start position moved inside the new pool', '', 3500); }
    this.scheduleSave();
  }

  // live parameter change (firmware only, no rebuild)
  setParam(name, value) {
    // single validating setter (range clamp, leak detector / positioning follow-up)
    const r = setParamValue(this.cfg, name, value);
    if (r.error) { toast(`${name}: ${r.error}`, 'bad', 4000); return r; }
    value = r.value;
    const structural = /^(FRAME_CONFIG|SCHED_LOOP_RATE|INS_|AHRS_TRIM|BARO_SPEC|EK3_)/.test(name) || /^MOT_\d+_DIRECTION$/.test(name);
    if (structural) this.changed({ immediate: true });
    else { this.sim.fw.applyParams({ [name]: value }); this.sim.cfg.params[name] = value; this.scheduleSave(); }
    return r;
  }

  // replace the vehicle of the active team (vehicle file import)
  async loadVehicle(cfg, stlBuffer) {
    this.closeDetection();
    this.cfg = cfg;
    this.stlRaw = null;
    if (stlBuffer) {
      try { this.stlRaw = this.withParts(parseSTL(stlBuffer)); } catch (e) { toast(`The STL inside the file could not be read: ${e.message}`, 'bad', 6000); }
      if (this.stlRaw) await idbPut(`stl:${this.team.id}`, stlBuffer);
    }
    if (cfg.model.source === 'stl' && !this.stlRaw) cfg.model.source = 'sample';
    this.firstRun = !(cfg.mass.mass > 0);
    await this.updateGeometry({ resetVehicle: true });
    this.saveNow();
    this.refreshPanel();
  }

  loadSTLFile(file) { return this.loadDesignFile(file); }
  async loadDesignFile(file) {
    const buf = await file.arrayBuffer();
    let parsed;
    try { parsed = parseMesh(buf, file.name); } catch (e) { toast(`Could not read ${file.name}: ${e.message}`, 'bad', 7000); return false; }
    if (parsed.triCount > 3_000_000) { toast('This design has more than 3 million triangles. Decimate it in your CAD tool first.', 'bad', 8000); return false; }
    const parts = parsed.parts || null; // named bodies (OBJ groups, ASCII STL solids)
    if (parsed.format !== 'binary' && parsed.format !== 'ascii') parsed = parseSTL(writeBinarySTL(parsed.positions, file.name)); // OBJ/PLY are stored as STL
    if (parts) parsed.parts = parts;
    this.stlRaw = parsed;
    const bb = bbox(parsed.positions);
    const m = this.cfg.model;
    m.source = 'stl';
    m.fileName = file.name;
    m.parts = parts && parts.length <= 2000 ? parts : null; // kept for "Find thrusters" after a reload (the stored STL has no names)
    m.units = guessUnits(bb.size);
    m.origin = 'bbox';
    m.offset = [0, 0, 0];
    m.color = m.color || '#dfe6ea';
    const ok = await idbPut(`stl:${this.team.id}`, /\.stl$/i.test(file.name) ? buf : writeBinarySTL(parsed.positions, file.name));
    if (!ok) toast('This browser cannot store the STL: you will need to load it again next time.', '', 6000);
    this.cfg.mass.mass = 0; // re-estimate neutral mass for the new hull until the user enters theirs
    this.firstRun = true;
    await this.updateGeometry({ resetVehicle: true });
    toast(`Loaded ${file.name}: ${parsed.triCount.toLocaleString()} triangles, ${m.units} units guessed`, 'good', 5000);
    await this.detectDesignThrusters({ auto: true });
    return true;
  }

  // part names saved with the vehicle (the browser stores OBJ/PLY designs as nameless STL)
  withParts(raw) {
    const p = this.cfg?.model?.parts;
    if (raw && !raw.parts && Array.isArray(p) && p.length && p.every((q) => q.start >= 0 && q.start + q.count <= raw.triCount)) raw.parts = p;
    return raw;
  }

  // ------------------------------------------------------------------ thrusters found in the design
  // Detection runs on the hull as placed in the vehicle frame (units, axes and
  // origin from Vehicle), proposes the thrusters as a custom frame (automatic
  // mixer) and keeps the previous layout until Accept / Discard.
  async detectDesignThrusters({ auto = false, rerun = false } = {}) {
    if (this.cfg.model.source !== 'stl' || !this.stlRaw || !this.vehiclePositions) {
      if (!auto) toast('Thruster detection works on an uploaded design: choose My design (STL) in Vehicle.', 'bad', 5000);
      return null;
    }
    this.busy('Looking for thrusters in the design…');
    await new Promise((r) => setTimeout(r, 30));
    let det;
    try {
      det = detectThrusters(this.vehiclePositions, { parts: this.stlRaw.parts, frameConfig: this.cfg.params.FRAME_CONFIG ?? 2, cg: cgOf(this.cfg, this.geo) });
    } catch (e) {
      this.busy(null);
      console.error(e);
      if (!auto) toast(`Thruster detection failed: ${e.message}`, 'bad', 6000);
      return null;
    }
    this.busy(null);
    if (!det.thrusters.length) {
      if (rerun) { this.discardDetection('No thrusters found with these units and axes: previous thruster layout restored.'); return det; }
      const how = 'Name the thruster parts T200_1, T200_2… in your CAD export (OBJ or ASCII STL), or place them in Frame.';
      toast(`${det.summary}: your thruster layout is unchanged. ${det.notes.join(' ')} ${how}`, auto ? '' : 'bad', 9000);
      return det;
    }
    this.applyDetection(det);
    return det;
  }

  applyDetection(det) {
    const cfg = this.cfg, P = cfg.params;
    const prev = this.pendingDetection?.prev ?? { thrusters: JSON.parse(JSON.stringify(cfg.thrusters)), mixerSource: cfg.mixer.source };
    const t200 = /^t200/.test(cfg.thrusters[0]?.preset || '') ? cfg.thrusters[0].preset : 't200-16v'; // keep the T200 voltage in use
    cfg.thrusters = det.thrusters.map((d, i) => {
      const t = thrusterFromPreset({ name: d.name, pos: d.pos.slice(), axis: d.axis.slice() }, d.type === 't200' ? t200 : d.preset);
      if (d.type === 'generic') t.propDiameter = d.propDiameter;
      // axis = push for a positive output; spin direction follows MOT_n_DIRECTION, as for a frame layout
      t.polarity = (P[`MOT_${i + 1}_DIRECTION`] ?? 1) < 0 ? -1 : 1;
      return t;
    });
    cfg.mixer.source = 'auto';
    this.pendingDetection = { det, prev };
    this.changed({ immediate: true });
    this.showDetection();
    this.refreshPanelIf('frame');
    this.refreshPanelIf('thrusters');
  }

  showDetection() {
    const p = this.pendingDetection;
    if (!p) return;
    const el = thrusterNotice(this, p.det, {
      accept: () => this.acceptDetection(),
      discard: () => this.discardDetection(),
      flip: (i) => this.flipDetected(i),
      openFrame: () => this.openPanel('frame'),
    });
    if (this.detectEl?.isConnected) this.detectEl.replaceWith(el); else this.root.append(el);
    this.detectEl = el;
    const sure = p.det.directions === 'frame';
    this.scene.setThrusterPreview(this.cfg.thrusters.map((t, i) => ({ pos: t.pos, axis: t.axis, od: p.det.thrusters[i]?.od, confidence: Math.min(p.det.thrusters[i]?.confidence ?? 0, sure ? 1 : 0.4) })));
  }

  closeDetection() {
    this.pendingDetection = null;
    this.detectEl?.remove();
    this.detectEl = null;
    this.scene?.setThrusterPreview(null);
  }

  flipDetected(i) {
    const t = this.cfg.thrusters[i];
    if (!t) return;
    t.axis = t.axis.map((v) => -v || 0);
    this.changed({ immediate: true });
    this.showDetection();
    this.refreshPanelIf('frame');
    this.refreshPanelIf('thrusters');
  }

  acceptDetection() {
    const n = this.cfg.thrusters.length;
    this.closeDetection();
    this.scheduleSave();
    toast(`${n} thruster${n > 1 ? 's' : ''} kept as your custom frame (Frame panel)`, 'good');
  }

  discardDetection(msg = 'Previous thruster layout restored') {
    const p = this.pendingDetection;
    if (!p) return;
    this.cfg.thrusters = p.prev.thrusters;
    this.cfg.mixer.source = p.prev.mixerSource;
    this.closeDetection();
    this.changed({ immediate: true });
    this.refreshPanelIf('frame');
    this.refreshPanelIf('thrusters');
    toast(msg, '', 5000);
  }

  busy(text) {
    if (!this.busyEl) { this.busyEl = h('div', { class: 'toast', style: { position: 'absolute', top: '70px', left: '50%', transform: 'translateX(-50%)', zIndex: 65 } }); }
    if (!text) { this.busyEl.remove(); return; }
    this.busyEl.textContent = text;
    if (!this.busyEl.isConnected) this.root.append(this.busyEl);
  }

  // ------------------------------------------------------------------ actions
  toggleArm() {
    if (this.gcsBlocked()) return;
    const fw = this.sim.fw;
    if (fw.armed) fw.disarm('gcs');
    else if (!fw.arm('gcs')) { const m = fw.messages[fw.messages.length - 1]; toast(m ? m.text : 'Arming failed', 'bad'); }
  }
  setMode(m) {
    if (this.gcsBlocked()) { this.modeSelect.value = String(this.sim.fw.mode); return; }
    if (!this.sim.fw.setMode(m, 'gcs')) { const msg = this.sim.fw.messages[this.sim.fw.messages.length - 1]; toast(msg ? msg.text : 'Mode change failed', 'bad'); }
    this.modeSelect.value = String(this.sim.fw.mode);
  }
  togglePause() {
    this.sim.paused = !this.sim.paused;
    this.playBtn.replaceChildren(icon(this.sim.paused ? 'play' : 'pause'));
  }
  async resetVehicle() {
    const ok = this.sim.fw.armed ? await confirmDialog('Reset the vehicle?', 'The vehicle is armed. Resetting disarms it and puts it back at the start position.', { ok: 'Reset' }) : true;
    if (!ok) return;
    this.sim.reset();
    this.modeSelect.value = String(this.sim.fw.mode);
    this.dock?.reset?.();
  }
  hotkey(code, e) {
    const modeKeys = { Digit1: MODE.MANUAL, Digit2: MODE.STABILIZE, Digit3: MODE.ALT_HOLD, Digit4: MODE.ACRO, Digit5: MODE.POSHOLD, Digit6: MODE.SURFACE };
    if (code in modeKeys) { this.setMode(modeKeys[code]); return true; }
    switch (code) {
      case 'Enter': this.toggleArm(); return true;
      case 'Equal': case 'NumpadAdd': this.sim.fw.js.gainStep(1); return true;
      case 'Minus': case 'NumpadSubtract': this.sim.fw.js.gainStep(-1); return true;
      case 'KeyV': { const i = CAMERAS.findIndex(([k]) => k === this.scene.cameraMode); const n = CAMERAS[(i + 1) % CAMERAS.length][0]; this.camSelect.value = n; this.scene.setCameraMode(n); toast(`Camera: ${CAMERAS.find(([k]) => k === n)[1]}`); return true; }
      case 'KeyG': this.toggleDock(); return true;
      case 'KeyP': this.togglePause(); return true;
      case 'Backspace': this.resetVehicle(); return true;
      case 'Escape': if (this.panelOpen) { this.openPanel(null); return true; } return false;
    }
    return false;
  }

  // ------------------------------------------------------------------ loop
  loop(t) {
    const dt = Math.min(0.1, Math.max(0, (t - this.last) / 1000));
    this.last = t;
    this.frame++;
    if (this.sim && !this.simError) {
      try {
        // a panel can drive the vehicle (Frame > Try it); otherwise keyboard / gamepad
        const msg = this.pilotOverride ? this.pilotOverride.msg : this.input.sample(dt, this.cfg);
        this.sim.setPilotInput(msg);
        const t0 = performance.now();
        this.sim.advance(dt);
        this.simCost = (this.simCost ?? 0) * 0.95 + (performance.now() - t0) * 0.05;
        // camera mount and other servo outputs run in firmware time (paused / sped up with the sim);
        // the camera view follows the tilt servo (SERVOn_FUNCTION 7, MNT1_*)
        const js = this.sim.fw.js;
        js.update?.(this.sim.fw.time);
        const tilt = js.cameraTiltDeg?.(), pan = js.cameraPanDeg?.();
        this.scene.camTilt = Number.isFinite(tilt) ? (tilt * Math.PI) / 180 : 0;
        this.scene.camPan = Number.isFinite(pan) ? (pan * Math.PI) / 180 : 0;
        this.drainFirmwareSaves();
        this.scene.update(this.sim, dt);
        this.hud.draw(this.sim, { paused: this.sim.paused, cameraMode: this.scene.cameraMode, gcsLinkDown: !!this.gcsLinkDown });
        if (this.frame % 6 === 0) this.updateTopbar();
        if (this.frame % 12 === 0) this.activePanel?.tick?.();
        if (this.dock && !this.dockHost.hidden && this.frame % 3 === 0) this.dock.tick();
      } catch (e) {
        console.error(e);
        this.simError = e.message;
        toast(`Simulation stopped: ${e.message}. Reset the vehicle or fix the setting that caused it.`, 'bad', 10000);
      }
    }
    requestAnimationFrame((tt) => this.loop(tt));
  }

  // parameters the firmware saved itself (set_and_save: JS_GAIN_* on gain buttons, SERVOn_FUNCTION migrations at boot)
  drainFirmwareSaves() {
    const q = this.sim.fw.js?.pendingSaves;
    if (!q?.length) return;
    for (const [n, v] of q.splice(0)) {
      this.cfg.params[n] = v;
      if (this.cfg.extraParams && n in this.cfg.extraParams) delete this.cfg.extraParams[n];
    }
    this.scheduleSave();
  }

  // GUIDED "go to": click the water in the Top view while in GUIDED; mission/target markers
  updateGuidedUi() {
    const fw = this.sim.fw;
    const on = this.scene.cameraMode === 'top' && MODE.GUIDED !== undefined && fw.mode === MODE.GUIDED && !this.gcsLinkDown;
    if (on !== !!this.scene.groundPickHandler) this.scene.setGroundPickHandler?.(on ? (p) => this.guidedGoto([p[0], p[1], this.sim.plant.position[2]]) : null);
    if (fw.mode !== MODE.GUIDED) this.guidedTarget = null;
    const sig = JSON.stringify([this.cfg.mission || [], this.guidedTarget]);
    if (sig !== this.wpSig) { this.wpSig = sig; this.scene.setWaypoints?.(this.cfg.mission || [], this.guidedTarget); }
  }

  updateTopbar() {
    const fw = this.sim.fw;
    this.updateGuidedUi();
    this.armBtn.textContent = fw.armed ? 'ARMED' : 'DISARMED';
    this.armBtn.classList.toggle('armed', fw.armed);
    this.armBtn.setAttribute('aria-pressed', fw.armed ? 'true' : 'false');
    this.syncModeOptions();
    this.gainChip.replaceChildren('Gain ', h('b', {}, `${Math.round(fw.gain * 100)}%`));
    const src = this.input.source;
    const name = src === 'gamepad' ? shortName(this.input.gamepadName) : src === 'touch' ? 'Touch sticks' : this.input.gamepadBlocked ? 'Keyboard (controller blocked here)' : 'Keyboard';
    this.inputChip.textContent = name;
    const rtf = this.sim.timeScale * (this.sim.paused ? 0 : 1);
    this.timeChip.textContent = `${this.sim.time.toFixed(1)} s · ${rtf.toFixed(2)}×`;
  }
}
