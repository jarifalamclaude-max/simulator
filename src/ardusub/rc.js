// Port of RC_Channel / RC_Channels as ArduSub uses them (integer maths kept as in C++).
// 16 channels (RC_Channels NUM_RC_CHANNELS), MAVLink overrides with RC_OVERRIDE_TIME,
// RC_OPTIONS (IGNORE_OVERRIDES) and RCn_OPTION. ArduSub has no RC receiver path in the
// simulator (only MANUAL_CONTROL -> overrides), and it never runs read_aux_all(), so
// RCn_OPTION only matters as a marker the mount looks up (MOUNT1_ROLL/PITCH/YAW).

const trunc = Math.trunc;
const clampI = (v, lo, hi) => (v < lo ? lo : v > hi ? hi : v);

export const NUM_RC_CHANNELS = 16;
// RC_Channels::Option (RC_Channel.h:510-525)
export const RC_OPTION = { IGNORE_RECEIVER: 1 << 0, IGNORE_OVERRIDES: 1 << 1, IGNORE_FAILSAFE: 1 << 2, FPORT_PAD: 1 << 3, LOG_DATA: 1 << 4, ARMING_CHECK_THROTTLE: 1 << 5, ARMING_SKIP_CHECK_RPY: 1 << 6, ALLOW_SWITCH_REV: 1 << 7 };
export const RC_OPTIONS_BITS = ['Ignore RC receiver', 'Ignore MAVLink overrides', 'Ignore receiver failsafe bit', 'FPort pad', 'Log RC input bytes', 'Arming check throttle for 0 input', 'Skip arming check for neutral R/P/Y', 'Allow switch reverse', 'CRSF passthrough telemetry', 'Suppress CRSF mode/rate message', 'Multiple receivers', 'CRSF link quality as RSSI', 'Annotate CRSF mode on disarm', 'ELRS 420 kbaud'];
// RCn_OPTION values that ArduSub acts on (mount targeting); others are listed for completeness only
export const RC_AUX_FUNCTIONS = { 0: 'Do Nothing', 9: 'Camera Trigger', 27: 'Retract Mount1', 28: 'Relay1 On/Off', 31: 'Motor Emergency Stop', 34: 'Relay2 On/Off', 35: 'Relay3 On/Off', 36: 'Relay4 On/Off', 41: 'ArmDisarm (4.1 and lower)', 46: 'RC Override Enable', 153: 'ArmDisarm', 166: 'Camera Record Video', 167: 'Camera Zoom', 168: 'Camera Manual Focus', 169: 'Camera Auto Focus', 212: 'Mount1 Roll', 213: 'Mount1 Pitch', 214: 'Mount1 Yaw', 215: 'Mount2 Roll', 216: 'Mount2 Pitch', 217: 'Mount2 Yaw', 300: 'Scripting1', 301: 'Scripting2' };
export const AUX = { MOUNT1_ROLL: 212, MOUNT1_PITCH: 213, MOUNT1_YAW: 214 };
// RCn defaults: RC_Channel.cpp:64-105 (MIN 1100, TRIM 1500, MAX 1900, DZ 0) and ArduSub
// defaults_table (RC3_TRIM 1100, RC7_OPTION 214, RC8_OPTION 213: Parameters.h:398-413).
export const rcDefault = (i, k) => ({ MIN: 1100, MAX: 1900, TRIM: i === 3 ? 1100 : 1500, DZ: 0, REVERSED: 0, OPTION: i === 7 ? AUX.MOUNT1_YAW : i === 8 ? AUX.MOUNT1_PITCH : 0 })[k];
const rcParam = (P, i, k) => P?.[`RC${i}_${k}`] ?? rcDefault(i, k);

export class RCChannel {
  constructor({ min = 1100, max = 1900, trim = 1500, dz = 0, reversed = false, option = 0 } = {}, owner = null) {
    this.min = min; this.max = max; this.trim = trim; this.deadZone = dz; this.reversed = !!reversed; this.option = option;
    this.owner = owner;
    this.type = 'angle'; this.highIn = 4500;
    this.radioIn = trim;
    this.controlIn = 0;
    this.overrideValue = 0; this.lastOverrideMs = 0;
  }
  configure({ min, max, trim, dz, reversed, option }) {
    if (min !== undefined) this.min = min;
    if (max !== undefined) this.max = max;
    if (trim !== undefined) this.trim = trim;
    if (dz !== undefined) this.deadZone = dz;
    if (reversed !== undefined) this.reversed = !!reversed;
    if (option !== undefined) this.option = option;
  }
  setAngle(a) { this.type = 'angle'; this.highIn = a; }
  setRange(h) { this.type = 'range'; this.highIn = h; }
  // RC_Channel::set_override (RC_Channel.cpp:484). Without an owner, or with
  // owner.deferRead false (default), the override is read at once (as if
  // read_input() ran right after); with deferRead it waits for readInput().
  setOverride(pwm, nowMs) {
    const o = this.owner;
    if (o && !o.gcsOverridesEnabled) return;
    this.overrideValue = trunc(pwm) & 0xffff;
    this.lastOverrideMs = nowMs ?? o?.nowMs ?? 0;
    if (o) o.hasNewOverrides = true;
    if (!o || !o.deferRead) this.update(this.lastOverrideMs);
  }
  clearOverride() { this.overrideValue = 0; this.lastOverrideMs = 0; }
  // RC_Channel::has_override (RC_Channel.cpp:501): RC_OVERRIDE_TIME > 0 times out, 0 disables, < 0 never times out
  hasOverride(nowMs = this.owner?.nowMs ?? this.lastOverrideMs) {
    if (this.overrideValue === 0) return false;
    const t = this.owner?.overrideTimeout ?? 3;
    if (!(t > 0) && t !== 0) return true;
    if (t === 0) return false;
    return nowMs - this.lastOverrideMs < trunc(Math.fround(t * 1e3));
  }
  // RC_Channel::update (RC_Channel.cpp:284). No receiver in the simulator.
  update(nowMs) {
    const ignore = this.owner ? (this.owner.options & RC_OPTION.IGNORE_OVERRIDES) !== 0 : false;
    if (this.hasOverride(nowMs) && !ignore) this.radioIn = this.overrideValue;
    else return false;
    this.recompute();
    return true;
  }
  recompute() {
    this.controlIn = this.type === 'range' ? this.pwmToRangeDz(this.deadZone) : this.pwmToAngleDzTrim(this.deadZone, this.trim);
    return this.controlIn;
  }
  pwmToAngleDzTrim(dz, trim) {
    const hi = trim + dz, lo = trim - dz;
    const rev = this.reversed ? -1 : 1;
    const r = clampI(this.radioIn, this.min, this.max);
    if (r > hi && this.max !== hi) return trunc((rev * trunc(this.highIn * (r - hi))) / (this.max - hi));
    if (r < lo && lo !== this.min) return trunc((rev * trunc(this.highIn * (r - lo))) / (lo - this.min));
    return 0;
  }
  pwmToRangeDz(dz) {
    let r = clampI(this.radioIn, this.min, this.max);
    if (this.reversed) r = this.max - (r - this.min);
    const lo = this.min + dz;
    if (r > lo && this.max > lo) return trunc((this.highIn * (r - lo)) / (this.max - lo));
    return 0;
  }
  getControlMid() {
    if (this.type === 'range') {
      const r = trunc((this.min + this.max) / 2);
      const lo = this.min + this.deadZone;
      if (this.max - lo <= 0) return 0; // dead zone covers the whole range
      return trunc((this.highIn * (r - lo)) / (this.max - lo));
    }
    return 0;
  }
  // RC_Channel::norm_input (RC_Channel.cpp:403)
  normInput() {
    const rev = this.reversed ? -1 : 1;
    let ret;
    if (this.radioIn < this.trim) {
      if (this.min >= this.trim) return 0;
      ret = (rev * (this.radioIn - this.trim)) / (this.trim - this.min);
    } else {
      if (this.max <= this.trim) return 0;
      ret = (rev * (this.radioIn - this.trim)) / (this.max - this.trim);
    }
    return Math.max(-1, Math.min(1, ret));
  }
  // RC_Channel::norm_input_dz (RC_Channel.cpp:421), used by AP_Mount for RC targeting
  normInputDz() {
    const dzMin = this.trim - this.deadZone, dzMax = this.trim + this.deadZone;
    const rev = this.reversed ? -1 : 1;
    let ret = 0;
    if (this.radioIn < dzMin && dzMin > this.min) ret = (rev * (this.radioIn - dzMin)) / (dzMin - this.min);
    else if (this.radioIn > dzMax && this.max > dzMax) ret = (rev * (this.radioIn - dzMax)) / (this.max - dzMax);
    return Math.max(-1, Math.min(1, ret));
  }
}

// RC_Channels: the 16 channels plus RC_OVERRIDE_TIME / RC_OPTIONS state.
export class RCChannels {
  constructor(P = {}) {
    this.gcsOverridesEnabled = true;
    this.hasNewOverrides = false;
    this.deferRead = false;
    this.nowMs = 0;
    this.all = [];
    for (let i = 1; i <= NUM_RC_CHANNELS; i++) this.all.push(new RCChannel({ min: rcParam(P, i, 'MIN'), max: rcParam(P, i, 'MAX'), trim: rcParam(P, i, 'TRIM'), dz: rcParam(P, i, 'DZ'), reversed: rcParam(P, i, 'REVERSED'), option: rcParam(P, i, 'OPTION') }, this));
    this.configureGlobals(P);
  }
  configureGlobals(P) {
    this.overrideTimeout = P?.RC_OVERRIDE_TIME ?? 3;
    this.options = (P?.RC_OPTIONS ?? RC_OPTION.ARMING_CHECK_THROTTLE) >>> 0;
  }
  channel(i) { return this.all[i]; }
  // RC_Channels::read_input (RC_Channels.cpp:73): only runs when new overrides arrived
  readInput(nowMs = this.nowMs) {
    this.nowMs = nowMs;
    if (!this.hasNewOverrides) return false;
    this.hasNewOverrides = false;
    let ok = false;
    for (const c of this.all) ok = c.update(nowMs) || ok;
    return ok;
  }
  clearOverrides() { for (const c of this.all) c.clearOverride(); }
  setGcsOverridesEnabled(v) { this.gcsOverridesEnabled = !!v; if (!v) this.clearOverrides(); }
  hasActiveOverrides(nowMs = this.nowMs) { return this.all.some((c) => c.hasOverride(nowMs)); }
  overrideMask(nowMs = this.nowMs) { return this.all.reduce((m, c, i) => (c.hasOverride(nowMs) ? m | (1 << i) : m), 0); }
  // RC_Channels::find_channel_for_option
  findChannelForOption(opt) { return this.all.find((c) => c.option === opt) ?? null; }
}

// ArduSub channel map: 1 pitch, 2 roll, 3 throttle, 4 yaw, 5 forward, 6 lateral (7 pan, 8 tilt, 9-16 aux)
export function makeSubChannels(P) {
  const rc = new RCChannels(P);
  const ch = rc.all;
  const [pitch, roll, throttle, yaw, forward, lateral] = ch;
  roll.setAngle(4500); pitch.setAngle(4500); yaw.setAngle(4500); forward.setAngle(4500); lateral.setAngle(4500);
  throttle.setRange(1000);
  for (let i = 0; i < 8; i++) ch[i].setOverride(1500);
  return Object.assign(rc, { pitch, roll, throttle, yaw, forward, lateral });
}

export function configureSubChannels(rc, P) {
  rc.all.forEach((c, i) => c.configure({ min: rcParam(P, i + 1, 'MIN'), max: rcParam(P, i + 1, 'MAX'), trim: rcParam(P, i + 1, 'TRIM'), dz: rcParam(P, i + 1, 'DZ'), reversed: rcParam(P, i + 1, 'REVERSED'), option: rcParam(P, i + 1, 'OPTION') }));
  rc.configureGlobals?.(P);
  rc.all.forEach((c) => c.recompute());
}
