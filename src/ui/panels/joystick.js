// Controller panel: live gamepad state, axis mapping, button functions
// (BTNn_FUNCTION / BTNn_SFUNCTION for all 32 ArduSub buttons, the full 4.5.7
// list), the outputs the buttons drive (lights, camera mount, relays,
// actuators, SERVOn channels with live PWM), RC channels 1-16 and keyboard help.

import { h, numberField, selectField, checkbox, fmt, toast } from '../dom.js';
import { BUTTON_FUNCTIONS, BUTTON_FUNCTION_GROUPS, buttonFunctionLabel, SDL_BUTTON_NAMES, SERVO_FUNCTIONS, servoFunctionDefault, MOUNT_DEFAULTS } from '../../ardusub/joystick.js';
import { RC_AUX_FUNCTIONS, RC_OPTIONS_BITS, rcDefault, NUM_RC_CHANNELS } from '../../ardusub/rc.js';
import { KEY_HELP, shortName } from '../input.js';

// browser "standard gamepad" button indices
const STD_NAMES = ['A / Cross', 'B / Circle', 'X / Square', 'Y / Triangle', 'LB', 'RB', 'LT', 'RT', 'Back / Share', 'Start / Options', 'Left stick press', 'Right stick press', 'D-pad up', 'D-pad down', 'D-pad left', 'D-pad right', 'Guide / Home'];
const AXIS_NAMES = ['Left stick X', 'Left stick Y', 'Right stick X', 'Right stick Y'];
const CONTROLS = [['x', 'Forward / back'], ['y', 'Lateral (strafe)'], ['z', 'Up / down (throttle)'], ['r', 'Yaw']];
const TRIG = [['none', 'Not used'], ['pitch', 'Pitch input'], ['roll', 'Roll input'], ['up', 'Ascend'], ['down', 'Descend']];
const MNT_TYPES = { 0: 'None', 1: 'Servo', 2: '3DR Solo', 3: 'Alexmos', 4: 'SToRM32', 5: 'SToRM32 serial', 6: 'Gremsy', 7: 'BrushlessPWM', 8: 'Siyi', 9: 'Scripting', 10: 'Xacti', 11: 'Viewpro', 12: 'Topotek' };
const MNT_MODES = { 0: 'Retracted', 1: 'Neutral', 2: 'MAVLink targeting', 3: 'RC targeting', 4: 'GPS point', 5: 'SysID target', 6: 'Home location' };
const RELAY_FUNCTIONS = { 0: 'None', 1: 'Relay', 2: 'Ignition', 3: 'Parachute', 4: 'Camera', 5: 'Bus switch', 6: 'Engine starter' };
const RELAY_DEFAULTS = { 0: 'Off', 1: 'On', 2: 'No change' };
const CAM_TYPES = { 0: 'None', 1: 'Servo', 2: 'Relay', 3: 'GoPro in Solo gimbal', 4: 'Mount (Siyi)', 5: 'MAVLink', 6: 'MAVLink camera v2', 7: 'Scripting' };
const COMMON_SERVO = [0, 1, 6, 7, 8, 33, 34, 35, 36, 37, 38, 39, 40, 82, 83, 84, 85, 181, 182, 183, 184, 185, 186, 187, 188, 189, ...Array.from({ length: 16 }, (_, i) => 51 + i), -1];

export default {
  id: 'joystick', title: 'Controller', short: 'Joystick', icon: 'gamepad', sub: 'Gamepad, buttons, lights, camera mount, relays and servo outputs',
  create(app) {
    const cfg = app.cfg;
    const J = cfg.joystick;
    const el = h('div', { class: 'section', style: { gap: '18px' } });
    const save = () => app.scheduleSave();
    const P = (n, d) => cfg.params[n] ?? cfg.extraParams?.[n] ?? d;
    const set = (n, v) => app.setParam(n, v);
    const fw = () => app.sim.fw;
    const live = []; // [element, () => text]
    const liveCell = (fn, attrs = {}) => { const c = h('span', { class: 'mono-cell', ...attrs }, ''); live.push([c, fn]); return c; };
    const pnum = (n, d, o = {}) => numberField({ label: o.label ?? n, step: o.step ?? 1, min: o.min, max: o.max, unit: o.unit, hint: o.hint, get: () => P(n, d), set: (v) => set(n, v) });
    const psel = (n, d, values, o = {}) => {
      const opts = Object.entries(values).map(([k, l]) => [k, `${k} · ${l}`]);
      if (!(String(P(n, d)) in values)) opts.push([String(P(n, d)), `${P(n, d)} · (other)`]);
      return selectField({ label: o.label ?? n, options: opts, get: () => P(n, d), set: (v) => set(n, +v), hint: o.hint });
    };
    const details = (title, ...kids) => h('details', { class: 'section' }, h('summary', { style: { cursor: 'pointer' } }, h('b', {}, title)), ...kids);

    // ---- status + live ----
    const status = h('div', { class: 'card' });
    const axesBox = h('div', { class: 'bars' });
    const btnBox = h('div', { class: 'row', style: { gap: '4px' } });
    el.append(h('div', { class: 'section' }, h('h3', {}, 'Controller'), status, axesBox, btnBox));

    function drawStatus() {
      const inp = app.input;
      const pad = inp.padState;
      status.replaceChildren(
        pad ? h('div', { class: 'row', style: { justifyContent: 'space-between' } }, h('b', {}, shortName(pad.id)), h('span', { class: 'pill good' }, 'Connected'))
          : inp.gamepadBlocked ? h('div', {}, h('b', {}, 'Controllers are blocked in this view'), h('p', { class: 'note' }, 'Download the simulator (Logs & files → Save simulator page) and open the HTML file in Chrome or Edge. The keyboard works everywhere.'))
            : h('div', {}, h('b', {}, 'No controller detected'), h('p', { class: 'note' }, 'Plug in an Xbox, PlayStation or Logitech gamepad and press any button. Browsers only reveal a controller after a button press.')));
      if (!pad) { axesBox.replaceChildren(); btnBox.replaceChildren(); return; }
      axesBox.replaceChildren(...pad.axes.map((v, i) => h('div', { class: 'bar-row' }, h('span', { class: 'note' }, AXIS_NAMES[i] ?? `Axis ${i}`),
        h('div', { class: 'bar-track' }, h('div', { class: 'bar-fill', style: { left: v < 0 ? `${50 + v * 50}%` : '50%', width: `${Math.abs(v) * 50}%`, background: 'var(--accent)' } })),
        h('span', { class: 'mono note', style: { textAlign: 'right' } }, `${i}: ${fmt(v, 2)}`))));
      btnBox.replaceChildren(...pad.buttons.map((v, i) => h('span', { class: 'kbd', title: STD_NAMES[i] ?? `Button ${i}`, style: { background: v > 0.5 ? 'var(--accent)' : '', color: v > 0.5 ? 'var(--accent-ink)' : '' } }, i)));
    }
    drawStatus();

    // ---- axes ----
    const axisOpts = [[-1, 'None'], ...[0, 1, 2, 3, 4, 5, 6, 7].map((i) => [i, `${i} · ${AXIS_NAMES[i] ?? 'Axis ' + i}`])];
    const axSec = h('div', { class: 'section' }, h('h3', {}, 'Sticks'),
      h('p', { class: 'note' }, 'Default is the QGroundControl / BlueOS layout: left stick throttle and yaw, right stick forward and lateral. Press "Detect" and move one stick fully.'));
    for (const [k, label] of CONTROLS) {
      const m = J.axes[k] || (J.axes[k] = { axis: -1, invert: false });
      const sel = selectField({ label, options: axisOpts, get: () => m.axis, set: (v) => { m.axis = +v; save(); } });
      const det = h('button', { class: 'btn', onclick: () => detectAxis(app, (ax, sign) => { m.axis = ax; m.invert = sign < 0; sel.refresh(); inv.querySelector('input').checked = m.invert; save(); toast(`${label}: axis ${ax}${m.invert ? ' (inverted)' : ''}`, 'good'); }) }, 'Detect');
      const inv = checkbox('Invert', () => m.invert, (v) => { m.invert = v; save(); });
      axSec.append(h('div', { style: { display: 'grid', gridTemplateColumns: '1fr auto auto', gap: '8px', alignItems: 'end' } }, sel, det, inv));
    }
    axSec.append(h('div', { class: 'grid3' },
      numberField({ label: 'Dead zone', step: 0.01, min: 0, max: 0.4, get: () => J.deadzone, set: (v) => { J.deadzone = v; save(); } }),
      numberField({ label: 'Expo', step: 0.05, min: 0, max: 1, get: () => J.expo, set: (v) => { J.expo = v; save(); } }),
      numberField({ label: 'Send rate', unit: 'Hz', step: 1, min: 2, max: 100, get: () => J.sendRate, set: (v) => { J.sendRate = v; save(); }, hint: 'MANUAL_CONTROL messages per second (QGC sends 25-50)' })),
      h('div', { class: 'grid2' },
        selectField({ label: 'Left trigger (LT)', options: TRIG, get: () => J.triggers.left, set: (v) => { J.triggers.left = v; save(); } }),
        selectField({ label: 'Right trigger (RT)', options: TRIG, get: () => J.triggers.right, set: (v) => { J.triggers.right = v; save(); } })),
      h('p', { class: 'note' }, 'Roll and pitch inputs use the MANUAL_CONTROL s/t extension fields, as BlueOS Cockpit sends them for 6-DOF frames. A trigger set to Ascend or Descend does that on either side.'));
    el.append(axSec);

    // ---- gain & link ----
    el.append(h('div', { class: 'section' }, h('h3', {}, 'Pilot gain and link'),
      h('div', { class: 'grid2' },
        pnum('JS_GAIN_DEFAULT', 0.5, { step: 0.05, min: 0.1, max: 1 }), pnum('JS_GAIN_MIN', 0.25, { step: 0.05, min: 0.1, max: 0.8 }),
        pnum('JS_GAIN_MAX', 1, { step: 0.05, min: 0.2, max: 1 }), pnum('JS_GAIN_STEPS', 4, { min: 1, max: 10 }),
        pnum('JS_THR_GAIN', 1, { step: 0.05, min: 0.5, max: 4 }), pnum('JS_LIGHTS_STEPS', 8, { min: 1, max: 10 }),
        pnum('SYSID_MYGCS', 255, { min: 1, max: 255, hint: 'MANUAL_CONTROL is only accepted from this system id (the simulator GCS is 255, like QGC)' }),
        pnum('SYSID_THISMAV', 1, { min: 1, max: 255, hint: 'MANUAL_CONTROL must target this system id' })),
      h('p', { class: 'note' }, 'Gain buttons save JS_GAIN_MIN/MAX/STEPS clamped to 0.1-0.8 / min-1 / 1-10, as ArduSub does. The simulated GCS sends from system 255 to target 1: other SYSID values make the vehicle ignore the controller, as on the real vehicle.'),
      h('div', { class: 'row' }, h('span', { class: 'note' }, 'Gain now:'), liveCell(() => `${Math.round(fw().gain * 100)} %`), h('span', { class: 'note' }, 'Rejected messages:'), liveCell(() => String(fw().js.rejected ?? 0)))));

    // ---- buttons ----
    const fnSelect = (p) => {
      const cur = P(p, 0);
      const groups = BUTTON_FUNCTION_GROUPS.map(([g, list]) => h('optgroup', { label: g }, list.map((k) => h('option', { value: String(k) }, `${k} · ${buttonFunctionLabel(k)}`))));
      if (!(cur in BUTTON_FUNCTIONS)) groups.push(h('option', { value: String(cur) }, `${cur} · (other)`));
      const s = h('select', { 'aria-label': p, onchange: () => { set(p, +s.value); s.blur(); } }, groups);
      s.value = String(cur);
      return s;
    };
    const sdlOpts = [[-1, 'Not used'], ...Array.from({ length: 32 }, (_, i) => [i, `${i} · ${SDL_BUTTON_NAMES[i] ?? 'Button ' + i}`])];
    const nPad = Math.max(STD_NAMES.length, app.input.padState?.buttons.length ?? 0, ...Object.keys(J.buttonMap || {}).map((k) => +k + 1));
    const rows = Array.from({ length: nPad }, (_, std) => {
      const name = STD_NAMES[std] ?? `Pad button ${std}`;
      const sdl = J.buttonMap[std] ?? -1;
      const mapSel = h('select', { 'aria-label': `${name} sends button`, onchange: () => { const v = +mapSel.value; if (v < 0) delete J.buttonMap[std]; else J.buttonMap[std] = v; save(); app.refreshPanel(); } }, sdlOpts.map(([v, l]) => h('option', { value: String(v) }, l)));
      mapSel.value = String(sdl);
      const fn = (shift) => (sdl < 0 ? h('span', { class: 'note' }, '—') : fnSelect(`BTN${sdl}_${shift ? 'SFUNCTION' : 'FUNCTION'}`));
      return h('tr', {}, h('td', {}, name), h('td', {}, mapSel), h('td', {}, fn(false)), h('td', {}, fn(true)));
    });
    const btnLive = liveCell(() => {
      const js = fw().js, on = [];
      for (let i = 0; i < 32; i++) if ((js.buttons ?? js.buttonsPrev) & (1 << i)) on.push(i);
      return on.length ? `${on.join(', ')}${js.shift ? ' (shift)' : ''}` : 'none';
    });
    const allRows = Array.from({ length: 32 }, (_, i) => h('tr', {}, h('td', {}, `BTN${i}`), h('td', { class: 'note' }, SDL_BUTTON_NAMES[i] ?? ''), h('td', {}, fnSelect(`BTN${i}_FUNCTION`)), h('td', {}, fnSelect(`BTN${i}_SFUNCTION`))));
    el.append(h('div', { class: 'section' }, h('h3', {}, 'Buttons'),
      h('p', { class: 'note' }, 'Each physical button sends a QGroundControl button number (0-15 in MANUAL_CONTROL buttons, 16-31 in buttons2); ArduSub decides what it does with BTNn_FUNCTION, or BTNn_SFUNCTION while a button whose function is "shift" is held. Keyboard Shift presses that shift button.'),
      h('div', { class: 'row' }, h('span', { class: 'note' }, 'Pressed now:'), btnLive),
      h('div', { class: 'table-wrap' }, h('table', { class: 'data' }, h('thead', {}, h('tr', {}, h('th', {}, 'Button'), h('th', {}, 'Sends'), h('th', {}, 'Function'), h('th', {}, 'With shift'))), h('tbody', {}, rows))),
      details('All 32 ArduSub buttons (BTN0-BTN31)', h('div', { class: 'table-wrap' }, h('table', { class: 'data' }, h('thead', {}, h('tr', {}, h('th', {}, 'Param'), h('th', {}, 'QGC name'), h('th', {}, 'FUNCTION'), h('th', {}, 'SFUNCTION'))), h('tbody', {}, allRows))))));

    // ---- lights & camera ----
    const tiltTxt = () => { const t = fw().js.cameraTiltDeg?.(); return t === null || t === undefined ? 'no tilt servo' : `${fmt(t, 1)}°`; };
    el.append(h('div', { class: 'section' }, h('h3', {}, 'Lights and camera mount'),
      h('div', { class: 'row' }, h('span', { class: 'note' }, 'Lights 1:'), liveCell(() => `${Math.round(fw().js.lights1 / 10)} % → ${Math.round((fw().js.lightLevel?.(0) ?? 0) * 100)} % lamp`),
        h('span', { class: 'note' }, 'Lights 2:'), liveCell(() => `${Math.round(fw().js.lights2 / 10)} % → ${Math.round((fw().js.lightLevel?.(1) ?? 0) * 100)} % lamp`),
        h('span', { class: 'note' }, 'Video:'), liveCell(() => `source ${fw().js.cameraSource ?? 1}`)),
      h('div', { class: 'row' }, h('span', { class: 'note' }, 'Mount target:'), liveCell(() => { const m = fw().js.mount; return m ? `tilt ${fmt(m.pitch, 1)}° pan ${fmt(m.yaw, 1)}°` : '—'; }), h('span', { class: 'note' }, 'Camera tilt (servo):'), liveCell(tiltTxt)),
      h('p', { class: 'note' }, 'Lights are SRV outputs lights1/lights2 (SERVOn_FUNCTION 181/182) scaled 0-1000 onto SERVOn_MIN..MAX; the lamp brightness assumes a Lumen (1100 µs off, 1900 µs full). The camera tilts with the mount tilt servo (SERVOn_FUNCTION 7) when MNT1_TYPE is Servo; tilt buttons send RC8 as a rate input at MNT1_RC_RATE (RC8_OPTION 213), limited by MNT1_PITCH_MIN/MAX.'),
      P('MNT1_TYPE', MOUNT_DEFAULTS.MNT1_TYPE) === 0 ? h('p', { class: 'note warn' }, 'MNT1_TYPE = 0: this vehicle has no camera mount configured, so the tilt and pan buttons do nothing (as on the Pixhawk).') : null,
      h('div', { class: 'grid2' }, psel('MNT1_TYPE', MOUNT_DEFAULTS.MNT1_TYPE, MNT_TYPES, { hint: 'Only Servo (1) is simulated' }), psel('MNT1_DEFLT_MODE', 3, MNT_MODES),
        pnum('MNT1_RC_RATE', 30, { unit: 'deg/s', min: 0, max: 90, hint: '0 = the RC input is an angle, not a rate' }),
        pnum('MNT1_PITCH_MIN', -90, { unit: 'deg', min: -90, max: 90 }), pnum('MNT1_PITCH_MAX', 20, { unit: 'deg', min: -90, max: 90 }),
        pnum('MNT1_YAW_MIN', -180, { unit: 'deg', min: -180, max: 180 }), pnum('MNT1_YAW_MAX', 180, { unit: 'deg', min: -180, max: 180 }),
        pnum('MNT1_ROLL_MIN', -30, { unit: 'deg', min: -180, max: 180 }), pnum('MNT1_ROLL_MAX', 30, { unit: 'deg', min: -180, max: 180 }),
        psel('CAM1_TYPE', 0, CAM_TYPES, { hint: 'camera_trigger does nothing in ArduSub 4.5.7 (joystick.cpp:202); kept for export' }))));

    // ---- relays ----
    const relayRows = Array.from({ length: 6 }, (_, i) => {
      const k = i + 1;
      const fnS = h('select', { 'aria-label': `RELAY${k}_FUNCTION`, onchange: () => set(`RELAY${k}_FUNCTION`, +fnS.value) }, Object.entries(RELAY_FUNCTIONS).map(([v, l]) => h('option', { value: v }, `${v} · ${l}`)));
      fnS.value = String(P(`RELAY${k}_FUNCTION`, 0));
      const dS = h('select', { 'aria-label': `RELAY${k}_DEFAULT`, onchange: () => set(`RELAY${k}_DEFAULT`, +dS.value) }, Object.entries(RELAY_DEFAULTS).map(([v, l]) => h('option', { value: v }, l)));
      dS.value = String(P(`RELAY${k}_DEFAULT`, 0));
      const pin = h('input', { type: 'number', step: 1, style: { width: '5em' }, value: String(P(`RELAY${k}_PIN`, -1)), onchange: () => set(`RELAY${k}_PIN`, +pin.value) });
      const inv = h('input', { type: 'checkbox', onchange: () => set(`RELAY${k}_INVERTED`, inv.checked ? 1 : 0) }); inv.checked = !!P(`RELAY${k}_INVERTED`, 0);
      return h('tr', {}, h('td', {}, `Relay ${k}`), h('td', {}, fnS), h('td', {}, pin), h('td', {}, dS), h('td', {}, inv), h('td', {}, liveCell(() => (P(`RELAY${k}_FUNCTION`, 0) !== 1 ? 'not a relay' : `${fw().js.relays[i] ? 'ON' : 'off'} (pin ${fw().js.relayPinState?.(i) ? 'high' : 'low'})`))));
    });
    el.append(h('div', { class: 'section' }, h('h3', {}, 'Relays'),
      h('p', { class: 'note' }, 'relay_n_on/off/toggle/momentary buttons switch relay n only when RELAYn_FUNCTION is Relay (1), as AP_Relay does in 4.5. Momentary turns on at the press and off at the release.'),
      h('div', { class: 'table-wrap' }, h('table', { class: 'data' }, h('thead', {}, h('tr', {}, h('th', {}, ''), h('th', {}, 'FUNCTION'), h('th', {}, 'PIN'), h('th', {}, 'DEFAULT'), h('th', {}, 'INVERTED'), h('th', {}, 'State'))), h('tbody', {}, relayRows)))));

    // ---- actuators & scripting ----
    const actRows = Array.from({ length: 6 }, (_, i) => h('tr', {}, h('td', {}, `Actuator ${i + 1}`),
      h('td', {}, numberField({ label: `ACTUATOR${i + 1}_INC`, step: 0.005, min: 0, max: 1, get: () => P(`ACTUATOR${i + 1}_INC`, 0.01), set: (v) => set(`ACTUATOR${i + 1}_INC`, v) })),
      h('td', {}, liveCell(() => { const n = fw().js.findServo?.(184 + i); return n ? `SERVO${n}` : 'no SERVOn_FUNCTION ' + (184 + i); })),
      h('td', {}, liveCell(() => `${fmt((fw().js.actuators?.[i] ?? 0) * 100, 0)} %`))));
    el.append(h('div', { class: 'section' }, h('h3', {}, 'Actuators and script buttons'),
      h('p', { class: 'note' }, 'actuator_n buttons move a 0-1 value by ACTUATORn_INC (min / max / centre / toggles; momentary buttons centre on release) that drives the SERVO channel whose function is Actuator n (184-189). At boot ArduSub gives legacy servo_1-3 buttons a disabled SERVO9-11.'),
      h('div', { class: 'table-wrap' }, h('table', { class: 'data' }, h('thead', {}, h('tr', {}, h('th', {}, ''), h('th', {}, 'Increment'), h('th', {}, 'Output'), h('th', {}, 'Value'))), h('tbody', {}, actRows))),
      h('div', { class: 'row' }, h('span', { class: 'note' }, 'Script buttons 1-4:'), liveCell(() => (fw().js.scriptButtons ?? []).map((b, i) => `${i + 1}:${b.pressed ? 'down' : 'up'}/${b.count}`).join('  ')))));

    // ---- SERVO outputs ----
    const servoFnOpts = (cur) => {
      const list = COMMON_SERVO.map((v) => [v, `${v} · ${SERVO_FUNCTIONS[v] ?? 'function ' + v}`]);
      if (!COMMON_SERVO.includes(cur)) list.push([cur, `${cur} · ${SERVO_FUNCTIONS[cur] ?? '(other)'}`]);
      return list;
    };
    const servoRows = Array.from({ length: 16 }, (_, i) => {
      const n = i + 1, fnN = `SERVO${n}_FUNCTION`;
      const cur = P(fnN, servoFunctionDefault(n));
      const fnS = h('select', { 'aria-label': fnN, onchange: () => { set(fnN, +fnS.value); fnS.blur(); } }, servoFnOpts(cur).map(([v, l]) => h('option', { value: String(v) }, l)));
      fnS.value = String(cur);
      const num = (k, d) => { const inp = h('input', { type: 'number', step: 1, style: { width: '5.2em' }, 'aria-label': `SERVO${n}_${k}`, value: String(P(`SERVO${n}_${k}`, d)), onchange: () => set(`SERVO${n}_${k}`, +inp.value), onkeydown: (e) => { if (e.key === 'Enter') inp.blur(); } }); return inp; };
      const rev = h('input', { type: 'checkbox', 'aria-label': `SERVO${n}_REVERSED`, onchange: () => set(`SERVO${n}_REVERSED`, rev.checked ? 1 : 0) }); rev.checked = !!P(`SERVO${n}_REVERSED`, 0);
      return h('tr', {}, h('td', {}, `SERVO${n}`), h('td', {}, fnS), h('td', {}, num('MIN', 1100)), h('td', {}, num('TRIM', 1500)), h('td', {}, num('MAX', 1900)), h('td', {}, rev),
        h('td', {}, liveCell(() => { const o = fw().js.servoOutputs?.()[i]; return o?.pwm === null || o?.pwm === undefined ? '—' : `${o.pwm} µs`; })));
    });
    el.append(h('div', { class: 'section' }, h('h3', {}, 'Servo outputs'),
      h('p', { class: 'note' }, 'What the Pixhawk sends on each output, live. Motors come from the mixer; lights, mount, video switch, actuators and RCIN pass-through follow the buttons. "—" is an output the simulator does not drive.'),
      h('div', { class: 'table-wrap' }, h('table', { class: 'data' }, h('thead', {}, h('tr', {}, h('th', {}, ''), h('th', {}, 'FUNCTION'), h('th', {}, 'MIN'), h('th', {}, 'TRIM'), h('th', {}, 'MAX'), h('th', {}, 'REV'), h('th', {}, 'PWM'))), h('tbody', {}, servoRows)))));

    // ---- RC channels ----
    const rcRows = Array.from({ length: NUM_RC_CHANNELS }, (_, i) => {
      const n = i + 1;
      const num = (k, lo, hi) => { const inp = h('input', { type: 'number', step: 1, min: lo, max: hi, style: { width: '5.2em' }, 'aria-label': `RC${n}_${k}`, value: String(P(`RC${n}_${k}`, rcDefault(n, k))), onchange: () => { const v = +inp.value; if (v >= lo && v <= hi) set(`RC${n}_${k}`, v); else toast(`RC${n}_${k} must be ${lo}-${hi}`, 'bad'); }, onkeydown: (e) => { if (e.key === 'Enter') inp.blur(); } }); return inp; };
      const rev = h('input', { type: 'checkbox', 'aria-label': `RC${n}_REVERSED`, onchange: () => set(`RC${n}_REVERSED`, rev.checked ? 1 : 0) }); rev.checked = !!P(`RC${n}_REVERSED`, 0);
      const optCur = P(`RC${n}_OPTION`, rcDefault(n, 'OPTION'));
      const opt = h('select', { 'aria-label': `RC${n}_OPTION`, onchange: () => { set(`RC${n}_OPTION`, +opt.value); opt.blur(); } }, [...Object.entries(RC_AUX_FUNCTIONS), ...(optCur in RC_AUX_FUNCTIONS ? [] : [[optCur, '(other)']])].map(([v, l]) => h('option', { value: String(v) }, `${v} · ${l}`)));
      opt.value = String(optCur);
      return h('tr', {}, h('td', {}, `RC${n}`), h('td', {}, num('MIN', 800, 2200)), h('td', {}, num('TRIM', 800, 2200)), h('td', {}, num('MAX', 800, 2200)), h('td', {}, num('DZ', 0, 200)), h('td', {}, rev), h('td', {}, opt),
        h('td', {}, liveCell(() => { const c = fw().rc?.all?.[i]; return c ? `${c.radioIn}${c.hasOverride?.() ? ' ovr' : ''}` : '—'; })));
    });
    const optBits = RC_OPTIONS_BITS.map((label, b) => checkbox(`${b}: ${label}`, () => (P('RC_OPTIONS', 32) >>> 0) & (1 << b), (v) => { const cur = P('RC_OPTIONS', 32) >>> 0; set('RC_OPTIONS', v ? cur | (1 << b) : cur & ~(1 << b)); }));
    el.append(h('div', { class: 'section' }, h('h3', {}, 'RC channels'),
      h('p', { class: 'note' }, 'The joystick reaches ArduSub as RC overrides on channels 1-8 (1 pitch, 2 roll, 3 throttle, 4 yaw, 5 forward, 6 lateral, 7 pan, 8 tilt). ArduSub does not run RC aux switches; RCn_OPTION only marks the mount input channels (213 Mount1 Pitch, 214 Mount1 Yaw). Overrides time out after RC_OVERRIDE_TIME (0 disables them, -1 never times out).'),
      h('div', { class: 'grid2' }, pnum('RC_OVERRIDE_TIME', 3, { unit: 's', step: 0.5, min: -1, max: 120 }), pnum('RC_FS_TIMEOUT', 1, { unit: 's', step: 0.1, min: 0.5, max: 10, hint: 'Receiver failsafe (no receiver in the simulator)' }),
        pnum('RC_PROTOCOLS', 1, { hint: 'Receiver protocol mask (no receiver in the simulator)' }), pnum('RC_SPEED', 490, { unit: 'Hz', hint: 'ESC update rate (exported only)' })),
      details('RC_OPTIONS bits', h('div', { class: 'section' }, optBits)),
      h('div', { class: 'table-wrap' }, h('table', { class: 'data' }, h('thead', {}, h('tr', {}, h('th', {}, ''), h('th', {}, 'MIN'), h('th', {}, 'TRIM'), h('th', {}, 'MAX'), h('th', {}, 'DZ'), h('th', {}, 'REV'), h('th', {}, 'OPTION'), h('th', {}, 'PWM in'))), h('tbody', {}, rcRows)))));

    // ---- keyboard ----
    el.append(h('div', { class: 'section' }, h('h3', {}, 'Keyboard'),
      h('div', { class: 'table-wrap' }, h('table', { class: 'data' }, h('tbody', {}, KEY_HELP.map(([k, d]) => h('tr', {}, h('td', {}, h('span', { class: 'kbd' }, k)), h('td', { style: { whiteSpace: 'normal' } }, d))))))));

    const tick = () => {
      drawStatus();
      for (const [c, f] of live) { try { const t = f(); if (c.textContent !== t) c.textContent = t; } catch { c.textContent = '—'; } }
    };
    tick();
    return { el, tick };
  },
};

// Wait for one stick axis to move past 0.6 and report it.
function detectAxis(app, done) {
  const start = app.input.padState?.axes.slice();
  if (!start) { toast('Connect a controller first', 'bad'); return; }
  toast('Move the stick fully in the positive direction (forward, right, up or yaw right)…', '', 4000);
  const t0 = performance.now();
  const poll = () => {
    const a = app.input.padState?.axes;
    if (a) for (let i = 0; i < a.length; i++) if (Math.abs(a[i] - (start[i] ?? 0)) > 0.6) { done(i, a[i] - (start[i] ?? 0)); return; }
    if (performance.now() - t0 < 6000) setTimeout(poll, 50); else toast('No stick movement detected', 'bad');
  };
  poll();
}
