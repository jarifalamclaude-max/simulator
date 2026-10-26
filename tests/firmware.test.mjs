import test from 'node:test';
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import { sampleGeometry } from '../src/vehicle/sample.js';
import { analyzeGeometry } from '../src/geometry/analyze.js';
import { defaultConfig, applyParamFile } from '../src/vehicle/config.js';
import { parseParamFile } from '../src/ardusub/params.js';
import { Simulation, MODE } from '../src/sim/simulation.js';
import { RCChannel } from '../src/ardusub/rc.js';
import { Motors6DOF, SPOOL } from '../src/ardusub/motors.js';
import { frameTable } from '../src/alloc/frames.js';
import { sqrtController, invSqrtController } from '../src/ardusub/control.js';
import { waterDensity } from '../src/hydro/water.js';

const geo = analyzeGeometry(sampleGeometry().positions);
const duburi = parseParamFile(readFileSync(new URL('../profiles/duburi_pixhawk_param.param', import.meta.url), 'utf8')).params;

function makeSim({ mixer = 'frame', sync = true, start = [0, 0, 1.5] } = {}) {
  const cfg = defaultConfig();
  applyParamFile(cfg, duburi, { syncPolarity: sync });
  cfg.mixer.source = mixer;
  cfg.mass.mass = waterDensity(cfg.environment.temperature) * geo.volume * 0.985; // slightly positive
  cfg.start = { position: start, yaw: 0 };
  return new Simulation(cfg, geo);
}
const deg = (r) => (r * 180) / Math.PI;

test('param file parses (Mission Planner format) and maps known params', () => {
  assert.equal(duburi.FRAME_CONFIG, 2);
  assert.equal(duburi.MOT_2_DIRECTION, -1);
  const cfg = defaultConfig();
  const rep = applyParamFile(cfg, duburi);
  assert.ok(rep.applied > 100 && rep.kept > 100);
  assert.deepEqual(rep.polarityChanged, [2, 3, 7, 8]);
});

test('RC channel scaling matches ArduPilot integer maths', () => {
  const c = new RCChannel({ min: 1100, max: 1900, trim: 1500, dz: 30 });
  c.setAngle(4500);
  c.setOverride(1700); assert.equal(c.controlIn, Math.trunc((4500 * (1700 - 1530)) / (1900 - 1530)));
  c.setOverride(1510); assert.equal(c.controlIn, 0);
  const t = new RCChannel({ min: 1100, max: 1900, trim: 1100, dz: 30 });
  t.setRange(1000);
  t.setOverride(1500); assert.equal(t.normInput(), 0.5);
  assert.equal(t.getControlMid(), Math.trunc((1000 * (1500 - 1130)) / (1900 - 1130)));
});

test('sqrt controller and its inverse agree', () => {
  for (const e of [0.01, 0.5, 3]) {
    const out = sqrtController(e, 6, 2, 0);
    assert.ok(Math.abs(invSqrtController(out, 6, 2) - e) < 1e-9);
  }
});

test('vectored-6DOF mixer reproduces firmware maths (group normalisation)', () => {
  const m = new Motors6DOF();
  const P = { MOT_PWM_MIN: 1100, MOT_PWM_MAX: 1900, MOT_SPOOL_TIME: 0.5 };
  m.configure({ factors: frameTable(2), variant: 'vectored6dof', params: P });
  m.arm(); m.setDesiredSpoolState(SPOOL.THROTTLE_UNLIMITED);
  m.setThrottle(0.5); m.setForward(1); m.setYaw(1);
  for (let i = 0; i < 400; i++) m.output(0.0025);
  // forward 1 + yaw 1 saturates horizontal motors; normalised by yfl_max = 2
  assert.equal(m.spool, SPOOL.THROTTLE_UNLIMITED);
  assert.ok(Math.abs(m.thrust[0] - (1 * 1 + 1 * -1) / 2) < 1e-9); // motor 1: yaw +1, fwd -1
  assert.ok(Math.abs(m.thrust[3] - (1 + 1) / 2) < 1e-9); // motor 4: yaw +1, fwd +1
  assert.equal(m.pwm[3], 1900);
});

test('arming spools up after MOT_SPOOL_TIME, outputs neutral before', () => {
  const sim = makeSim();
  sim.fw.setMode(MODE.STABILIZE);
  assert.ok(sim.fw.arm());
  sim.runFor(0.2);
  assert.ok(sim.fw.motors.pwm.every((p) => p === 1500), 'neutral during ground idle');
  sim.runFor(1.0);
  assert.equal(sim.fw.motors.spool, SPOOL.THROTTLE_UNLIMITED);
});

for (const mixer of ['frame', 'auto']) {
  test(`stabilize holds level and heading with no pilot input (${mixer} mixer)`, () => {
    const sim = makeSim({ mixer });
    sim.fw.setMode(MODE.STABILIZE);
    sim.fw.arm();
    sim.runFor(8);
    const e = sim.state().euler;
    assert.ok(Math.abs(deg(e[0])) < 3 && Math.abs(deg(e[1])) < 3, `attitude ${e.map(deg)}`);
    assert.ok(Math.abs(deg(e[2])) < 3, `heading drift ${deg(e[2])}`);
  });

  test(`depth hold keeps depth within 5 cm (${mixer} mixer)`, () => {
    const sim = makeSim({ mixer });
    sim.fw.setMode(MODE.ALT_HOLD);
    sim.runFor(0.5); // disarmed in depth hold: controller relaxes like on the real vehicle
    sim.fw.arm();
    sim.runFor(3);
    const d0 = sim.plant.position[2];
    sim.runFor(10);
    const d1 = sim.plant.position[2];
    assert.ok(Math.abs(d1 - d0) < 0.05, `depth moved ${d0} -> ${d1}`);
  });

  test(`pilot yaw input turns, release holds the new heading (${mixer} mixer)`, () => {
    const sim = makeSim({ mixer });
    sim.fw.setMode(MODE.STABILIZE);
    sim.fw.arm();
    sim.runFor(1.5);
    sim.setPilotInput({ r: 600 });
    sim.runFor(2);
    sim.setPilotInput({ r: 0 });
    sim.runFor(1);
    const yaw1 = deg(sim.state().euler[2]);
    assert.ok(Math.abs(yaw1) > 20, `turned only ${yaw1} deg`);
    sim.runFor(4);
    const yaw2 = deg(sim.state().euler[2]);
    assert.ok(Math.abs(yaw2 - yaw1) < 3, `heading drift ${yaw1} -> ${yaw2}`);
  });
}

test('forward stick moves the vehicle forward', () => {
  const sim = makeSim();
  sim.fw.setMode(MODE.ALT_HOLD);
  sim.fw.arm();
  sim.runFor(1);
  sim.setPilotInput({ x: 800 });
  sim.runFor(5);
  assert.ok(sim.plant.position[0] > 1, `x = ${sim.plant.position[0]}`);
  assert.ok(Math.abs(sim.plant.position[1]) < 0.5);
});

test('joystick buttons arm and change mode via BTN params (QGC numbering)', () => {
  const sim = makeSim();
  sim.setPilotInput({ buttons: 1 << 6 }); // Start = arm
  sim.runFor(0.1);
  sim.setPilotInput({ buttons: 0 });
  sim.runFor(0.1);
  assert.ok(sim.fw.armed);
  sim.setPilotInput({ buttons: 1 << 2 }); // X = depth hold
  sim.runFor(0.1);
  sim.setPilotInput({ buttons: 0 });
  sim.runFor(0.1);
  assert.equal(sim.fw.mode, MODE.ALT_HOLD);
  sim.setPilotInput({ buttons: 1 << 11 }); // D-pad up = gain +
  sim.runFor(0.1);
  sim.setPilotInput({ buttons: 0 });
  sim.runFor(0.1);
  assert.ok(sim.fw.gain > 0.5);
  sim.setPilotInput({ buttons: 1 << 4 }); // Back = disarm
  sim.runFor(0.1);
  assert.ok(!sim.fw.armed);
});

test('mis-wired thrusters (MOT_DIRECTION without matching hardware) destabilise the vehicle', () => {
  const sim = makeSim({ sync: false });
  sim.fw.setMode(MODE.STABILIZE);
  sim.fw.arm();
  sim.setPilotInput({ r: 400 });
  sim.runFor(6);
  const e = sim.state().euler.map(deg);
  const ok = Math.abs(e[0]) < 5 && Math.abs(e[1]) < 5;
  assert.ok(!ok || Math.abs(e[2]) < 5, 'expected loss of control or no yaw response');
});
