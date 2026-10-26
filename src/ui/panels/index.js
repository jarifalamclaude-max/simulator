// Setup rail order: build the vehicle, check what it can do, fly and tune.
import vehicle from './vehicle.js';
import thrusters from './thrusters.js';
import frame from './frame.js';
import mixer from './mixer.js';
import hydro from './hydro.js';
import cfd from './cfd.js';
import env from './environment.js';
import params from './params.js';
import joystick from './joystick.js';
import motors from './motors.js';
import tuning from './tuning.js';
import mission from './mission.js';
import logs from './logs.js';
import team from './team.js';
import help from './help.js';
import autotune from './autotune.js';

export const PANELS = [vehicle, thrusters, frame, mixer, hydro, cfd, env, params, joystick, motors, tuning, mission, logs, team, help, autotune];
