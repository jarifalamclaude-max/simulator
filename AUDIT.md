# ArduSub 4.5.7 fidelity audit

Every subsystem of the simulator was compared line by line with the ArduPilot source at tag `ArduSub-4.5.7`, and each finding was checked by an independent verifier. Below: what was wrong in the previous version and what changed. All findings are fixed in this version unless marked *partial*.

**109 confirmed findings**: 5 critical, 20 major, 84 minor; plus 82 groups of missing ArduSub features and parameters, added in this version (a few only in part: no GPS arming checks, no windmilling prop drag, no wave radiation damping, approximate T500 table, frozen-geometry CFD for rotation).

## Motors / attitude / PID

| Severity | File | What was wrong |
|---|---|---|
| major | `src/physics/thruster.js` | Reversed prop/ESC polarity flips the thrust sign instead of mirroring the curve about 1500 us: strong and weak thrust land on the wrong rotation |
| major | `src/physics/thruster.js` | Thrust has no supply-voltage dependence (fixed preset curve, no battery sag) |
| major | `src/physics/thruster.js` | Reverse-thrust jet-hull deduction (deductionRev) is computed but never used |
| major | `src/ardusub/sub.js` | Simulator blocks arming on AC_AttitudeControl::pre_arm_checks, which ArduSub 4.5.7 never calls |
| major | `src/ardusub/pid.js` | AC_PID_2D integrator anti-windup freezes the whole increment instead of limiting integrator length |
| minor | `src/ardusub/motors.js` | Default automatic and custom mixers fly a non-firmware saturation scheme ('priority'), so what is tuned in the sim is not what the exported table does on ArduSub *(partial)* |
| minor | `src/ardusub/motors.js` | MOT_THST_HOVER is taken from the user's parameters; ArduSub forces it to 0.5 at boot and clamps get_throttle_hover() |
| minor | `src/ardusub/motors.js` | AP_Motors6DOF battery-current limiter (MOT_BAT_CURR_MAX / MOT_BAT_CURR_TC) is not ported |
| minor | `src/physics/thruster.js` | Rotor lag: first-order on thrust with a magnitude-only time-constant choice; braking through a reversal uses the spool-up constant *(partial)* |
| minor | `src/ardusub/motors.js` | MOT_n_DIRECTION is coerced to +/-1; ArduPilot multiplies by the raw int8 value |
| minor | `src/ardusub/motors.js` | setDesiredSpoolState accepts requests while disarmed |
| minor | `src/ardusub/motors.js` | Spool logic ignores MOT_SPOOL_TIM_DN, the GROUND_IDLE->GROUND_IDLE ramp, and the MOT_SAFE_DISARM/MOT_SAFE_TIME gate |
| minor | `src/ardusub/motors.js` | Motor test path differs from ArduSub: the PWM is clamped, untested motors are forced to 1500, and mixer and spool logic keep running |
| minor | `src/ardusub/motors.js` | The mixer table and MOT_* values can be swapped while armed *(partial)* |
| minor | `src/physics/thruster.js` | Parametric thrust curve sampling puts non-zero thrust inside the ESC deadband |
| minor | `src/ardusub/attitude.js` | preArmChecks ignores the FF branch, and setParams never passes ATC_RAT_*_FF to the PIDs *(partial)* |
| minor | `src/ardusub/attitude.js` | relaxAttitudeControllers zeroes euler_rate_target at +/-90 deg pitch instead of leaving it unchanged |
| minor | `src/ardusub/attitude.js` | setThrottleOut never sets throttle_avg_max *(partial)* |
| minor | `src/ardusub/attitude.js` | rateControllerRun skips update_throttle_rpy_mix and control_monitor_update *(partial)* |
| minor | `src/core/math.js` | wrap180cd returns -18000 where ArduPilot returns +18000 |
| minor | `src/core/math.js` | Axis-angle helpers use a 1e-12 zero threshold instead of FLT_EPSILON |
| minor | `src/ardusub/control.js` | calcLowpassAlpha returns 1 for dt=0 (ArduPilot returns 0), which affects the rate PID filters |
| minor | `src/ardusub/params.js` | ATC parameter ranges used by validateParam do not match the ArduSub docs |
| minor | `src/ardusub/poscontrol.js` | PosControl XY jerk limit ignores the attitude limits, and the attitude controller has no getters for them *(partial)* |
| minor | `src/ardusub/pid.js` | SMAX slew-rate limiter (Dmod) missing from AC_PID.update_all |
| minor | `src/ardusub/pid.js` | D_FF (derivative feed-forward) missing; getFF() omits DFF and targetDerivative is dead code |
| minor | `src/ardusub/pid.js` | AC_PID_2D has no NaN/Inf input guard, so a NaN stays in the integrator |
| minor | `src/ardusub/control.js` | calcLowpassAlpha returns 1 (pass-through) at dt=0 where ArduPilot returns 0 (hold) |
| minor | `src/core/math.js` | clamp() does not reproduce constrain_float's NaN-to-midpoint behaviour used by the integrator setters |
| minor | `src/ardusub/pid.js` | FLT_EPSILON guards replaced by plain > 0 (derivative divide, 2-D limit dot product, length clamps) |
| minor | `src/ardusub/poscontrol.js` | PSC_VELXY_FF never wired into AC_PID_2D, and the XY integrator init omits the -vel_target*ff term |
| minor | `src/ardusub/control.js` | LowPassFilter2p bypasses the filter at cutoff >= fs/2 instead of clamping cutoff to 0.4*fs |
| minor | `src/ardusub/params.js` | PID parameter metadata ranges do not match ArduSub 4.5.7 |

## Modes, failsafes, depth & position control

| Severity | File | What was wrong |
|---|---|---|
| critical | `src/ardusub/sub.js` | Unsanitised ANGLE_MAX is used as a divisor in POSHOLD: NaN thrust, physics diverges |
| major | `src/ardusub/sub.js` | requires_GPS is inverted for SURFACE and POSHOLD: SURFACE (and the leak failsafe) works without a position fix |
| major | `src/ardusub/sub.js` | Pre-arm checks are not ArduSub's: an extra ATC gain check, while the Sub checks are missing |
| major | `src/sim/simulation.js` | A keep-state rebuild recreates the firmware and loses latched failsafe and pilot state mid-dive |
| major | `src/ardusub/poscontrol.js` | initXYController seeds the XY velocity integrator from a stale accelTarget instead of the current lean-angle target |
| major | `src/ardusub/poscontrol.js` | initXYController never resets roll/pitch targets to the attitude target, which changes POSHOLD output during pilot repositioning and on entry |
| minor | `src/vehicle/config.js` | Default position_ok() is true and is not derived from the vehicle's parameters *(partial)* |
| minor | `src/ardusub/sub.js` | Bottom detector uses a consecutive, loop-rate-scaled counter instead of ArduSub's cumulative 400-tick counter |
| minor | `src/ardusub/sub.js` | SURFACE returns right after switching to ALT_HOLD instead of finishing the loop |
| minor | `src/ardusub/sub.js` | Pilot-input failsafe runs at 10 Hz, starts inactive, and prints a message ArduSub never sends |
| minor | `src/ardusub/motors.js` | MOT_THST_HOVER from the param file changes depth hold, but ArduSub forces it to 0.5 at boot |
| minor | `src/ardusub/control.js` | LowPassFilter2p and calcLowpassAlpha edge cases differ from ArduPilot |
| minor | `src/ardusub/poscontrol.js` | PSC_ANGLE_MAX (and the lean-angle override) is ignored; XY accel limit always uses ANGLE_MAX |
| minor | `src/ardusub/poscontrol.js` | PSC_VELXY_FF is not passed to the XY velocity PID |
| minor | `src/ardusub/pid.js` | AC_PID_2D integrator anti-windup differs: the port freezes the integrator instead of allowing rotation at constant length |
| minor | `src/ardusub/pid.js` | Z accel PID silently drops PSC_ACCZ_SMAX, PSC_ACCZ_D_FF, PSC_ACCZ_NTF and PSC_ACCZ_NEF |
| minor | `src/ardusub/poscontrol.js` | jerkMaxXY is undefined until setMaxSpeedAccelXY and is never limited by attitude rate or acceleration limits |
| minor | `src/ardusub/poscontrol.js` | XY accel limit is a plain length clamp instead of limit_accel_xy (cross-track priority); fwd_pitch_is_limited is never computed *(partial)* |
| minor | `src/ardusub/sub.js` | Disarmed ALT_HOLD returns before control_depth(), so update_z_controller does not run while disarmed as it does in ArduSub |
| minor | `src/ardusub/motors.js` | Hover throttle used by the depth controller is neither clamped nor forced to 0.5 as ArduSub does |
| minor | `src/ardusub/sub.js` | WPNAV_ACCEL <= 0 is passed raw instead of falling back to 250 cm/s^2 |
| minor | `src/ardusub/sub.js` | depthTargetOverride hook (used by AUTO-TUNE) drives AC_PosControl in a way ArduSub never does |

## Joystick, RC, parameters, UI

| Severity | File | What was wrong |
|---|---|---|
| critical | `src/sim/simulation.js` | Live rebuild while armed trips the pilot-input failsafe (disarms) and wipes all joystick/controller state |
| major | `src/ardusub/params.js` | QGC .params export writes every parameter as type 9 (REAL32) and always as sysid 1 *(partial)* |
| major | `src/ui/input.js` | Left trigger 'Ascend'/'Descend' are inverted |
| major | `src/ui/hud.js` | HUD depth tape mixes frames: readout/tape use true CG depth, target marker uses ArduSub's estimate |
| major | `src/ui/input.js` | Focused form controls swallow flight keys; the topbar Mode select turns W/A/S/D and arrow keys into flight-mode changes |
| minor | `src/ui/panels/logs.js` | Logs panel stops updating after 200 autopilot messages |
| minor | `src/vehicle/config.js` | Button map 'Not used' is not persisted (normalizeConfig resurrects default mappings) |
| minor | `src/vehicle/config.js` | Leak detector is not re-derived from LEAKn_PIN/LEAKn_TYPE after the first import; type semantics wrong *(partial)* |
| minor | `src/ardusub/joystick.js` | BTN function enum incomplete; joystick panel cannot show or select valid 4.5.7 functions |
| minor | `src/ardusub/joystick.js` | Relay momentary press never turns the relay on; relays 3/4, actuators and script buttons unhandled |
| minor | `src/ardusub/joystick.js` | throttleBase not truncated to int16 as in joystick.cpp |
| minor | `src/ardusub/joystick.js` | Light step not truncated to uint16 |
| minor | `src/ardusub/params.js` | No ranges on RC/ACRO/FS params, and the tuning/joystick panels bypass validateParam; RC3_DZ can produce a NaN depth target |
| minor | `src/ardusub/params.js` | MOT_THST_HOVER is editable and used, but ArduSub forces it to 0.5 at boot |
| minor | `src/ui/app.js` | Camera tilt is integrated in the render loop with fixed constants and wall-clock time |
| minor | `src/ui/app.js` | New Simulation on geometry/team change resets paused and timeScale while the UI still shows the old state |
| minor | `src/ui/input.js` | Keyboard Shift is hard-wired to SDL button 5 |
| minor | `src/ui/scene3d.js` | Frame-rate-dependent decay in render-loop effects |

## Sensors, physics plant, hydrodynamics

| Severity | File | What was wrong |
|---|---|---|
| critical | `src/geometry/analyze.js` | Sealed-hull mode: buoyancy elements built from shell voxels while volume/CB come from the filled cavity -> buoyancy discontinuity |
| major | `src/physics/body.js` | Wave/gust fluid acceleration produces no inertial (Froude-Krylov + added-mass) force; M_A acts on nu_dot instead of nu_r_dot |
| major | `src/physics/body.js` | Penalty contact uses absolute stiffness/damping (30000 N/m, 900 Ns/m per point) independent of vehicle mass/inertia -> unstable tumbling/NaN for small vehicles |
| major | `src/vehicle/model.js` | Default added-mass estimate (bbox-inscribed solid ellipsoid) is far from a real open-frame ROV, and is not moved to the CG |
| major | `src/vehicle/model.js` | Auto inertia tensor scaled with the wrong volume (density = mass/geo.volume, integral taken over the voxel solid) |
| minor | `src/ardusub/sensors.js` | Depth ground-pressure handling diverges from ArduSub 4.5.7: no 'altitude>0 -> update_calibration' ratchet, no stored BAROn_GND_PRESS boot reference, no BARO_ALT_OFFSET *(partial)* |
| minor | `src/ardusub/sensors.js` | Depth sample timing: instantaneous 50 Hz samples, whereas ArduSub's baro frontend updates at 10 Hz with averaged MS5837 conversions *(partial)* |
| minor | `src/ardusub/control.js` | LowPassFilter2p diverges from ArduPilot for high cutoffs; IMU filters run at loop rate instead of the IMU backend rate |
| minor | `src/ardusub/sensors.js` | EK3_HGT_DELAY is used both as the physical sensor latency and as the filter's compensation delay |
| minor | `src/ardusub/sensors.js` | Sensor reset leaves IMU filter and attitude-error state from the previous run |
| minor | `src/ardusub/sensors.js` | AHRS_TRIM applied to a perfectly aligned simulated board tilts the sim vehicle by the user's real trim |
| minor | `src/physics/body.js` | Flooded water is added as weight but not as mass/inertia |
| minor | `src/physics/body.js` | Added mass kept at the full deep-water value when partly or fully out of the water |
| minor | `src/physics/body.js` | Floor and wall contact are exclusive per point (else-if) |
| minor | `src/physics/environment.js` | EOS-80 temperature of 0 degC silently replaced by 20 degC |
| minor | `src/physics/environment.js` | Waves use deep-water dispersion and exponential decay regardless of water depth |
| minor | `src/vehicle/model.js` | User 6x6 added-mass / damping matrices are not symmetrised or checked for positive definiteness *(partial)* |
| minor | `src/geometry/analyze.js` | Projected-area rotational drag integrals are taken about the solid centroid, not the CG |
| minor | `src/vehicle/shapes.js` | Torpedo shape is longer than the requested length by 0.3*D |
| minor | `src/vehicle/config.js` | Default config enables perfect XY positioning although the reference vehicle has no underwater XY source *(partial)* |

## Simulation loop, AUTO-TUNE, CFD

| Severity | File | What was wrong |
|---|---|---|
| critical | `src/cfd/study.js` | GPU CFD run never terminates on coarse grids: one force sample per frame vs convergence() requiring >=4 samples per window before the tMax check |
| critical | `src/sim/simulation.js` | keepState rebuild (any param edit) trips the pilot-input failsafe and disarms the vehicle |
| major | `src/sim/tuning.js` | Depth step-test target is set from true CG depth but the controller tracks baro depth (step size and steady error wrong, worst in seawater) |
| major | `src/cfd/study.js` | Smagorinsky eddy viscosity equals the molecular viscosity at the Re=2000 run, biasing the C_inf + K/Re extrapolation |
| minor | `src/sim/tuning.js` | Auto-tune suggests gains outside ArduPilot's parameter ranges |
| minor | `src/ardusub/sensors.js` | Depth feedback updated at 50 Hz instead of ArduSub's 10 Hz barometer task *(partial)* |
| minor | `src/cfd/cpu.js` | Rotational damping from a non-rotating mask with a moving-wall BC: face-normal wall speed becomes mass transpiration *(partial)* |
| minor | `src/sim/tuning.js` | Attitude step tests score true attitude but command in the AHRS frame (AHRS_TRIM offset counted as error) |
| minor | `src/sim/tuning.js` | Pitch divergence can never be flagged (Euler pitch bounded at ±90°) |
| minor | `src/cfd/study.js` | Reynolds-fit clamp zeroes diagonal K but keeps off-diagonal K, giving an indefinite (energy-injecting) DL |
| minor | `src/core/convergence.js` | GCI: extrapolation order clamp of 3 is above LBM's formal order of 2, and the two-grid case uses Fs=1.25 instead of 3 |
| minor | `src/sim/simulation.js` | reset() does not reseed the RNG or reset the environment clock and turbulence state (runs are not reproducible) |
| minor | `src/sim/simulation.js` | Periodic tasks use `t >= next; next = t + 1/rate` and drift to lower rates; failsafe and 3 Hz rates differ from the ArduSub scheduler |
| minor | `src/ardusub/sub.js` | Bottom-detector counter resets on every non-bottom loop and scales with SCHED_LOOP_RATE |
| minor | `src/cfd/cpu.js` | Momentum exchange on moving (rotating) walls lacks the Galilean-invariance correction *(partial)* |
| minor | `src/sim/tuning.js` | Auto-tune tunes each axis against the other axes' old gains, and single-sign CFD runs store even (lift) couplings as odd u\|u\| terms |

## Added ArduSub features and parameters

- **Unused-in-Sub multicopter MOT_* parameters are absent from the parameter table** (motors)
- **Battery-current output limiter and battery monitor** (motors)
- **MOT_SPOOL_TIM_DN, MOT_SAFE_DISARM, MOT_SAFE_TIME in the spool state machine** (motors)
- **MOT_PWM_TYPE output protocol and endpoint forcing** (motors)
- **MOT_PWM_MIN/MAX conversion from RC3 and Sub defaults** (motors)
- **Output channel mapping and ESC update rate** (motors)
- **MAV_CMD_DO_MOTOR_TEST protocol and the motor testing order** (motors)
- **MOTOR_DETECT flight mode (automatic MOT_n_DIRECTION detection)** (motors)
- **Propeller reaction torque** (motors)
- **Electrical and rotor-dynamics model of the thruster** (motors)
- **get_throttle_hover clamp and MOT_THST_HOVER read-only semantics** (motors)
- **Sub-specific input_euler_angle_roll_pitch_slew_yaw** (attitude)
- **Remaining AC_AttitudeControl input functions** (attitude)
- **Target reset and EKF-reset handling** (attitude)
- **Crash check based on attitude error** (attitude)
- **Throttle-vs-attitude mix machinery** (attitude)
- **Angle boost** (attitude)
- **Rate PID slew limiter (SMAX), notch filters and the full PID info** (attitude)
- **Control monitor (CTRL oscillation diagnostics)** (attitude)
- **Autotune and system-identification hooks** (attitude)
- **Accessor and runtime-setter parity** (attitude)
- **Legacy ATC_RAT_*_FILT parameter conversion on import** (attitude)
- **SlewLimiter / SMAX gain reduction (with Dmod and slew_rate outputs)** (pid)
- **Derivative feed-forward D_FF / DFF** (pid)
- **Target/error notch filters NTF/NEF and the AP_Filters bank** (pid)
- **Complete AP_PIDInfo plus PID_TUNING telemetry and PID logging** (pid)
- **Missing AC_PID / AC_PID_Basic / AC_PID_2D API surface** (pid)
- **AC_P_1D.set_error_limits and AC_P_2D.set_error_max, with their getters** (pid)
- **PSC_VELXY_FF parameter (and feed-forward wiring)** (pid)
- **AC_P class for the ATC_ANG_*_P angle controllers (and AC_PI / AC_PI_2D for completeness)** (pid)
- **GCS heartbeat failsafe** (failsafes)
- **Crash check failsafe** (failsafes)
- **EKF variance failsafe and its effect on position_ok** (failsafes)
- **Internal temperature failsafe** (failsafes)
- **Depth-sensor failsafe, no-depth-sensor operation and control_check_barometer** (failsafes / mode entry)
- **Battery failsafe** (failsafes)
- **Terrain failsafe (AUTO/GUIDED)** (failsafes)
- **SURFTRAK mode (21)** (flight modes)
- **MOTOR_DETECT mode (20)** (flight modes)
- **GUIDED (4), AUTO (3), CIRCLE (7) modes and their navigation parameters** (flight modes)
- **Arming configuration and Sub-specific arming behaviour** (arming)
- **Motor-test protocol (MAV_CMD_DO_MOTOR_TEST)** (arming / motors_output)
- **Pilot link-loss injection and RC override details** (failsafes (pilot input))
- **Leak detector driver semantics** (failsafes (leak))
- **Parameter ranges/metadata for mode and failsafe params** (params)
- **Missing PSC / AC_PID parameters in PARAM_DEFS (kept verbatim but have no effect)** (poscontrol)
- **SURFTRAK mode and the AC_PosControl terrain-offset API** (poscontrol)
- **Missing vertical kinematic entry points** (poscontrol)
- **Missing horizontal kinematic/utility functions** (poscontrol)
- **Vertical controller health ratio and vibration compensation** (poscontrol)
- **EKF position-reset handling and timing diagnostics** (poscontrol)
- **PSC logging / telemetry** (poscontrol)
- **GCS depth/position targets** (poscontrol)
- **ArduSub top-level parameters missing from params.js** (params)
- **Joystick button functions 76-87 and 112-138 plus their handlers** (joystick)
- **RC channels 9-16 and RC option/override parameters** (rc)
- **Lights, camera mount, relay and servo-output parameters** (joystick / outputs)
- **Leak detector parameters (used but not defined)** (params / failsafe)
- **Motor library parameters not defined** (params (motors))
- **Attitude/position/nav library parameters not defined** (params (controllers))
- **Arming, sensor and AHRS parameters relevant to simulated behaviour** (params (arming/sensors))
- **Parameter-type metadata and legacy-name conversion for .param import/export** (params import/export)
- **Joystick 4.5.7 extras not modelled** (joystick)
- **Gyro/accelerometer bias, bias random walk and the resulting EKF yaw drift without a yaw source** (sensors)
- **Barometer parameters and behaviour** (sensors)
- **Temperature sensing and the internal-temperature failsafe** (sensors)
- **No-external-depth-sensor path and depth sensor health** (sensors)
- **IMU configuration parameters** (sensors)
- **EKF3 height/velocity source selection, tuning and EKF failsafe** (sensors)
- **Leak detector logic level, type defaulting and cooldown** (sensors)
- **Rangefinder (downward) sensor** (sensors)
- **Compass / magnetometer model** (sensors)
- **Free-surface / wall effects on hydrodynamics and wave excitation** (plant)
- **Crash-check, EKF and depth-sensor failsafes (fifty_hz_loop)** (sim loop / failsafes)
- **GCS-heartbeat, internal-temperature and terrain failsafes (three_hz_loop)** (sim loop / failsafes)
- **Battery monitor and battery failsafe** (sim loop / failsafes)
- **AP_Baro accumulate/update split (10 Hz depth) and in-flight baro recalibration** (sim loop / sensors)
- **one_hz_loop tasks** (sim loop)
- **Rangefinder task and SURFTRAK mode** (sim loop / modes)
- **Tether turn counter** (sim loop)
- **Auto-tune leaves out most loop parameters** (tuning)
- **No transform of CFD damping when the CG moves** (cfd -> plant)
