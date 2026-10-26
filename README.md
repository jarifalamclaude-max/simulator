# DUBURI — ROV Simulator

Put your ROV in a virtual pool instead of the water. Upload your 3D design, check
that your thruster layout really gives roll, pitch and yaw, fly it with a gamepad
through a faithful port of the ArduSub 4.5.7 controllers, and tune the parameters
before the first pool session. Everything runs in one HTML file in the browser.

**Open it online:** <https://jarifalam13.github.io/DUBURI_SIM/> (Chrome or Edge recommended; needs WebGL2).

**Documentation:** <https://jarifalam13.github.io/DUBURI_SIM/guide/> — user guide with screenshots, the physics and control equations, and all 2708 parameters.

**Or offline:** download `dist/duburi-sim.html` and double-click it. Nothing to install.

## What it does

| | |
|---|---|
| **Your design** | Upload STL (binary/ASCII), OBJ or PLY — top-bar *Upload design* or drop the file anywhere. Pick units and which axis is forward/up; the hull is voxelised for volume, centre of buoyancy, inertia, projected areas and contact points. The thrusters in the file are found automatically — by part name (`T200_3`, `Thruster 5` in OBJ groups or ASCII STL solids) or by duct shape (T200 / T500 / generic) — matched to your `FRAME_CONFIG` for motor order and push direction, and proposed as a custom frame (Accept, Flip, Discard). |
| **Frame** | Picture cards for every ArduSub `FRAME_CONFIG` and the automatic mixer, a top and side drawing of your thrusters, and command sliders that light up the thrusters that fire and show the force/torque that results — or fly the vehicle with them. |
| **Custom frame builder** | Add up to 12 thrusters (T200 / T500 / generic / your measured table), drag them on the top and side drawings (5 mm steps), set direction (presets, ±45° rotate, flip), mirror, remove, reverse `MOT_n_DIRECTION`, start from any built-in layout, and copy the resulting mixer as `add_motor_raw_6dof()` lines for a custom ArduSub build. |
| **Body shape without CAD** | Simple open-frame, closed-box or torpedo bodies with your dimensions and a duct at every thruster — or your STL/OBJ/PLY, or the sample ROV. |
| **Thruster layout → DOF** | Place thrusters on the model (click the hull). The allocator finds the largest set of independently controllable axes — full 6-DOF when the layout allows it, otherwise the maximum possible — and reports authority and cross-coupling per axis. Or fly the exact ArduSub `FRAME_CONFIG` table. |
| **Your parameter file** | All 2708 ArduSub 4.5.7 parameters, generated from the ArduPilot source (`tools/gen-params.mjs`) with types, defaults, ranges, units, value lists and bitmasks. About 670 drive the simulation; the rest (calibration, EKF tuning, serial, logging, …) are kept exactly and written back on export. Mission Planner and QGC files round-trip losslessly; legacy names are converted. |
| **ArduSub 4.5.7 port** | `AP_Motors6DOF` (mixers, saturation, full spool state machine, battery current limiter, `MOT_PWM_TYPE`, `SERVOn_FUNCTION` routing, motor test), `AC_PID` with SMAX slew limiter, D_FF and notch filters, `AC_AttitudeControl_Sub`, `AC_PosControl` (depth and XY), `joystick.cpp` with every `BTNn_FUNCTION`, RC1–16, the AP_Scheduler task table at ArduSub rates, AP_Arming_Sub checks, all modes (Manual, Stabilize, Acro, Depth Hold, SurfTrak, Position Hold, Surface, Guided, Auto, Circle, Motor Detect) and all failsafes (pilot input, GCS, leak, pressure, temperature, battery, EKF, crash, depth sensor, terrain). Every change is listed in [AUDIT.md](AUDIT.md). |
| **Fly** | Keyboard or gamepad (Gamepad API, QGC button numbering), Cockpit/QGC-style HUD, chase/orbit/camera/top views, live target-vs-actual graphs, CSV logs. |
| **Pool & water** | Any length, width and depth — test tank to open sea — with a flat floor or a shallow end and slope; wall height; presets; fresh or sea water; current, turbulence and linear waves. |
| **Pressure** | Absolute pressure = air pressure + water column, with compressible water (UNESCO EOS-80). Buoyancy changes with depth (water density, hull compressibility). Bar02 / Bar30 / custom depth sensor with range limit and resolution, read through AP_Baro's formula (9800 Pa/m × `BARO_SPEC_GRAV`). Housings leak past their depth rating (orifice flow against the Boyle-compressed air inside) → leak detector → ArduSub failsafes. |
| **Hydrodynamics** | 6-DOF Fossen plant with full 6×6 added mass and its Coriolis/Munk terms, coupled linear + quadratic damping, surface-aware buoyancy, thruster in-flow loss, ventilation and jet–hull interaction. |
| **Added mass (BEM)** | Constant-source panel method on a smooth hull mesh, GMRES, three resolutions with Richardson extrapolation and a GCI-style uncertainty. |
| **Drag & damping (CFD)** | Lattice-Boltzmann (D3Q19, regularised BGK + Smagorinsky LES, Bouzidi interpolated walls with moving-wall term, momentum exchange) in a virtual tow tank on the GPU (WebGL2), with grid convergence (Celik et al. 2008 GCI) and Reynolds extrapolation C(Re) = C∞ + K/Re. Streamlines and hull pressure shown on the 3D model. |
| **AUTO-TUNE** | One click: converged BEM added mass + CFD damping for *your* design, then closed-loop step tests to tune roll, pitch, yaw and depth gains. Every value is shown, editable and only applied when you say so. Mass, CG and thrust you entered are never overwritten. |
| **Teams** | DUBURI wordmark with per-team name, logo and colour; each team keeps its own vehicle in the browser. |

## Quick start

1. **Upload design** (top bar). Set units and axes in *Vehicle*.
2. Enter what you know: real mass and CG, thruster positions/directions/polarity, measured thrust tables if you have them.
3. **Pool & water**: your pool size and depth, water type, housing depth rating.
4. **Frame**: pick the frame and try each axis with the sliders (details in *Frame & mixer*).
5. **AUTO-TUNE** → review → *Apply selected*.
6. Fly: `Enter` arms, `1`–`6` change mode, `G` shows graphs. Gamepad works in Chrome/Edge.
7. *Save .param* and load it on the Pixhawk with QGroundControl or Mission Planner.

## Validation

All numbers below are reproducible with the tools in `tools/` and the tests in `tests/` (185 automated tests, `npm test`).

| Check | Result |
|---|---|
| Sphere drag, CFD pipeline at Re = 100 (`npm run validate:cfd`) | 8 / 12 / 16 cells per diameter: Cd 1.311 / 1.189 / 1.143 → Richardson 1.074 vs 1.088 standard curve (−1.2 %) |
| GPU (WebGL2) vs CPU lattice-Boltzmann (`node tools/gpu-check.mjs`) | state difference ~4 × 10⁻⁷, with and without interpolated walls and rotation |
| Panel method vs Capytaine on identical meshes (`tools/bem-crosscheck.py`) | within ~0.2–1 % (largest: ROV heave 2.1 % at 3 966 panels) |
| Panel method vs exact ellipsoid (Lamb) at 3 380 panels | +0.5 … +1.9 % |
| WebAssembly kernels vs JavaScript | BEM ≤ 2 × 10⁻⁵ relative, LBM ≤ 10⁻⁶; BEM 3.3× faster |
| EOS-80 seawater density and bulk modulus | all 14 UNESCO check values to < 0.01 ppm |
| Displaced volume (exact along rays, fine across) | thin 12.7 mm plates within 0.04 % (plain voxels: +31 %); slanted rod −0.04 %; sphere and box exact |
| Firmware port | mixer maths, RC scaling, sqrt controller, spool logic, button mapping and failsafes checked against the ArduSub source behaviour |

## Limits — read before trusting a number

- Not the ArduSub binary (no SITL): the control code is ported; the EKF is replaced by an estimator with realistic noise; no compass, GPS or MAVLink link.
- The tether is not modelled.
- The CFD runs at Re of a few thousand and extrapolates; reliable for open frames and boxy hulls (sharp-edge separation), less for smooth streamlined hulls. Each result carries its grid uncertainty.
- The panel method is ideal-flow; parts thinner than a voxel are thickened slightly.
- Water temperature is uniform with depth; thrusters and electronics don't change with pressure.
- Results are only as good as the mass, CG and thruster data you enter. Measure them.

## Development

```bash
npm install
npm test               # 185 unit tests (physics, firmware, controllers, params, joystick, allocation, BEM, CFD, WASM, pressure, shapes, tuning, thruster detection)
npm run build          # -> dist/duburi-sim.html (stand-alone), dist/artifact.html, docs/index.html (GitHub Pages)
npm run build:wasm     # recompile src/wasm/kernels.c (needs clang with wasm32)
npm run validate:cfd   # sphere drag through the full CFD pipeline
node tools/gpu-check.mjs                   # GPU vs CPU LBM (needs Playwright + Chromium)
node tools/bem-export.mjs bem.json && python3 tools/bem-crosscheck.py bem.json   # needs capytaine
```

```
src/
  core/       math, seeded RNG, grid-convergence (GCI)
  geometry/   STL/OBJ/PLY, voxelisation, mass properties, smooth hull extraction
  alloc/      ArduSub frame tables, thruster allocation & DOF analysis
  hydro/      water properties (EOS-80), BEM panel method, hull meshing
  cfd/        D3Q19 lattice-Boltzmann (CPU, WebGL2 GPU), tow-tank studies
  wasm/       C kernels compiled to WebAssembly (BEM, LBM)
  physics/    thruster model, environment (pool, pressure), 6-DOF plant
  ardusub/    ArduSub 4.5.7 port: params, motors, attitude, position, joystick, sensors, modes
  sim/        simulation loop, telemetry, step tests and auto-tune
  ui/         three.js scene, HUD, panels, graphs, jobs/worker
  worker/     background worker (BEM, CFD, auto-tune)
profiles/     DUBURI Pixhawk parameter file
tools/        validation and build tools
```

### Hosting

GitHub Pages serves `docs/index.html` from the `main` branch (Settings → Pages →
Deploy from a branch → `main`, `/docs`). After a change: `npm run build`, commit,
push; the site updates in a minute or two.

### Conventions

- World frame NED, origin at the still-water surface, pool centre. Body frame FRD, origin at the CG.
- Euler angles follow ArduPilot (3-2-1). Thruster positions in metres from the model origin.

## License

GPL-3.0-or-later (the control code is a port of ArduPilot, which is GPL-3.0). three.js is MIT.
