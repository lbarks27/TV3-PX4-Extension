# TV3 PX4 SIH Simulation

The active simulator path is PX4 Simulation-In-Hardware (SIH) with the custom `tv3_sih` module. Hawkeye is a viewer only; the physics source of truth is `tv3_sih`.

**SIH is intentionally a simplified deterministic plant** for controller development, gate validation, and hardware correlation rather than a high-fidelity aerospace simulator (see "Known Limitations and Simplifications" below).

The retired Gazebo workflow is no longer part of the active repo. Local copies may live under gitignored `deprecated/sim/gazebo/` or outside this checkout under `../deprecated-sim/gazebo/`.

## Prerequisites (macOS)

- Xcode Command Line Tools (`xcode-select --install`)
- Homebrew with `cmake`, `git`, and `python3`
- `brew install qt@5` (PX4 SITL links against Qt 5)
- Optional visualization: `brew tap px4/px4 && brew install px4/px4/hawkeye`
- Disk space for `../vendor/px4` and `../.work/px4-tv3` (cloned and prepared by the bootstrap scripts)

Vehicle manifests and flight profiles are JSON under `config/vehicles/*.json` and `config/flight_profiles/*.json`.

## First-Time Setup

```bash
./scripts/check_barebones.sh      # host tests + generate bare-bones assets
./scripts/bootstrap_px4.sh        # clone PX4 v1.16.1 into ../vendor/px4
./scripts/prepare_px4_tree.sh     # patched worktree + ROMFS overlays
./scripts/build_sih.sh            # PX4 SITL build with tv3_sih
```

## Daily Workflow

```bash
# Default lander hover-window gate:
./scripts/run_sitl_sih.sh

# Automated Phase 1 gate (headless, archives ULog, runs review):
./scripts/check_hover_window.sh

# Review the newest archived run (full tool matrix: docs/data_visualization.md):
./scripts/setup_python_env.sh --profile viz   # once
./scripts/validate_viz_commands.sh            # optional headless smoke test
./scripts/plot_ulog.sh --latest               # 2D timeseries PNG
./scripts/tv3_replay.sh --latest              # interactive 3D trajectory (PyVista)
./scripts/tv3_replay.sh --latest --rerun      # timed playback (Rerun)
```

Switch vehicles with `TV3_VEHICLE_CONFIG=config/vehicles/tv3_v1.json`. Load a scenario with `TV3_FLIGHT_PROFILE=config/flight_profiles/single_engine_ascent.json`.

## Default Gate

The first required scenario gate is:

```bash
TV3_VEHICLE_CONFIG=config/vehicles/tv3_lander_v1.json \
TV3_FLIGHT_PROFILE=config/flight_profiles/lander_hover_window.json \
./scripts/run_sitl_sih.sh
```

Those are also the defaults, so `./scripts/run_sitl_sih.sh` is enough for the current lander hover-window gate.

## Build

```bash
./scripts/bootstrap_px4.sh
./scripts/build_px4.sh --target sih
```

`build_px4.sh --target sih` prepares `../.work/px4-tv3`, points `EXTERNAL_MODULES_LOCATION` at the no-space symlink `../.work/tv3-px4-extension`, and builds `px4_sitl_default` with the external `tv3_sih` module.

## Run

```bash
./scripts/run_sitl_sih.sh
```

The launcher sets:

- `PX4_SIMULATOR=sihsim`
- `PX4_SIM_MODEL=tv3_lander`
- `PX4_SIM_SPEED_FACTOR=1` (real-time lockstep; do not raise for routine runs)
- `PX4_SYS_AUTOSTART=11002`
- `TV3_VEHICLE_CONFIG=config/vehicles/tv3_lander_v1.json`
- `TV3_FLIGHT_PROFILE=config/flight_profiles/lander_hover_window.json`

It also syncs the TV3 logger profile into the PX4 rootfs, starts the profile command runner by default, and archives new ULogs into `logs/sim/YYYY-MM-DD/<run-id>/` on exit.

Disable automatic profile commands with:

```bash
TV3_RUN_PROFILE_COMMANDS=0 ./scripts/run_sitl_sih.sh
```

For a visual run, use two terminals so Hawkeye is already listening before PX4
starts streaming the SIH viewer feed:

```bash
# Terminal 1
./scripts/run_hawkeye.sh
```

```bash
# Terminal 2
TV3_VEHICLE_CONFIG=config/vehicles/tv3_lander_v1.json \
TV3_FLIGHT_PROFILE=config/flight_profiles/lander_hover_window.json \
TV3_LOG_RUN_ID=manual-hawkeye-visual \
./scripts/run_sitl_sih.sh
```

`run_sitl_sih.sh` starts `scripts/run_profile_commands.py` by default. For the
default hover-window profile, the runner arms at `t=0` and sends TV3 launch
command `31010` with `param1=1` at `t=1`.

If a previous viewer or PX4 instance is still using the ports, stop it before
starting the next visual run:

```bash
pkill -f '/opt/homebrew/bin/hawkeye -udp 19410' || true
pkill -f 'px4_sitl_default.*/bin/px4|run_sitl_sih.sh' || true
```

## Profile Commands

`scripts/run_profile_commands.py` reads the active flight profile and sends the command timeline over MAVLink. For `lander_hover_window.json`, it arms at `t=0` and sends TV3 launch command `31010` with `param1=1` at `t=1`.

Default MAVLink endpoint:

```text
udpin:0.0.0.0:14540
```

Override it with `TV3_MAVLINK_URL`.

## Visualization

Start Hawkeye separately:

```bash
./scripts/run_hawkeye.sh
```

PX4 SIH publishes the viewer stream on UDP `19410` and includes `HIL_STATE_QUATERNION` for visualization. If Hawkeye is not installed, set `HAWKEYE_CMD` to the executable command or install `Hawkeye.app` in `/Applications`.

QGroundControl connects through the normal PX4 SITL GCS link on UDP `18570`.

To drive the same launch path from QGC in SIH that hardware uses, install the
TV3 MAVLink actions:

```bash
./scripts/install_qgc_actions.sh
```

Restart QGC after installing. The Fly View action list should include `TV3
Launch`, `TV3 Abort`, and `TV3 Reset`. Use `TV3 Launch` for tv3 launch in
sim and hardware; do not remap QGC's generic `Takeoff` button to ignition.

On macOS with the PX4 Homebrew tap, Hawkeye can be installed with:

```bash
brew tap px4/px4
brew install px4/px4/hawkeye
```

The current Hawkeye CLI supports built-in vehicle shapes with `-mc`, `-fw`, and
`-ts`; the TV3 SIH path treats Hawkeye as a pose viewer, not the physics source.

## Review Artifacts

SITL ULogs are archived under:

```text
logs/sim/YYYY-MM-DD/<run-id>/
```

Each archive includes copied `.ulg` files, `manifest.txt`, `logger_topics.txt` when available, the active `vehicle.json`, and the active `flight_profile.json`.

## Ports And Endpoints

| Service | Default endpoint | Notes |
| --- | --- | --- |
| Profile command runner | `udpin:0.0.0.0:14540` | Override with `TV3_MAVLINK_URL` |
| QGroundControl (SITL) | UDP `18570` | Normal PX4 GCS link |
| Hawkeye viewer | UDP `19410` | Visualization only; not physics truth |

## Troubleshooting

- **Stale PX4 or Hawkeye process**: run the `pkill` commands in the visual-run section above before restarting.
- **Hawkeye not found**: set `HAWKEYE_CMD` to the executable, install via Homebrew, or place `Hawkeye.app` in `/Applications`.
- **Profile commands do not arm/launch**: confirm QGC is not holding the serial/UDP link exclusively; check `TV3_RUN_PROFILE_COMMANDS` is not `0`.
- **Missing ULog topics**: run `./scripts/sync_sitl_logger_topics.sh` or start a new run so `run_sitl_sih.sh` copies the generated logger profile before boot.
- **Build fails after PX4 update**: rerun `./scripts/prepare_px4_tree.sh` to reapply patches, then `./scripts/build_sih.sh`.

## Known Limitations and Simplifications

The SIH plant (`tv3_sih`) and the TV3 PX4 extension prioritize deterministic, fast SITL for controller validation, hardware parity, and gates (hover window, waypoint track, etc.) over high-fidelity physics. The following are intentional or current-state simplifications. Items that are not addressed in the current codebase are explicitly called out.

### SIH Physics Model
- **Forward-only wrench plant**: As of recent cleanup, the plant is strictly forward: net body force/torque is computed directly from per-engine chamber thrust (from `tv3_motor_model` + thrust curves) rotated by the applied gimbal angles (from `tv3_engine_command`). No guidance `required_thrust_n` scaling and no torque-to-gimbal synthesis (previously present as workarounds) remain in the plant.
- **Fixed diagonal inertia**: `RK_I**` (sourced from `physical_model` by the asset generator) is loaded once and held constant. `vehicle_mass_kg()` and `current_com_body()` correctly migrate total mass and COM as motors burn, but the inertia tensor does not update. **Not addressed**: full variable inertia about the moving COM using parallel-axis theorem on depleting motor masses + their local inertias. Short-duration profiles make this acceptable for now.
- **Rail and ground contact**: Purely kinematic.
  - Rail: instantaneous zeroing of horizontal position/velocity and body rates while below rail length. No sliding friction or release transient.
  - Ground: hard z=0 clamp + fixed empirical damping factors. No spring, restitution, or contact wrench.
  These are minimal models sufficient for rail-launched ascent and basic deck contact in current gates.
- **Actuator model**: Only a slew-rate limiter on gimbal angles inside the plant (honoring `RK_TVC_SLEW_DPS`). Backlash, servo position lag, and hysteresis declared in manifests are not simulated (current vehicles use 0 backlash).
- **No environmental effects**: No aerodynamic drag, base drag, wind, turbulence, or nozzle effects (beyond optional `RK_SIH_RATE_DAMP`, default 0). `RK_SIH_RATE_DAMP` was previously default 2.5 for "numerical taming"; now defaults to 0 and is only a sim aid.
- **Integration**: First-order Euler for translation + simple quaternion integration + normalize at fixed ~400 Hz step. dt derived from the manipulated SITL monotonic clock.
- **No variable-mass rocket effects**: Propellant expulsion carries no relative velocity momentum in the model.

### Control and Allocation Path in SIH
- Gimbal angles on `tv3_engine_command` (roll/primary + yaw/splay secondary) come from `tv3_mode_manager`, which forwards allocator servos + computes collective splay yaw when `RK_GD_ENABLE` and guidance thrust solution is valid.
- The allocator (via patch) and `tv3_att_control` produce torques; splay throttle for lander is handled outside the allocator as a thrust modulator.
- **Not addressed in SIH**: the full nonlinear kinematics are present and match the Python reference model, but the complete closed-loop "commanded wrench → allocator → actuator commands → plant" for all cases relies on mode_manager bridging.

### Broader Extension Complexity and Duplication
- **Custom modules**: `tv3_mode_manager`, `tv3_guidance`, `tv3_att_control`, `tv3_motor_model`, load-cell modules, and `tv3_sih` implement TV3-specific behaviors (solid-motor ignition confirmation via load cell, splay-as-throttle, custom command 31010, per-engine state, mass reporting from curves). Stock PX4 components are reused (control allocator via CA_RK, attitude, uORB pubs, SITL sensors) where possible.
- **Dual parameter sets**: `CA_RK_*` (for patched `ActuatorEffectivenessTV3` and allocator) + `RK_*` (for TV3 modules). Both are generated from the same vehicle manifest by `tools/generate_vehicle_assets.py`. This duplication exists because the allocator lives in a PX4 patch.
- **Manifest richness vs runtime use**: `config/vehicles/*.json` contains detailed `physical_model` (links, joints, inertias about origin, CAD refs) for intake, validation (`check_physical_manifests.sh`), and future use. Runtime only consumes a flattened subset via generated params + `tv3_motor_model` curves.
- **Allocator implementation**: TV3 effectiveness is supplied by a ~772-line patch (`patches/px4/0001-tv3-control-allocation.patch`) adding `EffectivenessSource::TV3` and `ActuatorEffectivenessTV3`. Not yet an upstream or pure external module.
- **Model duplication (thrust selection, splay, geometry)**:
  - Thrust fallback (filtered → measured → expected) is reimplemented in C++ modules and mirrored in Python tools for offline checks.
  - Collective splay yaw computation (`collective_throttle_yaw_deg` using acos or search) exists in `tv3_mode_manager` and the Python allocator/reference.
  - Gimbal direction math is duplicated between `tv3_sih` (`engine_thrust_dir_body_angles`) and `tools/tv3_control_allocator.py` (`plant_thrust_direction`).
  **Not addressed**: a single shared implementation (would require new common library, headers, or moving logic to Python-only reference + codegen).
- **Script and generator surface**: ~30 scripts + generators + schemas + staging for microSD, barebones, checks, plots. This supports reproducible gates, hardware bring-up, and Monte Carlo, at the cost of surface area.
- **Physical model data**: Many fields are still "preliminary" (see `data_status` in JSON). Measured bench data is required before flight_ready.

### Items Explicitly Not Simplified or Fixed Now
- Removing or collapsing custom modules (would lose required TV3 rocket/lander behaviors and hardware compatibility).
- Unifying RK_* / CA_RK_* without changes to the PX4 patch or allocator.
- Full 6DOF variable-mass variable-inertia dynamics, aero model, or high-fidelity contact (would increase complexity and require more manifest data + tuning; current fidelity is matched to gates and ULog review).
- Moving the allocator out of the patch.
- Reducing the generator/manifest footprint (used for validation, CAD correlation, and both sim + hardware paths).

These limitations are accepted for the current phase (primarily Phase 2 bench + early SIH gates). Higher fidelity or further simplification will be tracked in `docs/completion_roadmap.md` and `docs/implementation_phases.md`.

See also [docs/control_mixer.md](control_mixer.md) (for allocator/splay details) and [docs/completion_roadmap.md](completion_roadmap.md).

See also [docs/data_visualization.md](data_visualization.md) for ULog plotting and [docs/completion_roadmap.md](completion_roadmap.md) for phase gates.
