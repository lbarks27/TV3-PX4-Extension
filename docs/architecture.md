# TV3 Repository Architecture and Components

This document describes the major pieces of the TV3 PX4 Extension repo, what each does, and how they work together to achieve the project's goals.

## Main Purpose

The repo provides an **out-of-tree PX4 extension** plus a supporting **host/tooling pipeline** for thrust-vector-controlled TV3 vehicles (currently `tv3_v1` single-engine ascent and `tv3_lander_v1` three-engine splay-throttle lander).

Vehicle manifests under `config/vehicles/*.json` are the **source of truth**. They drive generation of PX4 runtime parameters, SD-card payloads, motor reference data, and logger profiles. Flight profiles in `config/flight_profiles/` define scenario-specific targets and parameters that can be overlaid for SITL runs.

The overall objective is to go from checked-in configuration → generated runtime assets → simulated or real execution → logged data → automated validation and human review, repeatedly and reproducibly.

## How the Major Components Relate

The system follows a clear pipeline:

1. **Manifests + profiles** (`config/vehicles/*.json`, `config/flight_profiles/*.json`) describe the vehicle, engines, propulsion/load-cell semantics, physical data provenance, and scenario targets.
2. **Generators + validators** convert those manifests into:
   - Generated PX4 parameters (`RK_*` and allocator `CA_RK_*`)
   - SD-card runtime payload and startup configuration
   - Logger topic profiles for rich ULog capture
3. **PX4 integration** (runtime) executes using:
   - Custom uORB messages
   - Custom PX4 modules
   - PX4 patches (primarily for control allocation)
   - ROMFS overlays for startup ordering
4. **Simulation and hardware** both consume the same generated artifacts:
   - SIH (Simulation-In-Hardware) runs inside PX4 using the `tv3_sih` plant; Hawkeye is a viewer only.
   - Hardware uses the same runtime layout on a microSD card for the Cube Orange Plus.
5. **Validation and review** close the loop:
   - Host unit tests protect math and semantics.
   - Gate scripts run scenarios and assert success criteria against logs.
   - Visualization tools (2D plots, 3D replay) turn ULogs into reviewable artifacts.

## Major Components

### 1. PX4 Extension Runtime Code

These are the modules and definitions that run on the flight controller (SITL or hardware).

- **uORB messages** (`msg/*.msg`): Define TV3-specific topics such as `Tv3Status`, `Tv3EngineCommand`, `Tv3LoadCell`, `Tv3Thrust`, `Tv3MotorReference`, `Tv3ModeStatus`, `Tv3GuidanceStatus`, etc.
- **Custom modules** (`src/modules/`):
  - `tv3_motor_model`: Motor thrust curve handling and reference state.
  - `tv3_load_cell` (+ `tv3_load_cell_telemetry`): Load-cell driver integration and calibrated thrust signals.
  - `tv3_mode_manager`: Ignition, launch, abort, reset, burnout, and coast state machine.
  - `tv3_att_control`: Attitude/rate PID mixer that produces body wrench setpoints (`vehicle_torque_setpoint` / `vehicle_thrust_setpoint`).
  - `tv3_guidance`: Waypoint, hover, landing, and envelope-aware guidance (enabled per-manifest via `guidance.enable`).
  - `tv3_sih`: The simplified deterministic SIH plant used for controller development and gate validation.
- **Control allocator patch** (`patches/px4/`): Still present for geometry parameters (`CA_RK_*`) and to keep `control_allocator_status` logging; however the small-angle `ActuatorEffectivenessTV3` servo outputs are **bypassed** at runtime. Command synthesis for TVC is performed by a weighted projected-GD joint (torque + thrust) solver inside `tv3_mode_manager` using the full nonlinear kinematics.
- **Startup / ROMFS overlays** (`overlay/ROMFS/`): Control module start order and enable TV3 behaviors for both POSIX SITL and NuttX hardware images.

### 2. Vehicle and Scenario Configuration

- **Vehicle manifests** (`config/vehicles/tv3_*.json`): The authoritative description of a vehicle, including:
  - Physical properties (masses, COM, inertia references)
  - Engine geometry, thrust axes, gimbal limits, thrust fractions
  - Load-cell calibration and channel mapping
  - State machine thresholds (launch, ignition timeout, burnout, etc.)
  - Controller gains and torque limits
  - Guidance enablement and parameters
  - `data_status` provenance tracking (measured / preliminary / placeholder) used by completion gates
- **Flight profiles** (`config/flight_profiles/*.json`): Scenario overlays that supply mission targets (hover window, waypoint track, ignition sequences, abort paths, etc.) without mutating the base vehicle manifest.
- **Schemas** (`config/schemas/`): `vehicle_intake_schema.json` and `flight_profile_schema.json` define the expected structure and enable deterministic validation.

### 3. Host-Side Generators and Validators

These Python tools turn manifests into runnable artifacts and enforce consistency.

- **Manifest validation** (`tools/validate_vehicle_manifest.py`): Checks schema compliance, axis orthogonality, thrust fraction sums, engine geometry rules, data provenance requirements, and more.
- **Runtime asset generation** (`tools/generate_vehicle_assets.py`): The central generator. It produces:
  - PX4 parameter files (`tv3_*.params`) containing both `RK_*` (TV3 modules) and `CA_RK_*` (allocator) values
  - Motor catalog and per-motor curve/spec CSVs under the runtime motors directory
  - Logger topic profile (`logger_topics.txt`)
  - Active flight profile JSON when a profile is supplied
- **Motor / thrust curve tooling**:
  - `tools/generate_motor_catalog.py` and `tools/tv3_motor_catalog.py`
  - Consume `config/thrust_curves/` (inventory + per-motor specs/dynamics CSVs)
  - Produce normalized catalog assets and compute allocator thrust fields (reference, minimum, fallback)
- **Control and allocation math** (`tools/tv3_control_allocator.py`):
  - Offline constrained allocator and reachability solver
  - Mirrors the PX4 allocator's small-angle TVC model and the SIH plant's nonlinear splay/pitch/yaw thrust model
  - Used by guidance envelope checks and the Phase 4 control mixer gate
- **Guidance envelope and Monte Carlo** (`tools/tv3_guidance_envelope.py`, `tools/run_guidance_monte_carlo.py`): Higher-level checks that guidance only claims solutions inside the remaining vehicle capability.

### 4. Orchestration Scripts

Shell and Python scripts that tie build, run, validation, and archiving together.

- **Bootstrap and build**:
  - `scripts/bootstrap_px4.sh`, `scripts/prepare_px4_tree.sh`
  - `scripts/build_sih.sh`, `scripts/build_sitl.sh`, `scripts/build_px4.sh`, `scripts/build_nuttx.sh`
- **Simulation runs and log collection**:
  - `scripts/run_sitl_sih.sh`: Launches SIH with the active vehicle + profile, archives ULogs under `logs/sim/YYYY-MM-DD/<run-id>/`
  - `scripts/run_profile_commands.py`: Sends the TV3 launch / command timeline over MAVLink
  - `scripts/run_hawkeye.sh`: Starts the Hawkeye UDP viewer (visualization only)
  - `scripts/sync_sitl_logger_topics.sh`: Ensures the generated logger profile is present before boot
- **Gate and completion scripts** (see `docs/completion_roadmap.md`):
  - `scripts/check_barebones.sh`, `scripts/check_hover_window.sh`, `scripts/check_physical_manifests.sh`, `scripts/check_propulsion_semantics.sh`, `scripts/check_control_mixer.sh`, etc.
  - `scripts/check_completion_status.sh` + `tools/report_completion_status.py` aggregate results into `docs/completion_status.md`
- **Hardware deployment**:
  - `scripts/stage_microsd.sh`: Validates manifest and copies generated runtime payload to a microSD card
  - `scripts/complete_phase2_bench.sh`: Captures bench measurements (load cell, TVC limits, etc.) back into the vehicle manifest

### 5. Visualization and Log Replay

Tools that turn ULogs into reviewable artifacts.

- **2D timeseries plots**: `scripts/plot_ulog.sh` + `tools/plot_ulog.py` (matplotlib)
- **Interactive 3D spatial review** (PyVista):
  - `scripts/view_vehicle_frame.sh`
  - `scripts/plot_ulog_replay.sh`
  - `scripts/plot_ulog_engines.sh`
- **Timed playback / scrubber** (Rerun):
  - `scripts/tv3_replay.sh --rerun` (produces one .rrd per sim with trajectory+engines+guidance)
  - Scene builders support trajectory, per-engine thrust, and guidance metrics on a shared `sim_time` timeline
- Core helpers live in `tools/ulog_replay_common.py`, `tools/viz_common.py`, `tools/rerun_replay.py`, `tools/pyvista_viz.py`, `tools/scene_builders.py`, etc.

Hawkeye receives a UDP pose stream on port 19410 but is not the physics source of truth.

### 6. Tests

Host-side tests protect assumptions and math that are difficult to debug on target.

- Location: `tests/`
- Coverage areas include:
  - Control allocator reachability and plant model agreement (`test_control_allocator.py`)
  - Guidance envelope calculations (`test_guidance_envelope.py`)
  - Propulsion semantics and load-cell state transitions, including synthetic ADC replay (`test_propulsion_semantics.py`)
  - Vehicle manifest validation and asset generation parity (`test_vehicle_manifest_validation.py`, `test_vehicle_assets.py`)
  - Plotting and replay smoke tests
  - Gimbal axis construction and motor catalog behavior
- Fixtures under `tests/fixtures/` provide minimal ULogs and load-cell traces for deterministic testing.

## Cross-References

- [Simulation Guide](simulation.md) — how to build and run SIH, ports, daily workflow, limitations.
- [Data Visualization](data_visualization.md) — full visualization stack, logger topics, replay usage.
- [Control Mixer](control_mixer.md) — attitude mixer, allocator geometry, splay throttle, and reachability model.
- [Hardware Flight Workflow](hardware_flight_workflow.md) — flashing, microSD staging, QGC setup, parameter checks, launch-day flow.
- [Completion Roadmap](completion_roadmap.md) — phases, exit criteria, and gate scripts.
- [Completion Status](completion_status.md) — live progress dashboard (regenerated by scripts).

See the README for quick-start commands and the overall status of the project.