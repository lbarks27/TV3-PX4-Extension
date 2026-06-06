# TV3 Completion Roadmap

This document describes the path from the current software stack to a flight-ready TV3 repo. "Completion" does not mean every future idea is implemented. It means the repo can repeatedly prove, from checked-in configuration and generated artifacts, that the selected vehicle can be simulated, bench-tested, flown, logged, and reviewed against its mission.

For instructions on actually building and running the current stack in SITL (including QGroundControl), see [docs/simulation.md](simulation.md).

For this repo, completion has two vehicle targets:

- `tv3_v1`: the default single-engine TVC ascent vehicle. This is the first free-flight gate.
- `tv3_lander_v1`: the three-engine solid-motor lander. This is the later hover, waypoint, and precision-landing vehicle.

The first vehicle proves the launch loop, motor/load-cell path, TVC attitude control, and flight data workflow. The second vehicle proves multi-engine sequencing, splay-based thrust control, constrained allocation, guidance envelope checks, hover/waypoint behavior, and landing.

## Current State

The repo is now past the "skeleton" stage:

- Host tests and `./scripts/check_barebones.sh` validate the generated asset path.
- `config/vehicles/tv3_v1.yaml` remains the default single-engine ascent manifest.
- `config/vehicles/tv3_lander_v1.yaml` adds a three-engine lander manifest.
- `config/schemas/vehicle_intake_schema.yaml` defines the measured-data intake contract.
- `config/flight_profiles/*.yaml` defines checked-in SITL scenario targets that can be overlaid on vehicle manifests.
- `tools/generate_vehicle_assets.py` generates runtime params, motor placeholders, Gazebo assets, and JSBSim assets from a selected manifest.
- `rocket_motor_model`, `rocket_load_cell`, and `rocket_mode_manager` have per-engine command/state plumbing while preserving aggregate topics.
- `rocket_guidance` is compiled but manifest-gated; it starts only when the selected vehicle enables guidance.
- `src/gazebo_plugins/tv3_rocket` builds a first-pass Gazebo system plugin that applies motor thrust, splay cosine loss, and geometry-derived torque.
- `tools/rocket_allocator.py` provides a host-side constrained reachability solver for the three-engine lander.
- Host tests (`tests/`) + `./scripts/check_barebones.sh` cover asset generation, param fidelity, allocator reachability, and manifest-driven startup differences between the two vehicles.

Important limits remain:

- The physical manifests still contain preliminary mass, CG, inertia, engine geometry, and actuator values.
- The Gazebo plugin is a first physics hook, not yet a fully closed-loop PX4/Gazebo actuator and load-cell model.
- The allocator exists as a host solver, but the flight control mixer still needs the shared constrained solver path.
- Guidance has an envelope check, but the launch-to-landing solution is not proven in 6DoF SITL.
- Hardware ignition, load-cell calibration, and restrained/tethered validation are still flight blockers.

## Source Of Truth

Everything should continue to flow from manifest data and measured evidence:

- Vehicle manifests: `config/vehicles/*.yaml`
- Flight profiles and scenario targets: `config/flight_profiles/*.yaml`
- Intake schema: `config/schemas/vehicle_intake_schema.yaml`
- Motor catalog and curves: generated from vendor/reference data into `build/motors`
- Runtime params and SD-card payloads: generated output under `build/`
- Gazebo/JSBSim assets: generated output under `build/`
- PX4 external modules and messages: `src/` and `msg/`
- Test evidence: host test output, SITL logs, ULog files, bench calibration reports, and flight-review notes

Do not hand-edit generated runtime or sim assets as source. If a value matters, put it in a manifest, schema, generator, or measured-data file.

## Phase 0: Stabilize The Baseline

Goal: make the current stack boring to build and easy to reproduce.

Work:

- Keep `tv3_v1` as the default manifest and `guidance.enable: 0`.
- Keep `tv3_lander_v1` available through `TV3_VEHICLE_CONFIG`.
- Remove stale local path assumptions and verify `.work/tv3-px4-extension` points to this checkout.
- Keep generated outputs disposable under `build/`.
- Add a short release/checkpoint note after major verified changes.

Exit criteria:

- `python3 -m unittest discover -s tests -v` passes.
- `./scripts/check_barebones.sh` passes.
- `./scripts/prepare_px4_tree.sh` succeeds from a clean worktree.
- `make -C ../.work/px4-tv3 px4_sitl_default DONT_RUN=1` builds.
- The generated default airframe starts no guidance by default.

## Phase 1: Replace Provisional Physical Data

Goal: the manifests represent the real vehicles closely enough for sim and control design.

Work:

- Fill the intake schema with measured values for both vehicles:
  - total mass and dry mass
  - current CG and expected CG migration
  - inertia tensor about the declared reference frame
  - engine mount positions and thrust axes
  - TVC/splay axes, limits, trims, backlash, and slew rates
  - load-cell channel map, tare, scale, noise, and timeout
  - ignition outputs and expected continuity behavior
  - vendor/reference motor curves and measured load-cell curve overlays
- Add a repeatable measured-data import path rather than copying values manually.
- Add schema validation tests for required measured fields and unit conventions.

Exit criteria:

- Both vehicle manifests validate against the schema.
- Generated PX4 params match measured geometry and actuator limits.
- Generated Gazebo inertials are traceable to measured or CAD-derived values.
- The repo clearly labels any remaining placeholders as non-flight values.

## Phase 2: Prove Propulsion And Load-Cell Semantics

Goal: flight software sees motor state the same way in SITL, bench tests, and flight.

Work:

- Finish per-engine ignition sequencing:
  - command engine N
  - wait for load-cell confirmation
  - dwell
  - advance to engine N+1
  - abort or hold on timeout/fault
- Add per-engine fault semantics for missing load cell, stale data, false positive, under-thrust, over-thrust, and burnout.
- Keep aggregate `rocket_thrust` and `rocket_motor_reference` behavior stable for compatibility.
- Add ADC replay tests using recorded or synthetic load-cell traces.
- Add a bench calibration report template for tare, scale, noise, and threshold selection.

Exit criteria:

- Host tests cover nominal ignition, delayed ignition, failed ignition, false-positive rejection, burnout, and stale sensor cases.
- SITL can show reference-thrust ignition and load-cell-confirmed ignition using the same state transitions.
- Bench ADC replay reproduces the expected `rocket_engine_state` and aggregate thrust behavior.

## Phase 3: Close The Gazebo 6DoF Loop

Goal: Gazebo becomes the authoritative closed-loop simulation environment.

Work:

- Couple PX4 per-engine commands to the Gazebo rocket plugin.
- Simulate per-engine thrust from the same motor references used by PX4.
- Apply TVC/splay actuator dynamics, slew limits, trims, and saturation.
- Feed simulated per-engine load-cell readings back into PX4.
- Update mass, CG, and inertia as propellant burns.
- Add scenario launch scripts for:
  - single-engine ascent
  - lander ignition sequence
  - hover window
  - waypoint track
  - landing approach
  - abort/fault paths

Exit criteria:

- `tv3_v1` leaves the rail, responds to TVC, burns out, and coasts in Gazebo.
- `tv3_lander_v1` shows per-engine force response and sequencing in Gazebo.
- SITL logs include rocket topics, actuator commands, guidance status, and enough vehicle state for post-run review.
- A failed engine or stale load cell produces a clear abort or no-solution result.

## Phase 4: Finish The Control Mixer

Goal: requested torque and net thrust become reachable, bounded engine commands.

Work:

- Move from the host-only allocator helper to a shared constrained solver used by tests and flight code.
- Inputs:
  - desired roll, pitch, yaw torque
  - desired net thrust
  - current per-engine thrust
  - engine geometry
  - TVC/splay limits
  - slew limits
  - failed or unavailable engines
- Outputs:
  - per-engine pitch/yaw/splay commands
  - saturation flags
  - clear unreachable result
  - residual torque/thrust error
- Keep roll control explicit. If roll is only possible through canted geometry and engine placement, the solver must say so; do not hide roll authority assumptions in trims.
- Add allocator tests across nominal, saturated, failed-engine, low-thrust, high-thrust, and burnout conditions.

Exit criteria:

- The solver passes host tests and PX4 build.
- The flight mixer and host allocator agree on representative cases.
- Saturation is visible in logs and does not masquerade as good control.
- Hover and landing guidance can query whether the requested thrust/torque envelope is reachable before committing to a solution.

## Phase 5: Prove Guidance In Simulation

Goal: guidance only claims a solution when the remaining vehicle envelope can actually execute it.

Work:

- Keep guidance disabled for `tv3_v1` by default.
- Enable guidance only for explicit SITL validation manifests.
- Extend guidance checks from simple thrust margin to a reachable envelope:
  - remaining impulse
  - available thrust over time
  - thrust-to-weight ratio
  - torque authority
  - landing reserve
  - waypoint acceptance
  - abort corridor
- Add deterministic scenario tests first, then Monte Carlo sweeps over mass, thrust curves, wind, sensor noise, ignition delay, and actuator lag.
- Define success metrics before tuning:
  - max position error
  - landing dispersion
  - saturation time
  - minimum thrust margin
  - abort correctness
  - touchdown velocity and attitude

Exit criteria:

- `tv3_v1` ascent guidance remains off unless explicitly requested.
- `tv3_lander_v1` can complete launch, waypoint, hover/descend, and landing scenarios in Gazebo with logged margins.
- Guidance reports no-solution for impossible profiles instead of producing unsafe setpoints.
- Scenario results are reproducible from checked-in configs and generated artifacts.

## Phase 6: Bench And Hardware-In-The-Loop Gates

Goal: real sensors and outputs behave like the simulated interfaces.

Work:

- Calibrate load cells and store calibration evidence.
- Verify no-thrust false-positive rejection.
- Verify igniter command continuity and timing without live motors.
- Replay ADC traces into the software path.
- Run restrained motor/load-cell ignition confirmation.
- Confirm TVC/splay actuator limits, trims, and measured slew rates.
- Confirm SD-card payload generation and PX4 startup on the target autopilot.
- Confirm ULog captures the rocket-specific topics needed for review.

Exit criteria:

- Bench tests reproduce expected `rocket_mode_manager` transitions.
- Ignition cannot advance without load-cell confirmation unless an explicit test mode allows it.
- A failed load-cell channel prevents flight or forces the configured abort path.
- Logs contain enough data to compare real thrust, expected thrust, commands, and vehicle response.

## Phase 7: Flight Gates

Goal: fly only the vehicle and mission that have passed the matching evidence gates.

Gate A: single-engine ascent free flight

- Vehicle: `tv3_v1`
- Guidance: off unless separately reviewed
- Required evidence:
  - measured physical manifest
  - motor/load-cell bench confirmation
  - Gazebo ascent response
  - TVC command sanity
  - abort/reset command verification
  - launch-day checklist

Gate B: three-engine restrained/tethered lander tests

- Vehicle: `tv3_lander_v1`
- Guidance: sim-validation mode only until bench and fixture tests pass
- Required evidence:
  - load-cell-confirmed ignition sequence
  - allocator saturation tests
  - Gazebo hover window
  - actuator thermal/mechanical checks
  - fixture/tether safety review

Gate C: three-engine waypoint and landing flight

- Vehicle: `tv3_lander_v1`
- Guidance: enabled only after repeated SITL and bench passes
- Required evidence:
  - launch-to-landing Gazebo pass
  - Monte Carlo dispersion report
  - load-cell and motor-curve agreement
  - reachable envelope margins
  - abort behavior for key failures
  - post-test review from tethered/fixture runs

## Completion Definition

The repo is complete when all of these are true:

- Both vehicle manifests are measured-data-backed and schema-valid.
- Asset generation is the only path from manifests to runtime/sim outputs.
- Host tests cover schema, generation, propulsion, load-cell faults, ignition sequence, allocator reachability, and guidance no-solution cases.
- Gazebo 6DoF SITL can run the required scenarios for both vehicles.
- The control mixer produces bounded per-engine commands and clear unreachable results.
- Guidance can prove or reject launch, waypoint, hover, and landing demands from the current remaining envelope.
- Bench procedures validate load cells, igniters, actuator limits, ADC replay, and restrained ignition behavior.
- Flight gates are documented with pass/fail evidence and ULog review requirements.
- The first free-flight gate remains single-engine ascent until the lander gates pass.

## Immediate Next Work

Recommended next tasks, in order:

1. Add real measured fields to `config/vehicles/tv3_v1.yaml`.
2. Add a schema validation test that fails on missing measured flight-critical fields.
3. Add ULog topic configuration for all `rocket_*` topics and actuator/guidance evidence.
4. Close the PX4-to-Gazebo plugin command path for per-engine thrust/TVC/splay commands.
5. Add SITL scripts for `tv3_v1` ascent and `tv3_lander_v1` force-response smoke tests.
6. Promote `tools/rocket_allocator.py` into a shared solver design that flight code can use.
7. Add allocator saturation and failed-engine tests.
8. Add load-cell ADC replay tests.
9. Build the bench calibration report template.
10. Run and archive the first end-to-end `tv3_v1` Gazebo ascent report.
