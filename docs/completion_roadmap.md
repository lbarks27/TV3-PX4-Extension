# TV3 Completion Roadmap

This document describes the path from the current software stack to a flight-ready TV3 repo. "Completion" means the repo can repeatedly prove, from checked-in configuration and archived logs, that the selected vehicle can be simulated, bench-tested, flown, logged, and reviewed against its mission.

For build and run instructions, see [docs/simulation.md](simulation.md).

## Current State

- `tv3_v1` remains the default single-engine TVC ascent manifest.
- `tv3_lander_v1` is the active three-engine lander validation manifest.
- PX4 SIH plus the external `tv3_sih` module is now the active simulator path.
- Hawkeye is visualization only and consumes the PX4 SIH MAVLink stream on UDP `19410`.
- `tools/generate_vehicle_assets.py` generates runtime params, motor placeholders, and logger topics from a selected manifest.
- The retired Gazebo workflow is archived under `deprecated/sim/gazebo/`; large generated payloads and old run logs are archived under `../deprecated-sim/gazebo/`.
- `tv3_motor_model`, `tv3_load_cell`, `tv3_mode_manager`, `tv3_att_control`, `tv3_guidance`, and the allocator plumbing remain the active flight-software path.

Important limits remain:

- Physical manifests still contain preliminary mass, CG, inertia, engine geometry, and actuator values.
- `tv3_sih` is deterministic first-pass 6DoF physics, not a final correlated vehicle model.
- Guidance and allocation are observable but not yet proven across the full launch-to-landing envelope.
- Hardware ignition, load-cell calibration, and restrained/tethered validation are still flight blockers.

## Source Of Truth

- Vehicle manifests: `config/vehicles/*.yaml`
- Flight profiles and scenario targets: `config/flight_profiles/*.yaml`
- Intake schema: `config/schemas/vehicle_intake_schema.yaml`
- Runtime params and SD-card payloads: generated output under `build/`
- PX4 external modules and messages: `src/` and `msg/`
- Test evidence: host test output, SIH logs, ULog files, bench calibration reports, and flight-review notes

Do not hand-edit generated runtime or sim outputs as source. If a value matters, put it in a manifest, schema, generator, or measured-data file.

## Phase 0: Stabilize SIH Baseline

Goal: make the active simulator boring to build and easy to reproduce.

Work:

- Keep `tv3_v1` as the default manifest and `guidance.enable: 0`.
- Use `tv3_lander_v1` plus `lander_hover_window.yaml` as the first required SIH gate.
- Keep `RK_LC_SRC=1` for SIH so load-cell confirmation follows reference thrust.
- Keep generated outputs disposable under `build/`.
- Keep old visual-simulator material in `deprecated/` only.

Exit criteria:

- `python3 -m unittest discover -s tests -v` passes.
- `./scripts/check_barebones.sh` passes.
- `./scripts/build_sih.sh` builds the PX4 SITL binary with `tv3_sih`.
- `./scripts/run_sitl_sih.sh` starts `PX4_SIMULATOR=sihsim`, `PX4_SIM_MODEL=tv3_lander`, and archives a ULog.

## Phase 1: Prove Lander Hover Window

Goal: validate the first required lander scenario in SIH before expanding scope.

Required profile:

```text
TV3_VEHICLE_CONFIG=config/vehicles/tv3_lander_v1.yaml
TV3_FLIGHT_PROFILE=config/flight_profiles/lander_hover_window.yaml
```

Pass criteria:

- Required profile topics exist in the ULog.
- Three engines ignite and complete the configured sequence.
- No stale load-cell or motor-reference faults occur.
- The vehicle reaches the hover window.
- Position error stays within `acceptance_m` for at least `review.min_hover_s`.
- Allocator saturation is logged without sustained uncontrolled saturation.

Exit script:

```bash
./scripts/check_hover_window.sh
```

## Phase 2: Replace Provisional Physical Data

Goal: manifests represent the real vehicles closely enough for simulation and control design.

Work:

- Fill measured mass, CG, inertia, rail, and torque-limit data.
- Fill engine mount positions, thrust axes, TVC/splay axes, trims, backlash, and slew rates.
- Fill load-cell channel maps, tare, scale, noise, and timeout data.
- Promote `data_status.fields` entries from `preliminary` or `placeholder` to `measured` as bench evidence arrives.

Infrastructure now in repo (no measured numbers required):

- Machine-readable intake schema: `config/schemas/vehicle_intake_schema.yaml`
- Manifest validator with unit checks and PX4 param parity: `tools/validate_vehicle_manifest.py`
- Both manifests declare `data_status.flight_ready: false` and per-field provenance
- Generator rejects manifests that fail intake validation

Exit script (structural gate; passes before measured data exists):

```bash
./scripts/check_physical_manifests.sh
```

Exit criteria:

- Both vehicle manifests validate against the schema.
- Generated PX4 params match manifest geometry and actuator limits.
- Remaining placeholders are clearly labeled as non-flight values.
- `data_status.flight_ready` becomes `true` only after measured fields are promoted and revalidated.

## Phase 3: Propulsion And Load-Cell Semantics

Goal: flight software sees motor state the same way in SIH, bench tests, and flight.

Work:

- Finish per-engine ignition sequencing and fault semantics.
- Add tests for delayed ignition, failed ignition, false-positive rejection, burnout, and stale sensor cases.
- Add ADC replay tests using recorded or synthetic load-cell traces.
- Add a bench calibration report template.

Exit criteria:

- SIH shows reference-thrust ignition and load-cell-confirmed ignition using the same state transitions.
- Bench ADC replay reproduces expected `tv3_engine_state` and aggregate thrust behavior.

## Phase 4: Control Mixer

Goal: requested torque and net thrust become reachable, bounded engine commands.

Work:

- Promote the host allocator into a shared constrained solver.
- Cover nominal, saturated, failed-engine, low-thrust, high-thrust, and burnout cases.
- Keep unreachable results explicit in logs.

Exit criteria:

- Flight mixer and host allocator agree on representative cases.
- Hover and landing guidance can query reachability before committing to a solution.

## Phase 5: Guidance And Monte Carlo

Goal: guidance only claims a solution when the remaining vehicle envelope can execute it.

Work:

- Extend guidance checks from simple thrust margin to remaining impulse, available thrust over time, torque authority, landing reserve, and abort corridor.
- Add deterministic scenario tests, then Monte Carlo sweeps over mass, thrust curves, wind, sensor noise, ignition delay, and actuator lag.

Exit criteria:

- `tv3_lander_v1` can complete launch, waypoint, hover/descend, and landing scenarios in SIH with logged margins.
- Guidance reports no-solution for impossible profiles.

## Phase 6: Bench And Hardware Gates

Goal: real sensors and outputs behave like the simulated interfaces.

Work:

- Calibrate load cells and store calibration evidence.
- Verify igniter command continuity and timing without live motors.
- Run restrained motor/load-cell ignition confirmation.
- Confirm TVC/splay actuator limits, trims, and measured slew rates.
- Confirm SD-card payload generation and PX4 startup on the target autopilot.

Exit criteria:

- Bench tests reproduce expected `tv3_mode_manager` transitions.
- Ignition cannot advance without load-cell confirmation unless an explicit test mode allows it.
- Logs contain enough data to compare real thrust, expected thrust, commands, and vehicle response.

## Flight Gates

Gate A: `tv3_v1` single-engine ascent free flight.

Gate B: `tv3_lander_v1` restrained/tethered lander tests.

Gate C: `tv3_lander_v1` waypoint and landing flight.

Each gate requires measured physical manifests, matching bench evidence, ULog review, and documented pass/fail artifacts.
