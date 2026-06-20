# TV3 Completion Roadmap

This document is the **plan**: phases, goals, exit criteria, and gate scripts. It should change rarely.

For **current progress**, gate results, manifest provenance counts, and evidence links, see the generated dashboard in [docs/completion_status.md](completion_status.md). Refresh it with:

```bash
./scripts/check_completion_status.sh
./scripts/check_completion_status.sh --run-gates fast   # rerun fast gate scripts
./scripts/check_completion_status.sh --run-gates all    # include slow SIH hover gate
```

Manual notes and status overrides live in [config/completion_status.json](../config/completion_status.json).

"Completion" means the repo can repeatedly prove, from checked-in configuration and archived logs, that the selected vehicle can be simulated, bench-tested, flown, logged, and reviewed against its mission.

For build and run instructions, see [docs/simulation.md](simulation.md).

## Status Vocabulary

| Status | Meaning |
| --- | --- |
| `not_started` | No meaningful work or tooling yet |
| `in_progress` | Partial progress; exit criteria not fully met |
| `structural` | Repo tooling/tests in place; not mission- or hardware-proven |
| `verified` | Exit script passes and evidence artifacts exist |
| `blocked` | Known external blocker (set via `status_override` in the status JSON) |

Phase 2 field-level progress is derived from `data_status.fields` in each vehicle manifest (`measured`, `preliminary`, `placeholder`). Do not duplicate those counts here.

## Source Of Truth

- Vehicle manifests: `config/vehicles/*.json`
- Flight profiles and scenario targets: `config/flight_profiles/*.json`
- Intake schema: `config/schemas/vehicle_intake_schema.json`
- Runtime params and SD-card payloads: generated output under `build/`
- PX4 external modules and messages: `src/` and `msg/`
- Test evidence: host test output, SIH logs, ULog files, bench calibration reports, and flight-review notes

Do not hand-edit generated runtime or sim outputs as source. If a value matters, put it in a manifest, schema, generator, or measured-data file.

## Phase 0: Stabilize SIH Baseline

Goal: make the active simulator boring to build and easy to reproduce.

Work:

- Keep `tv3_v1` as the default manifest and `guidance.enable: 0`.
- Use `tv3_lander_v1` plus `lander_hover_window.json` as the first required SIH gate.
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
TV3_VEHICLE_CONFIG=config/vehicles/tv3_lander_v1.json
TV3_FLIGHT_PROFILE=config/flight_profiles/lander_hover_window.json
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

Exit scripts:

```bash
./scripts/check_physical_manifests.sh          # schema + generator gate
./scripts/stage_microsd.sh                     # deploy runtime payload to SD card
./scripts/complete_phase2_bench.sh             # MAVLink capture into manifest
```

Hardware bench order: flash TV3 firmware → stage microSD → calibrate load cell →
run `complete_phase2_bench.sh` with QGC closed. See
[docs/hardware_flight_workflow.md](hardware_flight_workflow.md).

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

Exit script:

```bash
./scripts/check_propulsion_semantics.sh
```

Exit criteria:

- SIH shows reference-thrust ignition and load-cell-confirmed ignition using the same state transitions.
- Bench ADC replay reproduces expected `tv3_engine_state` and aggregate thrust behavior.

## Phase 4: Control Mixer

See [control_mixer.md](control_mixer.md) for the attitude mixer, PX4 allocator, triple-engine
TVC geometry, splay throttle, and reachability model.

Goal: requested torque and net thrust become reachable, bounded engine commands.

Work:

- Promote the host allocator into a shared constrained solver.
- Cover nominal, saturated, failed-engine, low-thrust, high-thrust, and burnout cases.
- Keep unreachable results explicit in logs.

Exit script:

```bash
./scripts/check_control_mixer.sh
```

Exit criteria:

- Flight mixer and host allocator agree on representative cases.
- Hover and landing guidance can query reachability before committing to a solution.

## Phase 5: Guidance And Monte Carlo

Goal: guidance only claims a solution when the remaining vehicle envelope can execute it.

Work:

- Extend guidance checks from simple thrust margin to remaining impulse, available thrust over time, torque authority, landing reserve, and abort corridor.
- Add deterministic scenario tests, then Monte Carlo sweeps over mass, thrust curves, wind, sensor noise, ignition delay, and actuator lag.

Exit script:

```bash
./scripts/check_guidance_monte_carlo.sh
```

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
