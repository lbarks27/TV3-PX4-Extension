# TV3 Implementation Phases (Historical)

> **Note**: The active, detailed plan from the current state to flight-ready gates lives in [docs/completion_roadmap.md](completion_roadmap.md). This file is retained for historical context on the original product intent and high-level phasing.

For the full from-current-state-to-flight-ready plan, see `docs/completion_roadmap.md`.

## Product Intent

TV3 is an out-of-tree PX4 extension for thrust-vector-controlled TV3 vehicles. The intended product is not a standalone flight app; it is a set of PX4 modules, uORB topics, vehicle definitions, ROMFS overlays, SIH simulation modules, and host-side generators that let selected vehicle manifests drive SITL, hardware runtime assets, and motor-reference data.

The core loop is:

1. Load a selected motor reference and expected mass/thrust curve.
2. Convert load-cell or reference thrust into a trusted `tv3_thrust` signal.
3. Gate launch, ignition, boost, burnout, coast, abort, and reset through `tv3_mode_manager`.
4. Publish thrust and torque setpoints for PX4 control allocation and TVC actuation.
5. When explicitly enabled by a manifest, layer waypoint/apogee/landing guidance on top of the launch loop after checking the remaining thrust/control envelope.

## Current Bare-Bones Product

The repo now defaults to the smallest useful launch/boost slice:

- Built by default: `tv3_motor_model`, `tv3_load_cell`, `tv3_mode_manager`, `tv3_att_control`, and manifest-gated `tv3_guidance`.
- Started by default in SITL overlays: control allocator, engine control, motor model, load cell, mode manager, and attitude control.
- Disabled by default at startup: autonomous guidance starts only when the selected vehicle manifest sets `guidance.enable: 1`.
- Verified locally: motor catalog normalization, vehicle asset generation, and generated `RK_*` parameter names matching firmware definitions.
- Generated runtime and simulation assets live under `build/`; tracked runtime files are limited to SD-card startup templates.

This gives us a narrow product we can reason about: configure a tv3, load motor data, arm, command launch/abort/reset, detect ignition/burnout, and publish TVC-relevant setpoints. The current repo also includes a three-engine lander manifest, per-engine propulsion state plumbing, the `tv3_sih` simulation module, and a host allocator helper; those are the starting point for the completion roadmap.

## Phase Gates

Phase 1: Bare-bones launch loop

- Keep the active module set small.
- Use reference thrust in SITL and load-cell input on hardware.
- Keep guidance off until the launch/boost loop is observable and boring.
- Smoke test with `./scripts/check_barebones.sh`.

Phase 2: Motor and load-cell confidence

- Validate real motor catalog coverage.
- Add calibration workflow for tare/scale/timeout.
- Add fault-injection tests for stale thrust, missing channel, bad scale, and no reference.

Phase 3: Guidance re-entry

- Use a guidance-enabled manifest so `prepare_px4_tree.sh` starts `tv3_guidance` in the generated startup overlay.
- Decide which phases are commanded by PX4 trajectory setpoints versus tv3-only status.
- Add tests for standby, ascent, apogee, waypoint, landing, complete, and abort transitions.

Phase 4: PX4 allocator and SITL fidelity

- Refresh the PX4 patch against the selected PX4 tag.
- Run SIH asset generation from the vehicle JSON manifest.
- Verify TVC actuator outputs under expected and measured thrust changes.

Phase 5: Hardware readiness

- Stage NuttX SD-card payloads.
- Add preflight checks for motor selection, load-cell validity, command source, arming state, and GCS-loss policy.
- Produce a repeatable launch-day checklist from the same configuration surface.
