# TV3 Flight Profiles

Flight profiles live here. A profile is a checked-in scenario target that can be
loaded on top of a vehicle manifest without changing the vehicle's measured
geometry, hardware, propulsion, or controller configuration.

Use profiles for simulator runs such as single-engine ascent, lander ignition
sequence checks, hover windows, waypoint tracks, landing approaches, and
abort/fault paths. Vehicle definitions stay in `config/vehicles/*.yaml`.

## Loading A Profile

Generate assets with an explicit profile:

```bash
./tools/generate_vehicle_assets.py \
	--vehicle config/vehicles/tv3_lander_v1.yaml \
	--flight-profile config/flight_profiles/lander_hover_window.yaml \
	--output build/lander_hover_window
```

The repo scripts also accept `TV3_FLIGHT_PROFILE`:

```bash
TV3_VEHICLE_CONFIG=config/vehicles/tv3_lander_v1.yaml \
TV3_FLIGHT_PROFILE=config/flight_profiles/lander_hover_window.yaml \
./scripts/check_barebones.sh
```

For SITL, use the profile during asset generation or full worktree preparation:

```bash
TV3_VEHICLE_CONFIG=config/vehicles/tv3_lander_v1.yaml \
TV3_FLIGHT_PROFILE=config/flight_profiles/lander_waypoint_track.yaml \
./scripts/prepare_px4_tree.sh
```

Today the generator applies the profile's `guidance` block to generated PX4
`RK_GD_*` params and overlays `mission_profile` metadata for traceability.
`scripts/run_sitl_sih.sh` starts `scripts/run_profile_commands.py` by default,
which executes profile command timelines such as arm and launch over MAVLink.

## Current Starter Profiles

- `single_engine_ascent.yaml`: default `tv3_v1` ascent gate with guidance off.
- `lander_ignition_sequence.yaml`: three-engine sequencing smoke scenario.
- `lander_hover_window.yaml`: short local hover window for `tv3_lander_v1`.
- `lander_waypoint_track.yaml`: nominal waypoint and landing approach scenario.
- `lander_abort_fault_path.yaml`: future fault-injection and abort review case.
