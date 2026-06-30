# TV3 Flight Profiles

Flight profiles live here. A profile is a checked-in scenario target that can be
loaded on top of a vehicle manifest without changing the vehicle's measured
geometry, hardware, propulsion, or controller configuration.

Use profiles for simulator runs such as single-engine ascent, lander ignition
sequence checks, hover windows, waypoint tracks, landing approaches, and
abort/fault paths. Vehicle definitions stay in `config/vehicles/*.json`.

## Loading A Profile

Generate assets with an explicit profile:

```bash
./tools/generate_vehicle_assets.py \
	--vehicle config/vehicles/tv3_lander_v1.json \
	--flight-profile config/flight_profiles/lander_hover_window.json \
	--output build/lander_hover_window
```

The repo scripts also accept `TV3_FLIGHT_PROFILE`:

```bash
TV3_VEHICLE_CONFIG=config/vehicles/tv3_lander_v1.json \
TV3_FLIGHT_PROFILE=config/flight_profiles/lander_hover_window.json \
./scripts/check_barebones.sh
```

For SITL, use the profile during asset generation or full worktree preparation:

```bash
TV3_VEHICLE_CONFIG=config/vehicles/tv3_lander_v1.json \
TV3_FLIGHT_PROFILE=config/flight_profiles/lander_waypoint_track.json \
./scripts/prepare_px4_tree.sh
```

Today the generator should apply the profile's `control.phases` block to `RK_CTRL_*`
params (per-module modes keyed by `tv3_sm_status` lifecycle mode). Legacy `guidance`
blocks are deprecated in the simplified stack. `mission_profile` remains traceability
metadata. `scripts/run_sitl_sih.sh` starts `scripts/run_profile_commands.py` by default,
which executes profile command timelines such as arm and launch over MAVLink.

See `config/schemas/flight_profile_schema.json` and `config/flight_profiles/lander_boost_upright.json`
for the v2 `control.phases` shape.

## Current Starter Profiles (lander precision focus)

- `lander_boost_upright.json`: simplified stack boost-upright gate (v2 `control.phases`).
- `lander_hover_window.json`: short local hover for `tv3_lander_v1`.
- `lander_offset_hover_window.json`: lateral translate + hover.
- `lander_waypoint_track.json`: launch + waypoints + landing.
- `lander_precision_land.json`: tight 0.5 m acceptance + low descent rates for the real-vehicle goal (recommended).
- `lander_ignition_sequence.json`, `lander_splay_throttle.json`, `lander_abort_fault_path.json`: supporting.

Experimental / legacy profiles have been moved to `_archive/config/flight_profiles/`. `single_engine_ascent` and many boost variants are de-emphasized (lander focus).