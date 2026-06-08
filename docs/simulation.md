# Running the TV3 PX4 Rocket in Gazebo SITL

This guide explains how to run the TV3 rocket simulation using PX4's Gazebo (gz) simulator and how to connect it to QGroundControl (QGC). The goal is to have a **reproducible workflow** that any developer can follow on macOS (the primary development platform for this repo).

## Prerequisites (macOS)

1. **Homebrew** (package manager)
2. **Gazebo Harmonic** (gz sim 8.x)
grok   ```bash
   brew install gz-harmonic
   ```
   Verify:
   ```bash
   gz sim --version   # Should report 8.x
   ```
3. **Qt 5** (required by some PX4 SITL tools)
   ```bash
   brew install qt@5
   ```
4. Other common tools:
   ```bash
   brew install python3 git
   ```

> **Note**: The repository directory name contains a space (`TV3 PX4 Extension`). All scripts quote paths carefully, but be aware when writing your own commands.

## Repository Layout

The project expects this layout next to the extension:

```
TV3/
├── TV3 PX4 Extension/     ← this repo (you are here)
├── vendor/
│   └── px4/               ← cloned by bootstrap_px4.sh
└── .work/
    ├── px4-tv3/           ← worktree created by prepare_px4_tree.sh
    └── tv3-px4-extension  ← symlink to this repo (for EXTERNAL_MODULES_LOCATION)
```

The scripts (`bootstrap_px4.sh`, `prepare_px4_tree.sh`, etc.) enforce this layout automatically.

## First-Time / Full Setup (Slow but Complete)

Run these steps **once** (or after a `git clean` / major PX4 version change). This will take a long time (10–40+ minutes) because it checks out PX4 + submodules and does a full build.

From inside the extension directory:

```bash
# 1. Generate bare-bones assets and run host tests
./scripts/check_barebones.sh

# 2. Clone / update the PX4 source (goes to ../vendor/px4)
./scripts/bootstrap_px4.sh

# 3. Create a patched worktree + apply overlays + generate models
./scripts/prepare_px4_tree.sh

# 4. Build SITL (without immediately launching the simulator)
./scripts/build_sitl.sh
```

After this you have a fully prepared `../.work/px4-tv3` tree with your rocket modules, gz model, and patched control allocator. The `check_barebones.sh` step also runs the host tests that validate generated parameters match the firmware definitions in `src/modules/flight_modes/rocket_params.c`.

## Fast / Repeated Runs (Recommended)

After the first full setup, **do not** run `prepare_px4_tree.sh` or `build_sitl.sh` every time — they destroy the worktree and force a full rebuild.

Use the fast launcher instead:

```bash
./scripts/run_sitl_gazebo_fast.sh
```

(Or run the direct `make` invocation shown inside that script.)

To use the three-engine lander instead of the default single-engine ascent vehicle:

```bash
TV3_VEHICLE_CONFIG=config/vehicles/tv3_lander_v1.yaml ./scripts/run_sitl_gazebo_fast.sh
```

## Flight Profiles

Flight profiles live in `config/flight_profiles/`. They describe repeatable SITL
scenarios such as ascent, lander ignition sequence, hover window, waypoint
track, landing approach, and abort/fault cases. Vehicle manifests remain the
source for vehicle geometry, hardware, propulsion, and controller data.

The asset generator can overlay a profile's `guidance` block on the selected
vehicle manifest:

```bash
./tools/generate_vehicle_assets.py \
	--vehicle config/vehicles/tv3_lander_v1.yaml \
	--flight-profile config/flight_profiles/lander_hover_window.yaml \
	--output build/lander_hover_window
```

The repo scripts accept the same selection through environment variables:

```bash
TV3_VEHICLE_CONFIG=config/vehicles/tv3_lander_v1.yaml \
TV3_FLIGHT_PROFILE=config/flight_profiles/lander_waypoint_track.yaml \
./scripts/prepare_px4_tree.sh
```

Use `TV3_FLIGHT_PROFILE` during asset generation or full worktree preparation
before expecting the fast launcher to reflect a changed scenario. The fast
launcher consumes the already-prepared PX4 worktree and generated runtime
params.

This starts:
- Gazebo server + GUI
- The selected model (`tv3_rocket` or `tv3_lander`)
- Your custom rocket modules (`rocket_mode_manager`, `rocket_att_control`, etc.)
- `gz_bridge` connected to the model

The launch scripts also sync the TV3 ULog topic profile into the SITL rootfs so post-run Matplotlib review includes the custom rocket topics. When the sim exits, new `.ulg` files are archived under `logs/sim/YYYY-MM-DD/<run-id>/` in this checkout. See [data_visualization.md](data_visualization.md) for the archive and plotting workflow.

## Connecting to QGroundControl

### MAVLink Ports Used by This SITL Instance

From `px4-rc.mavlink` (for `px4_instance=0`):

- **GCS / QGroundControl link**: UDP port **18570** (localhost)
- Onboard / offboard link: UDP 14580
- Other auxiliary ports also exist (see the log for the full list)

You will see this in the SITL console / log when it starts:

```
INFO  [mavlink] mode: Normal, data rate: 4000000 B/s on udp port 18570 remote port 14550
```

### Steps to Connect QGC

1. Start QGroundControl (latest stable recommended).
2. In QGC go to **Application Settings → Comm Links**.
3. Click **Add** → choose **UDP**.
4. Configure:
   - Name: `TV3 SITL`
   - Port: `18570`
   - (Optional) Add a second link for the offboard port 14580 if you want to test MAVSDK/offboard later.
5. Click **OK**, then select the new link and click **Connect**.
6. You should see a vehicle appear. Because `MAV_TYPE=9` (Rocket), QGC has limited vehicle-specific UI. You will mostly see:
   - Attitude / position
   - QGC messages from the rocket mode manager state machine
   - Parameter editor (all the `RK_*` and `CA_RK_*` parameters)
   - MAVLink console

**Tips**:
- The link is restricted to localhost by default for security (`MAV_0_BROADCAST=0`). This is fine for local QGC.
- If QGC does not auto-discover the link, the manual UDP 18570 entry almost always works.

### TV3 State Machine Access From QGC

The native QGC UI does not decode the custom `rocket_status` or
`rocket_mode_status` uORB topics as first-class widgets. This repo exposes the
state machine to QGC in three practical ways:

1. **QGC messages**: `rocket_mode_manager` sends MAVLink status text whenever
   the state or fault changes. Watch the QGC message area for:
   - `TV3 state DISARMED_SAFE`
   - `TV3 state ARMED_STANDBY`
   - `TV3 state READY`
   - `TV3 state IGNITION_PENDING`
   - `TV3 state BOOST`
   - `TV3 state COAST`
   - `TV3 state ABORT fault <reason>`
2. **MAVLink Console**: In QGC, open **Analyze Tools -> MAVLink Console** and
   run:
   ```sh
   rocket_mode_manager status
   listener rocket_status
   listener rocket_mode_status
   listener vehicle_command_ack
   ```
3. **ULog review**: `rocket_status` and `rocket_mode_status` are recorded in
   the repo-local sim log archive. Use `./tools/plot_ulog.py --latest` after a
   run for post-flight review.

### TV3 Commands From QGC

The firmware accepts `MAV_CMD_USER_1` / command `31010` through
`rocket_mode_manager`:

| Action | `param1` | Equivalent PX4 shell command |
| --- | ---: | --- |
| Launch | `1` | `rocket_mode_manager launch` |
| Abort | `2` | `rocket_mode_manager abort` |
| Reset | `3` | `rocket_mode_manager reset` |

Invalid `param1` values are denied with `vehicle_command_ack`.

For QGC Fly View buttons, install this repo's action file:

```bash
./scripts/install_qgc_actions.sh
```

This copies `config/qgc/TV3RocketActions.json` into:

```text
~/Documents/QGroundControl/MavlinkActions/TV3RocketActions.json
```

Restart QGC after installing. The Fly View action list will include
`TV3 Launch`, `TV3 Abort`, and `TV3 Reset`. QGC loads MAVLink action files only
at startup.

You can also use the MAVLink Console shell commands directly:

```sh
commander arm
rocket_mode_manager launch
rocket_mode_manager abort
rocket_mode_manager reset
```

The state machine still enforces its internal gates: the vehicle must be armed,
`RK_ENABLE` must be enabled, and a motor must be loaded before `READY` can
advance to launch.

## Stopping the Simulation

The cleanest ways:

```bash
# From another terminal
pkill -f "gz sim"
pkill -f "/px4_sitl_default/bin/px4"
```

Or kill the specific PIDs shown in `ps aux | grep px4`.

Closing the Gazebo GUI will usually also shut down the server.

## Known Limitations (Bare-Bones Phase)

- The default single-engine Gazebo model (`tv3_rocket`) keeps `include_tv3_plugin: false` in `config/vehicles/tv3_v1.yaml` for the first ascent gate.
- The three-engine lander manifest (`config/vehicles/tv3_lander_v1.yaml`) enables the `libtv3_rocket_gz` system plugin so Gazebo applies the first pass of motor thrust, splay cosine loss, and geometry-derived torque to the rigid body.
- Vehicle structure is CAD-only in Gazebo. Until renderable meshes are placed in `assets/cad/tv3_v1/` or `assets/cad/lander/`, the generator omits the structure instead of drawing a procedural approximation.
- Both manifests render body-frame X/Y/Z orientation arrows pinned at the body center of mass. These are visual-only aids and do not add collisions, mass, or joints.
- Joint markers are also visual-only: `tv3_v1` renders declared TVC joint origins/axes from `physical_model.joints`, while the lander renders engine pivot markers from the manifest engine positions.
- Animated lander nozzle visuals must be separate meshes named `engine_nozzle_0`, `engine_nozzle_1`, and `engine_nozzle_2`; the Gazebo plugin moves those from command-truth pitch, yaw, and splay values.
- QGC has very little built-in support for `MAV_TYPE_ROCKET`. Expect generic vehicle views and no 3D rocket model.
- The `prepare_px4_tree.sh` script **always** deletes and recreates the worktree. This is safe but slow. Use the "Fast" method above for daily work.
- On macOS, Gazebo sometimes has rendering or input focus quirks — restarting the GUI usually resolves them.

## Troubleshooting

### "prepare_px4_tree.sh takes forever / uses huge amounts of bandwidth"

This is expected the first time (full PX4 + submodule checkout). Subsequent runs of the full prepare are wasteful. Prefer the fast launch method.

### QGC does not see any vehicle

- Confirm the port with:
  ```bash
  lsof -i UDP:18570
  ```
- Manually add the UDP link on port 18570 in QGC (see above).
- Check the SITL log for `mavlink` startup lines.

### "ERROR [init] Gazebo simulation dependencies not found"

Make sure `gz sim --version` reports ≥ 8.0 and that you are using a supported Gazebo Harmonic installation.

### The rocket falls / doesn't move

Check which manifest was prepared. `tv3_v1.yaml` (the default) intentionally has the Gazebo physics plugin disabled (`include_tv3_plugin: false`). Use `TV3_VEHICLE_CONFIG=config/vehicles/tv3_lander_v1.yaml` when you want the three-engine lander with the first-pass physics plugin enabled.

## Related Files

- `scripts/run_sitl_gazebo_fast.sh` — recommended daily launcher (avoids repeated prepare)
- `scripts/run_sitl_gazebo.sh` — full launcher that calls prepare every time (useful for CI or after clean)
- `overlay/ROMFS/init.d-posix/airframes/11000_gz_tv3_rocket` + `tv3_rocket_common.*`
- `config/vehicles/tv3_v1.yaml` — default single-engine ascent manifest (guidance disabled)
- `config/vehicles/tv3_lander_v1.yaml` — three-engine lander manifest (guidance + Gazebo plugin enabled by default)
- `config/flight_profiles/` — checked-in SITL scenario targets loaded with `TV3_FLIGHT_PROFILE`
- `tools/generate_vehicle_assets.py` — regenerates SDF, params, and runtime payloads from a vehicle yaml
- `tools/plot_ulog.py` — creates TV3-focused Matplotlib review plots from PX4 ULog files
- `docs/data_visualization.md` — ULog topic profile and plotting workflow
- `tools/generate_motor_catalog.py` + `scripts/bootstrap_thrust_curves.sh` — motor curve ingestion pipeline (populates `build/motors/`)

---

This document aims to be the single source of truth for anyone who wants to run the simulation and connect it to a ground station. Please update it when the workflow or port numbers change.

For the current implementation status and what remains before flight gates, see [docs/completion_roadmap.md](completion_roadmap.md).
