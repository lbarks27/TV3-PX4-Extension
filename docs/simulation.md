# TV3 PX4 SIH Simulation

The active simulator path is PX4 Simulation-In-Hardware (SIH) with the custom `tv3_sih` module. Hawkeye is a viewer only; the physics source of truth is `tv3_sih`.

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

# Plot the newest archived run:
./scripts/setup_viz_env.sh        # once
./scripts/plot_ulog.sh --latest
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
./scripts/build_sih.sh
```

`build_sih.sh` prepares `../.work/px4-tv3`, points `EXTERNAL_MODULES_LOCATION` at the no-space symlink `../.work/tv3-px4-extension`, and builds `px4_sitl_default` with the external `tv3_sih` module.

## Run

```bash
./scripts/run_sitl_sih.sh
```

The launcher sets:

- `PX4_SIMULATOR=sihsim`
- `PX4_SIM_MODEL=tv3_lander`
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

See also [docs/data_visualization.md](data_visualization.md) for ULog plotting and [docs/completion_roadmap.md](completion_roadmap.md) for phase gates.
