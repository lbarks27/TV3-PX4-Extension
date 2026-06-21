# TV3 Data Visualization

This repo supports a PX4-first data path for detailed SITL, flight, and ground-test review:

1. Generate a TV3 ULog topic profile with the runtime assets.
2. Sync that profile into the PX4 SITL rootfs before launch.
3. Archive the resulting `.ulg` under `logs/`.
4. Review archived logs with the TV3 viz stack below.

## Visualization Stack

| Use case | Tool | Entry point |
|----------|------|-------------|
| Live SITL 3D pose | **Hawkeye** | `./scripts/run_hawkeye.sh` |
| Interactive 3D spatial review (no timeline) | **PyVista** | `./scripts/view_vehicle_frame.sh`, `./scripts/tv3_replay.sh` |
| Timed log playback (scrubbable timeline) | **Rerun** | `./scripts/tv3_replay.sh --latest --scene trajectory` (or `engines`, `guidance`, `all`) |
| Static PNG export | **PyVista** | `./scripts/tv3_replay.sh -o *.png` or `view_vehicle_frame.sh --save` |
| Static 2D ULog timeseries review | **Matplotlib** | `./scripts/plot_ulog.sh` |

Hawkeye is a live UDP viewer only (port `19410`). PyVista opens an interactive 3D window you can orbit and zoom — it shows a single snapshot in time (pick with `--time`), not a scrubber. Rerun is for full timed playback across the log. Use `-o file.png` when you need a headless snapshot for reports or CI.

## Install And Validate

Run once before any plotting or replay:

```bash
./scripts/setup_viz_env.sh
```

This creates or updates `../.work/tv3-viz-venv`, which avoids installing packages into Homebrew's externally managed Python.

**Always use the repo shell wrappers** (`./scripts/plot_ulog.sh`, `./scripts/tv3_replay.sh`, etc.). They activate the viz venv and prepend its `bin/` directory to `PATH` so the Rerun viewer executable is found. Calling `python3 tools/...` directly will fail with missing `pyvista` / `rerun-sdk` unless you manage the venv yourself.

`plot_ulog_replay.sh` and `plot_ulog_engines.sh` remain as deprecated aliases forwarding to `tv3_replay.sh`.

Headless smoke test (no GUI windows):

```bash
./scripts/validate_viz_commands.sh
```

## Logger Topic Profile

`tools/generate_vehicle_assets.py` writes the TV3 logger profile to:

```text
build/barebones/runtime/etc/logging/logger_topics.txt
build/barebones/runtime/fs/microsd/etc/logging/logger_topics.txt
```

PX4 reads `etc/logging/logger_topics.txt` from its storage directory at boot. The SIH launcher calls `scripts/sync_sitl_logger_topics.sh`, which copies the generated profile into:

```text
../.work/px4-tv3/build/px4_sitl_default/rootfs/etc/logging/logger_topics.txt
```

That profile includes core PX4 state and control-allocation topics plus TV3-specific topics such as:

```text
tv3_status
tv3_thrust
tv3_motor_reference
tv3_engine_command
tv3_engine_state
tv3_guidance_status
vehicle_local_position_groundtruth
vehicle_torque_setpoint
vehicle_thrust_setpoint
actuator_servos
actuator_motors
```

If you need to sync the profile manually before a run:

```bash
./scripts/sync_sitl_logger_topics.sh
```

## Log Archive

SITL first writes PX4 ULogs to the local PX4 rootfs:

```text
../.work/px4-tv3/build/px4_sitl_default/rootfs/log/YYYY-MM-DD/HH_MM_SS.ulg
```

`scripts/run_sitl_sih.sh` archives new logs automatically on exit:

```text
logs/sim/YYYY-MM-DD/<run-id>/
```

Each archive directory includes copied `.ulg` files plus `manifest.txt`; SITL archives also include `logger_topics.txt`, `flight_profile.json`, and `vehicle.json` when available. The binary log payloads are ignored by git, but they stay in the project checkout for local analysis.

Use a stable run ID when you want a predictable folder name:

```bash
TV3_LOG_RUN_ID=lander-smoke-001 ./scripts/run_sitl_sih.sh
```

Archive flight-hardware or ground-test logs copied from QGroundControl, an SD card, or another source with:

```bash
./scripts/archive_px4_logs.sh --kind flight --source /path/to/log.ulg --run-id flight-001
./scripts/archive_px4_logs.sh --kind ground --source /path/to/log.ulg --run-id load-cell-bench-001
```

## Live SITL (Hawkeye)

Start Hawkeye before launching SITL so the UDP stream is already listening:

```bash
./scripts/run_hawkeye.sh
```

Then run SITL in a second terminal (see [simulation.md](simulation.md) for the full workflow). Hawkeye receives pose updates on UDP `19410`; it is not the physics source of truth.

Install Hawkeye on macOS with:

```bash
brew tap px4/px4 && brew install px4/px4/hawkeye
```

## Interactive 3D Review (PyVista)

These open a PyVista window. Orbit with the mouse; there is no timeline scrubber. Use `--time` to choose which log instant to display (default: last frame).

Vehicle frame with per-engine roll/yaw sliders:

```bash
./scripts/view_vehicle_frame.sh
```

Four-panel overview (interactive):

```bash
./scripts/view_vehicle_frame.sh --overview
```

Flight path plus vehicle attitude at one instant:

```bash
./scripts/plot_ulog_replay.sh --latest
./scripts/plot_ulog_replay.sh --latest --time 12.5 --camera track
```

Engine mounts and thrust vectors at one instant:

```bash
./scripts/plot_ulog_engines.sh --latest
./scripts/plot_ulog_engines.sh --latest --time 8.0
```

Export PNG snapshots (headless, no window):

```bash
./scripts/view_vehicle_frame.sh --save build/vehicle_frame/tv3_lander_v1.png
./scripts/plot_ulog_replay.sh --latest -o /tmp/trajectory.png --time 12.5
./scripts/plot_ulog_engines.sh --latest -o /tmp/engines.png --time 12.5
```

Camera presets for PyVista: `iso`, `top`, `side`, `front`, `forward_up`, `overview`, `track`.

## Timed Log Playback (Rerun)

Use Rerun when you want to scrub through time. Guidance metrics are Rerun-only (no PyVista 3D scene).

```bash
./scripts/plot_ulog_replay.sh --latest --rerun
./scripts/plot_ulog_replay.sh --latest --scene all -o /tmp/tv3_unified.rrd
./scripts/plot_ulog_replay.sh --latest --scene guidance
./scripts/plot_ulog_engines.sh --latest --rerun
```

`--scene all` writes one Rerun recording with trajectory, per-engine thrust, and guidance metrics on a shared `sim_time` timeline (seconds from log start).

Save a recording for offline review (headless, no viewer window):

```bash
./scripts/plot_ulog_replay.sh --latest --rerun -o /tmp/tv3_trajectory.rrd
./scripts/plot_ulog_replay.sh --latest --scene guidance -o /tmp/tv3_guidance.rrd
./scripts/plot_ulog_engines.sh --latest --rerun -o /tmp/tv3_engines.rrd
```

Re-open a saved recording:

```bash
rerun /tmp/tv3_trajectory.rrd
```

Or invoke the venv binary directly:

```bash
../.work/tv3-viz-venv/bin/rerun /tmp/tv3_trajectory.rrd
```

Pass an explicit log path instead of `--latest`:

```bash
./scripts/plot_ulog_replay.sh logs/sim/YYYY-MM-DD/<run-id>/HH_MM_SS.ulg
```

## 2D Timeseries Review (Matplotlib)

Run the sim normally, then plot the newest archived ULog:

```bash
./scripts/run_sitl_sih.sh
./scripts/plot_ulog.sh --latest
```

The plot is saved beside the `.ulg` as:

```text
<log-name>.tv3_review.png
```

You can also pass an explicit log path:

```bash
./scripts/plot_ulog.sh logs/sim/YYYY-MM-DD/<run-id>/HH_MM_SS.ulg
```

To see what a log actually contains:

```bash
./scripts/plot_ulog.sh --latest --list-topics
```

If a panel says a topic is missing, the log was probably recorded before the TV3 profile was synced, or that module did not publish during the run. Start a new run after the sync step and check again.

## Troubleshooting

| Symptom | Fix |
|---------|-----|
| `missing dependency: install pyvista` / `rerun-sdk` | Run `./scripts/setup_viz_env.sh`, then use `./scripts/...` wrappers |
| `Rerun viewer not found on PATH` | Use repo scripts (they prepend `../.work/tv3-viz-venv/bin` to `PATH`) |
| `gRPC has been unable to connect` with `--rerun` | Same as above — viewer binary was not found |
| PyVista window does not appear | You passed `-o *.png` (headless export) — omit `-o` for interactive |
| Guidance PNG rejected | Expected — guidance is Rerun-only; use `--rerun` or `-o file.rrd` |
| Rerun flight lasts ~50 ms | Select the **`sim_time`** timeline in the time panel — not `log_time` (wall-clock export time) |
| Rerun playback looks choppy | Default replay uses native ULog rate (~50 Hz). Use `--fps 10` only if you want a lighter `.rrd` |
| `ULog not found` with `--latest` | Run SITL first so logs land under `logs/sim/`, or pass an explicit `.ulg` path |

## SIH Ground-Truth Topics

For simulator-owned truth data, prefer ULog topics emitted by `tv3_sih`:

```text
vehicle_attitude_groundtruth
vehicle_angular_velocity_groundtruth
vehicle_local_position_groundtruth
vehicle_global_position_groundtruth
```

Use ULog for PX4 controller state, vehicle estimates, commands, and TV3 module outputs. Hawkeye is visualization only.