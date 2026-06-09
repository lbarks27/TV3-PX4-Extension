# TV3 Data Visualization

This repo supports a PX4-first data path for detailed SITL, flight, and ground-test review:

1. Generate a TV3 ULog topic profile with the runtime assets.
2. Sync that profile into the PX4 SITL rootfs before launch.
3. Archive the resulting `.ulg` under `logs/`.
4. Plot the archived `.ulg` with Matplotlib.

## Install Plotting Dependencies

```bash
./scripts/setup_viz_env.sh
```

This creates or updates `../.work/tv3-viz-venv`, which avoids installing packages into Homebrew's externally managed Python.

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
rocket_status
rocket_thrust
rocket_motor_reference
rocket_engine_command
rocket_engine_state
rocket_guidance_status
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

Each archive directory includes copied `.ulg` files plus `manifest.txt`; SITL archives also include `logger_topics.txt`, `flight_profile.yaml`, and `vehicle.yaml` when available. The binary log payloads are ignored by git, but they stay in the project checkout for local analysis.

Use a stable run ID when you want a predictable folder name:

```bash
TV3_LOG_RUN_ID=lander-smoke-001 ./scripts/run_sitl_sih.sh
```

Archive flight-hardware or ground-test logs copied from QGroundControl, an SD card, or another source with:

```bash
./scripts/archive_px4_logs.sh --kind flight --source /path/to/log.ulg --run-id flight-001
./scripts/archive_px4_logs.sh --kind ground --source /path/to/log-folder --run-id load-cell-bench-001
```

## Plot a SITL Run

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

## SIH And Hawkeye

For simulator-owned truth data, prefer ULog topics emitted by `tv3_sih`:

```text
vehicle_attitude_groundtruth
vehicle_angular_velocity_groundtruth
vehicle_local_position_groundtruth
vehicle_global_position_groundtruth
```

Hawkeye is a viewer on UDP `19410`; it is not the physics source of truth. Use ULog for PX4 controller state, vehicle estimates, commands, and TV3 rocket module outputs.
