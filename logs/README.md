# TV3 Run Logs

This directory is the local archive for PX4 ULog evidence from TV3 simulation, flight-hardware, and ground-test runs on this machine.

Logs are stored by kind, date, and run ID:

```text
logs/
  sim/YYYY-MM-DD/<run-id>/
  flight/YYYY-MM-DD/<run-id>/
  ground/YYYY-MM-DD/<run-id>/
```

Each run directory should contain the copied `.ulg` files plus `manifest.txt`. SITL archives also include the synced `logger_topics.txt`, `vehicle.json`, and `flight_profile.json` when those files are available.

The log payloads are ignored by git because `.ulg` files can be large. Keep durable analysis notes, plots, or selected evidence in docs if they need to be committed.

## Simulation

`scripts/run_sitl_sih.sh` and `scripts/run_sitl_sih_headless.sh` archive new SITL `.ulg` files automatically when the run exits.

Use `TV3_LOG_RUN_ID` to choose a stable run directory:

```bash
TV3_LOG_RUN_ID=lander-smoke-001 ./scripts/run_sitl_sih.sh
```

## Flight And Ground Tests

Archive PX4 `.ulg` files copied from QGroundControl, an SD card, or another hardware source with:

```bash
./scripts/archive_px4_logs.sh --kind flight --source /path/to/log.ulg --run-id flight-001
./scripts/archive_px4_logs.sh --kind ground --source /path/to/qgc-log-folder --run-id load-cell-bench-001
```

`archive_px4_logs.sh` copies `.ulg` files only. Bench capture JSON under `logs/ground/bench_capture_*.json` is written by `complete_phase2_bench.sh` and should stay in place (reference it from `config/completion_status.json` evidence).

Use `--vehicle-config config/vehicles/<vehicle>.json` and `--notes "short note"` when useful.