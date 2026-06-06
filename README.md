# TV3 PX4 Rocket Extension

Out-of-tree PX4 modules, uORB messages, and tooling for thrust-vector-controlled solid-motor rockets.

- Single-engine TVC ascent vehicle (`tv3_v1`)
- Three-engine splay-throttle lander vehicle (`tv3_lander_v1`)

Designed for `EXTERNAL_MODULES_LOCATION` builds against PX4 v1.16.1. Vehicle manifests in `config/vehicles/` drive generated SITL assets, runtime payloads, and motor reference data.
Flight profiles in `config/flight_profiles/` define scenario targets that can be loaded on top of those vehicle manifests for SITL runs.

## Documentation

- **[Simulation Guide](docs/simulation.md)** — macOS prerequisites, first-time setup, recommended daily workflow, QGroundControl connection, ports, and troubleshooting.
- **[Completion Roadmap](docs/completion_roadmap.md)** — current state, detailed phase plan, flight gates, and immediate next work for both vehicles.
- **[Implementation Phases](docs/implementation_phases.md)** — high-level product intent (see roadmap for the active plan).

## Quick Start

```bash
./scripts/check_barebones.sh      # run tests + generate bare-bones assets
./scripts/bootstrap_px4.sh        # clone PX4 into ../vendor/px4 (first time)
./scripts/prepare_px4_tree.sh     # one-time: create patched worktree + overlays
./scripts/build_sitl.sh           # initial SITL build

# After first setup, use the fast path for daily runs:
./scripts/run_sitl_gazebo_fast.sh
```

Switch vehicles with `TV3_VEHICLE_CONFIG=config/vehicles/tv3_lander_v1.yaml`.
Load a flight scenario with `TV3_FLIGHT_PROFILE=config/flight_profiles/lander_hover_window.yaml`.

See [docs/simulation.md](docs/simulation.md) for the full reproducible workflow, how to connect QGC, and why the fast launcher is preferred after initial setup.

## Status & Roadmap

See [docs/completion_roadmap.md](docs/completion_roadmap.md) for what is implemented, what remains, and the evidence gates before free flight.
