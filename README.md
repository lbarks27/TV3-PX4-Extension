# TV3 PX4 Rocket Extension

Out-of-tree PX4 modules, uORB messages, and tooling for thrust-vector-controlled solid-motor rockets.

- Single-engine TVC ascent vehicle (`tv3_v1`)
- Three-engine splay-throttle lander vehicle (`tv3_lander_v1`)

Designed for `EXTERNAL_MODULES_LOCATION` builds against PX4 v1.16.1. Vehicle manifests in `config/vehicles/` drive generated SIH/SITL runtime payloads, JSBSim assets, and motor reference data.
Flight profiles in `config/flight_profiles/` define scenario targets that can be loaded on top of those vehicle manifests for SITL runs.

## Documentation

- **[Simulation Guide](docs/simulation.md)** — macOS prerequisites, first-time setup, recommended daily workflow, QGroundControl connection, ports, and troubleshooting.
- **[Hardware Flight Workflow](docs/hardware_flight_workflow.md)** — Cube Orange Plus upload, microSD runtime staging, QGC setup, parameter checks, and launch-day software flow.
- **[Completion Roadmap](docs/completion_roadmap.md)** — current state, detailed phase plan, flight gates, and immediate next work for both vehicles.
- **[Implementation Phases](docs/implementation_phases.md)** — high-level product intent (see roadmap for the active plan).

## Quick Start

```bash
./scripts/check_barebones.sh      # run tests + generate bare-bones assets
./scripts/bootstrap_px4.sh        # clone PX4 into ../vendor/px4 (first time)
./scripts/prepare_px4_tree.sh     # one-time: create patched worktree + overlays
./scripts/build_sih.sh            # initial PX4 SIH SITL build

# Default validation gate:
./scripts/run_sitl_sih.sh
```

Switch vehicles with `TV3_VEHICLE_CONFIG=config/vehicles/tv3_lander_v1.yaml`.
Load a flight scenario with `TV3_FLIGHT_PROFILE=config/flight_profiles/lander_hover_window.yaml`.

See [docs/simulation.md](docs/simulation.md) for the full reproducible workflow, QGC/Hawkeye ports, and scenario command runner behavior.

## Status & Roadmap

See [docs/completion_roadmap.md](docs/completion_roadmap.md) for what is implemented, what remains, and the evidence gates before free flight.
