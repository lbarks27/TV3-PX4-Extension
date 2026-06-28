# TV3 PX4 Extension

Out-of-tree PX4 modules, uORB messages, and tooling for thrust-vector-controlled TV3 vehicles.

Focus: three-engine splay-throttle lander (`tv3_lander_v1`) for launch + lateral translation + precision landing (target 0.5 m window on real hardware).

- Three-engine splay-throttle lander vehicle (`tv3_lander_v1`)
- (tv3_v1 single-engine ascent support is secondary / legacy)

Designed for `EXTERNAL_MODULES_LOCATION` builds against PX4 v1.16.1. Vehicle manifests in `config/vehicles/` drive generated SIH/SITL runtime payloads and motor reference data.
Flight profiles in `config/flight_profiles/` define scenario targets that can be loaded on top of those vehicle manifests for SITL runs.

Use `TV3_FLIGHT_PROFILE=config/flight_profiles/lander_precision_land.json` (acceptance 0.5 m) for tight landing work.

## Documentation

- **[Simulation Guide](docs/simulation.md)** — macOS prerequisites, first-time setup, recommended daily workflow, QGroundControl connection, ports, and troubleshooting.
- **[Hardware Flight Workflow](docs/hardware_flight_workflow.md)** — Cube Orange Plus upload, microSD runtime staging, QGC setup, parameter checks, and launch-day software flow.
- **[Completion Roadmap](docs/completion_roadmap.md)** — phase plan, exit criteria, and gate scripts.
- **[Completion Status](docs/completion_status.md)** — generated progress dashboard (refresh with `./scripts/check_completion_status.sh`).
- **[Implementation Phases](docs/implementation_phases.md)** — high-level product intent (see roadmap for the active plan).
- **[Control Mixer](docs/control_mixer.md)** — attitude mixer, PX4 allocator, triple-engine TVC geometry, splay throttle, and reachability checks.

## Quick Start

```bash
./scripts/check_barebones.sh      # run tests + generate bare-bones assets
./scripts/bootstrap_px4.sh        # clone PX4 into ../vendor/px4 (first time)
./scripts/prepare_px4_tree.sh     # one-time: create patched worktree + overlays
./scripts/build_sih.sh            # initial PX4 SIH SITL build

# Default validation gate:
./scripts/run_sitl_sih.sh
```

Switch vehicles with `TV3_VEHICLE_CONFIG=config/vehicles/tv3_lander_v1.json`.
Load a flight scenario with `TV3_FLIGHT_PROFILE=config/flight_profiles/lander_hover_window.json`.

Hardware bench (Cube Orange Plus + microSD):

```bash
./scripts/build_nuttx.sh cubepilot_cubeorangeplus_default   # build + flash TV3 firmware
./scripts/stage_microsd.sh                                 # SD card attached to this Mac
./scripts/complete_phase2_bench.sh --body-mass-kg <kg>      # QGC closed; captures into manifest
```

See [docs/simulation.md](docs/simulation.md) for prerequisites, the full SIH workflow, QGC/Hawkeye ports, profile command runner behavior, and troubleshooting.

## Status & Roadmap

See [docs/completion_roadmap.md](docs/completion_roadmap.md) for what is implemented, what remains, and the evidence gates before free flight.
