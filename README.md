# TV3 PX4 Rocket Extension

This repository is a standalone PX4 extension for a thrust-vector-controlled model rocket. It is designed to stay out-of-tree and be built into PX4 with `EXTERNAL_MODULES_LOCATION`. The PX4 checkout, build worktrees, and thrust-curves source repo live next to this repo in the parent `TV3` folder; the extension source and vehicle config live here.

## Scope

- PX4 `v1.16.1` integration baseline
- out-of-tree modules and uORB messages
- powered-ascent flight-mode state machine, preliminary waypoint guidance, and TVC control mixer
- ADS1115-backed load-cell integration through `adc_report`
- offline motor-catalog validation and normalization
- Gazebo Sim and JSBSim vehicle-asset generation from one shared vehicle definition

## Architecture Map

The extension is organized by responsibility, not by PX4 boilerplate:

- `src/modules/control_mixer/`: thrust-vector control mixer and attitude loop
- `src/modules/vehicle_models/`: motor model and load-cell model
- `src/modules/guidance/`: launch, waypoint, and landing guidance
- `src/modules/flight_modes/`: launch / abort / reset state machine
- `msg/`: rocket-specific uORB messages
- `patches/`: PX4 patch set that enables rocket control allocation
- `overlay/`: POSIX ROMFS overlays for SITL airframes and startup hooks
- `runtime/`: generated NuttX SD-card assets
- `config/vehicles/`: shared rocket definitions that drive generated assets
- `tools/`: motor normalization, bootstrap, and asset-generation tooling
- `tests/`: host-side tests for the data pipeline and generators

## Module Map

```text
src/
  modules/
    control_mixer/
      rocket_att_control.cpp
    vehicle_models/
      rocket_motor_model.cpp
      rocket_load_cell.cpp
    guidance/
      rocket_guidance.cpp
    flight_modes/
      rocket_mode_manager.cpp
msg/
  RocketCommand.msg
  RocketStatus.msg
  RocketThrust.msg
  RocketMotorReference.msg
  RocketModeStatus.msg
  RocketGuidanceStatus.msg
  RocketLoadCell.msg
overlay/
  ROMFS/
patches/
  px4/
runtime/
  nuttx/
config/
  vehicles/
tools/
tests/
```

The top-level folders are deliberately split by role:

- `src/` and `msg/` are the extension source tree consumed by PX4.
- `overlay/` and `patches/` are build inputs, not generated output.
- `runtime/` holds the generated SD-card payload that gets staged to the vehicle.
- `config/vehicles/` stays in git as the source-of-truth vehicle definition.
- `tools/` and `tests/` cover catalog normalization, asset generation, and validation.

## Quick Start

1. Bootstrap PX4:

   ```bash
   ./scripts/bootstrap_px4.sh
   ```

2. Prepare a patched PX4 worktree:

   ```bash
   ./scripts/prepare_px4_tree.sh
   ```

   This applies the repository's `patches/px4/0001-rocket-control-allocation.patch`, which adds the rocket-specific control-allocation path and reserves `MAV_CMD_USER_1 (31010)` for launch/abort/reset handling.

3. Build Gazebo Sim SITL:

   ```bash
   ./scripts/run_sitl_gazebo.sh
   ```

4. Build JSBSim SITL:

   ```bash
   ./scripts/run_sitl_jsbsim.sh
   ```

5. Clone or refresh the thrust-curves source repo:

   ```bash
   ./scripts/bootstrap_thrust_curves.sh
   ```

6. Generate a normalized motor catalog from the thrust-curves repo:

   ```bash
   ./tools/generate_motor_catalog.py \
     --output ./build/motors
   ```

   By default this reads from `../vendor/Thrust-Curves-Apogee`. Pass `--source` only if you want a different catalog checkout.

7. Generate runtime and simulation assets from the shared vehicle definition:

   ```bash
   ./tools/generate_vehicle_assets.py \
     --vehicle ./config/vehicles/tv3_v1.yaml \
     --output ./build/generated
   ```

## Runtime Layout

The SD-card payload is generated, not hand-edited:

```text
/fs/microsd/etc/config.txt
/fs/microsd/etc/extras.txt
/fs/microsd/tv3/airframes/tv3_v1.params
/fs/microsd/tv3/motors/catalog.csv
/fs/microsd/tv3/motors/<motor-id>/curve.csv
/fs/microsd/tv3/motors/<motor-id>/specs.csv
```

`config/vehicles/tv3_v1.yaml` is the source config that generates the `tv3_v1.params` file, so it stays in the repo. The generated `runtime/nuttx/fs/microsd/` tree is ignored by git.
