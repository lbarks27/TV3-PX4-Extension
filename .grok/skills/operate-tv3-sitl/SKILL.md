---
name: operate-tv3-sitl
description: Operate the TV3 PX4 Gazebo SITL workflow from the local TV3 PX4 Extension checkout. Prepare or refresh the PX4 worktree, launch the fast daily Gazebo simulation (prefer ./scripts/run_sitl_gazebo_fast.sh after initial setup), select vehicle manifests (tv3_v1, tv3_lander_v1, or others), inspect running PX4/Gazebo processes, verify QGroundControl UDP ports (18570/14580), stop stale simulations, and report complete run context. Always surface the active TV3_VEHICLE_CONFIG, worktree path, external modules symlink, motor root, and listening ports. Use when the user mentions TV3 SITL, PX4 Gazebo, run_sitl_gazebo_fast, tv3_v1, tv3_lander_v1, QGroundControl, launch simulation, prepare PX4 worktree, stop simulation, SITL status, or runs /operate-tv3-sitl. Never treat basic infrastructure or launch checks as proof of hover, landing, or flight readiness.
---

# Operate TV3 SITL

## Core Rule

Prefer the fast launcher after initial setup:

```bash
./scripts/run_sitl_gazebo_fast.sh
```

Use full prepare/build only when the worktree is missing or stale, PX4 patches changed, generated startup/model assets need a clean refresh, the selected manifest must be re-applied into ROMFS/model outputs, or the user asks for full setup.

Do not report a Gazebo launch as proof of hover, landing, or flight readiness. Treat it as infrastructure evidence unless scenario logs and gate-specific checks prove more.

## Repo Orientation

This checkout (the one containing this skill):

```text
/Users/liambarkley/Developer/TV3/TV3 PX4 Extension
```

Expected sibling layout:

```text
TV3/
├── TV3 PX4 Extension/
├── vendor/px4/
└── .work/
    ├── px4-tv3/
    └── tv3-px4-extension -> current checkout
```

Start by confirming the active repo and state:

```bash
pwd
git status --short --branch
./scripts/check_barebones.sh
```

For a quick non-mutating environment check, use the bundled helper from this skill:

```bash
.grok/skills/operate-tv3-sitl/scripts/tv3-sitl-status.sh
```

The helper auto-detects when you are already inside the repo root. You can also pass an explicit path:

```bash
.grok/skills/operate-tv3-sitl/scripts/tv3-sitl-status.sh "/path/to/TV3 PX4 Extension"
```

## Vehicle Selection

If the user names a manifest, use it after verifying it exists under `config/vehicles/` or as an explicit path.

Default:

```bash
TV3_VEHICLE_CONFIG=config/vehicles/tv3_v1.yaml
```

Use this for the default single-engine ascent path. Its Gazebo physics plugin is intentionally disabled in the current manifest, so an inert/falling model may be expected during bare-bones work.

Lander:

```bash
TV3_VEHICLE_CONFIG=config/vehicles/tv3_lander_v1.yaml
```

Use this when the user asks for lander behavior, force response, multi-engine behavior, the TV3 Gazebo plugin, hover-related bring-up, or guidance-enabled SITL validation. Report that this is plugin/guidance infrastructure unless a specific scenario proves hover or landing.

For other vehicles:

```bash
find config/vehicles -maxdepth 1 -name '*.yaml' -print
rg -n "name:|include_tv3_plugin:|guidance:" config/vehicles/<vehicle>.yaml
```

Then report what is known from the manifest: model name, plugin enabled/disabled, guidance enabled/disabled, and any uncertainty.

## Launch Decision Tree

1. Inspect current state:

```bash
.grok/skills/operate-tv3-sitl/scripts/tv3-sitl-status.sh
```

2. If a sim is already running, report the process and port state before launching another instance.

3. If `../.work/px4-tv3` is missing, the symlink points at a different checkout, or the user asks for clean setup, run the full path:

```bash
./scripts/check_barebones.sh
./scripts/bootstrap_px4.sh
./scripts/prepare_px4_tree.sh
./scripts/build_sitl.sh
```

`prepare_px4_tree.sh` deletes and recreates `../.work/px4-tv3`, updates PX4 submodules, applies patches, copies overlays, generates assets, and toggles `rocket_guidance start` according to the selected manifest. It can be slow and quiet during PX4/submodule work.

4. Otherwise use the fast path:

```bash
./scripts/run_sitl_gazebo_fast.sh
```

For a selected manifest:

```bash
TV3_VEHICLE_CONFIG=config/vehicles/tv3_lander_v1.yaml ./scripts/run_sitl_gazebo_fast.sh
```

Keep the long-running PX4/Gazebo command session active while the user wants the simulation running. Do not send a final answer while a launch command is still starting unless the user only asked for a status update.

## Runtime Values To Report

Every run summary should include:

```text
repo path
worktree path
external modules symlink target
TV3_VEHICLE_CONFIG
TV3_MOTOR_ROOT
PX4_SYS_AUTOSTART
PX4_GZ_WORLD
active PX4/Gazebo process IDs
active QGroundControl/offboard UDP ports
```

Expected defaults:

```bash
PX4_SYS_AUTOSTART=11000
PX4_GZ_WORLD=default
TV3_VEHICLE_CONFIG=config/vehicles/tv3_v1.yaml
```

The fast launcher chooses `TV3_MOTOR_ROOT` from `build/motors` when a full motor catalog exists, otherwise from `build/barebones/runtime/fs/microsd/tv3/motors`.

## QGroundControl

Expected local QGC link:

```text
UDP 18570
```

Optional onboard/offboard link:

```text
UDP 14580
```

Verify with:

```bash
lsof -nP -iUDP:18570 -iUDP:14580 2>/dev/null
```

The repo routes `MAV_CMD_USER_1` / command `31010` through `rocket_mode_manager` for launch, abort, and reset style commands. Do not guess command parameters; inspect the module or docs before issuing mission-changing commands.

## Stopping Stale Simulations

Before killing anything, inspect:

```bash
pgrep -af "gz sim|gazebo|px4_sitl_default|bin/px4|gz_bridge"
```

If the user asked to stop the sim or stale processes clearly block a requested launch, stop with:

```bash
pkill -f "gz sim"
pkill -f "/px4_sitl_default/bin/px4"
```

Then verify:

```bash
pgrep -af "gz sim|gazebo|px4_sitl_default|bin/px4|gz_bridge" || true
lsof -nP -iUDP:18570 -iUDP:14580 2>/dev/null || true
```

## Troubleshooting

If QGC cannot connect, check `UDP:18570`, then the SITL console for MAVLink startup.

If PX4/Gazebo appears to use the wrong checkout, check:

```bash
readlink ../.work/tv3-px4-extension
```

If the rocket falls or does not move, check the selected manifest. `tv3_v1` intentionally has `include_tv3_plugin: false`; use `tv3_lander_v1` when plugin force response is expected.

If the full launcher or build fails after PX4 patch changes, inspect rejects and build logs in `../.work/px4-tv3`; do not edit generated `build/` outputs as source.

If ULog evidence matters, verify actual logged topics. Do not assume custom `rocket_*` topics are recorded just because modules publish them.

## Reporting Template

Use concise run reports:

```text
TV3 SITL status:
- vehicle: <manifest>
- worktree: <path, exists/missing>
- external modules: <symlink target>
- motor root: <path>
- running: <PX4/Gazebo PIDs or none>
- ports: <18570/14580 listeners or none>
- command: <launcher command used>
- boundary: <what this run proves and does not prove>
```
