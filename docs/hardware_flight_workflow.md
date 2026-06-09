# TV3 Hardware Flight Workflow

This guide describes the repo-specific process for getting the TV3 PX4
extension onto a Cube Orange Plus, connecting from QGroundControl (QGC),
checking the generated parameters, and running the software side of a launch-day
flow.

This is not a range safety plan. Use it only inside an approved test plan with
the required launch authority, site rules, arming controls, pyrotechnic handling
controls, and recovery procedures.

## Current Hardware Target

The default hardware vehicle is `config/vehicles/tv3_v1.yaml`:

- Autopilot: Cube Orange Plus
- Carrier: mini carrier
- Telemetry: RFD900
- GPS: Here4 RTK rover plus Here4 RTK base
- Vehicle: single-engine TVC ascent vehicle
- Guidance: disabled by default

The PX4 target for this board is:

```bash
cubepilot_cubeorangeplus_default
```

Verify target availability in the prepared PX4 worktree when changing boards or
PX4 versions:

```bash
make -C ../.work/px4-tv3 list_config_targets | rg 'cubeorangeplus|cubeorange'
```

## Build And Flash Firmware

Run the hardware build from the repo root:

```bash
./scripts/build_nuttx.sh cubepilot_cubeorangeplus_default
```

That script:

- prepares `../.work/px4-tv3`
- builds PX4 with `EXTERNAL_MODULES_LOCATION` pointed at this repo
- generates TV3 runtime assets under `build/nuttx/cubepilot_cubeorangeplus_default`
- stages the microSD payload under `../.work/cubepilot_cubeorangeplus_default_runtime`

Flash the Cube Orange Plus from the prepared PX4 worktree:

```bash
make -C ../.work/px4-tv3 cubepilot_cubeorangeplus_default upload \
	EXTERNAL_MODULES_LOCATION="$(pwd)"
```

PX4's make flow accepts `make <target> upload` as the build-and-upload path. Use
`force-upload` only when you intentionally need to bypass uploader checks.

## Stage The MicroSD Runtime Payload

Firmware upload is only half of this repo's hardware deployment. The flight
controller also needs the generated runtime payload on its microSD card.

After the build, copy the staged payload to the Cube's microSD card:

```text
../.work/cubepilot_cubeorangeplus_default_runtime/etc/*        -> /fs/microsd/etc/
../.work/cubepilot_cubeorangeplus_default_runtime/fs/microsd/* -> /fs/microsd/
```

The staged payload should include:

- `etc/config.txt`
- `etc/extras.txt`
- `etc/logging/logger_topics.txt`
- `tv3/airframes/tv3_v1.params`
- `tv3/motors/`

On boot, PX4 reads `/fs/microsd/etc/config.txt` and
`/fs/microsd/etc/extras.txt`. The TV3 `extras.txt` imports the generated
airframe parameters and starts:

```sh
control_allocator start
internal_combustion_engine_control start
rocket_motor_model start
rocket_load_cell start
rocket_mode_manager start
rocket_att_control start
```

Guidance is not started for `tv3_v1` by default. Keep it disabled for the first
single-engine ascent gate unless it has its own evidence review.

## Connect From QGroundControl

Start with USB for first boot checks. Move to the RFD900 link only after the
board boots cleanly and the telemetry serial port is configured for the actual
carrier wiring.

This repo declares RFD900 as the `tv3_v1` telemetry hardware, but it does not
currently ship a hardware MAVLink serial parameter profile. Configure the
selected TELEM port, baud rate, and MAVLink instance in QGC using the standard
PX4 parameter workflow for the port you wired.

Install the repo's QGC MAVLink actions:

```bash
./scripts/install_qgc_actions.sh
```

Restart QGC after installing. The action file is copied to:

```text
~/Documents/QGroundControl/MavlinkActions/TV3RocketActions.json
```

The Fly View action list should then include:

- `TV3 Launch`
- `TV3 Abort`
- `TV3 Reset`

Do not remap QGC's generic `Takeoff` button to rocket ignition. The TV3
hardware and SIH paths intentionally use the same explicit `TV3 Launch` action
so the operator sees a rocket-specific command with `Abort` and `Reset` nearby.
The launch action is valid for sim and hardware only after the normal arming,
state-machine, range, and pad-safety checks pass.

These actions send `MAV_CMD_USER_1` command `31010`:

| Action | `param1` | PX4 shell equivalent |
| --- | ---: | --- |
| Launch | `1` | `rocket_mode_manager launch` |
| Abort | `2` | `rocket_mode_manager abort` |
| Reset | `3` | `rocket_mode_manager reset` |

Use QGC's MAVLink Console for software visibility:

```sh
rocket_mode_manager status
listener rocket_status
listener rocket_mode_status
listener vehicle_command_ack
```

QGC does not provide first-class widgets for the custom TV3 uORB topics, so
expect to use status text, the parameter editor, MAVLink Console, and post-run
ULog review.

## Configure And Verify Parameters

Treat `config/vehicles/tv3_v1.yaml` as the source of truth for generated flight
parameters. Do not hand-edit a different one-off parameter set in QGC and then
let it drift from the manifest.

The generator writes the hardware airframe params to:

```text
tv3/airframes/tv3_v1.params
```

The main generated parameter groups are:

- `RK_*`: rocket state machine, ignition, load cell, mass, TVC, and guidance
  values
- `CA_RK_*`: rocket control-allocation geometry and thrust values

Before field use, verify at least:

- `RK_ENABLE=1`
- `RK_CMD_SRC` matches the intended command source
- `RK_MOT_IDX` and `RK_ENG*_MOT` match the loaded motor data
- `RK_IGNITION_MS`, `RK_IGN_TO_MS`, and ignition sequence values are correct
- `RK_LAUNCH_THR_N`, `RK_BURNOUT_N`, and burn dwell/min/max values are correct
- `RK_LC_*` load-cell source, channel, tare, scale, filter, and timeout are
  correct
- `RK_TVC_MAX_DEG`, `RK_TVC_SLEW_DPS`, and torque limits match measured actuator
  behavior
- `CA_AIRFRAME=16`
- `CA_RK_*` engine positions, axes, trims, thrust values, and limits match the
  measured vehicle
- `RK_GD_ENABLE=0` for the first `tv3_v1` ascent gate unless guidance has been
  separately reviewed

After parameter edits in QGC, save the parameters and keep the saved file with
the test record. If a field value becomes part of the baseline vehicle, update
the YAML manifest and regenerate the runtime payload rather than relying on the
QGC-only copy.

## Bench And Ground-Test Gates

Run the repo smoke gate before hardware work:

```bash
./scripts/check_barebones.sh
```

Before launch-day use, confirm:

- the Cube boots the flashed firmware
- the microSD payload is present and readable
- TV3 modules start without shell errors
- QGC can connect over USB and over the intended telemetry link
- QGC shows the generated `RK_*` and `CA_RK_*` parameters
- `rocket_mode_manager status` reports the expected state
- load-cell tare and scale are measured with known loads
- no-thrust false positives are rejected
- igniter output continuity and timing are verified without live motors
- TVC endpoints, trims, direction, and slew rate are measured
- abort and reset commands are verified from QGC
- ULog captures the TV3 topics needed for review

Archive ground-test logs into the repo:

```bash
./scripts/archive_px4_logs.sh --kind ground --source /path/to/log-folder --run-id bench-001
```

## Launch-Day Software Flow

Use this only as the software checklist inside the approved range procedure.

Before arming:

- confirm the vehicle, motor, and parameter file match the test card
- confirm the microSD payload and log space
- power the ground station, telemetry radio, and flight controller
- verify QGC link quality, battery status, sensor status, and GPS/RTK status if
  used
- check QGC messages for TV3 startup errors
- run `rocket_mode_manager status`
- confirm the vehicle is not reporting a TV3 fault state
- confirm `TV3 Abort` and `TV3 Reset` are available in QGC
- confirm the range-safe arming and pyrotechnic controls are in the expected
  state

At the pad:

- arm only when the range procedure permits it
- confirm the TV3 state machine reaches the expected armed/ready state
- keep QGC visible for status text and command acknowledgements
- send `TV3 Launch` only after the range lead gives the launch command
- send `TV3 Abort` if the range procedure or TV3 fault response calls for abort

After flight or abort:

- safe the vehicle before handling
- disarm from QGC or the approved local control path
- copy the `.ulg` files from QGC, the microSD card, or another hardware source
- archive the flight logs:

```bash
./scripts/archive_px4_logs.sh --kind flight --source /path/to/flight.ulg --run-id flight-001
```

Review at least:

- state-machine transitions
- `vehicle_command_ack` for launch, abort, and reset commands
- load-cell confirmation versus expected motor thrust
- commanded and measured TVC response
- control allocator outputs and saturation
- burnout detection and coast transition
- estimator and sensor health

Do not change the manifest, parameters, or field procedure for another launch
until the archived logs explain the previous run well enough to support that
change.

## Verification Boundary

This workflow is derived from the current repo scripts and checked-in vehicle
configuration. It has not, by itself, proven that a specific Cube Orange Plus,
carrier, radio, igniter circuit, load cell, actuator stack, motor, or range setup
is flight-ready. Board-side flashing, SD-card boot, module startup, telemetry,
actuator, ignition, and logging behavior must be verified on the actual
hardware.
