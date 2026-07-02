# TV3 PX4 Extension

Out-of-tree PX4 modules and uORB messages for thrust-vector-controlled TV3 vehicles.

TV3 owns operational flight phases through `tv3_state_machine`. PX4 commander remains the arming/disarm shell; standard PX4 flight modes are hidden when `RK_ENABLE=1`.

Built for `EXTERNAL_MODULES_LOCATION` against PX4 v1.16.1.

## Architecture

```text
QGC / MAVLink cmd 31010  -->  tv3_state_machine  -->  tv3_sm_status
                                      |                tv3_sm_modes
                                      v
                            (future: guidance, attitude, mixer)
```

PX4 `nav_state` stays on a non-selectable Manual placeholder. TV3 modes (`READY`, `BOOST`, `COAST`, `ABORT`, etc.) live on `tv3_sm_status`.

## Layout

```text
TV3 PX4 Extension/
├── msg/                     # tv3_sm_* and load-cell input topics
├── src/
│   ├── CMakeLists.txt
│   ├── lib/
│   └── modules/
│       ├── tv3_params/
│       └── tv3_state_machine/
├── overlay/ROMFS/           # airframe defaults + module startup
├── patches/px4/             # commander mode-menu patch
└── scripts/
```

## Quick Start

```bash
./scripts/bootstrap_px4.sh      # clone PX4 into ../vendor/px4 (first time)
./scripts/prepare_px4_tree.sh   # patched worktree + ROMFS overlay
./scripts/build_sitl.sh         # build px4_sitl_default with this extension
```

Run SITL with the TV3 airframe (after build):

```text
make -C ../.work/px4-tv3 px4_sitl_default
./build/px4_sitl_default/bin/px4 ./build/px4_sitl_default/etc -s etc/init.d-posix-airframes/11002_tv3_lander
```

Operator commands (PX4 shell or QGC actions in `config/qgc/TV3Actions.json`):

```text
tv3_state_machine launch
tv3_state_machine abort
tv3_state_machine reset
tv3_state_machine status
listener tv3_sm_status
listener tv3_sm_modes
```

MAVLink vehicle command `31010` with `param1`: `1=launch`, `2=abort`, `3=reset`.

## Adding Downstream Modules

Guidance, attitude, and mixer modules should subscribe to `tv3_sm_modes` and remain mode consumers. Extend `tv3_common.post` to start them after the state machine.
