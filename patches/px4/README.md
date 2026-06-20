# PX4 Patch Stack

This directory carries the core upstream PX4 delta needed by the extension:

- `0001-tv3-control-allocation.patch` — supplies `ActuatorEffectivenessTV3`, the TV3 airframe type (16), related parameters, and control allocation wiring.
- `0002-tv3-qgc-flight-modes.patch` — when `RK_ENABLE=1`, limits PX4 `AVAILABLE_MODES` to a non-selectable Manual placeholder so QGC stops listing multicopter flight modes for `MAV_TYPE_ROCKET`.
- `0004-ads1115-differential-loadcell.patch` — retunes the ADS1115 driver for differential A0–A1 load-cell reads, faster sampling, and ±0.256 V PGA on the Cube Orange Plus bench path.

The patch is applied during `scripts/prepare_px4_tree.sh`, which then performs additional integration steps (module.yaml edits, allocator registration, SIH startup overlays, commander command passthrough, build flag adjustments, etc.) so the full tv3 feature set works against the selected PX4 tag while keeping the majority of the behavior out-of-tree in this repo.

The extension publishes the primary topics (`tv3_status`, `tv3_thrust`, `tv3_engine_*`, etc.) plus a few compatibility aliases for the allocator during the transition. The patch should be refreshed when rebasing to a newer PX4 baseline.
