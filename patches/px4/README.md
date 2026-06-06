# PX4 Patch Stack

This directory carries the core upstream PX4 delta needed by the extension:

- `0001-rocket-control-allocation.patch` — supplies `ActuatorEffectivenessRocket`, the Rocket airframe type (16), related parameters, and control allocation wiring.

The patch is applied during `scripts/prepare_px4_tree.sh`, which then performs additional integration steps (module.yaml edits, allocator registration, Gazebo plugin wiring, commander command passthrough, build flag adjustments, etc.) so the full rocket feature set works against the selected PX4 tag while keeping the majority of the behavior out-of-tree in this repo.

The extension publishes the primary topics (`rocket_status`, `rocket_thrust`, `rocket_engine_*`, etc.) plus a few compatibility aliases for the allocator during the transition. The patch should be refreshed when rebasing to a newer PX4 baseline.
