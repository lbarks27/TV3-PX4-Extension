# PX4 Patch Stack

This directory carries the smallest upstream PX4 delta needed by the extension:

- `0001-rocket-control-allocation.patch`

The patch was imported from the earlier TV3 wrapper and is kept separate from the extension modules so the rocket-specific behavior remains out-of-tree. The current extension publishes both the new public topics (`rocket_status`, `rocket_thrust`) and compatibility aliases (`rocket_mode_status`, `rocket_load_cell`) so the allocator patch can continue to consume the older topic names until the patch is refreshed.
