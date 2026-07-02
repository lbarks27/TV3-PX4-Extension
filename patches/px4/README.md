# PX4 Patch Stack

Patches applied by `scripts/prepare_px4_tree.sh` against the PX4 worktree:

- `0002-tv3-qgc-flight-modes.patch` — when `RK_ENABLE=1`, hides standard PX4 selectable flight modes from GCS so TV3 owns operational modes via `tv3_state_machine`.

Refresh patches when rebasing to a newer PX4 tag.
