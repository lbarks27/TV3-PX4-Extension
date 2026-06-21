#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
REPO_ROOT=$(cd -- "${SCRIPT_DIR}/.." && pwd)
PORT="${HAWKEYE_UDP_PORT:-19410}"
MESH="${TV3_HAWKEYE_MESH:-${REPO_ROOT}/assets/meshes/tv3_lander_v1.obj}"
export TV3_HAWKEYE_MESH_ROT="${TV3_HAWKEYE_MESH_ROT:-tv3_manifest}"
# Fixed-wing slot: mirror_axes=0 (full mesh). Multicopter slots mirror quarter meshes.
HAWKEYE_ARGS="${TV3_HAWKEYE_ARGS:--fw}"

if [ -f "${MESH}" ]; then
	"$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/install_hawkeye_model.sh"
fi

if [ -n "${HAWKEYE_CMD:-}" ]; then
	# shellcheck disable=SC2086
	exec ${HAWKEYE_CMD} ${HAWKEYE_ARGS} -udp "${PORT}"
fi

if command -v hawkeye >/dev/null 2>&1; then
	# shellcheck disable=SC2086
	exec hawkeye ${HAWKEYE_ARGS} -udp "${PORT}"
fi

if [ -d "/Applications/Hawkeye.app" ]; then
	# shellcheck disable=SC2086
	exec open -a Hawkeye --args ${HAWKEYE_ARGS} -udp "${PORT}"
fi

cat >&2 <<EOF
Hawkeye was not found.

Set HAWKEYE_CMD to the Hawkeye executable command, or install Hawkeye.app in /Applications.
The TV3 SIH MAVLink viewer stream is UDP ${PORT}; PX4 sends HIL_STATE_QUATERNION there when PX4_SIMULATOR=sihsim.
Using -fw (fixed-wing slot, no mirror) with a TV3 mesh override installed by install_hawkeye_model.sh.
EOF
exit 1
