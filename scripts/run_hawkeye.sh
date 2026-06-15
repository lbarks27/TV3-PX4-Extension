#!/usr/bin/env bash

set -euo pipefail

PORT="${HAWKEYE_UDP_PORT:-19410}"
MESH="${TV3_HAWKEYE_MESH:-/Users/liambarkley/Downloads/TV3+Hub+Test+1.obj}"

if [ -f "${MESH}" ]; then
	"$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/install_hawkeye_model.sh"
fi

if [ -n "${HAWKEYE_CMD:-}" ]; then
	exec ${HAWKEYE_CMD} -udp "${PORT}"
fi

if command -v hawkeye >/dev/null 2>&1; then
	exec hawkeye -fw -udp "${PORT}"
fi

if [ -d "/Applications/Hawkeye.app" ]; then
	exec open -a Hawkeye --args -fw -udp "${PORT}"
fi

cat >&2 <<EOF
Hawkeye was not found.

Set HAWKEYE_CMD to the Hawkeye executable command, or install Hawkeye.app in /Applications.
The TV3 SIH MAVLink viewer stream is UDP ${PORT}; PX4 sends HIL_STATE_QUATERNION there when PX4_SIMULATOR=sihsim.
Using -fw (fixed-wing slot, no mirror) with a TV3 mesh override installed by install_hawkeye_model.sh.
EOF
exit 1
