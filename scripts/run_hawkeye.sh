#!/usr/bin/env bash

set -euo pipefail

PORT="${HAWKEYE_UDP_PORT:-19410}"

if [ -n "${HAWKEYE_CMD:-}" ]; then
	exec ${HAWKEYE_CMD} -udp "${PORT}"
fi

if command -v hawkeye >/dev/null 2>&1; then
	exec hawkeye -ts -udp "${PORT}"
fi

if [ -d "/Applications/Hawkeye.app" ]; then
	exec open -a Hawkeye --args -ts -udp "${PORT}"
fi

cat >&2 <<EOF
Hawkeye was not found.

Set HAWKEYE_CMD to the Hawkeye executable command, or install Hawkeye.app in /Applications.
The TV3 SIH MAVLink viewer stream is UDP ${PORT}; PX4 sends HIL_STATE_QUATERNION there when PX4_SIMULATOR=sihsim.
Using -ts (tailsitter) shape for upright rocket visualization with the SIH 90deg initial pitch.
EOF
exit 1
