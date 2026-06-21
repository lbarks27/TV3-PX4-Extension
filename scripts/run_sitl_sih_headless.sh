#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)

export TV3_VEHICLE_CONFIG="${TV3_VEHICLE_CONFIG:-config/vehicles/tv3_lander_v1.json}"
export TV3_FLIGHT_PROFILE="${TV3_FLIGHT_PROFILE:-config/flight_profiles/lander_hover_window.json}"
export TV3_SIM_DURATION_S="${TV3_SIM_DURATION_S:-90}"
export TV3_LOG_RUN_ID="${TV3_LOG_RUN_ID:-$(date -u +%Y%m%dT%H%M%SZ)-phase1-headless}"
export TV3_RUN_PROFILE_COMMANDS="${TV3_RUN_PROFILE_COMMANDS:-1}"
export TV3_HAWKEYE_VIS="${TV3_HAWKEYE_VIS:-0}"
export PX4_SIM_SPEED_FACTOR="${PX4_SIM_SPEED_FACTOR:-1}"
export TV3_REUSE_PX4_WORKTREE="${TV3_REUSE_PX4_WORKTREE:-1}"
export TV3_PX4_DAEMON="${TV3_PX4_DAEMON:-1}"

if [ "${TV3_SKIP_BUILD:-0}" != "1" ]; then
	"${SCRIPT_DIR}/build_sih.sh" >/dev/null
fi

"${SCRIPT_DIR}/run_sitl_sih.sh" &
RUNNER_PID=$!

cleanup_sim() {
	pkill -INT -x px4 >/dev/null 2>&1 || true
	kill -INT "${RUNNER_PID}" >/dev/null 2>&1 || true
	wait "${RUNNER_PID}" >/dev/null 2>&1 || true
	sleep 1
	pkill -9 -x px4 >/dev/null 2>&1 || true
}
trap cleanup_sim EXIT INT TERM

sleep "${TV3_SIM_DURATION_S}"
cleanup_sim
trap - EXIT INT TERM