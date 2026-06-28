#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
REPO_ROOT=$(cd -- "${SCRIPT_DIR}/.." && pwd)
TV3_ROOT=$(cd -- "${REPO_ROOT}/.." && pwd)

export TV3_VEHICLE_CONFIG="${TV3_VEHICLE_CONFIG:-config/vehicles/tv3_lander_v1.json}"
export TV3_FLIGHT_PROFILE="${TV3_FLIGHT_PROFILE:-config/flight_profiles/lander_hover_window.json}"
export PX4_SIMULATOR="${PX4_SIMULATOR:-sihsim}"
export PX4_SIM_MODEL="${PX4_SIM_MODEL:-tv3_lander}"
export PX4_SYS_AUTOSTART="${PX4_SYS_AUTOSTART:-11002}"
export HAWKEYE_UDP_PORT="${HAWKEYE_UDP_PORT:-19410}"
# Hawkeye 0.3.x maps unknown MAV_TYPE values (including TV3's MAV_TYPE_ROCKET=9)
# to the stock quad model. MAV_TYPE_FIXED_WING keeps the TV3 mesh in the -fw slot.
export TV3_HAWKEYE_VIS="${TV3_HAWKEYE_VIS:-1}"
export TV3_SIH_IDEAL="${TV3_SIH_IDEAL:-0}"
export TV3_SIMULATOR="${TV3_SIMULATOR:-sih}"

# Non-interactive runs (CI, redirected logs, headless wrapper) should not block on pxh>.
if [ "${TV3_PX4_INTERACTIVE:-0}" != "1" ] && [ ! -t 1 ]; then
	export TV3_PX4_DAEMON="${TV3_PX4_DAEMON:-1}"
fi

if [ "${TV3_PX4_KILL_STALE:-1}" = "1" ]; then
	if pgrep -x px4 >/dev/null 2>&1; then
		echo "stopping stale px4 instance before SIH launch"
		pkill -INT -x px4 >/dev/null 2>&1 || true
		sleep 1
		pkill -9 -x px4 >/dev/null 2>&1 || true
	fi
fi

WORKTREE=$("${SCRIPT_DIR}/prepare_px4_tree.sh")
MODULES_LOCATION="${TV3_ROOT}/.work/tv3-px4-extension"
QT5_PREFIX=/opt/homebrew/opt/qt@5

ln -sfn "${REPO_ROOT}" "${MODULES_LOCATION}"

if [ -d "${QT5_PREFIX}" ]; then
	CMAKE_PREFIX_PATH="${QT5_PREFIX}${CMAKE_PREFIX_PATH:+:${CMAKE_PREFIX_PATH}}"
fi

MARKER_DIR="${REPO_ROOT}/build/run-markers"
mkdir -p "${MARKER_DIR}"
MARKER=$(mktemp "${MARKER_DIR}/sih.XXXXXX")
touch "${MARKER}"

profile_name=$(python3 - "${REPO_ROOT}" "${TV3_FLIGHT_PROFILE}" <<'PY'
from pathlib import Path
import json
import sys

repo = Path(sys.argv[1])
profile = Path(sys.argv[2])
if not profile.is_absolute():
    profile = repo / profile
data = json.loads(profile.read_text())
print(data.get("name", profile.stem))
PY
)
vehicle_name=$(python3 - "${REPO_ROOT}" "${TV3_VEHICLE_CONFIG}" <<'PY'
from pathlib import Path
import json
import sys

repo = Path(sys.argv[1])
vehicle = Path(sys.argv[2])
if not vehicle.is_absolute():
    vehicle = repo / vehicle
data = json.loads(vehicle.read_text())
print(data.get("name", vehicle.stem))
PY
)

export TV3_LOG_RUN_ID="${TV3_LOG_RUN_ID:-$(date -u +%Y%m%dT%H%M%SZ)-${vehicle_name}-${profile_name}-sih}"

GENERATED_ROOT="${REPO_ROOT}/build/barebones"
GENERATOR_ARGS=(
	"${REPO_ROOT}/tools/generate_vehicle_assets.py"
	--vehicle "${TV3_VEHICLE_CONFIG}"
	--output "${GENERATED_ROOT}"
)
if [ -n "${TV3_FLIGHT_PROFILE}" ]; then
	GENERATOR_ARGS+=(--flight-profile "${TV3_FLIGHT_PROFILE}")
fi
"${GENERATOR_ARGS[@]}" >/dev/null

PROFILE_RUNNER_PID=""
PX4_PID=""
cleanup() {
	status=$?
	trap - EXIT INT TERM
	if [ -n "${PROFILE_RUNNER_PID}" ] && kill -0 "${PROFILE_RUNNER_PID}" >/dev/null 2>&1; then
		kill "${PROFILE_RUNNER_PID}" >/dev/null 2>&1 || true
		wait "${PROFILE_RUNNER_PID}" >/dev/null 2>&1 || true
	fi

	if [ -n "${PX4_PID:-}" ] && kill -0 "${PX4_PID}" >/dev/null 2>&1; then
		kill -INT "${PX4_PID}" >/dev/null 2>&1 || true
		wait "${PX4_PID}" >/dev/null 2>&1 || true
	fi

	PX4_BUILD_DIR="${WORKTREE}/build/px4_sitl_default"
	for LOG_SOURCE in "${PX4_BUILD_DIR}/rootfs/log" "${PX4_BUILD_DIR}/log"; do
		if [ -d "${LOG_SOURCE}" ]; then
			"${SCRIPT_DIR}/archive_px4_logs.sh" \
				--kind sim \
				--source "${LOG_SOURCE}" \
				--worktree "${WORKTREE}" \
				--vehicle-config "${TV3_VEHICLE_CONFIG}" \
				--run-id "${TV3_LOG_RUN_ID}" \
				--since-marker "${MARKER}" \
				--simulator "${TV3_SIMULATOR}" \
				--notes "PX4 SIH tv3_sih run; Hawkeye viewer UDP ${HAWKEYE_UDP_PORT}" || true
			break
		fi
	done

	rm -f "${MARKER}"
	exit "${status}"
}
trap cleanup EXIT INT TERM

echo "=== TV3 PX4 SIH ==="
echo "worktree:           ${WORKTREE}"
echo "vehicle:            ${TV3_VEHICLE_CONFIG}"
echo "profile:            ${TV3_FLIGHT_PROFILE}"
echo "PX4_SIMULATOR:      ${PX4_SIMULATOR}"
echo "PX4_SIM_MODEL:      ${PX4_SIM_MODEL}"
echo "PX4_SYS_AUTOSTART:  ${PX4_SYS_AUTOSTART}"
echo "Hawkeye UDP:        ${HAWKEYE_UDP_PORT}"
echo "TV3_HAWKEYE_VIS:    ${TV3_HAWKEYE_VIS}"
echo "TV3_SIH_IDEAL:      ${TV3_SIH_IDEAL}"
echo "TV3_LOG_RUN_ID:     ${TV3_LOG_RUN_ID}"
echo "TV3_PX4_DAEMON:     ${TV3_PX4_DAEMON:-0}"
echo

PX4_BUILD_DIR="${WORKTREE}/build/px4_sitl_default"

if [ "${TV3_SKIP_BUILD:-0}" != "1" ] || [ ! -x "${PX4_BUILD_DIR}/bin/px4" ]; then
	env CMAKE_PREFIX_PATH="${CMAKE_PREFIX_PATH:-}" EXTERNAL_MODULES_LOCATION="${MODULES_LOCATION}" \
		make -C "${WORKTREE}" px4_sitl_default DONT_RUN=1
fi
mkdir -p "${PX4_BUILD_DIR}/etc/logging"
cp -p "${REPO_ROOT}/build/barebones/runtime/etc/logging/logger_topics.txt" \
	"${PX4_BUILD_DIR}/etc/logging/logger_topics.txt"

SITL_MICROSD="${PX4_BUILD_DIR}/fs/microsd"
mkdir -p "${SITL_MICROSD}"
rsync -a "${REPO_ROOT}/build/barebones/runtime/fs/microsd/" "${SITL_MICROSD}/"
mkdir -p "${PX4_BUILD_DIR}/rootfs/fs"
ln -sfn "${SITL_MICROSD}" "${PX4_BUILD_DIR}/rootfs/fs/microsd"
PARAM_BASENAME="${vehicle_name}.params"
if [ -f "${SITL_MICROSD}/tv3/airframes/${PARAM_BASENAME}" ]; then
	cp -p "${SITL_MICROSD}/tv3/airframes/${PARAM_BASENAME}" "${SITL_MICROSD}/tv3/airframes/active.params"
	awk 'BEGIN { print "# generated by run_sitl_sih.sh" } NF >= 5 { print "param set " $3 " " $4 }' \
		"${SITL_MICROSD}/tv3/airframes/${PARAM_BASENAME}" > "${SITL_MICROSD}/tv3/airframes/active.params.sh"
	if [ "${TV3_HAWKEYE_VIS}" != "0" ]; then
		echo "param set MAV_TYPE 1" >> "${SITL_MICROSD}/tv3/airframes/active.params.sh"
	fi
fi
export TV3_MOTOR_ROOT="${TV3_MOTOR_ROOT:-${SITL_MICROSD}/tv3/motors}"

if [ "${TV3_RUN_PROFILE_COMMANDS:-1}" != "0" ]; then
	"${SCRIPT_DIR}/run_profile_commands.py" --flight-profile "${TV3_FLIGHT_PROFILE}" &
	PROFILE_RUNNER_PID=$!
fi

cd "${PX4_BUILD_DIR}"
PX4_ARGS=(-s etc/init.d-posix/rcS 0)
if [ "${TV3_PX4_DAEMON:-0}" = "1" ]; then
	PX4_ARGS=(-d "${PX4_ARGS[@]}")
fi

env EXTERNAL_MODULES_LOCATION="${MODULES_LOCATION}" \
	TV3_MOTOR_ROOT="${TV3_MOTOR_ROOT}" \
	TV3_SIH_IDEAL="${TV3_SIH_IDEAL}" \
	PX4_SIM_MODEL="${PX4_SIM_MODEL}" \
	PX4_SIMULATOR="${PX4_SIMULATOR}" \
	./bin/px4 "${PX4_ARGS[@]}" &
PX4_PID=$!

# px4 -d daemonizes: the shell child returns immediately while the sim keeps running.
if [ "${TV3_PX4_DAEMON:-0}" = "1" ]; then
	while kill -0 "${PX4_PID}" >/dev/null 2>&1 || pgrep -x px4 >/dev/null 2>&1; do
		sleep 1
	done
else
	wait "${PX4_PID}"
fi
