#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
REPO_ROOT=$(cd -- "${SCRIPT_DIR}/.." && pwd)
TV3_ROOT=$(cd -- "${REPO_ROOT}/.." && pwd)
WORKTREE=$("${SCRIPT_DIR}/prepare_px4_tree.sh")
SIM_TARGET="${1:-gz_tv3_rocket}"
QT5_PREFIX=/opt/homebrew/opt/qt@5
MODULES_LOCATION="${TV3_ROOT}/.work/tv3-px4-extension"
PX4_BUILD_DIR="${WORKTREE}/build/px4_sitl_default"

ln -sfn "${REPO_ROOT}" "${MODULES_LOCATION}"

if [ -d "${QT5_PREFIX}" ]; then
	CMAKE_PREFIX_PATH="${QT5_PREFIX}${CMAKE_PREFIX_PATH:+:${CMAKE_PREFIX_PATH}}"
fi

export PX4_SYS_AUTOSTART="${PX4_SYS_AUTOSTART:-11000}"
export PX4_GZ_WORLD="${PX4_GZ_WORLD:-default}"
export TV3_VEHICLE_CONFIG="${TV3_VEHICLE_CONFIG:-${REPO_ROOT}/config/vehicles/tv3_v1.yaml}"
export TV3_FLIGHT_PROFILE="${TV3_FLIGHT_PROFILE:-}"
if [ -f "${REPO_ROOT}/build/motors/catalog.csv" ]; then
	DEFAULT_MOTOR_ROOT="${REPO_ROOT}/build/motors"
else
	DEFAULT_MOTOR_ROOT="${REPO_ROOT}/build/barebones/runtime/fs/microsd/tv3/motors"
fi
export TV3_MOTOR_ROOT="${TV3_MOTOR_ROOT:-${DEFAULT_MOTOR_ROOT}}"

env CMAKE_PREFIX_PATH="${CMAKE_PREFIX_PATH:-}" EXTERNAL_MODULES_LOCATION="${MODULES_LOCATION}" \
	make -C "${WORKTREE}" px4_sitl_default DONT_RUN=1

"${SCRIPT_DIR}/sync_sitl_logger_topics.sh" "${WORKTREE}"

if [ "${TV3_RESET_SIM_PARAMS:-1}" != "0" ]; then
	rm -f \
		"${PX4_BUILD_DIR}/parameters.bson" \
		"${PX4_BUILD_DIR}/parameters_backup.bson" \
		"${PX4_BUILD_DIR}/rootfs/parameters.bson" \
		"${PX4_BUILD_DIR}/rootfs/parameters_backup.bson"
fi

VEHICLE_NAME=$(basename -- "${TV3_VEHICLE_CONFIG%.*}")
PROFILE_NAME=""
if [ -n "${TV3_FLIGHT_PROFILE}" ]; then
	PROFILE_NAME="-$(basename -- "${TV3_FLIGHT_PROFILE%.*}")"
fi
export TV3_LOG_RUN_ID="${TV3_LOG_RUN_ID:-$(date -u +%Y%m%dT%H%M%SZ)-${VEHICLE_NAME}${PROFILE_NAME}-${SIM_TARGET}}"
TV3_LOG_MARKER=$(mktemp "${TMPDIR:-/tmp}/tv3-sitl-log-start.XXXXXX")

archive_sitl_logs() {
	local status=$?
	set +e
	local log_source=""
	for candidate in "${PX4_BUILD_DIR}/log" "${PX4_BUILD_DIR}/rootfs/log"; do
		if [ -e "${candidate}" ]; then
			log_source="${candidate}"
			break
		fi
	done
	if [ -n "${log_source}" ]; then
		"${SCRIPT_DIR}/archive_px4_logs.sh" \
			--kind sim \
			--source "${log_source}" \
			--since-marker "${TV3_LOG_MARKER}" \
			--run-id "${TV3_LOG_RUN_ID}" \
			--vehicle-config "${TV3_VEHICLE_CONFIG}" \
			--worktree "${WORKTREE}"
	else
		echo "No PX4 SITL log directory found to archive under ${PX4_BUILD_DIR}"
	fi
	rm -f "${TV3_LOG_MARKER}"
	exit "${status}"
}
trap archive_sitl_logs EXIT

echo "TV3_VEHICLE_CONFIG: ${TV3_VEHICLE_CONFIG}"
echo "TV3_FLIGHT_PROFILE: ${TV3_FLIGHT_PROFILE:-<none>}"
echo "TV3_LOG_RUN_ID: ${TV3_LOG_RUN_ID}"
echo "TV3_RESET_SIM_PARAMS: ${TV3_RESET_SIM_PARAMS:-1}"

env CMAKE_PREFIX_PATH="${CMAKE_PREFIX_PATH:-}" EXTERNAL_MODULES_LOCATION="${MODULES_LOCATION}" \
	make -C "${WORKTREE}" px4_sitl_default "${SIM_TARGET}"
