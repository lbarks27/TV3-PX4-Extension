#!/usr/bin/env bash
#
# Fast / repeated-run launcher for TV3 Gazebo SITL.
#
# After you have done the one-time heavy setup (check_barebones.sh +
# prepare_px4_tree.sh + build_sitl.sh), use this script instead of
# run_sitl_gazebo.sh. It avoids the expensive "rm -rf worktree + full
# submodule update + re-patch" cycle on every launch.
#
# Usage:
#   ./scripts/run_sitl_gazebo_fast.sh
#
# It will:
#   - Ensure the EXTERNAL_MODULES_LOCATION symlink is correct
#   - Set the standard TV3 environment variables
#   - Run the exact same make target that the normal launcher uses
#
# For full instructions (including QGroundControl connection and the
# first-time setup), see docs/simulation.md.

set -euo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
REPO_ROOT=$(cd -- "${SCRIPT_DIR}/.." && pwd)
TV3_ROOT=$(cd -- "${REPO_ROOT}/.." && pwd)
WORKTREE="${TV3_ROOT}/.work/px4-tv3"
MODULES_LOCATION="${TV3_ROOT}/.work/tv3-px4-extension"
QT5_PREFIX=/opt/homebrew/opt/qt@5
PX4_BUILD_DIR="${WORKTREE}/build/px4_sitl_default"

# Ensure the external modules symlink points at the current checkout
# (this is the key step that lets PX4 find our out-of-tree rocket modules)
ln -sfn "${REPO_ROOT}" "${MODULES_LOCATION}"

if [ -d "${QT5_PREFIX}" ]; then
	CMAKE_PREFIX_PATH="${QT5_PREFIX}${CMAKE_PREFIX_PATH:+:${CMAKE_PREFIX_PATH}}"
fi

export PX4_SYS_AUTOSTART="${PX4_SYS_AUTOSTART:-11000}"
export PX4_GZ_WORLD="${PX4_GZ_WORLD:-default}"
export TV3_VEHICLE_CONFIG="${TV3_VEHICLE_CONFIG:-${REPO_ROOT}/config/vehicles/tv3_v1.yaml}"
export TV3_FLIGHT_PROFILE="${TV3_FLIGHT_PROFILE:-}"

python3 - "${REPO_ROOT}" "${WORKTREE}" "${TV3_VEHICLE_CONFIG}" "${TV3_FLIGHT_PROFILE}" <<'PY'
from pathlib import Path
import shutil
import subprocess
import sys

import yaml

repo_root = Path(sys.argv[1])
worktree = Path(sys.argv[2])
vehicle_config = Path(sys.argv[3])
flight_profile = Path(sys.argv[4]) if sys.argv[4] else None

if not vehicle_config.is_absolute():
    vehicle_config = repo_root / vehicle_config
if flight_profile is not None and not flight_profile.is_absolute():
    flight_profile = repo_root / flight_profile

generated_assets = repo_root / "build/barebones"
generator_args = [
    sys.executable,
    str(repo_root / "tools/generate_vehicle_assets.py"),
    "--vehicle",
    str(vehicle_config),
    "--output",
    str(generated_assets),
]
if flight_profile is not None:
    generator_args.extend(["--flight-profile", str(flight_profile)])
subprocess.run(generator_args, check=True)

vehicle = yaml.safe_load(vehicle_config.read_text())
gazebo_model_name = vehicle.get("gazebo", {}).get("model_name", vehicle["name"])
source_model = generated_assets / "gazebo" / vehicle["name"]
target_model = worktree / "Tools/simulation/gz/models" / gazebo_model_name
if target_model.exists():
    shutil.rmtree(target_model)
shutil.copytree(source_model, target_model)

launcher_model_name = "tv3_rocket"
launcher_model = worktree / "Tools/simulation/gz/models" / launcher_model_name
if gazebo_model_name != launcher_model_name:
    if launcher_model.exists():
        shutil.rmtree(launcher_model)
    shutil.copytree(source_model, launcher_model)
    model_config = launcher_model / "model.config"
    model_sdf = launcher_model / "model.sdf"
    model_config.write_text(model_config.read_text().replace(f"<name>{gazebo_model_name}</name>", f"<name>{launcher_model_name}</name>"))
    model_sdf.write_text(model_sdf.read_text().replace(f'<model name="{gazebo_model_name}">', f'<model name="{launcher_model_name}">'))

print(f"synced Gazebo model {gazebo_model_name} from {source_model} into {target_model}")
PY

# Choose the motor catalog location (prefer a full generated one if present)
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

SIM_TARGET="gz_tv3_rocket"
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

echo "=== TV3 Gazebo SITL (fast path) ==="
echo "WORKTREE:            ${WORKTREE}"
echo "EXTERNAL_MODULES:    ${MODULES_LOCATION}"
echo "TV3_MOTOR_ROOT:      ${TV3_MOTOR_ROOT}"
echo "PX4_SYS_AUTOSTART:   ${PX4_SYS_AUTOSTART}"
echo "TV3_VEHICLE_CONFIG:  ${TV3_VEHICLE_CONFIG}"
echo "TV3_FLIGHT_PROFILE:  ${TV3_FLIGHT_PROFILE:-<none>}"
echo "TV3_LOG_RUN_ID:      ${TV3_LOG_RUN_ID}"
echo "TV3_RESET_SIM_PARAMS:${TV3_RESET_SIM_PARAMS:-1}"
echo

env CMAKE_PREFIX_PATH="${CMAKE_PREFIX_PATH:-}" EXTERNAL_MODULES_LOCATION="${MODULES_LOCATION}" \
	make -C "${WORKTREE}" px4_sitl_default "${SIM_TARGET}"
