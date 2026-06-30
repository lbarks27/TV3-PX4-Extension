#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
REPO_ROOT=$(cd -- "${SCRIPT_DIR}/.." && pwd)
TV3_ROOT=$(cd -- "${REPO_ROOT}/.." && pwd)

export TV3_VEHICLE_CONFIG="${TV3_VEHICLE_CONFIG:-config/vehicles/tv3_lander_v1.json}"
export TV3_FLIGHT_PROFILE="${TV3_FLIGHT_PROFILE:-config/flight_profiles/lander_boost_upright.json}"
export PX4_SIMULATOR="${PX4_SIMULATOR:-sihsim}"
export PX4_SIM_MODEL="${PX4_SIM_MODEL:-tv3_lander}"

WORKTREE=$("${SCRIPT_DIR}/prepare_px4_tree.sh")
MODULES_LOCATION="${TV3_ROOT}/.work/tv3-px4-extension"
QT5_PREFIX=/opt/homebrew/opt/qt@5

ln -sfn "${REPO_ROOT}" "${MODULES_LOCATION}"

if [ -d "${QT5_PREFIX}" ]; then
	CMAKE_PREFIX_PATH="${QT5_PREFIX}${CMAKE_PREFIX_PATH:+:${CMAKE_PREFIX_PATH}}"
fi

env CMAKE_PREFIX_PATH="${CMAKE_PREFIX_PATH:-}" EXTERNAL_MODULES_LOCATION="${MODULES_LOCATION}" \
	make -C "${WORKTREE}" px4_sitl_default DONT_RUN=1

if [ -d "${WORKTREE}/build/px4_sitl_default" ]; then
	env EXTERNAL_MODULES_LOCATION="${MODULES_LOCATION}" \
		cmake --build "${WORKTREE}/build/px4_sitl_default" --target modules__simulation__tv3_sih
fi

printf 'TV3 SIH SITL build ready in %s\n' "${WORKTREE}"
