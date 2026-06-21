#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
REPO_ROOT=$(cd -- "${SCRIPT_DIR}/.." && pwd)
TV3_ROOT=$(cd -- "${REPO_ROOT}/.." && pwd)

TARGET="sih"
NUTTX_TARGET=""

usage() {
	cat <<'EOF'
usage: build_px4.sh --target sih|sitl|nuttx [nuttx_board_target]

  sih    Build PX4 SITL with tv3_sih (default TV3 lander SIH profile env)
  sitl   Build generic PX4 SITL without tv3_sih
  nuttx  Build NuttX firmware target (requires board target argument)
EOF
}

while [ $# -gt 0 ]; do
	case "$1" in
	--target)
		TARGET=$2
		shift 2
		;;
	-h | --help)
		usage
		exit 0
		;;
	*)
		if [ "${TARGET}" = "nuttx" ] && [ -z "${NUTTX_TARGET}" ]; then
			NUTTX_TARGET=$1
			shift
		else
			echo "unknown argument: $1" >&2
			usage
			exit 1
		fi
		;;
	esac
done

case "${TARGET}" in
sih)
	export TV3_VEHICLE_CONFIG="${TV3_VEHICLE_CONFIG:-config/vehicles/tv3_lander_v1.json}"
	export TV3_FLIGHT_PROFILE="${TV3_FLIGHT_PROFILE:-config/flight_profiles/lander_hover_window.json}"
	export PX4_SIMULATOR="${PX4_SIMULATOR:-sihsim}"
	export PX4_SIM_MODEL="${PX4_SIM_MODEL:-tv3_lander}"
	export PX4_SIM_SPEED_FACTOR="${PX4_SIM_SPEED_FACTOR:-1}"

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
	;;
sitl)
	WORKTREE=$("${SCRIPT_DIR}/prepare_px4_tree.sh")
	MODULES_LOCATION="${TV3_ROOT}/.work/tv3-px4-extension"
	QT5_PREFIX=/opt/homebrew/opt/qt@5

	ln -sfn "${REPO_ROOT}" "${MODULES_LOCATION}"

	if [ -d "${QT5_PREFIX}" ]; then
		CMAKE_PREFIX_PATH="${QT5_PREFIX}${CMAKE_PREFIX_PATH:+:${CMAKE_PREFIX_PATH}}"
	fi

	env CMAKE_PREFIX_PATH="${CMAKE_PREFIX_PATH:-}" EXTERNAL_MODULES_LOCATION="${MODULES_LOCATION}" \
		make -C "${WORKTREE}" px4_sitl_default DONT_RUN=1

	printf 'PX4 SITL build ready in %s\n' "${WORKTREE}"
	;;
nuttx)
	if [ -z "${NUTTX_TARGET}" ]; then
		usage
		exit 1
	fi
	exec "${SCRIPT_DIR}/build_nuttx.sh" "${NUTTX_TARGET}"
	;;
*)
	echo "unknown target: ${TARGET}" >&2
	usage
	exit 1
	;;
esac
