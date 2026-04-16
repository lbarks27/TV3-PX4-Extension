#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
REPO_ROOT=$(cd -- "${SCRIPT_DIR}/.." && pwd)
WORKTREE=$("${SCRIPT_DIR}/prepare_px4_tree.sh")
QT5_PREFIX=/opt/homebrew/opt/qt@5

if [ -d "${QT5_PREFIX}" ]; then
	CMAKE_PREFIX_PATH="${QT5_PREFIX}${CMAKE_PREFIX_PATH:+:${CMAKE_PREFIX_PATH}}"
fi

PX4_SYS_AUTOSTART="${PX4_SYS_AUTOSTART:-11001}" \
PX4_SIM_MODEL="${PX4_SIM_MODEL:-none}" \
env CMAKE_PREFIX_PATH="${CMAKE_PREFIX_PATH:-}" EXTERNAL_MODULES_LOCATION="${REPO_ROOT}" \
	make -C "${WORKTREE}" px4_sitl_default none
