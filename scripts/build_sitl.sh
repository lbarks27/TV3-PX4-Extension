#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
REPO_ROOT=$(cd -- "${SCRIPT_DIR}/.." && pwd)
TV3_ROOT=$(cd -- "${REPO_ROOT}/.." && pwd)
WORKTREE=$("${SCRIPT_DIR}/prepare_px4_tree.sh")
MODULES_LOCATION="${TV3_ROOT}/.work/tv3-px4-extension"
QT5_PREFIX=/opt/homebrew/opt/qt@5

ln -sfn "${REPO_ROOT}" "${MODULES_LOCATION}"

if [ -d "${QT5_PREFIX}" ]; then
	CMAKE_PREFIX_PATH="${QT5_PREFIX}${CMAKE_PREFIX_PATH:+:${CMAKE_PREFIX_PATH}}"
fi

env CMAKE_PREFIX_PATH="${CMAKE_PREFIX_PATH:-}" EXTERNAL_MODULES_LOCATION="${MODULES_LOCATION}" \
	make -C "${WORKTREE}" px4_sitl_default DONT_RUN=1
