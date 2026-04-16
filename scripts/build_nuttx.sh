#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
REPO_ROOT=$(cd -- "${SCRIPT_DIR}/.." && pwd)
TV3_ROOT=$(cd -- "${REPO_ROOT}/.." && pwd)
WORKTREE=$("${SCRIPT_DIR}/prepare_px4_tree.sh")
TARGET="${1:-${PX4_NUTTX_TARGET:-}}"

if [ -z "${TARGET}" ]; then
	echo "usage: $0 <px4_target>"
	exit 1
fi

make -C "${WORKTREE}" "${TARGET}" EXTERNAL_MODULES_LOCATION="${REPO_ROOT}"

STAGE_ROOT="${TV3_ROOT}/.work/${TARGET}_runtime"
mkdir -p "${STAGE_ROOT}/etc"
mkdir -p "${STAGE_ROOT}/fs/microsd"
rsync -a "${REPO_ROOT}/runtime/nuttx/etc/" "${STAGE_ROOT}/etc/"
rsync -a "${REPO_ROOT}/runtime/nuttx/fs/microsd/" "${STAGE_ROOT}/fs/microsd/"

printf 'staged runtime assets in %s\n' "${STAGE_ROOT}"
