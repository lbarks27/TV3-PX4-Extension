#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
REPO_ROOT=$(cd -- "${SCRIPT_DIR}/.." && pwd)
TV3_ROOT=$(cd -- "${REPO_ROOT}/.." && pwd)
WORKTREE=$("${SCRIPT_DIR}/prepare_px4_tree.sh")
MODULES_LOCATION="${TV3_ROOT}/.work/tv3-px4-extension"
TARGET="${1:-${PX4_NUTTX_TARGET:-}}"
VEHICLE_CONFIG="${TV3_VEHICLE_CONFIG:-${REPO_ROOT}/config/vehicles/tv3_v1.yaml}"

if [ -z "${TARGET}" ]; then
	echo "usage: $0 <px4_target>"
	exit 1
fi

ln -sfn "${REPO_ROOT}" "${MODULES_LOCATION}"

make -C "${WORKTREE}" "${TARGET}" EXTERNAL_MODULES_LOCATION="${MODULES_LOCATION}"

ASSET_ROOT="${REPO_ROOT}/build/nuttx/${TARGET}"
rm -rf "${ASSET_ROOT}"
"${REPO_ROOT}/tools/generate_vehicle_assets.py" \
	--vehicle "${VEHICLE_CONFIG}" \
	--output "${ASSET_ROOT}"

STAGE_ROOT="${TV3_ROOT}/.work/${TARGET}_runtime"
mkdir -p "${STAGE_ROOT}/etc"
mkdir -p "${STAGE_ROOT}/fs/microsd"
rsync -a "${ASSET_ROOT}/runtime/etc/" "${STAGE_ROOT}/etc/"
rsync -a "${ASSET_ROOT}/runtime/fs/microsd/" "${STAGE_ROOT}/fs/microsd/"

if [ -f "${REPO_ROOT}/build/motors/catalog.csv" ]; then
	mkdir -p "${STAGE_ROOT}/fs/microsd/tv3/motors"
	rsync -a "${REPO_ROOT}/build/motors/" "${STAGE_ROOT}/fs/microsd/tv3/motors/"
fi

printf 'staged runtime assets in %s\n' "${STAGE_ROOT}"
