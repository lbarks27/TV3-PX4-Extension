#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
REPO_ROOT=$(cd -- "${SCRIPT_DIR}/.." && pwd)
TV3_ROOT=$(cd -- "${REPO_ROOT}/.." && pwd)
VENDOR_DIR="${TV3_ROOT}/vendor/px4"
PX4_REMOTE="${PX4_REMOTE:-https://github.com/PX4/PX4-Autopilot}"
PX4_REF="${PX4_REF:-v1.16.1}"

mkdir -p "${TV3_ROOT}/vendor"

if [ ! -d "${VENDOR_DIR}/.git" ]; then
	git clone --depth 1 --branch "${PX4_REF}" "${PX4_REMOTE}" "${VENDOR_DIR}"
else
	git -C "${VENDOR_DIR}" fetch --tags origin "${PX4_REF}" --depth 1 >/dev/null 2>&1 || true
	git -C "${VENDOR_DIR}" fetch --all --tags >/dev/null 2>&1 || true
	git -C "${VENDOR_DIR}" checkout -q "${PX4_REF}"
fi

git -C "${VENDOR_DIR}" submodule update --init --recursive

printf '%s\n' "${VENDOR_DIR}"
