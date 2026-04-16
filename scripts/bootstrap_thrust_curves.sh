#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
REPO_ROOT=$(cd -- "${SCRIPT_DIR}/.." && pwd)
TV3_ROOT=$(cd -- "${REPO_ROOT}/.." && pwd)
THRUST_DIR="${TV3_ROOT}/vendor/Thrust-Curves-Apogee"
THRUST_REMOTE="${THRUST_REMOTE:-https://github.com/lbarks27/Thrust-Curves-Apogee.git}"

mkdir -p "${TV3_ROOT}/vendor"

if [ ! -d "${THRUST_DIR}/.git" ]; then
	git clone --depth 1 "${THRUST_REMOTE}" "${THRUST_DIR}"
else
	git -C "${THRUST_DIR}" fetch --depth 1 origin HEAD >/dev/null 2>&1 || true
	git -C "${THRUST_DIR}" pull --ff-only >/dev/null 2>&1 || true
fi

printf '%s\n' "${THRUST_DIR}"
