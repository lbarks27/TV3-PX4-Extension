#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
REPO_ROOT=$(cd -- "${SCRIPT_DIR}/.." && pwd)
VENV="${REPO_ROOT}/.venv-bench"

cd "${REPO_ROOT}"

if [ ! -d "${VENV}" ]; then
	python3 -m venv "${VENV}"
	"${VENV}/bin/pip" install -q -r "${REPO_ROOT}/requirements-bench.txt"
fi

CONNECT="${TV3_MAVLINK_CONNECT:-/dev/cu.usbmodem01}"

exec "${VENV}/bin/python" "${REPO_ROOT}/tools/watch_load_cell.py" \
	--connect "${CONNECT}" \
	"$@"