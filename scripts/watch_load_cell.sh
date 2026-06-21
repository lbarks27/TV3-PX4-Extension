#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
REPO_ROOT=$(cd -- "${SCRIPT_DIR}/.." && pwd)
VENV="${REPO_ROOT}/.venv-bench"

cd "${REPO_ROOT}"

"${SCRIPT_DIR}/setup_python_env.sh" --profile bench >/dev/null

CONNECT="${TV3_MAVLINK_CONNECT:-/dev/cu.usbmodem01}"

exec "${VENV}/bin/python" "${REPO_ROOT}/tools/watch_load_cell.py" \
	--connect "${CONNECT}" \
	"$@"
