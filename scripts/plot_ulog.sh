#!/usr/bin/env bash
#
# Run the TV3 ULog plotter with the repo-managed viz Python environment.

set -euo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
REPO_ROOT=$(cd -- "${SCRIPT_DIR}/.." && pwd)
TV3_ROOT=$(cd -- "${REPO_ROOT}/.." && pwd)
VENV="${TV3_VIZ_VENV:-${TV3_ROOT}/.work/tv3-viz-venv}"
PYTHON="${TV3_VIZ_PYTHON:-${VENV}/bin/python}"

if [ ! -x "${PYTHON}" ]; then
	echo "TV3 viz Python not found at ${PYTHON}" >&2
	echo "Run ./scripts/setup_viz_env.sh first." >&2
	exit 1
fi

exec "${PYTHON}" "${REPO_ROOT}/tools/plot_ulog.py" "$@"
