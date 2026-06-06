#!/usr/bin/env bash
#
# Create/update the Python environment used by TV3 ULog plotting tools.

set -euo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
REPO_ROOT=$(cd -- "${SCRIPT_DIR}/.." && pwd)
TV3_ROOT=$(cd -- "${REPO_ROOT}/.." && pwd)
VENV="${TV3_VIZ_VENV:-${TV3_ROOT}/.work/tv3-viz-venv}"

python3 -m venv "${VENV}"
"${VENV}/bin/python" -m pip install --upgrade pip
"${VENV}/bin/python" -m pip install -r "${REPO_ROOT}/requirements-viz.txt"

printf 'TV3 viz Python: %s\n' "${VENV}/bin/python"
