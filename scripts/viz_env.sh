#!/usr/bin/env bash
#
# Shared TV3 visualization environment for shell wrappers.
# Source this from viz scripts; do not execute directly.

: "${SCRIPT_DIR:?SCRIPT_DIR must be set by the caller}"

REPO_ROOT=$(cd -- "${SCRIPT_DIR}/.." && pwd)
TV3_ROOT=$(cd -- "${REPO_ROOT}/.." && pwd)
VENV="${TV3_VIZ_VENV:-${TV3_ROOT}/.work/tv3-viz-venv}"
PYTHON="${TV3_VIZ_PYTHON:-${VENV}/bin/python}"

if [ ! -x "${PYTHON}" ]; then
	echo "TV3 viz Python not found at ${PYTHON}" >&2
	echo "Run ./scripts/setup_viz_env.sh first." >&2
	exit 1
fi

# Rerun viewer spawn and `rerun file.rrd` both resolve the executable from PATH.
export PATH="${VENV}/bin:${PATH}"