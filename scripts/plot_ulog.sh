#!/usr/bin/env bash
#
# Run the TV3 ULog plotter with the repo-managed viz Python environment.

set -euo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
# shellcheck source=scripts/viz_env.sh
source "${SCRIPT_DIR}/viz_env.sh"

exec "${PYTHON}" "${REPO_ROOT}/tools/plot_ulog.py" "$@"