#!/usr/bin/env bash
#
# Render TV3 vehicle manifests with the repo-managed viz Python environment.

set -euo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
# shellcheck source=scripts/viz_env.sh
source "${SCRIPT_DIR}/viz_env.sh"

exec "${PYTHON}" "${REPO_ROOT}/tools/view_vehicle_frame.py" "$@"