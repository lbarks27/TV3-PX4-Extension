#!/usr/bin/env bash
#
# Unified TV3 ULog replay (trajectory, engines, guidance, all).

set -euo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
# shellcheck source=scripts/viz_env.sh
source "${SCRIPT_DIR}/viz_env.sh"

exec "${PYTHON}" "${REPO_ROOT}/tools/tv3_replay.py" "$@"
