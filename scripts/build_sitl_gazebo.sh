#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
REPO_ROOT=$(cd -- "${SCRIPT_DIR}/.." && pwd)
WORKTREE=$("${SCRIPT_DIR}/prepare_px4_tree.sh")

make -C "${WORKTREE}" px4_sitl_default EXTERNAL_MODULES_LOCATION="${REPO_ROOT}" DONT_RUN=1
