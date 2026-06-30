#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
REPO_ROOT=$(cd -- "${SCRIPT_DIR}/.." && pwd)

cd "${REPO_ROOT}"

if [ -d "${REPO_ROOT}/tests" ]; then
	PYTHONDONTWRITEBYTECODE=1 python3 -m unittest tests.test_gimbal_lm tests.test_gimbal_lm_convergence -v
fi

printf 'control-mixer gate passed\n'
