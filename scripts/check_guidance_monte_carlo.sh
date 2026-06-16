#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
REPO_ROOT=$(cd -- "${SCRIPT_DIR}/.." && pwd)

cd "${REPO_ROOT}"

PYTHONDONTWRITEBYTECODE=1 python3 -m unittest tests.test_guidance_envelope -v

if [ ! -f "${REPO_ROOT}/config/flight_profiles/lander_impossible_guidance.yaml" ]; then
	printf 'missing impossible guidance profile\n' >&2
	exit 1
fi

python3 "${REPO_ROOT}/tools/run_guidance_monte_carlo.py" \
	--samples 32 \
	--seed 5 >/dev/null

printf 'Phase 5 guidance-and-monte-carlo gate passed\n'