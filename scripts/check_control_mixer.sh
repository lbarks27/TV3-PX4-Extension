#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
REPO_ROOT=$(cd -- "${SCRIPT_DIR}/.." && pwd)

cd "${REPO_ROOT}"

PYTHONDONTWRITEBYTECODE=1 python3 -m unittest tests.test_control_allocator -v

python3 "${REPO_ROOT}/tools/tv3_allocator.py" \
	--vehicle "${REPO_ROOT}/config/vehicles/tv3_lander_v1.yaml" \
	--thrust 620 \
	--torque 0 0 0 >/dev/null

python3 "${REPO_ROOT}/tools/tv3_allocator.py" \
	--vehicle "${REPO_ROOT}/config/vehicles/tv3_lander_v1.yaml" \
	--thrust 100 \
	--torque 0 0 0 | python3 -c 'import json,sys; data=json.load(sys.stdin); assert not data["reachable"]; assert data["reason"] == "net thrust outside splay envelope"'

printf 'Phase 4 control-mixer gate passed\n'