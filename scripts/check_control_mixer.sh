#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
REPO_ROOT=$(cd -- "${SCRIPT_DIR}/.." && pwd)

cd "${REPO_ROOT}"

PYTHONDONTWRITEBYTECODE=1 python3 -m unittest tests.test_control_allocator -v

HOVER_THRUST_N=$(cd "${REPO_ROOT}" && PYTHONDONTWRITEBYTECODE=1 PYTHONPATH="${REPO_ROOT}" python3 - <<'PY'
from pathlib import Path
from tools.tv3_control_allocator import load_manifest, vehicle_full_thrust_n

print(f"{vehicle_full_thrust_n(load_manifest(Path('config/vehicles/tv3_lander_v1.json'))):.3f}")
PY
)

python3 "${REPO_ROOT}/tools/tv3_allocator.py" \
	--vehicle "${REPO_ROOT}/config/vehicles/tv3_lander_v1.json" \
	--thrust "${HOVER_THRUST_N}" \
	--torque 0 0 0 >/dev/null

python3 "${REPO_ROOT}/tools/tv3_allocator.py" \
	--vehicle "${REPO_ROOT}/config/vehicles/tv3_lander_v1.json" \
	--thrust 95 \
	--torque 0 0 0 | python3 -c 'import json,sys; data=json.load(sys.stdin); assert not data["reachable"]; assert data["reason"] == "net thrust outside splay envelope"'

printf 'Phase 4 control-mixer gate passed\n'