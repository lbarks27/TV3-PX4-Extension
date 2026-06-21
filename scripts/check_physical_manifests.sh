#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
REPO_ROOT=$(cd -- "${SCRIPT_DIR}/.." && pwd)

cd "${REPO_ROOT}"

"${SCRIPT_DIR}/run_tests.sh"

./tools/validate_vehicle_manifest.py

for vehicle in config/vehicles/tv3_v1.json config/vehicles/tv3_lander_v1.json; do
	./tools/generate_vehicle_assets.py --vehicle "${vehicle}" --output "${REPO_ROOT}/build/physical_manifest/$(basename "${vehicle}" .json)"
done

printf 'Phase 2 physical-manifest gate passed (manifests validated; flight_ready remains false until measured data is supplied)\n'