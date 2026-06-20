#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
REPO_ROOT=$(cd -- "${SCRIPT_DIR}/.." && pwd)
VENV="${REPO_ROOT}/.venv-bench"

cd "${REPO_ROOT}"

if [ ! -d "${VENV}" ]; then
	python3 -m venv "${VENV}"
fi

"${VENV}/bin/pip" install -q pymavlink pyserial

CONNECT="${TV3_MAVLINK_CONNECT:-/dev/cu.usbmodem01}"
VEHICLE="${TV3_VEHICLE_CONFIG:-config/vehicles/tv3_v1.json}"

printf 'Phase 2 bench capture: vehicle=%s connect=%s\n' "${VEHICLE}" "${CONNECT}"
printf 'Requires TV3 firmware on the Cube and a staged microSD card (./scripts/stage_microsd.sh).\n'
printf 'Close QGroundControl if serial open fails (Resource busy).\n'

"${VENV}/bin/python" "${REPO_ROOT}/tools/capture_bench_manifest.py" \
	--vehicle "${VEHICLE}" \
	--connect "${CONNECT}" \
	--sample-seconds "${TV3_BENCH_SAMPLE_S:-30}" \
	--update-manifest \
	"$@"

./tools/validate_vehicle_manifest.py "${VEHICLE}"

./tools/generate_vehicle_assets.py \
	--vehicle "${REPO_ROOT}/${VEHICLE}" \
	--output "${REPO_ROOT}/build/physical_manifest/$(basename "${VEHICLE}" .json)"

printf 'Phase 2 bench capture complete. Review logs/ground/bench_capture_*.json and remaining non-measured fields.\n'