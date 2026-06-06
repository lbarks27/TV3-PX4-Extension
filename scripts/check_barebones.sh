#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
REPO_ROOT=$(cd -- "${SCRIPT_DIR}/.." && pwd)
OUTPUT_ROOT="${REPO_ROOT}/build/barebones"
VEHICLE_CONFIG="${TV3_VEHICLE_CONFIG:-${REPO_ROOT}/config/vehicles/tv3_v1.yaml}"
FLIGHT_PROFILE="${TV3_FLIGHT_PROFILE:-}"
if [[ "${VEHICLE_CONFIG}" != /* ]]; then
	VEHICLE_CONFIG="${REPO_ROOT}/${VEHICLE_CONFIG}"
fi
if [ -n "${FLIGHT_PROFILE}" ] && [[ "${FLIGHT_PROFILE}" != /* ]]; then
	FLIGHT_PROFILE="${REPO_ROOT}/${FLIGHT_PROFILE}"
fi

cd "${REPO_ROOT}"

PYTHONDONTWRITEBYTECODE=1 python3 -m unittest discover -s tests -v
rm -rf "${OUTPUT_ROOT}"
GENERATOR_ARGS=(
	--vehicle "${VEHICLE_CONFIG}"
	--output "${OUTPUT_ROOT}"
)
if [ -n "${FLIGHT_PROFILE}" ]; then
	GENERATOR_ARGS+=(--flight-profile "${FLIGHT_PROFILE}")
fi
./tools/generate_vehicle_assets.py "${GENERATOR_ARGS[@]}"

printf 'bare-bones assets generated in %s\n' "${OUTPUT_ROOT}"
