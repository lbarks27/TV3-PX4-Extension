#!/usr/bin/env bash
#
# Copy the generated TV3 ULog topic profile into the PX4 SITL storage root.
# PX4's logger reads ${PX4_STORAGEDIR}/etc/logging/logger_topics.txt at boot.

set -euo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
REPO_ROOT=$(cd -- "${SCRIPT_DIR}/.." && pwd)
TV3_ROOT=$(cd -- "${REPO_ROOT}/.." && pwd)
WORKTREE="${1:-${TV3_ROOT}/.work/px4-tv3}"
VEHICLE_CONFIG="${TV3_VEHICLE_CONFIG:-${REPO_ROOT}/config/vehicles/tv3_v1.yaml}"
FLIGHT_PROFILE="${TV3_FLIGHT_PROFILE:-}"
GENERATED_ROOT="${REPO_ROOT}/build/barebones"
SOURCE="${GENERATED_ROOT}/runtime/etc/logging/logger_topics.txt"
PX4_BUILD_DIR="${WORKTREE}/build/px4_sitl_default"
DESTS=(
	"${PX4_BUILD_DIR}/etc/logging/logger_topics.txt"
	"${PX4_BUILD_DIR}/rootfs/etc/logging/logger_topics.txt"
)

if [[ "${VEHICLE_CONFIG}" != /* ]]; then
	VEHICLE_CONFIG="${REPO_ROOT}/${VEHICLE_CONFIG}"
fi
if [ -n "${FLIGHT_PROFILE}" ] && [[ "${FLIGHT_PROFILE}" != /* ]]; then
	FLIGHT_PROFILE="${REPO_ROOT}/${FLIGHT_PROFILE}"
fi

GENERATOR_ARGS=(
	"${REPO_ROOT}/tools/generate_vehicle_assets.py"
	--vehicle "${VEHICLE_CONFIG}"
	--output "${GENERATED_ROOT}"
)
if [ -n "${FLIGHT_PROFILE}" ]; then
	GENERATOR_ARGS+=(--flight-profile "${FLIGHT_PROFILE}")
fi

"${GENERATOR_ARGS[@]}" >/dev/null

for dest in "${DESTS[@]}"; do
	mkdir -p "$(dirname -- "${dest}")"
	cp "${SOURCE}" "${dest}"
done

printf 'TV3 logger topics synced to %s and %s\n' "${DESTS[0]}" "${DESTS[1]}"
