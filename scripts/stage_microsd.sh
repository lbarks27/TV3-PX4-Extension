#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
REPO_ROOT=$(cd -- "${SCRIPT_DIR}/.." && pwd)
VEHICLE_CONFIG="${TV3_VEHICLE_CONFIG:-${REPO_ROOT}/config/vehicles/tv3_v1.json}"
STAGE_BUILD="${REPO_ROOT}/build/microsd_stage"
SD_MOUNT="${TV3_SD_MOUNT:-}"

usage() {
	cat <<'EOF'
Stage generated TV3 runtime assets onto a Cube microSD card.

Usage:
  ./scripts/stage_microsd.sh [--mount /Volumes/NO\ NAME]

Environment:
  TV3_SD_MOUNT       Absolute path to the mounted microSD volume
  TV3_VEHICLE_CONFIG Vehicle manifest JSON (default: config/vehicles/tv3_v1.json)

The card must be FAT32 and mounted on this machine (USB SD reader or direct slot).
PX4 expects:
  <mount>/etc/config.txt
  <mount>/etc/extras.txt
  <mount>/tv3/airframes/*.params
  <mount>/tv3/motors/*
EOF
}

while [ "$#" -gt 0 ]; do
	case "$1" in
		--mount)
			SD_MOUNT="${2:-}"
			shift 2
			;;
		-h | --help)
			usage
			exit 0
			;;
		*)
			printf 'unknown argument: %s\n' "$1" >&2
			usage >&2
			exit 1
			;;
	esac
done

if [ -z "${SD_MOUNT}" ]; then
	for candidate in /Volumes/NO\ NAME /Volumes/CUBE /Volumes/PIXHAWK; do
		if [ -d "${candidate}" ]; then
			SD_MOUNT="${candidate}"
			break
		fi
	done
fi

if [ -z "${SD_MOUNT}" ] || [ ! -d "${SD_MOUNT}" ]; then
	printf 'microSD mount not found. Set TV3_SD_MOUNT or pass --mount.\n' >&2
	printf 'Current /Volumes:\n' >&2
	ls -1 /Volumes >&2 || true
	exit 1
fi

printf 'validating %s\n' "${VEHICLE_CONFIG}"
PYTHONDONTWRITEBYTECODE=1 python3 "${REPO_ROOT}/tools/validate_vehicle_manifest.py" "${VEHICLE_CONFIG}" >/dev/null

rm -rf "${STAGE_BUILD}"
PYTHONDONTWRITEBYTECODE=1 python3 "${REPO_ROOT}/tools/generate_vehicle_assets.py" \
	--vehicle "${VEHICLE_CONFIG}" \
	--output "${STAGE_BUILD}"

ASSET_ETC="${STAGE_BUILD}/runtime/etc"
ASSET_ROOT="${STAGE_BUILD}/runtime/fs/microsd"

for required in \
	"${ASSET_ETC}/config.txt" \
	"${ASSET_ETC}/extras.txt" \
	"${ASSET_ROOT}/tv3/airframes/tv3_v1.params" \
	"${ASSET_ROOT}/tv3/motors/catalog.csv"; do
	if [ ! -f "${required}" ]; then
		printf 'missing generated asset: %s\n' "${required}" >&2
		exit 1
	fi
done

mkdir -p "${SD_MOUNT}/etc" "${SD_MOUNT}/tv3"

rsync -a "${ASSET_ETC}/" "${SD_MOUNT}/etc/"
rsync -a "${ASSET_ROOT}/" "${SD_MOUNT}/"

printf 'staged TV3 payload on %s\n' "${SD_MOUNT}"
printf '  etc/config.txt\n'
printf '  etc/extras.txt\n'
printf '  tv3/airframes/tv3_v1.params\n'
printf '  tv3/motors/catalog.csv\n'
printf 'Safely eject the card, insert into the Cube Orange Plus, then power-cycle.\n'
printf 'After boot, close QGC and run: ./scripts/complete_phase2_bench.sh\n'