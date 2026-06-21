#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
REPO_ROOT=$(cd -- "${SCRIPT_DIR}/.." && pwd)
# shellcheck source=scripts/_gate_common.sh
source "${SCRIPT_DIR}/_gate_common.sh"

OUTPUT_ROOT="${REPO_ROOT}/build/barebones"
VEHICLE_CONFIG="${TV3_VEHICLE_CONFIG:-config/vehicles/tv3_v1.json}"
FLIGHT_PROFILE="${TV3_FLIGHT_PROFILE:-}"

cd "${REPO_ROOT}"

gate_run_tests
gate_generate_assets "${VEHICLE_CONFIG}" "${OUTPUT_ROOT}" "${FLIGHT_PROFILE}"

if [ -x "${REPO_ROOT}/../.work/tv3-viz-venv/bin/python" ] || [ -x "${TV3_VIZ_VENV:-${REPO_ROOT}/../.work/tv3-viz-venv}/bin/python" ]; then
	"${SCRIPT_DIR}/validate_viz_commands.sh"
fi

printf 'bare-bones assets generated in %s\n' "${OUTPUT_ROOT}"
