#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
REPO_ROOT=$(cd -- "${SCRIPT_DIR}/.." && pwd)
# shellcheck source=scripts/_gate_common.sh
source "${SCRIPT_DIR}/_gate_common.sh"

export TV3_VEHICLE_CONFIG="${TV3_VEHICLE_CONFIG:-config/vehicles/tv3_lander_v1.json}"
export TV3_FLIGHT_PROFILE="${TV3_FLIGHT_PROFILE:-config/flight_profiles/lander_hover_window.json}"

cd "${REPO_ROOT}"

gate_run_tests
gate_generate_assets "${TV3_VEHICLE_CONFIG}" "${REPO_ROOT}/build/hover_window" "${TV3_FLIGHT_PROFILE}"

TV3_REUSE_PX4_WORKTREE=1 ./scripts/build_sih.sh

if [ "${TV3_SKIP_SIH_RUN:-0}" != "1" ]; then
	TV3_REUSE_PX4_WORKTREE=1 ./scripts/run_sitl_sih_headless.sh
fi

if [ -n "${TV3_REVIEW_ULOG:-}" ]; then
	./tools/review_flight_profile.py "${TV3_REVIEW_ULOG}" --flight-profile "${TV3_FLIGHT_PROFILE}"
else
	./tools/review_flight_profile.py --latest --flight-profile "${TV3_FLIGHT_PROFILE}"
fi
printf 'Phase 1 hover-window gate passed\n'
