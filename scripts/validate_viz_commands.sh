#!/usr/bin/env bash
#
# Headless smoke test for TV3 visualization entry points.
# Interactive viewers (Hawkeye, Rerun live, PyVista --interactive) are not launched.

set -euo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
REPO_ROOT=$(cd -- "${SCRIPT_DIR}/.." && pwd)
# shellcheck source=scripts/viz_env.sh
source "${SCRIPT_DIR}/viz_env.sh"

TMP_ROOT=$(mktemp -d "${TMPDIR:-/tmp}/tv3-viz-validate.XXXXXX")
trap 'rm -rf "${TMP_ROOT}"' EXIT

pass=0
fail=0

run_check() {
	local label=$1
	shift
	printf '== %s\n' "${label}"
	if "$@"; then
		pass=$((pass + 1))
		printf 'OK: %s\n\n' "${label}"
	else
		fail=$((fail + 1))
		printf 'FAIL: %s\n\n' "${label}"
	fi
}

cd "${REPO_ROOT}"

run_check "plot_ulog --list-topics" ./scripts/plot_ulog.sh --latest --list-topics >/dev/null
run_check "replay full .rrd (ignores --scene for Rerun; one file with all content)" ./scripts/tv3_replay.sh --latest --scene trajectory -o "${TMP_ROOT}/trajectory.rrd"
run_check "replay full .rrd via engines scene flag" ./scripts/tv3_replay.sh --latest --scene engines -o "${TMP_ROOT}/engines.rrd"
run_check "replay trajectory .png" ./scripts/tv3_replay.sh --latest --scene trajectory -o "${TMP_ROOT}/trajectory.png" --time 12.5
run_check "vehicle overview default" ./scripts/view_vehicle_frame.sh --save "${TMP_ROOT}/vehicle.png"

printf 'Validated %s checks: %s passed, %s failed\n' "$((pass + fail))" "${pass}" "${fail}"
if [ "${fail}" -ne 0 ]; then
	exit 1
fi
