#!/usr/bin/env bash
#
# Headless smoke test for TV3 visualization entry points.
# Interactive viewers (Hawkeye, Foxglove, PyVista --interactive) are not launched.

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

run_check "plot_ulog --latest" ./scripts/plot_ulog.sh --latest
run_check "plot_ulog --list-topics" ./scripts/plot_ulog.sh --latest --list-topics >/dev/null
run_check "vehicle overview default" ./scripts/view_vehicle_frame.sh --save "${TMP_ROOT}/vehicle.png"

printf 'Validated %s checks: %s passed, %s failed\n' "$((pass + fail))" "${pass}" "${fail}"
if [ "${fail}" -ne 0 ]; then
	exit 1
fi