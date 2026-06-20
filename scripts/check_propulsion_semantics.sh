#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
REPO_ROOT=$(cd -- "${SCRIPT_DIR}/.." && pwd)

cd "${REPO_ROOT}"

PYTHONDONTWRITEBYTECODE=1 python3 -m unittest discover -s tests -v

if [ ! -f "${REPO_ROOT}/docs/templates/bench_calibration_report.md" ]; then
	printf 'missing bench calibration template\n' >&2
	exit 1
fi

for fixture in delayed_ignition failed_ignition false_positive_spike burnout stale_mid_burn lander_three_engine_sequence; do
	if [ ! -f "${REPO_ROOT}/tests/fixtures/load_cell_adc/${fixture}.csv" ]; then
		printf 'missing ADC fixture %s\n' "${fixture}" >&2
		exit 1
	fi
done

python3 "${REPO_ROOT}/tools/replay_load_cell_adc.py" \
	"${REPO_ROOT}/tests/fixtures/load_cell_adc/delayed_ignition.csv" \
	--vehicle "${REPO_ROOT}/config/vehicles/tv3_v1.json" >/dev/null

printf 'Phase 3 propulsion-semantics gate passed\n'
