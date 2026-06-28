#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
REPO_ROOT=$(cd -- "${SCRIPT_DIR}/.." && pwd)

RUN_GATES="none"
GATE_TIMEOUT_S=1800

usage() {
	cat <<'EOF'
Usage:
  check_completion_status.sh [--run-gates none|fast|all] [--gate-timeout-s SECONDS]

Updates config/completion_status.json and docs/completion_status.md from gate
scripts, vehicle manifest provenance, and archived log evidence.

Examples:
  ./scripts/check_completion_status.sh
  ./scripts/check_completion_status.sh --run-gates fast
  ./scripts/check_completion_status.sh --run-gates all --gate-timeout-s 3600
EOF
}

while [ "$#" -gt 0 ]; do
	case "$1" in
		--run-gates)
			RUN_GATES="${2:?missing value for --run-gates}"
			shift 2
			;;
		--gate-timeout-s)
			GATE_TIMEOUT_S="${2:?missing value for --gate-timeout-s}"
			shift 2
			;;
		-h | --help)
			usage
			exit 0
			;;
		*)
			echo "unknown argument: $1" >&2
			usage >&2
			exit 2
			;;
	esac
done

cd "${REPO_ROOT}"
python3 "${REPO_ROOT}/tools/report_completion_status.py" \
	--run-gates "${RUN_GATES}" \
	--gate-timeout-s "${GATE_TIMEOUT_S}"