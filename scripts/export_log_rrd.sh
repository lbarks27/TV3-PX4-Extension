#!/usr/bin/env bash
#
# Export a TV3 Rerun recording (.rrd) beside an archived PX4 ULog.
#
# Examples:
#   ./scripts/export_log_rrd.sh logs/sim/2026-06-21/run-001/12_34_56.ulg
#   ./scripts/export_log_rrd.sh /path/to/flight.ulg -o /path/to/flight.tv3.rrd

set -euo pipefail

usage() {
	cat <<'EOF'
Usage:
  export_log_rrd.sh ULOG.ulg [-o OUTPUT.rrd]

Writes a unified Rerun recording (trajectory, engines, guidance on sim_time).
Default output: <ulog-stem>.tv3.rrd beside the .ulg file.
EOF
}

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
# shellcheck source=scripts/viz_env.sh
source "${SCRIPT_DIR}/viz_env.sh"

ULOG=""
OUTPUT=""

while [ "$#" -gt 0 ]; do
	case "$1" in
		-o|--output)
			OUTPUT="${2:?missing value for --output}"
			shift 2
			;;
		-h|--help)
			usage
			exit 0
			;;
		-*)
			echo "unknown argument: $1" >&2
			usage >&2
			exit 2
			;;
		*)
			if [ -z "${ULOG}" ]; then
				ULOG="$1"
			else
				echo "unexpected extra argument: $1" >&2
				usage >&2
				exit 2
			fi
			shift
			;;
	esac
done

if [ -z "${ULOG}" ]; then
	echo "ULOG path is required" >&2
	usage >&2
	exit 2
fi

if [ ! -f "${ULOG}" ]; then
	echo "ULog not found: ${ULOG}" >&2
	exit 2
fi

if [ -z "${OUTPUT}" ]; then
	OUTPUT="${ULOG%.ulg}.tv3.rrd"
fi

exec "${PYTHON}" "${REPO_ROOT}/tools/tv3_replay.py" "${ULOG}" -o "${OUTPUT}"
