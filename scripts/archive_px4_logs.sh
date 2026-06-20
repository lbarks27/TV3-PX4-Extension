#!/usr/bin/env bash
#
# Archive PX4 ULog files into the repo-local TV3 log tree.
#
# Examples:
#   ./scripts/archive_px4_logs.sh --kind flight --source ~/Downloads/flight.ulg
#   ./scripts/archive_px4_logs.sh --kind ground --source /path/to/qgc/logs --run-id load-cell-bench-001

set -euo pipefail

usage() {
	cat <<'EOF'
Usage:
  archive_px4_logs.sh --kind sim|flight|ground --source FILE_OR_DIR [options]

Options:
  --archive-root PATH     Archive root. Defaults to <repo>/logs.
  --run-id ID             Run/test identifier. Defaults to UTC timestamp + kind.
  --vehicle-config PATH   Vehicle manifest to copy beside the logs.
  --worktree PATH         PX4 SITL worktree; copies logger profile when present.
  --simulator NAME        Simulator name written to manifest.txt.
  TV3_FLIGHT_PROFILE      Optional env var copied beside archived logs when set.
  --since-marker PATH     Only archive .ulg files newer than this marker file.
  --notes TEXT            Short notes written to manifest.txt.
  -h, --help              Show this help.
EOF
}

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
REPO_ROOT=$(cd -- "${SCRIPT_DIR}/.." && pwd)

KIND=""
SOURCE=""
ARCHIVE_ROOT="${TV3_LOG_ARCHIVE_ROOT:-${REPO_ROOT}/logs}"
RUN_ID="${TV3_LOG_RUN_ID:-}"
VEHICLE_CONFIG="${TV3_VEHICLE_CONFIG:-}"
FLIGHT_PROFILE="${TV3_FLIGHT_PROFILE:-}"
WORKTREE=""
SIMULATOR="${TV3_SIMULATOR:-${PX4_SIMULATOR:-}}"
SINCE_MARKER=""
NOTES=""

while [ "$#" -gt 0 ]; do
	case "$1" in
		--kind)
			KIND="${2:?missing value for --kind}"
			shift 2
			;;
		--source)
			SOURCE="${2:?missing value for --source}"
			shift 2
			;;
		--archive-root)
			ARCHIVE_ROOT="${2:?missing value for --archive-root}"
			shift 2
			;;
		--run-id)
			RUN_ID="${2:?missing value for --run-id}"
			shift 2
			;;
		--vehicle-config)
			VEHICLE_CONFIG="${2:?missing value for --vehicle-config}"
			shift 2
			;;
		--worktree)
			WORKTREE="${2:?missing value for --worktree}"
			shift 2
			;;
		--simulator)
			SIMULATOR="${2:?missing value for --simulator}"
			shift 2
			;;
		--since-marker)
			SINCE_MARKER="${2:?missing value for --since-marker}"
			shift 2
			;;
		--notes)
			NOTES="${2:?missing value for --notes}"
			shift 2
			;;
		-h|--help)
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

case "${KIND}" in
	sim|flight|ground)
		;;
	*)
		echo "--kind must be one of: sim, flight, ground" >&2
		exit 2
		;;
esac

if [ -z "${SOURCE}" ]; then
	echo "--source is required" >&2
	exit 2
fi

if [ ! -e "${SOURCE}" ]; then
	echo "log source does not exist: ${SOURCE}" >&2
	exit 2
fi

if [ -n "${SINCE_MARKER}" ] && [ ! -e "${SINCE_MARKER}" ]; then
	echo "marker does not exist: ${SINCE_MARKER}" >&2
	exit 2
fi

sanitize_id() {
	printf '%s' "$1" | tr '/:[:space:]' '----' | tr -cd '[:alnum:]_.-'
}

is_newer_than_marker() {
	local path="$1"
	if [ -z "${SINCE_MARKER}" ]; then
		return 0
	fi
	[ "${path}" -nt "${SINCE_MARKER}" ]
}

LOGS=()
if [ -f "${SOURCE}" ]; then
	if [[ "${SOURCE}" == *.ulg ]] && is_newer_than_marker "${SOURCE}"; then
		LOGS+=("${SOURCE}")
	fi
else
	while IFS= read -r -d '' log_path; do
		if is_newer_than_marker "${log_path}"; then
			LOGS+=("${log_path}")
		fi
	done < <(find "${SOURCE}" -type f -name '*.ulg' -print0)
fi

if [ "${#LOGS[@]}" -eq 0 ]; then
	echo "No PX4 .ulg files found to archive under ${SOURCE}"
	exit 0
fi

ARCHIVE_DAY=$(date -u +%F)
if [ -z "${RUN_ID}" ]; then
	RUN_ID="$(date -u +%Y%m%dT%H%M%SZ)-${KIND}"
fi
RUN_ID=$(sanitize_id "${RUN_ID}")
RUN_DIR="${ARCHIVE_ROOT}/${KIND}/${ARCHIVE_DAY}/${RUN_ID}"

mkdir -p "${RUN_DIR}"

for log_path in "${LOGS[@]}"; do
	cp -p "${log_path}" "${RUN_DIR}/$(basename -- "${log_path}")"
done

if [ -n "${VEHICLE_CONFIG}" ]; then
	if [[ "${VEHICLE_CONFIG}" != /* ]]; then
		VEHICLE_CONFIG="${REPO_ROOT}/${VEHICLE_CONFIG}"
	fi
	if [ -f "${VEHICLE_CONFIG}" ]; then
		cp -p "${VEHICLE_CONFIG}" "${RUN_DIR}/vehicle.json"
	fi
fi

if [ -n "${FLIGHT_PROFILE}" ]; then
	if [[ "${FLIGHT_PROFILE}" != /* ]]; then
		FLIGHT_PROFILE="${REPO_ROOT}/${FLIGHT_PROFILE}"
	fi
	if [ -f "${FLIGHT_PROFILE}" ]; then
		cp -p "${FLIGHT_PROFILE}" "${RUN_DIR}/flight_profile.json"
	fi
fi

if [ -n "${WORKTREE}" ] && [ -d "${WORKTREE}" ]; then
	PX4_BUILD_DIR="${WORKTREE}/build/px4_sitl_default"
	for LOGGER_PROFILE in \
		"${PX4_BUILD_DIR}/etc/logging/logger_topics.txt" \
		"${PX4_BUILD_DIR}/rootfs/etc/logging/logger_topics.txt"
	do
		if [ -f "${LOGGER_PROFILE}" ]; then
			cp -p "${LOGGER_PROFILE}" "${RUN_DIR}/logger_topics.txt"
			break
		fi
	done
fi

GENERATED_LOGGER_PROFILE="${REPO_ROOT}/build/barebones/runtime/etc/logging/logger_topics.txt"
if [ ! -f "${RUN_DIR}/logger_topics.txt" ] && [ -f "${GENERATED_LOGGER_PROFILE}" ]; then
	cp -p "${GENERATED_LOGGER_PROFILE}" "${RUN_DIR}/logger_topics.txt"
fi

{
	echo "kind: ${KIND}"
	echo "run_id: ${RUN_ID}"
	echo "archived_at_utc: $(date -u +%Y-%m-%dT%H:%M:%SZ)"
	echo "repo_root: ${REPO_ROOT}"
	echo "source: ${SOURCE}"
	echo "archive_dir: ${RUN_DIR}"
	echo "vehicle_config: ${VEHICLE_CONFIG}"
	echo "flight_profile: ${FLIGHT_PROFILE}"
	echo "worktree: ${WORKTREE}"
	echo "simulator: ${SIMULATOR}"
	echo "px4_sys_autostart: ${PX4_SYS_AUTOSTART:-}"
	echo "px4_simulator: ${PX4_SIMULATOR:-}"
	echo "px4_sim_model: ${PX4_SIM_MODEL:-}"
	echo "hawkeye_udp_port: ${HAWKEYE_UDP_PORT:-19410}"
	echo "tv3_motor_root: ${TV3_MOTOR_ROOT:-}"
	echo "log_count: ${#LOGS[@]}"
	if [ -n "${NOTES}" ]; then
		echo "notes: ${NOTES}"
	fi
	echo
	echo "logs:"
	for log_path in "${LOGS[@]}"; do
		echo "  - $(basename -- "${log_path}")"
	done
} > "${RUN_DIR}/manifest.txt"

echo "Archived ${#LOGS[@]} PX4 log(s) to ${RUN_DIR}"
