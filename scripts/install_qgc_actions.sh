#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
REPO_ROOT=$(cd "${SCRIPT_DIR}/.." && pwd)

SOURCE="${REPO_ROOT}/config/qgc/TV3Actions.json"
QGC_SAVE_ROOT="${QGC_SAVE_ROOT:-${HOME}/Documents/QGroundControl}"
QGC_ACTIONS_DIR="${QGC_ACTIONS_DIR:-${QGC_SAVE_ROOT}/MavlinkActions}"
TARGET="${QGC_ACTIONS_DIR}/TV3Actions.json"

if [ ! -f "${SOURCE}" ]; then
	echo "missing ${SOURCE}" >&2
	exit 1
fi

install -d "${QGC_ACTIONS_DIR}"
install -m 0644 "${SOURCE}" "${TARGET}"

echo "Installed ${TARGET}"
echo "Restart QGroundControl so it reloads MAVLink actions."
