#!/usr/bin/env bash
# Deprecated — use ./scripts/build_px4.sh --target sitl

set -euo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
exec "${SCRIPT_DIR}/build_px4.sh" --target sitl "$@"
