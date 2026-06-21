#!/usr/bin/env bash
#
# Deprecated — use ./scripts/tv3_replay.sh --scene engines instead.

set -euo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
echo "note: plot_ulog_engines.sh is deprecated; use ./scripts/tv3_replay.sh --scene engines" >&2
exec "${SCRIPT_DIR}/tv3_replay.sh" --scene engines "$@"
