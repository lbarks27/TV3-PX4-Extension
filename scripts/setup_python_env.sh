#!/usr/bin/env bash
#
# Create/update Python environments for TV3 host tooling.

set -euo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
REPO_ROOT=$(cd -- "${SCRIPT_DIR}/.." && pwd)
TV3_ROOT=$(cd -- "${REPO_ROOT}/.." && pwd)

PROFILE="all"

usage() {
	cat <<'EOF'
usage: setup_python_env.sh --profile viz|bench|all

  viz   Repo-parent viz venv (PyVista, Rerun, Matplotlib)
  bench In-repo bench venv (MAVLink, serial)
  all   Both environments
EOF
}

while [ $# -gt 0 ]; do
	case "$1" in
	--profile)
		PROFILE=$2
		shift 2
		;;
	-h | --help)
		usage
		exit 0
		;;
	*)
		echo "unknown argument: $1" >&2
		usage
		exit 1
		;;
	esac
done

setup_viz() {
	"${SCRIPT_DIR}/setup_viz_env.sh"
}

setup_bench() {
	local venv="${REPO_ROOT}/.venv-bench"
	python3 -m venv "${venv}"
	"${venv}/bin/python" -m pip install --upgrade pip
	"${venv}/bin/python" -m pip install -r "${REPO_ROOT}/requirements-bench.txt"
	printf 'TV3 bench Python: %s/bin/python\n' "${venv}"
}

case "${PROFILE}" in
viz)
	setup_viz
	;;
bench)
	setup_bench
	;;
all)
	setup_viz
	setup_bench
	;;
*)
	usage
	exit 1
	;;
esac
