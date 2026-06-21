#!/usr/bin/env bash
# Shared helpers for TV3 phase gate scripts.

set -euo pipefail

_gate_repo_root() {
	local script_dir
	script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[1]}")" && pwd)
	cd -- "${script_dir}/.." && pwd
}

gate_run_tests() {
	local repo_root
	repo_root=$(_gate_repo_root)
	cd "${repo_root}"
	"${repo_root}/scripts/run_tests.sh" "$@"
}

gate_resolve_path() {
	local repo_root=$1
	local path=$2
	if [[ "${path}" != /* ]]; then
		printf '%s/%s' "${repo_root}" "${path}"
	else
		printf '%s' "${path}"
	fi
}

gate_generate_assets() {
	local vehicle=$1
	local output_dir=$2
	local flight_profile=${3:-}
	local repo_root
	repo_root=$(_gate_repo_root)
	vehicle=$(gate_resolve_path "${repo_root}" "${vehicle}")
	rm -rf "${output_dir}"
	local generator_args=(--vehicle "${vehicle}" --output "${output_dir}")
	if [ -n "${flight_profile}" ]; then
		flight_profile=$(gate_resolve_path "${repo_root}" "${flight_profile}")
		generator_args+=(--flight-profile "${flight_profile}")
	fi
	"${repo_root}/tools/generate_vehicle_assets.py" "${generator_args[@]}"
	python3 "${repo_root}/tools/generate_vehicle_mesh.py" --vehicle "${vehicle}"
}
