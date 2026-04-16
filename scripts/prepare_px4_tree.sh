#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
REPO_ROOT=$(cd -- "${SCRIPT_DIR}/.." && pwd)
TV3_ROOT=$(cd -- "${REPO_ROOT}/.." && pwd)
VENDOR_DIR=$("${SCRIPT_DIR}/bootstrap_px4.sh")
PX4_REF="${PX4_REF:-v1.16.1}"
WORK_ROOT="${PX4_WORK_ROOT:-${TV3_ROOT}/.work}"
WORKTREE="${WORK_ROOT}/px4-tv3"

mkdir -p "${WORK_ROOT}"

git -C "${VENDOR_DIR}" worktree remove --force "${WORKTREE}" >/dev/null 2>&1 || true
rm -rf "${WORKTREE}"
git -C "${VENDOR_DIR}" worktree add --detach "${WORKTREE}" "${PX4_REF}" >/dev/null 2>&1
git -C "${WORKTREE}" submodule update --init --recursive --jobs 8 >/dev/null 2>&1

for patch in "${REPO_ROOT}"/patches/px4/*.patch; do
	[ -f "${patch}" ] || continue
	git -C "${WORKTREE}" apply --reject "${patch}" >/dev/null 2>&1 || true
done

python3 - "${REPO_ROOT}" "${WORKTREE}" <<'PY'
from pathlib import Path
import sys

repo_root = Path(sys.argv[1])
worktree = Path(sys.argv[2])
patch = (repo_root / "patches/px4/0001-rocket-control-allocation.patch").read_text()
module_yaml = worktree / "src/modules/control_allocator/module.yaml"
text = module_yaml.read_text()

text = text.replace("                15: Spacecraft 3D\n            default: 0\n", "                15: Spacecraft 3D\n                16: Rocket\n            default: 0\n", 1)

param_marker = "+        # rocket TVC\n"
rocket_marker = "+            16: # Rocket\n"
module_patch = patch.split("diff --git a/src/modules/control_allocator/module.yaml b/src/modules/control_allocator/module.yaml", 1)[1]
module_patch = module_patch.split("--- a/src/modules/control_allocator/VehicleActuatorEffectiveness/apogee_f10_mass_curve.hpp", 1)[0]
param_block_raw = module_patch.split(param_marker, 1)[1].split(rocket_marker, 1)[0]
rocket_block_raw = module_patch.split(rocket_marker, 1)[1]

def strip_plus(block: str) -> str:
	lines = []
	for line in block.splitlines():
		if line.startswith("+"):
			lines.append(line[1:])
	return "\n".join(lines)

param_block = strip_plus(param_block_raw).rstrip("\n")
rocket_block = strip_plus(rocket_block_raw).rstrip("\n")

if "# rocket TVC" not in text:
	insert_at = text.index("        # Tilts\n")
	text = text[:insert_at] + param_block + "\n\n" + text[insert_at:]

if "16: # Rocket" not in text:
	text = text.rstrip("\n") + "\n\n" + rocket_block + "\n"

module_yaml.write_text(text)

cmake_lists = worktree / "CMakeLists.txt"
cmake_text = cmake_lists.read_text()
cmake_text = cmake_text.replace("set(CMAKE_CXX_STANDARD 14)", "set(CMAKE_CXX_STANDARD 17)", 1)
cmake_lists.write_text(cmake_text)

common_flags = worktree / "cmake/px4_add_common_flags.cmake"
common_flags_text = common_flags.read_text()
common_flags_text = common_flags_text.replace("-Wdouble-promotion", "-Wno-double-promotion")
common_flags_text = common_flags_text.replace("-Werror", "-Wno-error")
common_flags.write_text(common_flags_text)
PY

if find "${WORKTREE}" -name '*.rej' -not -name 'module.yaml.rej' -print -quit | grep -q .; then
	echo "unexpected patch rejects remain in ${WORKTREE}" >&2
	exit 1
fi

if [ -d "${REPO_ROOT}/overlay/ROMFS/init.d-posix" ]; then
	rsync -a "${REPO_ROOT}/overlay/ROMFS/init.d-posix/" "${WORKTREE}/ROMFS/px4fmu_common/init.d-posix/" >/dev/null 2>&1
fi

printf '%s\n' "${WORKTREE}"
