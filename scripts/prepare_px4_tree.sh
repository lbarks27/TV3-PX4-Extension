#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
REPO_ROOT=$(cd -- "${SCRIPT_DIR}/.." && pwd)
TV3_ROOT=$(cd -- "${REPO_ROOT}/.." && pwd)
VENDOR_DIR=$("${SCRIPT_DIR}/bootstrap_px4.sh")
PX4_REF="${PX4_REF:-v1.16.1}"
WORK_ROOT="${PX4_WORK_ROOT:-${TV3_ROOT}/.work}"
WORKTREE="${WORK_ROOT}/px4-tv3"
ROMFS_DIR="${WORKTREE}/ROMFS/px4fmu_common/init.d-posix"
OVERLAY_DIR="${REPO_ROOT}/overlay/ROMFS/init.d-posix"

mkdir -p "${WORK_ROOT}"

if [ "${TV3_REUSE_PX4_WORKTREE:-0}" = "1" ] && [ -d "${WORKTREE}/.git" ]; then
	printf '%s\n' "${WORKTREE}"
	exit 0
fi

git -C "${VENDOR_DIR}" worktree prune >/dev/null 2>&1 || true
git -C "${VENDOR_DIR}" worktree remove --force "${WORKTREE}" >/dev/null 2>&1 || true
rm -rf "${WORKTREE}"
git -C "${VENDOR_DIR}" worktree add --detach "${WORKTREE}" "${PX4_REF}" >/dev/null 2>&1
git -C "${WORKTREE}" submodule update --init --recursive --jobs 8 -- \
	. \
	':!Tools/simulation/gz' \
	':!Tools/simulation/gazebo-classic/sitl_gazebo-classic' \
	':!Tools/simulation/flightgear/flightgear_bridge' \
	':!Tools/simulation/jmavsim/jMAVSim' \
	':!platforms/nuttx/NuttX/apps' \
	':!platforms/nuttx/NuttX/nuttx' >/dev/null 2>&1

for patch in "${REPO_ROOT}"/patches/px4/*.patch; do
	[ -f "${patch}" ] || continue
	git -C "${WORKTREE}" apply --reject "${patch}" >/dev/null 2>&1 || true
done

python3 - "${WORKTREE}" <<'PY'
from pathlib import Path
import sys

worktree = Path(sys.argv[1])
mode_mgmt = worktree / "src/modules/commander/ModeManagement.cpp"
text = mode_mgmt.read_text()
marker = '#include <px4_platform_common/events.h>'

if 'RK_ENABLE' not in text:
	if marker in text and '#include <lib/parameters/param.h>' not in text:
		text = text.replace(marker, '#include <lib/parameters/param.h>\n' + marker, 1)

	tv3_block = """
\t// TV3 uses tv3_state_machine for launch sequencing. Hide standard PX4 selectable
\t// modes from GCS menus when the TV3 manager is enabled (RK_ENABLE=1).
\tparam_t rk_enable = param_find("RK_ENABLE");

\tif (rk_enable != PARAM_INVALID) {
\t\tint32_t enabled = 0;

\t\tif (param_get(rk_enable, &enabled) == 0 && enabled > 0) {
\t\t\tvalid_nav_state_mask = (1u << vehicle_status_s::NAVIGATION_STATE_MANUAL);
\t\t\tcan_set_nav_state_mask = 0;
\t\t}
\t}
"""

	anchor = "\t\t}\n\t}\n}\n\n#endif /* CONSTRAINED_FLASH */"
	if anchor in text:
		text = text.replace(anchor, "\t\t}\n\t}" + tv3_block + "\n}\n\n#endif /* CONSTRAINED_FLASH */", 1)
		mode_mgmt.write_text(text)
PY

if [ -d "${OVERLAY_DIR}" ]; then
	mkdir -p "${ROMFS_DIR}/airframes"
	rsync -a "${OVERLAY_DIR}/" "${ROMFS_DIR}/" >/dev/null 2>&1
fi

python3 - "${ROMFS_DIR}" <<'PY'
from pathlib import Path
import sys

romfs = Path(sys.argv[1])
init_cmake = romfs / "CMakeLists.txt"
airframes_cmake = romfs / "airframes" / "CMakeLists.txt"
tv3_romfs = (
	"tv3_common.inc",
	"tv3_common.post",
	"11002_tv3_lander",
	"11002_tv3_lander.post",
)

if init_cmake.exists():
	text = init_cmake.read_text()
	if "rc.tv3_defaults" not in text:
		marker = "px4_add_romfs_files("
		insert = "\trc.tv3_defaults\n"
		idx = text.find(marker)
		if idx != -1:
			close = text.find(")", idx)
			text = text[:close] + insert + text[close:]
			init_cmake.write_text(text)

if airframes_cmake.exists():
	text = airframes_cmake.read_text()
	missing = [name for name in tv3_romfs if name not in text]
	if missing:
		block = "\npx4_add_romfs_files(\n" + "".join(f"\t{name}\n" for name in missing) + ")\n"
		airframes_cmake.write_text(text.rstrip() + block)
PY

printf '%s\n' "${WORKTREE}"
