#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
REPO_ROOT=$(cd -- "${SCRIPT_DIR}/.." && pwd)
TV3_ROOT=$(cd -- "${REPO_ROOT}/.." && pwd)
VENDOR_DIR=$("${SCRIPT_DIR}/bootstrap_px4.sh")
PX4_REF="${PX4_REF:-v1.16.1}"
WORK_ROOT="${PX4_WORK_ROOT:-${TV3_ROOT}/.work}"
WORKTREE="${WORK_ROOT}/px4-tv3"
VEHICLE_CONFIG="${TV3_VEHICLE_CONFIG:-${REPO_ROOT}/config/vehicles/tv3_v1.json}"
FLIGHT_PROFILE="${TV3_FLIGHT_PROFILE:-}"
if [[ "${VEHICLE_CONFIG}" != /* ]]; then
	VEHICLE_CONFIG="${REPO_ROOT}/${VEHICLE_CONFIG}"
fi
if [ -n "${FLIGHT_PROFILE}" ] && [[ "${FLIGHT_PROFILE}" != /* ]]; then
	FLIGHT_PROFILE="${REPO_ROOT}/${FLIGHT_PROFILE}"
fi

mkdir -p "${WORK_ROOT}"

if [ "${TV3_REUSE_PX4_WORKTREE:-0}" = "1" ] && [ -x "${WORKTREE}/build/px4_sitl_default/bin/px4" ]; then
	python3 - "${REPO_ROOT}" "${WORKTREE}" "${VEHICLE_CONFIG}" "${FLIGHT_PROFILE}" <<'PY'
from pathlib import Path
import subprocess
import sys

repo_root = Path(sys.argv[1])
worktree = Path(sys.argv[2])
vehicle_config = Path(sys.argv[3])
flight_profile = Path(sys.argv[4]) if sys.argv[4] else None
generated_assets = repo_root / "build/barebones"
generator_args = [
	sys.executable,
	str(repo_root / "tools/generate_vehicle_assets.py"),
	"--vehicle",
	str(vehicle_config),
	"--output",
	str(generated_assets),
]
if flight_profile is not None:
	generator_args.extend(["--flight-profile", str(flight_profile)])
subprocess.run(generator_args, check=True, stdout=subprocess.DEVNULL)
PY

	if [ -d "${REPO_ROOT}/overlay/ROMFS/init.d-posix" ]; then
		rsync -a "${REPO_ROOT}/overlay/ROMFS/init.d-posix/" "${WORKTREE}/ROMFS/px4fmu_common/init.d-posix/" >/dev/null 2>&1
	fi

	GENERATED_PARAM_FILE=$(find "${REPO_ROOT}/build/barebones/runtime/fs/microsd/tv3/airframes" -name '*.params' -print -quit 2>/dev/null || true)
	if [ -n "${GENERATED_PARAM_FILE}" ]; then
		python3 - "${GENERATED_PARAM_FILE}" "${WORKTREE}/ROMFS/px4fmu_common/init.d-posix/airframes/tv3_common.post" <<'PY'
from pathlib import Path
import sys

post_path = Path(sys.argv[2])

lines = post_path.read_text().splitlines()
lines = [line for line in lines if line.strip() != "tv3_guidance start"]

insert_after = "tv3_state_machine start"
try:
    index = lines.index(insert_after) + 1
except ValueError:
    index = len(lines)
lines.insert(index, "tv3_guidance start")

post_path.write_text("\n".join(lines) + "\n")
PY

		python3 - "${GENERATED_PARAM_FILE}" "${WORKTREE}/ROMFS/px4fmu_common/init.d-posix/airframes/tv3_common.inc" <<'PY'
from pathlib import Path
import sys

params = Path(sys.argv[1])
target = Path(sys.argv[2])
lines = [
	"param set-default CA_ROTOR_COUNT 0",
	"param set-default CA_SV_CS_COUNT 0",
]
for raw_line in params.read_text().splitlines():
	fields = raw_line.split("\t")
	if len(fields) >= 4:
		lines.append(f"param set-default {fields[2]} {fields[3]}")
target.write_text("\n".join(lines) + "\n")
PY
	fi

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
	case "$(basename "${patch}")" in
	0001-tv3-control-allocation.patch) continue ;;
	esac
	git -C "${WORKTREE}" apply --reject "${patch}" >/dev/null 2>&1 || true
done

python3 - "${REPO_ROOT}" "${WORKTREE}" "${VEHICLE_CONFIG}" "${FLIGHT_PROFILE}" <<'PY'
from pathlib import Path
import subprocess
import sys

repo_root = Path(sys.argv[1])
worktree = Path(sys.argv[2])
vehicle_config = Path(sys.argv[3])
flight_profile = Path(sys.argv[4]) if sys.argv[4] else None

def write_text_if_changed(path: Path, text: str) -> None:
	current = path.read_text() if path.exists() else None
	if current != text:
		path.parent.mkdir(parents=True, exist_ok=True)
		path.write_text(text)

commander = worktree / "src/modules/commander/Commander.cpp"
commander_text = commander.read_text()
if "case 31010:" not in commander_text:
	commander_text = commander_text.replace(
		"\tcase vehicle_command_s::VEHICLE_CMD_REQUEST_CAMERA_INFORMATION:\n"
		"\t\t/* ignore commands that are handled by other parts of the system */\n",
		"\tcase vehicle_command_s::VEHICLE_CMD_REQUEST_CAMERA_INFORMATION:\n"
		"\tcase 31010: // MAV_CMD_USER_1, handled by tv3_mode when enabled\n"
		"\t\t/* ignore commands that are handled by other parts of the system */\n",
		1,
	)
	write_text_if_changed(commander, commander_text)

mode_management = worktree / "src/modules/commander/ModeManagement.cpp"
mode_management_text = mode_management.read_text()
if "RK_ENABLE" not in mode_management_text:
	if '#include <px4_platform_common/events.h>' in mode_management_text:
		mode_management_text = mode_management_text.replace(
			"#include <px4_platform_common/events.h>\n",
			"#include <lib/parameters/param.h>\n"
			"#include <px4_platform_common/events.h>\n",
			1,
		)
	mode_management_text = mode_management_text.replace(
		"\t\t}\n"
		"\t}\n"
		"}\n\n"
		"#endif /* CONSTRAINED_FLASH */",
		"\t\t}\n"
		"\t}\n\n"
		"\t// TV3 uses tv3_state_machine for launch sequencing. Hide standard PX4 selectable\n"
		"\t// modes from GCS menus when the ascent manager is enabled (RK_ENABLE=1).\n"
		"\tparam_t rk_enable = param_find(\"RK_ENABLE\");\n\n"
		"\tif (rk_enable != PARAM_INVALID) {\n"
		"\t\tint32_t enabled = 0;\n\n"
		"\t\tif (param_get(rk_enable, &enabled) == 0 && enabled > 0) {\n"
		"\t\t\tvalid_nav_state_mask = (1u << vehicle_status_s::NAVIGATION_STATE_MANUAL);\n"
		"\t\t\tcan_set_nav_state_mask = 0;\n"
		"\t\t}\n"
		"\t}\n"
		"}\n\n"
		"#endif /* CONSTRAINED_FLASH */",
		1,
	)
	write_text_if_changed(mode_management, mode_management_text)

cmake_lists = worktree / "CMakeLists.txt"
cmake_text = cmake_lists.read_text()
cmake_text = cmake_text.replace("set(CMAKE_CXX_STANDARD 14)", "set(CMAKE_CXX_STANDARD 17)", 1)
cmake_lists.write_text(cmake_text)

common_flags = worktree / "cmake/px4_add_common_flags.cmake"
common_flags_text = common_flags.read_text()
common_flags_text = common_flags_text.replace("-Wdouble-promotion", "-Wno-double-promotion")
common_flags_text = common_flags_text.replace("-Werror", "-Wno-error")
common_flags.write_text(common_flags_text)

sitl_board = worktree / "boards/px4/sitl/default.px4board"
sitl_board_text = sitl_board.read_text()
for disabled_module in (
	"CONFIG_MODULES_SIMULATION_GZ_MSGS=y\n",
	"CONFIG_MODULES_SIMULATION_GZ_BRIDGE=y\n",
	"CONFIG_MODULES_SIMULATION_GZ_PLUGINS=y\n",
):
	sitl_board_text = sitl_board_text.replace(disabled_module, "")
if "CONFIG_MODULES_INTERNAL_COMBUSTION_ENGINE_CONTROL=y" not in sitl_board_text:
	sitl_board_text = sitl_board_text.replace(
		"CONFIG_MODULES_CONTROL_ALLOCATOR=y\n",
		"CONFIG_MODULES_CONTROL_ALLOCATOR=y\nCONFIG_MODULES_INTERNAL_COMBUSTION_ENGINE_CONTROL=y\n",
		1,
	)
	write_text_if_changed(sitl_board, sitl_board_text)

generated_assets = repo_root / "build/barebones"
generator_args = [
	sys.executable,
	str(repo_root / "tools/generate_vehicle_assets.py"),
	"--vehicle",
	str(vehicle_config),
	"--output",
	str(generated_assets),
]
if flight_profile is not None:
	generator_args.extend(["--flight-profile", str(flight_profile)])
subprocess.run(generator_args, check=True, stdout=subprocess.DEVNULL)
PY

if find "${WORKTREE}" -name '*.rej' -not -name 'module.yaml.rej' -print -quit | grep -q .; then
	echo "unexpected patch rejects remain in ${WORKTREE}" >&2
	exit 1
fi

if [ -d "${REPO_ROOT}/overlay/ROMFS/init.d-posix" ]; then
	rsync -a "${REPO_ROOT}/overlay/ROMFS/init.d-posix/" "${WORKTREE}/ROMFS/px4fmu_common/init.d-posix/" >/dev/null 2>&1
fi

GENERATED_PARAM_FILE=$(find "${REPO_ROOT}/build/barebones/runtime/fs/microsd/tv3/airframes" -name '*.params' -print -quit 2>/dev/null || true)
if [ -n "${GENERATED_PARAM_FILE}" ]; then
	python3 - "${GENERATED_PARAM_FILE}" "${WORKTREE}/ROMFS/px4fmu_common/init.d-posix/airframes/tv3_common.post" <<'PY'
from pathlib import Path
import sys

post_path = Path(sys.argv[2])

lines = post_path.read_text().splitlines()
lines = [line for line in lines if line.strip() != "tv3_guidance start"]

insert_after = "tv3_state_machine start"
try:
    index = lines.index(insert_after) + 1
except ValueError:
    index = len(lines)
lines.insert(index, "tv3_guidance start")

post_path.write_text("\n".join(lines) + "\n")
PY

	python3 - "${GENERATED_PARAM_FILE}" "${WORKTREE}/ROMFS/px4fmu_common/init.d-posix/airframes/tv3_common.inc" <<'PY'
from pathlib import Path
import sys

params = Path(sys.argv[1])
target = Path(sys.argv[2])
lines = [
	"param set-default CA_ROTOR_COUNT 0",
	"param set-default CA_SV_CS_COUNT 0",
]
for raw_line in params.read_text().splitlines():
	fields = raw_line.split("\t")
	if len(fields) >= 4:
		lines.append(f"param set-default {fields[2]} {fields[3]}")
target.write_text("\n".join(lines) + "\n")
PY
fi

printf '%s\n' "${WORKTREE}"
