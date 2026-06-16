#!/usr/bin/env bash

set -euo pipefail

# Hawkeye fixed-wing slot: mirror_axes=0 (multicopter slots mirror quarter meshes).
# TV3 SIH reports MAV_TYPE_ROCKET in firmware, but Hawkeye 0.3.x falls back to quad
# for unknown types. run_sitl_sih.sh sets MAV_TYPE_FIXED_WING during visual runs so
# Hawkeye keeps this px4_fixed_wing.obj override once MAVLink connects.
# TV3_HAWKEYE_MESH_ROT presets:
#   tv3_hover    - default: Fusion +Z long axis -> OBJ +Y for upright SIH hover in Hawkeye -fw
#   tv3_fw       - legacy body-frame mapping (usually appears on its side)
#   tv3_z_up     - keep Fusion +Z; upright before MAVLink connects, on its side once SIH runs
#   tv3_fw_flip  - same as tv3_fw but with -90 deg Y instead of +90 deg
#   identity     - scale/center only
#   custom       - set TV3_HAWKEYE_MESH_MATRIX to nine space-separated row-major values

SRC_OBJ="${TV3_HAWKEYE_MESH:-/Users/liambarkley/Downloads/TV3+Hub+Test+1.obj}"
SCALE="${TV3_HAWKEYE_MESH_SCALE:-0.001}"
CENTER="${TV3_HAWKEYE_MESH_CENTER:-1}"
ROT="${TV3_HAWKEYE_MESH_ROT:-tv3_hover}"
HAWKEYE_MODEL="${TV3_HAWKEYE_MODEL:-px4_fixed_wing}"

if [ ! -f "${SRC_OBJ}" ]; then
	echo "Hawkeye mesh not found: ${SRC_OBJ}" >&2
	echo "Set TV3_HAWKEYE_MESH to an OBJ export (with its .mtl alongside the source file)." >&2
	exit 1
fi

SRC_DIR="$(cd "$(dirname "${SRC_OBJ}")" && pwd)"
MTL_NAME="$(awk '/^mtllib / {print $2; exit}' "${SRC_OBJ}" | tr -d '\r')"
if [ -z "${MTL_NAME}" ]; then
	echo "OBJ has no mtllib directive: ${SRC_OBJ}" >&2
	exit 1
fi
SRC_MTL="${SRC_DIR}/${MTL_NAME}"
if [ ! -f "${SRC_MTL}" ]; then
	echo "MTL not found next to OBJ: ${SRC_MTL}" >&2
	exit 1
fi

case "$(uname -s)" in
Darwin)
	HAWKEYE_MODELS="${HOME}/Library/Application Support/hawkeye/models"
	;;
Linux)
	HAWKEYE_MODELS="${XDG_DATA_HOME:-${HOME}/.local/share}/hawkeye/models"
	;;
*)
	echo "Unsupported platform for Hawkeye model install." >&2
	exit 1
	;;
esac

mkdir -p "${HAWKEYE_MODELS}"

DST_OBJ="${HAWKEYE_MODELS}/${HAWKEYE_MODEL}.obj"
DST_MTL="${HAWKEYE_MODELS}/${HAWKEYE_MODEL}.mtl"

python3 - "${SRC_OBJ}" "${DST_OBJ}" "${SCALE}" "${CENTER}" "${ROT}" "${TV3_HAWKEYE_MESH_MATRIX:-}" <<'PY'
import math
import sys
from pathlib import Path


def mat3(rows):
    return [list(map(float, row)) for row in rows]


def multiply(a, b):
    out = [[0.0, 0.0, 0.0] for _ in range(3)]
    for i in range(3):
        for j in range(3):
            out[i][j] = sum(a[i][k] * b[k][j] for k in range(3))
    return out


def rot_x(deg):
    r = math.radians(deg)
    c, s = math.cos(r), math.sin(r)
    return mat3([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def rot_y(deg):
    r = math.radians(deg)
    c, s = math.cos(r), math.sin(r)
    return mat3([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def rot_z(deg):
    r = math.radians(deg)
    c, s = math.cos(r), math.sin(r)
    return mat3([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def invert(m):
    a, b, c = m[0]
    d, e, f = m[1]
    g, h, i = m[2]
    det = (
        a * (e * i - f * h)
        - b * (d * i - f * g)
        + c * (d * h - e * g)
    )
    if abs(det) < 1e-12:
        raise SystemExit("rotation matrix is not invertible")
    inv_det = 1.0 / det
    return mat3(
        [
            [(e * i - f * h) * inv_det, (c * h - b * i) * inv_det, (b * f - c * e) * inv_det],
            [(f * g - d * i) * inv_det, (a * i - c * g) * inv_det, (c * d - a * f) * inv_det],
            [(d * h - e * g) * inv_det, (b * g - a * h) * inv_det, (a * e - b * d) * inv_det],
        ]
    )


def apply(m, x, y, z):
    return (
        m[0][0] * x + m[0][1] * y + m[0][2] * z,
        m[1][0] * x + m[1][1] * y + m[1][2] * z,
        m[2][0] * x + m[2][1] * y + m[2][2] * z,
    )


def resolve_rotation(preset: str, custom: str):
    if preset == "identity":
        return mat3([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]])
    if preset == "custom":
        values = [float(v) for v in custom.split()]
        if len(values) != 9:
            raise SystemExit("TV3_HAWKEYE_MESH_MATRIX must contain 9 values")
        return [values[0:3], values[3:6], values[6:9]]
    scale_fix = 1.0 / 1.15
    s = mat3(
        [
            [scale_fix, 0.0, 0.0],
            [0.0, scale_fix, 0.0],
            [0.0, 0.0, scale_fix],
        ]
    )

    if preset == "tv3_hover":
        # Fusion export: long axis on +Z. SIH hover uses a 90 deg body-Y quaternion and Hawkeye
        # -fw applies Rx(+90) then Ry(+180) before that attitude. Map CAD +Z to OBJ +Y.
        return multiply(s, rot_x(-90.0))
    if preset in {"tv3_fw", "tv3_fw_flip"}:
        # Legacy mapping kept for manual experiments.
        cad_to_body = rot_y(90.0 if preset == "tv3_fw" else -90.0)
        hawkeye_base = multiply(rot_x(90.0), rot_y(180.0))
        body_to_obj = invert(hawkeye_base)
        return multiply(multiply(s, body_to_obj), cad_to_body)
    if preset == "tv3_z_up":
        return s
    raise SystemExit(f"Unknown TV3_HAWKEYE_MESH_ROT preset: {preset}")


src = Path(sys.argv[1])
dst = Path(sys.argv[2])
scale = float(sys.argv[3])
center = sys.argv[4] == "1"
preset = sys.argv[5]
custom = sys.argv[6]
rotation = resolve_rotation(preset, custom)

positions = []
with src.open(newline="") as handle:
    for raw in handle:
        line = raw.rstrip("\r\n")
        if line.startswith("v "):
            parts = line.split()
            x, y, z = float(parts[1]) * scale, float(parts[2]) * scale, float(parts[3]) * scale
            positions.append((x, y, z))

if not positions:
    raise SystemExit(f"No vertices found in {src}")

mins = [min(p[i] for p in positions) for i in range(3)]
maxs = [max(p[i] for p in positions) for i in range(3)]
offset = [(mins[i] + maxs[i]) * 0.5 for i in range(3)] if center else [0.0, 0.0, 0.0]

size = [maxs[i] - mins[i] for i in range(3)]
print(f"source vertices: {len(positions)}")
print(f"scaled size (m): {size[0]:.3f} x {size[1]:.3f} x {size[2]:.3f}")
print(f"rotation preset: {preset}")

vertex_idx = 0
normal_idx = 0
with src.open(newline="") as handle, dst.open("w", encoding="utf-8", newline="\n") as out:
    for raw in handle:
        line = raw.rstrip("\r\n")
        if line.startswith("mtllib "):
            out.write(f"mtllib {dst.stem}.mtl\n")
            continue
        if line.startswith("v "):
            x, y, z = positions[vertex_idx]
            vertex_idx += 1
            x -= offset[0]
            y -= offset[1]
            z -= offset[2]
            x, y, z = apply(rotation, x, y, z)
            out.write(f"v {x:.6f} {y:.6f} {z:.6f}\n")
            continue
        if line.startswith("vn "):
            parts = line.split()
            nx, ny, nz = float(parts[1]), float(parts[2]), float(parts[3])
            nx, ny, nz = apply(rotation, nx, ny, nz)
            length = math.sqrt(nx * nx + ny * ny + nz * nz)
            if length > 1e-12:
                nx /= length
                ny /= length
                nz /= length
            out.write(f"vn {nx:.6f} {ny:.6f} {nz:.6f}\n")
            normal_idx += 1
            continue
        out.write(line + "\n")

if vertex_idx != len(positions):
    raise SystemExit("vertex count mismatch while rewriting OBJ")
PY

cp "${SRC_MTL}" "${DST_MTL}"

echo "Installed Hawkeye TV3 mesh override:"
echo "  source:   ${SRC_OBJ}"
echo "  target:   ${DST_OBJ}"
echo "  scale:    ${SCALE} (Fusion mm -> m by default)"
echo "  rotation: ${ROT}"
echo "Launch with ./scripts/run_hawkeye.sh (uses -fw -> ${HAWKEYE_MODEL}.obj, no mirror duplicate)."