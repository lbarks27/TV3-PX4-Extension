"""Vehicle-frame geometry helpers derived from manifests (no visualization dependencies)."""

from __future__ import annotations

from typing import Sequence

from tools.manifest_io import engines_from_manifest
from tools.tv3_control_allocator import engines_from_vehicle
from tools.tv3_motor_catalog import load_motor_catalog, resolve_motor_id, thrust_basis_from_manifest

DEFAULT_LANDER_BODY_SIZE_M = (0.65, 0.25, 0.25)


def vec3(values: Sequence[float], default: tuple[float, float, float] = (0.0, 0.0, 0.0)) -> tuple[float, float, float]:
    if not values or len(values) < 3:
        return default
    return float(values[0]), float(values[1]), float(values[2])


def link_pose_xyz(link: dict) -> tuple[float, float, float]:
    pose = link.get("pose_m", [0.0, 0.0, 0.0])
    if len(pose) >= 3:
        return vec3(pose[:3])
    return (0.0, 0.0, 0.0)


def link_world_com(link: dict) -> tuple[float, float, float]:
    origin = link_pose_xyz(link)
    com = vec3(link.get("com_m", [0.0, 0.0, 0.0]))
    return origin[0] + com[0], origin[1] + com[1], origin[2] + com[2]


def body_size_m(manifest: dict, override: Sequence[float] | None) -> tuple[float, float, float]:
    if override is not None:
        return vec3(override, DEFAULT_LANDER_BODY_SIZE_M)

    for link in manifest.get("physical_model", {}).get("links", []) or []:
        if link.get("id") == "body" and "size_m" in link:
            return vec3(link["size_m"], DEFAULT_LANDER_BODY_SIZE_M)

    if manifest.get("name") == "tv3_lander_v1":
        return DEFAULT_LANDER_BODY_SIZE_M
    return (0.35, 0.12, 0.12)


def extent_points(manifest: dict, body_size: Sequence[float]) -> list[tuple[float, float, float]]:
    points: list[tuple[float, float, float]] = [(0.0, 0.0, 0.0)]

    body = manifest["vehicle"]
    points.append((body["body_com_x_m"], 0.0, 0.0))
    points.append((body["motor_com_x_m"], 0.0, 0.0))

    bx, by, bz = body["body_com_x_m"], 0.0, 0.0
    sx, sy, sz = vec3(body_size)
    points.extend(
        [
            (bx - sx / 2.0, by - sy / 2.0, bz - sz / 2.0),
            (bx + sx / 2.0, by + sy / 2.0, bz + sz / 2.0),
        ]
    )

    for link in manifest.get("physical_model", {}).get("links", []) or []:
        points.append(link_pose_xyz(link))
        points.append(link_world_com(link))

    for engine in engines_from_manifest(manifest):
        points.append(vec3(engine["position_m"]))

    for joint in manifest.get("physical_model", {}).get("joints", []) or []:
        points.append(vec3(joint.get("origin_m", [0.0, 0.0, 0.0])))

    return points


def motor_label(manifest: dict, engine: dict) -> str:
    motor_selection = manifest.get("motor_selection", {})
    catalog_source = motor_selection.get("catalog_source")
    motor_id = resolve_motor_id(engine, motor_selection)
    if catalog_source:
        entry = load_motor_catalog(str(catalog_source)).get(motor_id)
        if entry is not None:
            return entry.label
    return motor_id or "unassigned"


def summary_text(manifest: dict, *, build_stage: int = 3) -> str:
    body = manifest["vehicle"]
    engines = engines_from_manifest(manifest)
    frame = manifest.get("physical_model", {}).get("reference_frame", {})
    stage_labels = {
        1: "stage 1: thrust ref only",
        2: "stage 2: + primary axis (mount->origin)",
        3: "stage 3: + secondary axis",
    }
    lines = [
        f"{manifest.get('name', 'vehicle')} — vehicle frame",
        f"axis build: {stage_labels.get(build_stage, f'stage {build_stage}')}",
        f"origin: {frame.get('origin', 'nozzle_exit_center')}",
        f"body mass: {body['body_mass_kg']} kg @ x={body['body_com_x_m']} m",
        f"motor wet mass: {body['motor_loaded_mass_kg']} kg x {len(engines)}",
        f"splay max: {body['tvc_max_deg']} deg, slew: {body['tvc_slew_dps']} deg/s",
        f"allocator ref thrust: {body['ca_reference_thrust_n']} N",
    ]
    motor_selection = manifest.get("motor_selection", {})
    basis = thrust_basis_from_manifest(motor_selection)
    if motor_selection.get("catalog_source"):
        lines.append(f"motor catalog: {motor_selection['catalog_source']} ({basis})")
    for engine in engines:
        pos = vec3(engine["position_m"])
        engine_models = engines_from_vehicle(manifest)
        thrust_n = next(
            (model.thrust_n for model in engine_models if model.position_m == tuple(engine["position_m"])),
            body["ca_reference_thrust_n"],
        )
        lines.append(
            f"  {engine.get('id', 'engine')}: {motor_label(manifest, engine)} "
            f"@ {thrust_n:.1f} N  pos=({pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f})"
        )
    return "\n".join(lines)
