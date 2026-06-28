#!/usr/bin/env python3
"""Shared TV3 constrained control allocator for host validation and guidance checks.

Mirrors the PX4 ``ActuatorEffectivenessTV3`` small-angle TVC linearization for
torque authority and the SIH plant splay/pitch/yaw thrust model for net thrust.
"""

from __future__ import annotations

import itertools
import json
import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Iterable, Sequence

REASON_NONE = ""
REASON_NO_ENGINES = "no engines"
REASON_NO_ACTIVE_ENGINES = "no active engines"
REASON_THRUST_ENVELOPE = "net thrust outside splay envelope"
REASON_TORQUE_UNREACHABLE = "torque demand outside TVC authority"
REASON_INSUFFICIENT_THRUST_AFTER_FAULT = "insufficient active thrust after engine fault"

CONTROL_OK = 0
CONTROL_THRUST_ENVELOPE = 1
CONTROL_TORQUE_ENVELOPE = 2
CONTROL_NO_ACTIVE_ENGINES = 3


@dataclass(frozen=True)
class EngineGeometry:
    position_m: tuple[float, float, float]
    thrust_axis: tuple[float, float, float]
    roll_axis: tuple[float, float, float]
    yaw_axis: tuple[float, float, float]
    thrust_n: float
    roll_min_deg: float
    roll_max_deg: float
    yaw_min_deg: float
    yaw_max_deg: float
    splay_max_deg: float
    thrust_fraction: float = 1.0
    roll_trim: float = 0.0
    yaw_trim: float = 0.0


@dataclass(frozen=True)
class TorqueLimits:
    roll_nm: float = 0.0
    pitch_nm: float = 10.0
    yaw_nm: float = 10.0


@dataclass
class AllocationResult:
    reachable: bool
    reason: str = REASON_NONE
    control_unreachable_reason: int = CONTROL_OK
    saturated: bool = False
    score: float = math.inf
    torque_error_nm: float = math.inf
    thrust_error_n: float = math.inf
    achieved_torque_nm: tuple[float, float, float] = (0.0, 0.0, 0.0)
    achieved_thrust_n: float = 0.0
    unallocated_torque_nm: tuple[float, float, float] = (0.0, 0.0, 0.0)
    commands: tuple[tuple[float, float, float], ...] = ()
    full_thrust_n: float = 0.0
    min_splayed_thrust_n: float = 0.0


def dot(a: Sequence[float], b: Sequence[float]) -> float:
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def cross(a: Sequence[float], b: Sequence[float]) -> tuple[float, float, float]:
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def scale(a: Sequence[float], value: float) -> tuple[float, float, float]:
    return (a[0] * value, a[1] * value, a[2] * value)


def add(a: Sequence[float], b: Sequence[float]) -> tuple[float, float, float]:
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def sub(a: Sequence[float], b: Sequence[float]) -> tuple[float, float, float]:
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def norm(a: Sequence[float]) -> float:
    return math.sqrt(dot(a, a))


def normalize(a: Sequence[float], fallback: Sequence[float]) -> tuple[float, float, float]:
    length = norm(a)
    if length <= 1e-9:
        return tuple(fallback)
    return scale(a, 1.0 / length)


def constrain(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def load_manifest(path: Path | str) -> dict:
    return json.loads(Path(path).read_text())


BODY_FORWARD_AXIS = (1.0, 0.0, 0.0)


def mount_to_origin_axis(position_m: Sequence[float]) -> tuple[float, float, float]:
    """Unit vector from the engine mount toward the vehicle reference origin."""
    return normalize((-position_m[0], -position_m[1], -position_m[2]), BODY_FORWARD_AXIS)


def outward_radial_axis(position_m: Sequence[float]) -> tuple[float, float, float]:
    """Unit vector from the origin toward the mount (Y-Z ring placement)."""
    return normalize((position_m[0], position_m[1], position_m[2]), (0.0, 1.0, 0.0))


def roll_axis_perpendicular(
    thrust_axis: Sequence[float],
    yaw_axis: Sequence[float],
) -> tuple[float, float, float]:
    """Roll axis orthogonal to thrust and yaw; falls back through body +X when degenerate."""
    for candidate in (
        cross(thrust_axis, yaw_axis),
        cross(BODY_FORWARD_AXIS, thrust_axis),
        cross(BODY_FORWARD_AXIS, yaw_axis),
        cross(yaw_axis, thrust_axis),
    ):
        if norm(candidate) > 1e-6:
            return normalize(candidate, (0.0, -1.0, 0.0))
    return (0.0, -1.0, 0.0)


def gimbal_axes_from_mount(
    position_m: Sequence[float],
    thrust_axis: Sequence[float] | None = None,
) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
    """Return (roll_axis, yaw_axis) for a mount pose and nominal thrust direction."""
    thrust = (
        normalize(thrust_axis, outward_radial_axis(position_m))
        if thrust_axis is not None
        else outward_radial_axis(position_m)
    )
    yaw = mount_to_origin_axis(position_m)
    roll = roll_axis_perpendicular(thrust, yaw)
    return roll, yaw


def axes_close(
    actual: Sequence[float],
    expected: Sequence[float],
    *,
    tolerance: float = 0.02,
) -> bool:
    return norm(sub(actual, expected)) <= tolerance


def engines_from_vehicle(vehicle: dict) -> list[EngineGeometry]:
    from tools.tv3_engine_frame import build_engine_frame_axes
    from tools.tv3_motor_catalog import engine_thrust_n, load_motor_catalog

    body = vehicle["vehicle"]
    motor_selection = vehicle.get("motor_selection", {})
    catalog = (
        load_motor_catalog(str(motor_selection["catalog_source"]))
        if motor_selection.get("catalog_source")
        else None
    )
    engines = vehicle.get("propulsion", {}).get("engines")
    if not engines:
        engines = [
            {
                "position_m": [body["motor_com_x_m"], 0.0, 0.0],
                "thrust_axis": [1.0, 0.0, 0.0],
                "roll_axis": [0.0, -1.0, 0.0],
                "yaw_axis": [0.0, 0.0, -1.0],
                "thrust_fraction": 1.0,
                "gimbal": {
                    "roll_max_deg": body["tvc_max_deg"],
                    "yaw_max_deg": body["tvc_max_deg"],
                    "splay_max_deg": body["tvc_max_deg"],
                },
            }
        ]

    is_lander = vehicle.get("variant", {}).get("role") == "three_engine_lander"
    geometries: list[EngineGeometry] = []
    for engine in engines:
        gimbal = engine["gimbal"]
        position_m = tuple(engine["position_m"])
        if is_lander:
            frame = build_engine_frame_axes(position_m)
            thrust_axis = frame.thrust_axis
            roll_axis = frame.primary_axis
            yaw_axis = frame.secondary_axis
        else:
            thrust_axis = normalize(engine["thrust_axis"], outward_radial_axis(position_m))
            manifest_roll = engine.get("roll_axis", engine.get("pitch_axis"))
            manifest_yaw = engine.get("yaw_axis")
            if isinstance(manifest_roll, list) and isinstance(manifest_yaw, list):
                roll_axis = normalize(manifest_roll, (0.0, -1.0, 0.0))
                yaw_axis = normalize(manifest_yaw, (0.0, 0.0, -1.0))
            else:
                roll_axis, yaw_axis = gimbal_axes_from_mount(position_m, thrust_axis)
        roll_max_deg = float(gimbal.get("roll_max_deg", gimbal.get("pitch_max_deg", body["tvc_max_deg"])))
        yaw_max_deg = float(gimbal.get("yaw_max_deg", body["tvc_max_deg"]))
        geometries.append(
            EngineGeometry(
                position_m=position_m,
                thrust_axis=thrust_axis,
                roll_axis=roll_axis,
                yaw_axis=yaw_axis,
                thrust_n=engine_thrust_n(vehicle, engine, catalog=catalog),
                roll_min_deg=float(gimbal.get("roll_min_deg", gimbal.get("pitch_min_deg", -roll_max_deg))),
                roll_max_deg=roll_max_deg,
                yaw_min_deg=float(gimbal.get("yaw_min_deg", -yaw_max_deg)),
                yaw_max_deg=yaw_max_deg,
                splay_max_deg=gimbal.get("splay_max_deg", body["tvc_max_deg"]),
                thrust_fraction=engine.get("thrust_fraction", 1.0 / len(engines)),
                roll_trim=float(gimbal.get("roll_trim", gimbal.get("pitch_trim", 0.0))),
                yaw_trim=gimbal.get("yaw_trim", 0.0),
            )
        )
    return geometries


def vehicle_full_thrust_n(vehicle: dict) -> float:
    return sum(engine.thrust_n for engine in engines_from_vehicle(vehicle))


def torque_limits_from_vehicle(vehicle: dict) -> TorqueLimits:
    limits = vehicle["vehicle"].get("torque_limits_nm", {})
    return TorqueLimits(
        roll_nm=limits.get("roll", 0.0),
        pitch_nm=limits.get("pitch", 10.0),
        yaw_nm=limits.get("yaw", 10.0),
    )


def reference_thrust_from_vehicle(vehicle: dict) -> float:
    from tools.tv3_motor_catalog import allocator_thrust_fields_from_catalog

    catalog_fields = allocator_thrust_fields_from_catalog(vehicle)
    if catalog_fields is not None:
        return float(catalog_fields["ca_reference_thrust_n"])
    return float(vehicle["vehicle"]["ca_reference_thrust_n"])


def splay_max_deg_from_vehicle(vehicle: dict) -> float:
    throttle = vehicle.get("propulsion", {}).get("throttle", {})
    return float(throttle.get("max_splay_deg", vehicle["vehicle"]["tvc_max_deg"]))


def active_engine_mask(engine_count: int, active_mask: int | None = None) -> int:
    if active_mask is None:
        return (1 << engine_count) - 1 if engine_count > 0 else 0
    return active_mask & ((1 << engine_count) - 1)


def scaled_engines(
    engines: Sequence[EngineGeometry],
    *,
    active_mask: int | None = None,
    thrust_scales: Sequence[float] | None = None,
) -> list[EngineGeometry]:
    if not engines:
        return []

    mask = active_engine_mask(len(engines), active_mask)
    scales = list(thrust_scales) if thrust_scales is not None else [1.0] * len(engines)
    if len(scales) < len(engines):
        scales.extend([1.0] * (len(engines) - len(scales)))

    scaled: list[EngineGeometry] = []
    for index, engine in enumerate(engines):
        if not (mask & (1 << index)):
            continue
        thrust_scale = max(scales[index], 0.0)
        scaled.append(
            EngineGeometry(
                position_m=engine.position_m,
                thrust_axis=engine.thrust_axis,
                roll_axis=engine.roll_axis,
                yaw_axis=engine.yaw_axis,
                thrust_n=engine.thrust_n * thrust_scale,
                roll_min_deg=engine.roll_min_deg,
                roll_max_deg=engine.roll_max_deg,
                yaw_min_deg=engine.yaw_min_deg,
                yaw_max_deg=engine.yaw_max_deg,
                splay_max_deg=engine.splay_max_deg,
                thrust_fraction=engine.thrust_fraction,
                roll_trim=engine.roll_trim,
                yaw_trim=engine.yaw_trim,
            )
        )
    return scaled


def thrust_envelope(engines: Sequence[EngineGeometry]) -> tuple[float, float]:
    full_thrust = sum(plant_axial_thrust(engine, 0.0, 0.0) for engine in engines)
    min_splayed = max(0.0, sum(plant_axial_thrust(engine, 0.0, 90.0) for engine in engines))
    return full_thrust, min_splayed


def practical_thrust_floor(engines: Sequence[EngineGeometry]) -> float:
    """Minimum practical collective thrust using each engine's splay limit."""
    if not engines:
        return 0.0
    splay_max_deg = min(engine.splay_max_deg for engine in engines)
    return sum(plant_axial_thrust(engine, 0.0, splay_max_deg) for engine in engines)


def flight_effectiveness_torque(
    engine: EngineGeometry,
    *,
    gimbal_axis: Sequence[float],
    max_angle_deg: float,
    reference_thrust_n: float | None = None,
) -> tuple[float, float, float]:
    """Matches ActuatorEffectivenessTV3::computeTorque at full servo deflection."""
    thrust_axis = normalize(engine.thrust_axis, (1.0, 0.0, 0.0))
    axis = normalize(gimbal_axis, (0.0, -1.0, 0.0))
    thrust_scale = (reference_thrust_n or engine.thrust_n) * constrain(engine.thrust_fraction, 0.0, 1.0)
    max_angle_rad = math.radians(max_angle_deg)
    return scale(
        cross(engine.position_m, cross(axis, thrust_axis)),
        thrust_scale * max_angle_rad,
    )


def rotate_about_axis(
    vector: Sequence[float],
    axis: Sequence[float],
    angle_rad: float,
) -> tuple[float, float, float]:
    """Rotate *vector* about a unit *axis* (Rodrigues), matching SIH axis-angle thrust."""
    if abs(angle_rad) <= 1e-12:
        return tuple(vector)
    k = normalize(axis, (0.0, 0.0, 1.0))
    cos_a = math.cos(angle_rad)
    sin_a = math.sin(angle_rad)
    k_dot_v = dot(k, vector)
    return add(
        add(scale(vector, cos_a), scale(cross(k, vector), sin_a)),
        scale(k, k_dot_v * (1.0 - cos_a)),
    )


def coupled_yaw_axis(engine: EngineGeometry, roll_deg: float) -> tuple[float, float, float]:
    """Yaw hinge axis after roll: yaw is coupled to roll, roll is not coupled to yaw."""
    total_roll = roll_deg + engine.roll_trim
    if abs(total_roll) <= 1e-9:
        return engine.yaw_axis
    return rotate_about_axis(engine.yaw_axis, engine.roll_axis, math.radians(total_roll))


def plant_thrust_direction(
    engine: EngineGeometry,
    roll_deg: float,
    yaw_deg: float,
) -> tuple[float, float, float]:
    """Unit thrust direction: roll about fixed roll_axis, then yaw about roll-coupled yaw_axis."""
    reference = normalize(engine.thrust_axis, (1.0, 0.0, 0.0))
    total_roll = roll_deg + engine.roll_trim
    total_yaw = yaw_deg + engine.yaw_trim
    direction = reference
    if abs(total_roll) > 1e-9:
        direction = rotate_about_axis(direction, engine.roll_axis, math.radians(total_roll))
    if abs(total_yaw) > 1e-9:
        direction = rotate_about_axis(
            direction,
            coupled_yaw_axis(engine, roll_deg),
            math.radians(total_yaw),
        )
    return normalize(direction, reference)


def plant_force_vector(
    engine: EngineGeometry,
    roll_deg: float,
    yaw_deg: float,
) -> tuple[tuple[float, float, float], float]:
    """Returns body-frame thrust direction and full chamber magnitude after roll and secondary-axis yaw."""
    direction = plant_thrust_direction(engine, roll_deg, yaw_deg)
    return direction, engine.thrust_n


def plant_torque(
    engine: EngineGeometry,
    roll_deg: float,
    yaw_deg: float,
) -> tuple[float, float, float]:
    """Matches the SIH thrust-direction model used in tv3_sih."""
    direction, magnitude = plant_force_vector(engine, roll_deg, yaw_deg)
    force = scale(direction, magnitude)
    return cross(engine.position_m, force)


def plant_axial_thrust(engine: EngineGeometry, roll_deg: float, yaw_deg: float) -> float:
    direction, magnitude = plant_force_vector(engine, roll_deg, yaw_deg)
    return magnitude * direction[0]


def collective_throttle_yaw_deg(
    desired_net_thrust_n: float,
    engines: Sequence[EngineGeometry],
    *,
    roll_deg: float = 0.0,
    throttle_max_deg: float | None = None,
) -> float:
    """Solve identical secondary-axis yaw (splay) for collective throttle."""
    if not engines:
        return 0.0

    yaw_limit = throttle_max_deg
    if yaw_limit is None:
        yaw_limit = min(engine.yaw_max_deg for engine in engines)

    if desired_net_thrust_n <= 0.0:
        return min(90.0, yaw_limit)

    full_thrust = sum(plant_axial_thrust(engine, roll_deg, 0.0) for engine in engines)
    if full_thrust < 1e-3 or desired_net_thrust_n >= full_thrust - 1e-3:
        return 0.0

    low = 0.0
    high = yaw_limit
    for _ in range(24):
        mid = 0.5 * (low + high)
        net = sum(plant_axial_thrust(engine, roll_deg, mid) for engine in engines)
        if net > desired_net_thrust_n:
            low = mid
        else:
            high = mid

    return high


def collective_splay_deg(
    desired_net_thrust_n: float,
    total_chamber_thrust_n: float,
    splay_max_deg: float,
) -> float:
    """Backward-compatible alias: splay is secondary-axis collective yaw."""
    if total_chamber_thrust_n < 1e-3:
        return 0.0

    if desired_net_thrust_n <= 0.0:
        return min(90.0, splay_max_deg)
    if desired_net_thrust_n >= total_chamber_thrust_n - 1e-3:
        return 0.0
    ratio = max(0.0, min(1.0, desired_net_thrust_n / total_chamber_thrust_n))
    return max(0.0, min(splay_max_deg, math.degrees(math.acos(ratio))))


@dataclass(frozen=True)
class LmConfig:
    max_iter: int = 12
    torque_tol_nm: float = 0.15
    lambda0: float = 0.01
    thrust_weight: float = 1.0
    splay_weight: float = 0.1
    fd_eps: float = 0.01


@dataclass
class LmSolveResult:
    commands: tuple[tuple[float, float], ...] = ()
    residual_torque_nm: float = math.inf
    residual_thrust_n: float = math.inf
    cost: float = math.inf
    lambda_final: float = 0.0
    iterations_used: int = 0
    converged: bool = False
    demand_saturated: bool = False


def clamp_torque_demand(
    desired_torque_nm: Sequence[float],
    torque_limits: TorqueLimits | None = None,
) -> tuple[tuple[float, float, float], bool]:
    limits = torque_limits or TorqueLimits()
    clamped = (
        constrain(desired_torque_nm[0], -limits.roll_nm, limits.roll_nm),
        constrain(desired_torque_nm[1], -limits.pitch_nm, limits.pitch_nm),
        constrain(desired_torque_nm[2], -limits.yaw_nm, limits.yaw_nm),
    )
    saturated = any(
        abs(clamped[index] - desired_torque_nm[index]) > 1e-6 for index in range(3)
    )
    return clamped, saturated


def wrench_demand_feasible(
    engines: Sequence[EngineGeometry],
    desired_torque_nm: Sequence[float],
    desired_thrust_n: float,
    *,
    torque_limits: TorqueLimits | None = None,
) -> bool:
    if not engines:
        return False
    full_thrust, min_splayed = thrust_envelope(engines)
    min_thrust = max(min_splayed, practical_thrust_floor(engines))
    if desired_thrust_n > full_thrust + 1e-3 or desired_thrust_n < min_thrust - 1e-3:
        return False
    limits = torque_limits or TorqueLimits()
    roll, pitch, yaw = desired_torque_nm
    return (
        abs(roll) <= limits.roll_nm + 1e-6
        and abs(pitch) <= limits.pitch_nm + 1e-6
        and abs(yaw) <= limits.yaw_nm + 1e-6
    )


def plant_total_wrench(
    engines: Sequence[EngineGeometry],
    commands: Sequence[tuple[float, float]],
    *,
    active_mask: int | None = None,
) -> tuple[tuple[float, float, float], float]:
    torque = (0.0, 0.0, 0.0)
    thrust = 0.0
    for index, engine in enumerate(engines):
        if active_mask is not None and (active_mask & (1 << index)) == 0:
            continue
        roll, yaw = commands[index]
        torque = add(torque, plant_torque(engine, roll, yaw))
        thrust += plant_axial_thrust(engine, roll, yaw)
    return torque, thrust


def _mean_active_yaw(commands: Sequence[tuple[float, float]], active_indices: Sequence[int]) -> float:
    if not active_indices:
        return 0.0
    return sum(commands[index][1] for index in active_indices) / len(active_indices)


def _clip_commands(
    engines: Sequence[EngineGeometry],
    commands: list[tuple[float, float]],
) -> list[tuple[float, float]]:
    clipped: list[tuple[float, float]] = []
    for engine, (roll, yaw) in zip(engines, commands):
        clipped.append(
            (
                constrain(roll, engine.roll_min_deg, engine.roll_max_deg),
                constrain(yaw, engine.yaw_min_deg, engine.yaw_max_deg),
            )
        )
    return clipped


def _evaluate_lm_residual(
    engines: Sequence[EngineGeometry],
    commands: Sequence[tuple[float, float]],
    desired_torque_nm: Sequence[float],
    desired_thrust_n: float,
    active_indices: Sequence[int],
    config: LmConfig,
) -> tuple[list[float], float, float, float]:
    torque, thrust = plant_total_wrench(engines, commands, active_mask=None)
    torque_error = sub(torque, desired_torque_nm)
    thrust_error = desired_thrust_n - thrust
    mean_yaw = _mean_active_yaw(commands, active_indices)
    thrust_scale = max(abs(desired_thrust_n), 1.0)
    residual = [
        torque_error[0],
        torque_error[1],
        torque_error[2],
        config.thrust_weight * thrust_error / thrust_scale,
    ]
    for index in active_indices:
        residual.append(config.splay_weight * (commands[index][1] - mean_yaw))
    cost = 0.5 * sum(value * value for value in residual)
    return residual, cost, norm(torque_error), thrust_error


def _solve_linear_system(matrix_a: list[list[float]], vector_b: list[float]) -> list[float] | None:
    size = len(vector_b)
    a = [row[:] for row in matrix_a]
    b = vector_b[:]

    for col in range(size):
        pivot = col
        pivot_abs = abs(a[col][col])
        for row in range(col + 1, size):
            candidate = abs(a[row][col])
            if candidate > pivot_abs:
                pivot_abs = candidate
                pivot = row
        if pivot_abs < 1e-9:
            return None
        if pivot != col:
            a[col], a[pivot] = a[pivot], a[col]
            b[col], b[pivot] = b[pivot], b[col]
        inv_pivot = 1.0 / a[col][col]
        for row in range(col + 1, size):
            factor = a[row][col] * inv_pivot
            for k in range(col, size):
                a[row][k] -= factor * a[col][k]
            b[row] -= factor * b[col]

    for row in range(size - 1, -1, -1):
        total = b[row]
        for col in range(row + 1, size):
            total -= a[row][col] * b[col]
        if abs(a[row][row]) < 1e-9:
            return None
        b[row] = total / a[row][row]
    return b


def _lm_converged(
    torque_error_nm: float,
    thrust_error_n: float,
    desired_thrust_n: float,
    config: LmConfig,
) -> bool:
    if torque_error_nm >= config.torque_tol_nm:
        return False
    if config.thrust_weight <= 0.0:
        return True
    thrust_tol = max(1.0, abs(desired_thrust_n) * 0.05)
    return abs(thrust_error_n) < thrust_tol


def solve_gimbal_lm(
    engines: Sequence[EngineGeometry],
    desired_torque_nm: Sequence[float],
    desired_thrust_n: float,
    initial_commands: Sequence[tuple[float, float]],
    *,
    active_mask: int | None = None,
    config: LmConfig | None = None,
    torque_limits: TorqueLimits | None = None,
) -> LmSolveResult:
    """Bounded Levenberg-Marquardt solver for torque, net thrust, and symmetric splay."""
    cfg = config or LmConfig()
    result = LmSolveResult()
    clamped_torque, torque_saturated = clamp_torque_demand(desired_torque_nm, torque_limits)
    result.demand_saturated = torque_saturated
    desired_torque_nm = clamped_torque

    if not engines:
        result.commands = tuple(initial_commands)
        return result

    if not wrench_demand_feasible(engines, clamped_torque, desired_thrust_n, torque_limits=torque_limits):
        result.demand_saturated = True
        result.commands = tuple(_clip_commands(engines, list(initial_commands)))
        torque, thrust = plant_total_wrench(engines, result.commands)
        result.residual_torque_nm = norm(sub(torque, clamped_torque))
        result.residual_thrust_n = desired_thrust_n - thrust
        result.converged = False
        return result

    active_indices = [
        index
        for index, engine in enumerate(engines)
        if (active_mask is None or (active_mask & (1 << index)) != 0) and engine.thrust_n > 0.5
    ]
    if not active_indices:
        result.commands = tuple(initial_commands)
        return result

    commands = _clip_commands(engines, list(initial_commands))
    dof_count = len(engines) * 2
    state = [0.0] * dof_count
    for index, (roll, yaw) in enumerate(commands):
        state[2 * index] = math.radians(roll)
        state[2 * index + 1] = math.radians(yaw)

    def state_to_commands(values: Sequence[float]) -> list[tuple[float, float]]:
        converted = []
        for index in range(len(engines)):
            converted.append(
                (
                    math.degrees(values[2 * index]),
                    math.degrees(values[2 * index + 1]),
                )
            )
        return _clip_commands(engines, converted)

    def dof_active(dof: int) -> bool:
        engine = dof // 2
        return engine in active_indices

    residual, cost, torque_error_nm, thrust_error_n = _evaluate_lm_residual(
        engines,
        commands,
        desired_torque_nm,
        desired_thrust_n,
        active_indices,
        cfg,
    )
    result.cost = cost
    result.residual_torque_nm = torque_error_nm
    result.residual_thrust_n = thrust_error_n

    if _lm_converged(torque_error_nm, thrust_error_n, desired_thrust_n, cfg):
        result.commands = tuple(commands)
        result.converged = True
        return result

    lambda_value = max(cfg.lambda0, 1e-6)

    for iteration in range(max(cfg.max_iter, 1)):
        result.iterations_used = iteration + 1
        trial_commands = state_to_commands(state)
        base_residual, _, _, _ = _evaluate_lm_residual(
            engines,
            trial_commands,
            desired_torque_nm,
            desired_thrust_n,
            active_indices,
            cfg,
        )
        jacobian: list[list[float]] = [[0.0] * dof_count for _ in range(len(base_residual))]

        for dof in range(dof_count):
            if not dof_active(dof):
                continue
            plus_state = state[:]
            minus_state = state[:]
            plus_state[dof] += cfg.fd_eps
            minus_state[dof] -= cfg.fd_eps
            plus_residual, _, _, _ = _evaluate_lm_residual(
                engines,
                state_to_commands(plus_state),
                desired_torque_nm,
                desired_thrust_n,
                active_indices,
                cfg,
            )
            minus_residual, _, _, _ = _evaluate_lm_residual(
                engines,
                state_to_commands(minus_state),
                desired_torque_nm,
                desired_thrust_n,
                active_indices,
                cfg,
            )
            inv_2eps = 1.0 / (2.0 * cfg.fd_eps)
            for row in range(len(base_residual)):
                jacobian[row][dof] = (plus_residual[row] - minus_residual[row]) * inv_2eps

        normal = [[0.0] * dof_count for _ in range(dof_count)]
        gradient = [0.0] * dof_count
        for dof in range(dof_count):
            for other in range(dof_count):
                normal[dof][other] = sum(jacobian[row][dof] * jacobian[row][other] for row in range(len(base_residual)))
            normal[dof][dof] += lambda_value
            gradient[dof] = -sum(jacobian[row][dof] * base_residual[row] for row in range(len(base_residual)))

        delta = _solve_linear_system(normal, gradient)
        if delta is None:
            lambda_value = min(lambda_value * 10.0, 1e6)
            continue

        trial_state = [state[dof] + delta[dof] for dof in range(dof_count)]
        trial_commands = state_to_commands(trial_state)
        trial_residual, trial_cost, trial_torque_error, trial_thrust_error = _evaluate_lm_residual(
            engines,
            trial_commands,
            desired_torque_nm,
            desired_thrust_n,
            active_indices,
            cfg,
        )

        if trial_cost < cost:
            state = trial_state
            commands = trial_commands
            residual = trial_residual
            cost = trial_cost
            torque_error_nm = trial_torque_error
            thrust_error_n = trial_thrust_error
            lambda_value = max(lambda_value * 0.1, 1e-6)
            if _lm_converged(torque_error_nm, thrust_error_n, desired_thrust_n, cfg):
                result.converged = True
                break
        else:
            lambda_value = min(lambda_value * 10.0, 1e6)

    result.commands = tuple(commands)
    result.cost = cost
    result.residual_torque_nm = torque_error_nm
    result.residual_thrust_n = thrust_error_n
    result.lambda_final = lambda_value
    return result


def lm_converged(
    torque_error_nm: float,
    thrust_error_n: float,
    desired_thrust_n: float,
    config: LmConfig | None = None,
) -> bool:
    """Public mirror of firmware LM convergence tolerances."""
    return _lm_converged(torque_error_nm, thrust_error_n, desired_thrust_n, config or LmConfig())


def _torque_seed_hint(
    engines: Sequence[EngineGeometry],
    desired_torque_nm: Sequence[float],
) -> tuple[tuple[float, float], ...]:
    """Heuristic differential TVC seed when the grid oracle is coarse."""
    roll, pitch, yaw = desired_torque_nm
    seed = [(0.0, 0.0) for _ in engines]
    if len(engines) < 3:
        return tuple(seed)

    if abs(pitch) > 1e-3:
        sign = 1.0 if pitch > 0.0 else -1.0
        seed[0] = (sign * 3.0, seed[0][1])
        seed[1] = (-sign * 1.5, seed[1][1])
        seed[2] = (-sign * 1.5, seed[2][1])

    if abs(roll) > 1e-3:
        sign = 1.0 if roll > 0.0 else -1.0
        for index in range(len(engines)):
            roll_deg, yaw_deg = seed[index]
            seed[index] = (roll_deg, yaw_deg + sign * 2.0)

    if abs(yaw) > 1e-3:
        sign = 1.0 if yaw > 0.0 else -1.0
        for index in range(len(engines)):
            roll_deg, yaw_deg = seed[index]
            yaw_hint = sign * (2.0 if index % 2 == 0 else -2.0)
            seed[index] = (roll_deg, yaw_deg + yaw_hint)

    return tuple(seed)


def firmware_warm_start(
    engines: Sequence[EngineGeometry],
    desired_torque_nm: Sequence[float],
    desired_thrust_n: float,
    *,
    oracle_commands: Sequence[tuple[float, float]] | None = None,
    oracle_torque_error_nm: float | None = None,
) -> tuple[tuple[float, float], ...]:
    """Allocator seed plus collective splay yaw hint, matching tv3_control_mixer cold start."""
    oracle_error = oracle_torque_error_nm
    if oracle_commands is None:
        oracle = allocate(engines, desired_torque_nm, desired_thrust_n)
        seed = list(oracle.commands)
        if oracle_error is None:
            oracle_error = oracle.torque_error_nm
    else:
        seed = list(oracle_commands)
        if oracle_error is None:
            oracle_error = math.inf

    while len(seed) < len(engines):
        seed.append((0.0, 0.0))

    if oracle_error > 0.1 or not math.isfinite(oracle_error):
        seed = list(_torque_seed_hint(engines, desired_torque_nm))

    full_thrust, _min_splayed = thrust_envelope(engines)
    if desired_thrust_n < full_thrust - 1e-3:
        splay_deg = collective_throttle_yaw_deg(desired_thrust_n, engines)
        seed = [(roll, yaw + splay_deg) for roll, yaw in seed]

    return tuple(seed[: len(engines)])


@dataclass(frozen=True)
class LmSweepCase:
    desired_torque_nm: tuple[float, float, float]
    desired_thrust_n: float


@dataclass
class LmSweepResult:
    case: LmSweepCase
    reachable: bool
    oracle_reason: str = REASON_NONE
    oracle_torque_error_nm: float = math.inf
    oracle_thrust_error_n: float = math.inf
    warm_start: tuple[tuple[float, float], ...] = ()
    lm_converged: bool = False
    residual_torque_nm: float = math.inf
    residual_thrust_n: float = math.inf
    iterations_used: int = 0
    cost: float = math.inf


@dataclass
class LmSweepSummary:
    total_cases: int = 0
    reachable_count: int = 0
    unreachable_count: int = 0
    lm_converged_count: int = 0
    reachable_failed_count: int = 0
    convergence_rate: float = 0.0
    torque_residual_p50: float = math.nan
    torque_residual_p95: float = math.nan
    thrust_residual_p50: float = math.nan
    iterations_p50: float = math.nan
    iterations_max: int = 0
    config: LmConfig = field(default_factory=LmConfig)
    worst_failures: list[dict] = field(default_factory=list)


def _span_values(low: float, high: float, steps: int) -> list[float]:
    if steps <= 1 or abs(high - low) <= 1e-9:
        return [0.5 * (low + high)]
    if low <= 0.0 <= high:
        values = [low, 0.0, high]
        if steps > 3:
            for index in range(1, steps - 1):
                frac = index / (steps - 1)
                values.append(low + frac * (high - low))
        return sorted(set(values))
    return [low + index * (high - low) / (steps - 1) for index in range(steps)]


def generate_lm_sweep_cases(
    engines: Sequence[EngineGeometry],
    torque_limits: TorqueLimits | None = None,
    *,
    thrust_steps: int = 5,
    torque_steps: int = 3,
    full: bool = False,
) -> list[LmSweepCase]:
    """Build a demand grid inside the thrust envelope and torque limits."""
    if full:
        thrust_steps = max(thrust_steps, 10)
        torque_steps = max(torque_steps, 7)

    limits = torque_limits or TorqueLimits()
    full_thrust, min_splayed = thrust_envelope(engines)
    min_thrust = max(min_splayed, practical_thrust_floor(engines))
    thrust_values = _span_values(min_thrust, full_thrust, thrust_steps)
    roll_values = _span_values(-limits.roll_nm, limits.roll_nm, torque_steps)
    pitch_values = _span_values(-limits.pitch_nm, limits.pitch_nm, torque_steps)
    yaw_values = _span_values(-limits.yaw_nm, limits.yaw_nm, torque_steps)

    cases: list[LmSweepCase] = []
    seen: set[tuple[tuple[float, float, float], float]] = set()

    def add_case(torque: tuple[float, float, float], thrust: float) -> None:
        key = (torque, round(thrust, 6))
        if key in seen:
            return
        seen.add(key)
        cases.append(LmSweepCase(desired_torque_nm=torque, desired_thrust_n=thrust))

    for thrust in thrust_values:
        for roll in roll_values:
            for pitch in pitch_values:
                for yaw in yaw_values:
                    add_case((roll, pitch, yaw), thrust)

    add_case((0.0, 0.0, 0.0), full_thrust)
    add_case((0.0, 0.0, 0.0), 0.8 * full_thrust)
    return cases


def _percentile(values: Sequence[float], fraction: float) -> float:
    finite = sorted(value for value in values if math.isfinite(value))
    if not finite:
        return math.nan
    index = min(len(finite) - 1, max(0, int(round(fraction * (len(finite) - 1)))))
    return finite[index]


def _physically_feasible(
    case: LmSweepCase,
    engines: Sequence[EngineGeometry],
    torque_limits: TorqueLimits | None,
) -> bool:
    limits = torque_limits or TorqueLimits()
    full_thrust, _ = thrust_envelope(engines)
    min_thrust = practical_thrust_floor(engines)
    if case.desired_thrust_n > full_thrust + 1e-3 or case.desired_thrust_n < min_thrust - 1e-3:
        return False
    roll, pitch, yaw = case.desired_torque_nm
    if abs(roll) > limits.roll_nm + 1e-6:
        return False
    if abs(pitch) > limits.pitch_nm + 1e-6:
        return False
    if abs(yaw) > limits.yaw_nm + 1e-6:
        return False
    return bool(engines)


def _oracle_reachable(
    oracle: AllocationResult,
    desired_thrust_n: float,
    *,
    torque_tolerance_nm: float = 0.75,
    thrust_tolerance_frac: float = 0.08,
) -> bool:
    if oracle.reachable:
        return True
    thrust_tol = max(1.0, abs(desired_thrust_n) * thrust_tolerance_frac)
    return oracle.torque_error_nm <= torque_tolerance_nm and oracle.thrust_error_n <= thrust_tol


def _splay_oracle_reachable(
    engines: Sequence[EngineGeometry],
    case: LmSweepCase,
    oracle: AllocationResult,
    *,
    torque_tolerance_nm: float = 0.75,
    thrust_tolerance_frac: float = 0.08,
) -> bool:
    """Reachability after collective splay hint, matching tv3_control_mixer cold start."""
    full_thrust, _ = thrust_envelope(engines)
    if case.desired_thrust_n >= full_thrust - 1e-3:
        return False
    warm = firmware_warm_start(
        engines,
        case.desired_torque_nm,
        case.desired_thrust_n,
        oracle_commands=oracle.commands,
    )
    torque, thrust = plant_total_wrench(engines, warm)
    torque_error = norm(sub(torque, case.desired_torque_nm))
    thrust_error = abs(thrust - case.desired_thrust_n)
    thrust_tol = max(1.0, abs(case.desired_thrust_n) * thrust_tolerance_frac)
    return torque_error <= torque_tolerance_nm and thrust_error <= thrust_tol


def _warm_start_reachable(
    engines: Sequence[EngineGeometry],
    case: LmSweepCase,
    oracle: AllocationResult,
    *,
    torque_tolerance_nm: float = 2.0,
    thrust_tolerance_frac: float = 0.12,
) -> bool:
    """Loose plant check at the firmware warm start."""
    warm = firmware_warm_start(
        engines,
        case.desired_torque_nm,
        case.desired_thrust_n,
        oracle_commands=oracle.commands,
        oracle_torque_error_nm=oracle.torque_error_nm,
    )
    torque, thrust = plant_total_wrench(engines, warm)
    torque_error = norm(sub(torque, case.desired_torque_nm))
    thrust_error = abs(thrust - case.desired_thrust_n)
    thrust_tol = max(5.0, abs(case.desired_thrust_n) * thrust_tolerance_frac)
    return torque_error <= torque_tolerance_nm and thrust_error <= thrust_tol


def _sweep_reachable(
    case: LmSweepCase,
    engines: Sequence[EngineGeometry],
    oracle: AllocationResult,
    torque_limits: TorqueLimits | None,
    *,
    oracle_torque_tolerance_nm: float = 0.75,
    oracle_thrust_tolerance_frac: float = 0.08,
) -> bool:
    if not _physically_feasible(case, engines, torque_limits):
        return False
    return _oracle_reachable(
        oracle,
        case.desired_thrust_n,
        torque_tolerance_nm=oracle_torque_tolerance_nm,
        thrust_tolerance_frac=oracle_thrust_tolerance_frac,
    ) or _splay_oracle_reachable(
        engines,
        case,
        oracle,
        torque_tolerance_nm=oracle_torque_tolerance_nm,
        thrust_tolerance_frac=oracle_thrust_tolerance_frac,
    ) or _warm_start_reachable(
        engines,
        case,
        oracle,
        torque_tolerance_nm=max(2.0, oracle_torque_tolerance_nm * 2.0),
        thrust_tolerance_frac=max(0.12, oracle_thrust_tolerance_frac * 1.5),
    )


def run_lm_sweep(
    engines: Sequence[EngineGeometry],
    cases: Sequence[LmSweepCase] | None = None,
    *,
    torque_limits: TorqueLimits | None = None,
    config: LmConfig | None = None,
    full: bool = False,
    grid_steps: int = 5,
    oracle_torque_tolerance_nm: float = 0.75,
    oracle_thrust_tolerance_frac: float = 0.08,
) -> tuple[LmSweepSummary, list[LmSweepResult]]:
    """Run reachability oracle plus LM for each demand case."""
    cfg = config or LmConfig()
    if cases is None:
        cases = generate_lm_sweep_cases(engines, torque_limits, full=full)

    results: list[LmSweepResult] = []
    for case in cases:
        oracle = allocate(
            engines,
            case.desired_torque_nm,
            case.desired_thrust_n,
            torque_limits=torque_limits,
            grid_steps=grid_steps,
        )
        entry = LmSweepResult(
            case=case,
            reachable=_sweep_reachable(
                case,
                engines,
                oracle,
                torque_limits,
                oracle_torque_tolerance_nm=oracle_torque_tolerance_nm,
                oracle_thrust_tolerance_frac=oracle_thrust_tolerance_frac,
            ),
            oracle_reason=oracle.reason,
            oracle_torque_error_nm=oracle.torque_error_nm,
            oracle_thrust_error_n=oracle.thrust_error_n,
        )
        if not entry.reachable:
            results.append(entry)
            continue

        warm_start = firmware_warm_start(
            engines,
            case.desired_torque_nm,
            case.desired_thrust_n,
            oracle_commands=oracle.commands,
            oracle_torque_error_nm=oracle.torque_error_nm,
        )
        entry.warm_start = warm_start
        lm_result = solve_gimbal_lm(
            engines,
            case.desired_torque_nm,
            case.desired_thrust_n,
            warm_start,
            config=cfg,
        )
        entry.residual_torque_nm = lm_result.residual_torque_nm
        entry.residual_thrust_n = lm_result.residual_thrust_n
        entry.iterations_used = lm_result.iterations_used
        entry.cost = lm_result.cost
        entry.lm_converged = lm_result.converged or lm_converged(
            lm_result.residual_torque_nm,
            lm_result.residual_thrust_n,
            case.desired_thrust_n,
            cfg,
        )
        results.append(entry)

    reachable = [entry for entry in results if entry.reachable]
    converged = [entry for entry in reachable if entry.lm_converged]
    failed = [entry for entry in reachable if not entry.lm_converged]

    summary = LmSweepSummary(
        total_cases=len(results),
        reachable_count=len(reachable),
        unreachable_count=len(results) - len(reachable),
        lm_converged_count=len(converged),
        reachable_failed_count=len(failed),
        convergence_rate=(len(converged) / len(reachable)) if reachable else 1.0,
        torque_residual_p50=_percentile([entry.residual_torque_nm for entry in converged], 0.5),
        torque_residual_p95=_percentile([entry.residual_torque_nm for entry in converged], 0.95),
        thrust_residual_p50=_percentile([abs(entry.residual_thrust_n) for entry in converged], 0.5),
        iterations_p50=_percentile([float(entry.iterations_used) for entry in converged], 0.5),
        iterations_max=max((entry.iterations_used for entry in converged), default=0),
        config=cfg,
        worst_failures=sorted(
            [
                {
                    "torque_nm": list(entry.case.desired_torque_nm),
                    "thrust_n": entry.case.desired_thrust_n,
                    "residual_torque_nm": entry.residual_torque_nm,
                    "residual_thrust_n": entry.residual_thrust_n,
                    "iterations_used": entry.iterations_used,
                }
                for entry in failed
            ],
            key=lambda item: item["residual_torque_nm"],
            reverse=True,
        )[:10],
    )
    return summary, results


def tune_lm_config(
    engines: Sequence[EngineGeometry],
    *,
    torque_limits: TorqueLimits | None = None,
    cases: Sequence[LmSweepCase] | None = None,
    max_trials: int = 50,
    grid_steps: int = 5,
    full: bool = False,
    seed: int = 0,
) -> tuple[LmConfig, LmSweepSummary]:
    """Coarse random search for LmConfig that maximizes reachable convergence rate."""
    import random

    rng = random.Random(seed)
    if cases is None:
        cases = generate_lm_sweep_cases(engines, torque_limits, full=full)

    base = LmConfig()
    candidates: list[LmConfig] = [base]
    for _ in range(max(0, max_trials - 1)):
        candidates.append(
            LmConfig(
                max_iter=rng.choice([6, 8, 12, 16]),
                torque_tol_nm=rng.choice([0.1, 0.15, 0.2, 0.25]),
                lambda0=10 ** rng.uniform(-3.0, -1.0),
                thrust_weight=rng.choice([0.5, 1.0, 2.0, 4.0]),
                splay_weight=rng.choice([0.0, 0.05, 0.1, 0.2]),
                fd_eps=rng.choice([0.005, 0.01, 0.02]),
            )
        )

    best_config = base
    best_summary, _ = run_lm_sweep(
        engines,
        cases,
        torque_limits=torque_limits,
        config=base,
        grid_steps=grid_steps,
    )
    best_rate = best_summary.convergence_rate

    for candidate in candidates[1:]:
        summary, _ = run_lm_sweep(
            engines,
            cases,
            torque_limits=torque_limits,
            config=candidate,
            grid_steps=grid_steps,
        )
        if summary.convergence_rate > best_rate + 1e-9:
            best_rate = summary.convergence_rate
            best_config = candidate
            best_summary = summary

    return best_config, best_summary


def command_grid(engine: EngineGeometry, steps: int) -> list[tuple[float, float, float]]:
    def span_values(low: float, high: float) -> list[float]:
        if steps <= 1 or abs(high - low) <= 1e-6:
            return [constrain(0.0, low, high)]
        if low <= 0.0 <= high:
            return [low, 0.0, high]
        midpoint = 0.5 * (low + high)
        return [low, midpoint, high]

    return [
        (roll, yaw)
        for roll in span_values(engine.roll_min_deg, engine.roll_max_deg)
        for yaw in span_values(engine.yaw_min_deg, engine.yaw_max_deg)
    ]


def _command_saturated(
    engine: EngineGeometry,
    command: tuple[float, float],
    epsilon: float = 1e-3,
) -> bool:
    roll, yaw = command
    return (
        abs(roll - engine.roll_min_deg) <= epsilon
        or abs(roll - engine.roll_max_deg) <= epsilon
        or abs(yaw - engine.yaw_min_deg) <= epsilon
        or abs(yaw - engine.yaw_max_deg) <= epsilon
    )


def allocate(
    engines: Sequence[EngineGeometry],
    desired_torque_nm: Sequence[float],
    desired_thrust_n: float,
    *,
    active_mask: int | None = None,
    thrust_scales: Sequence[float] | None = None,
    torque_limits: TorqueLimits | None = None,
    grid_steps: int = 5,
    torque_tolerance_nm: float = 0.5,
    thrust_tolerance_frac: float = 0.05,
) -> AllocationResult:
    """Bounded grid solver for torque and net thrust with explicit unreachable results."""
    working = scaled_engines(engines, active_mask=active_mask, thrust_scales=thrust_scales)
    if not engines:
        return AllocationResult(
            reachable=False,
            reason=REASON_NO_ENGINES,
            control_unreachable_reason=CONTROL_NO_ACTIVE_ENGINES,
        )
    if not working:
        return AllocationResult(
            reachable=False,
            reason=REASON_NO_ACTIVE_ENGINES,
            control_unreachable_reason=CONTROL_NO_ACTIVE_ENGINES,
        )

    full_thrust, min_splayed = thrust_envelope(working)
    result = AllocationResult(
        reachable=False,
        full_thrust_n=full_thrust,
        min_splayed_thrust_n=min_splayed,
    )

    if desired_thrust_n > full_thrust + 1e-6 or desired_thrust_n < min_splayed - 1e-6:
        result.reason = REASON_THRUST_ENVELOPE
        result.control_unreachable_reason = CONTROL_THRUST_ENVELOPE
        return result

    desired = tuple(desired_torque_nm)
    if torque_limits is not None:
        if abs(desired[0]) > torque_limits.roll_nm + 1e-6:
            result.reason = REASON_TORQUE_UNREACHABLE
            result.control_unreachable_reason = CONTROL_TORQUE_ENVELOPE
            return result
        if abs(desired[1]) > torque_limits.pitch_nm + 1e-6:
            result.reason = REASON_TORQUE_UNREACHABLE
            result.control_unreachable_reason = CONTROL_TORQUE_ENVELOPE
            return result
        if abs(desired[2]) > torque_limits.yaw_nm + 1e-6:
            result.reason = REASON_TORQUE_UNREACHABLE
            result.control_unreachable_reason = CONTROL_TORQUE_ENVELOPE
            return result

    best: AllocationResult | None = None
    thrust_tolerance = max(1.0, abs(desired_thrust_n) * thrust_tolerance_frac)

    for commands in itertools.product(*(command_grid(engine, grid_steps) for engine in working)):
        torque = (0.0, 0.0, 0.0)
        thrust = 0.0
        saturated = False
        for engine, command in zip(working, commands):
            roll, yaw = command
            torque = add(torque, plant_torque(engine, roll, yaw))
            thrust += plant_axial_thrust(engine, roll, yaw)
            saturated = saturated or _command_saturated(engine, command)

        torque_error = norm(sub(torque, desired))
        thrust_error = abs(thrust - desired_thrust_n)
        score = torque_error + thrust_error * 0.02
        candidate = AllocationResult(
            reachable=False,
            score=score,
            torque_error_nm=torque_error,
            thrust_error_n=thrust_error,
            achieved_torque_nm=torque,
            achieved_thrust_n=thrust,
            unallocated_torque_nm=sub(desired, torque),
            commands=commands,
            saturated=saturated,
            full_thrust_n=full_thrust,
            min_splayed_thrust_n=min_splayed,
        )
        if best is None or candidate.score < best.score:
            best = candidate

    assert best is not None
    best.reachable = best.torque_error_nm <= torque_tolerance_nm and best.thrust_error_n <= thrust_tolerance
    if not best.reachable:
        best.reason = REASON_TORQUE_UNREACHABLE
        best.control_unreachable_reason = CONTROL_TORQUE_ENVELOPE
    return best


def allocate_from_vehicle(
    vehicle: dict | Path | str,
    desired_torque_nm: Sequence[float],
    desired_thrust_n: float,
    **kwargs,
) -> AllocationResult:
    manifest = load_manifest(vehicle) if not isinstance(vehicle, dict) else vehicle
    return allocate(
        engines_from_vehicle(manifest),
        desired_torque_nm,
        desired_thrust_n,
        torque_limits=torque_limits_from_vehicle(manifest),
        **kwargs,
    )


@dataclass
class MotorReferenceState:
    engine_count: int = 1
    active_mask: int = 1
    expected_thrust_n: float = 0.0
    expected_thrust_n_engine: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0, 0.0])
    expected_vehicle_mass_kg: float = 1.0
    loaded: bool = True


def motor_reference_from_thrust(
    vehicle: dict,
    *,
    thrust_n: float,
    active_mask: int | None = None,
    thrust_scales: Sequence[float] | None = None,
) -> MotorReferenceState:
    engines = vehicle.get("propulsion", {}).get("engines", [])
    engine_count = int(vehicle.get("propulsion", {}).get("engine_count", len(engines) or 1))
    mask = active_engine_mask(engine_count, active_mask)
    scales = list(thrust_scales) if thrust_scales is not None else [1.0] * engine_count
    if len(scales) < engine_count:
        scales.extend([1.0] * (engine_count - len(scales)))

    from tools.tv3_motor_catalog import engine_thrust_n, load_motor_catalog

    motor_selection = vehicle.get("motor_selection", {})
    catalog = (
        load_motor_catalog(str(motor_selection["catalog_source"]))
        if motor_selection.get("catalog_source")
        else None
    )

    per_engine = [0.0, 0.0, 0.0, 0.0]
    total = 0.0
    for index, engine in enumerate(engines[:engine_count]):
        if mask & (1 << index):
            thrust = engine_thrust_n(vehicle, engine, catalog=catalog) * max(scales[index], 0.0)
            per_engine[index] = thrust
            total += thrust

    if thrust_n > 0.0:
        total = thrust_n

    return MotorReferenceState(
        engine_count=engine_count,
        active_mask=mask,
        expected_thrust_n=total,
        expected_thrust_n_engine=per_engine,
        expected_vehicle_mass_kg=float(vehicle["vehicle"]["body_mass_kg"]),
        loaded=True,
    )


def thrust_envelope_from_reference(
    vehicle: dict,
    motor_reference: MotorReferenceState,
) -> tuple[float, float]:
    splay_max_deg = splay_max_deg_from_vehicle(vehicle)
    cos_splay = math.cos(math.radians(splay_max_deg))
    max_thrust = 0.0
    min_thrust = 0.0
    for index in range(min(motor_reference.engine_count, 4)):
        if motor_reference.active_mask & (1 << index):
            thrust = motor_reference.expected_thrust_n_engine[index]
            max_thrust += thrust
            min_thrust += thrust * cos_splay
    return max_thrust, min_thrust


def estimate_guidance_torque_nm(
    vehicle: dict,
    *,
    velocity_sp: Sequence[float],
    mass_kg: float,
    position_gain: float,
) -> tuple[float, float, float]:
    """Rough torque demand from horizontal velocity commands for reachability gating."""
    horiz = math.hypot(velocity_sp[0], velocity_sp[1])
    if horiz <= 1e-3:
        return (0.0, 0.0, 0.0)

    engines = engines_from_vehicle(vehicle)
    lever_arm = max((math.hypot(engine.position_m[1], engine.position_m[2]) for engine in engines), default=0.1)
    lever_arm = max(lever_arm, 0.05)
    lateral_accel = mass_kg * min(horiz * position_gain, 20.0)
    torque_mag = lateral_accel * lever_arm
    return (0.0, torque_mag, torque_mag)


def guidance_reachability(
    vehicle: dict | Path | str,
    motor_reference: MotorReferenceState,
    *,
    required_thrust_n: float,
    velocity_sp: Sequence[float] = (0.0, 0.0, 0.0),
    position_gain: float = 0.15,
) -> AllocationResult:
    """Check whether hover/landing guidance can commit to the current thrust and torque demand."""
    manifest = load_manifest(vehicle) if not isinstance(vehicle, dict) else vehicle
    engines = engines_from_vehicle(manifest)
    thrust_scales = [
        (
            motor_reference.expected_thrust_n_engine[index] / engine.thrust_n
            if engine.thrust_n > 1e-6
            else 0.0
        )
        for index, engine in enumerate(engines)
    ]
    desired_torque = estimate_guidance_torque_nm(
        manifest,
        velocity_sp=velocity_sp,
        mass_kg=max(motor_reference.expected_vehicle_mass_kg, 0.1),
        position_gain=position_gain,
    )
    working = scaled_engines(
        engines,
        active_mask=motor_reference.active_mask,
        thrust_scales=thrust_scales,
    )
    full_thrust, min_splayed = thrust_envelope(working)
    min_thrust = max(min_splayed, practical_thrust_floor(working))
    if required_thrust_n > 1e-3 and (
        required_thrust_n > full_thrust + 1e-3 or required_thrust_n < min_thrust - 1e-3
    ):
        return AllocationResult(
            reachable=False,
            reason=REASON_THRUST_ENVELOPE,
            control_unreachable_reason=CONTROL_THRUST_ENVELOPE,
            full_thrust_n=full_thrust,
            min_splayed_thrust_n=min_thrust,
        )

    result = allocate(
        engines,
        desired_torque,
        required_thrust_n,
        active_mask=motor_reference.active_mask,
        thrust_scales=thrust_scales,
        torque_limits=torque_limits_from_vehicle(manifest),
    )
    if result.reachable:
        return result

    max_thrust, _min_thrust = thrust_envelope_from_reference(manifest, motor_reference)
    if required_thrust_n > 1e-3 and required_thrust_n > max_thrust + 1e-3:
        result.reason = REASON_THRUST_ENVELOPE
        result.control_unreachable_reason = CONTROL_THRUST_ENVELOPE
    elif motor_reference.active_mask == 0:
        result.reason = REASON_NO_ACTIVE_ENGINES
        result.control_unreachable_reason = CONTROL_NO_ACTIVE_ENGINES
    return result


def flight_plant_torque_agreement(
    engine: EngineGeometry,
    *,
    reference_thrust_n: float,
    axis: Sequence[float],
    max_angle_deg: float,
) -> tuple[float, float, float]:
    """Torque at full commanded deflection using both flight and plant models."""
    flight = flight_effectiveness_torque(
        engine,
        gimbal_axis=axis,
        max_angle_deg=max_angle_deg,
        reference_thrust_n=reference_thrust_n,
    )
    if axis == engine.roll_axis:
        plant = plant_torque(engine, max_angle_deg, 0.0, 0.0)
    else:
        plant = plant_torque(engine, 0.0, max_angle_deg, 0.0)
    return flight, plant