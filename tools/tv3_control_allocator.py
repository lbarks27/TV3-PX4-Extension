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
    if not engines or desired_net_thrust_n <= 0.0:
        return 0.0

    full_thrust = sum(plant_axial_thrust(engine, roll_deg, 0.0) for engine in engines)
    if full_thrust < 1e-3 or desired_net_thrust_n >= full_thrust - 1e-3:
        return 0.0

    yaw_limit = throttle_max_deg
    if yaw_limit is None:
        yaw_limit = min(engine.yaw_max_deg for engine in engines)

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
    if total_chamber_thrust_n < 1e-3 or desired_net_thrust_n <= 0.0:
        return 0.0
    if desired_net_thrust_n >= total_chamber_thrust_n - 1e-3:
        return 0.0
    ratio = max(0.0, min(1.0, desired_net_thrust_n / total_chamber_thrust_n))
    return max(0.0, min(splay_max_deg, math.degrees(math.acos(ratio))))


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