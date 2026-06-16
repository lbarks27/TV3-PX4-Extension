#!/usr/bin/env python3
"""Shared TV3 constrained control allocator for host validation and guidance checks.

Mirrors the PX4 ``ActuatorEffectivenessTV3`` small-angle TVC linearization for
torque authority and the SIH plant splay/pitch/yaw thrust model for net thrust.
"""

from __future__ import annotations

import itertools
import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Iterable, Sequence

import yaml

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
    pitch_axis: tuple[float, float, float]
    yaw_axis: tuple[float, float, float]
    thrust_n: float
    pitch_max_deg: float
    yaw_max_deg: float
    splay_max_deg: float
    thrust_fraction: float = 1.0
    pitch_trim: float = 0.0
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
    return yaml.safe_load(Path(path).read_text())


def engines_from_vehicle(vehicle: dict) -> list[EngineGeometry]:
    body = vehicle["vehicle"]
    engines = vehicle.get("propulsion", {}).get("engines")
    if not engines:
        engines = [
            {
                "position_m": [body["motor_com_x_m"], 0.0, 0.0],
                "thrust_axis": [1.0, 0.0, 0.0],
                "pitch_axis": [0.0, -1.0, 0.0],
                "yaw_axis": [0.0, 0.0, -1.0],
                "thrust_fraction": 1.0,
                "gimbal": {
                    "pitch_max_deg": body["tvc_max_deg"],
                    "yaw_max_deg": body["tvc_max_deg"],
                    "splay_max_deg": body["tvc_max_deg"],
                },
            }
        ]

    return [
        EngineGeometry(
            position_m=tuple(engine["position_m"]),
            thrust_axis=tuple(engine["thrust_axis"]),
            pitch_axis=tuple(engine["pitch_axis"]),
            yaw_axis=tuple(engine["yaw_axis"]),
            thrust_n=body["ca_reference_thrust_n"] * engine.get("thrust_fraction", 1.0 / len(engines)),
            pitch_max_deg=engine["gimbal"].get("pitch_max_deg", body["tvc_max_deg"]),
            yaw_max_deg=engine["gimbal"].get("yaw_max_deg", body["tvc_max_deg"]),
            splay_max_deg=engine["gimbal"].get("splay_max_deg", body["tvc_max_deg"]),
            thrust_fraction=engine.get("thrust_fraction", 1.0 / len(engines)),
            pitch_trim=engine["gimbal"].get("pitch_trim", 0.0),
            yaw_trim=engine["gimbal"].get("yaw_trim", 0.0),
        )
        for engine in engines
    ]


def torque_limits_from_vehicle(vehicle: dict) -> TorqueLimits:
    limits = vehicle["vehicle"].get("torque_limits_nm", {})
    return TorqueLimits(
        roll_nm=limits.get("roll", 0.0),
        pitch_nm=limits.get("pitch", 10.0),
        yaw_nm=limits.get("yaw", 10.0),
    )


def reference_thrust_from_vehicle(vehicle: dict) -> float:
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
                pitch_axis=engine.pitch_axis,
                yaw_axis=engine.yaw_axis,
                thrust_n=engine.thrust_n * thrust_scale,
                pitch_max_deg=engine.pitch_max_deg,
                yaw_max_deg=engine.yaw_max_deg,
                splay_max_deg=engine.splay_max_deg,
                thrust_fraction=engine.thrust_fraction,
                pitch_trim=engine.pitch_trim,
                yaw_trim=engine.yaw_trim,
            )
        )
    return scaled


def thrust_envelope(engines: Sequence[EngineGeometry]) -> tuple[float, float]:
    full_thrust = sum(engine.thrust_n for engine in engines)
    min_splayed = sum(
        engine.thrust_n * math.cos(math.radians(engine.splay_max_deg)) for engine in engines
    )
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


def plant_torque(
    engine: EngineGeometry,
    pitch_deg: float,
    yaw_deg: float,
    splay_deg: float,
) -> tuple[float, float, float]:
    """Matches the SIH small-angle thrust-direction model used in tv3_sih."""
    thrust_axis = normalize(engine.thrust_axis, (1.0, 0.0, 0.0))
    pitch_axis = normalize(engine.pitch_axis, (0.0, -1.0, 0.0))
    yaw_axis = normalize(engine.yaw_axis, (0.0, 0.0, -1.0))
    pitch_rad = math.radians(pitch_deg)
    yaw_rad = math.radians(yaw_deg)
    splay_rad = math.radians(splay_deg)
    direction = thrust_axis
    direction = add(direction, scale(cross(pitch_axis, thrust_axis), pitch_rad))
    direction = add(direction, scale(cross(yaw_axis, thrust_axis), yaw_rad))
    direction = normalize(direction, thrust_axis)
    force = scale(direction, engine.thrust_n * math.cos(splay_rad))
    return cross(engine.position_m, force)


def plant_thrust(engine: EngineGeometry, splay_deg: float) -> float:
    return engine.thrust_n * math.cos(math.radians(splay_deg))


def command_grid(engine: EngineGeometry, steps: int) -> list[tuple[float, float, float]]:
    def values(limit: float) -> list[float]:
        if steps <= 1 or limit <= 0:
            return [0.0]
        return [-limit, 0.0, limit]

    splay_values = [0.0, engine.splay_max_deg]
    return [
        (pitch, yaw, splay)
        for pitch in values(engine.pitch_max_deg)
        for yaw in values(engine.yaw_max_deg)
        for splay in splay_values
    ]


def _command_saturated(
    engine: EngineGeometry,
    command: tuple[float, float, float],
    epsilon: float = 1e-3,
) -> bool:
    pitch, yaw, splay = command
    return (
        abs(abs(pitch) - engine.pitch_max_deg) <= epsilon
        or abs(abs(yaw) - engine.yaw_max_deg) <= epsilon
        or abs(abs(splay) - engine.splay_max_deg) <= epsilon
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
            pitch, yaw, splay = command
            torque = add(torque, plant_torque(engine, pitch, yaw, splay))
            thrust += plant_thrust(engine, splay)
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

    per_engine = [0.0, 0.0, 0.0, 0.0]
    total = 0.0
    reference = float(vehicle["vehicle"]["ca_reference_thrust_n"])
    for index, engine in enumerate(engines[:engine_count]):
        if mask & (1 << index):
            fraction = engine.get("thrust_fraction", 1.0 / max(engine_count, 1))
            thrust = reference * fraction * max(scales[index], 0.0)
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

    max_thrust, min_thrust = thrust_envelope_from_reference(manifest, motor_reference)
    if required_thrust_n > max_thrust + 1e-3 or required_thrust_n < min_thrust - 1e-3:
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
    if axis == engine.pitch_axis:
        plant = plant_torque(engine, max_angle_deg, 0.0, 0.0)
    else:
        plant = plant_torque(engine, 0.0, max_angle_deg, 0.0)
    return flight, plant