#!/usr/bin/env python3

from __future__ import annotations

import argparse
import itertools
import json
import math
from dataclasses import dataclass
from pathlib import Path

import yaml


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


def dot(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def cross(a: tuple[float, float, float], b: tuple[float, float, float]) -> tuple[float, float, float]:
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def scale(a: tuple[float, float, float], value: float) -> tuple[float, float, float]:
    return (a[0] * value, a[1] * value, a[2] * value)


def add(a: tuple[float, float, float], b: tuple[float, float, float]) -> tuple[float, float, float]:
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def norm(a: tuple[float, float, float]) -> float:
    return math.sqrt(dot(a, a))


def normalize(a: tuple[float, float, float], fallback: tuple[float, float, float]) -> tuple[float, float, float]:
    length = norm(a)
    if length <= 1e-9:
        return fallback
    return scale(a, 1.0 / length)


def small_angle_torque(
    engine: EngineGeometry,
    pitch_deg: float,
    yaw_deg: float,
    splay_deg: float,
) -> tuple[float, float, float]:
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


def allocate(
    engines: list[EngineGeometry],
    desired_torque_nm: tuple[float, float, float],
    desired_thrust_n: float,
    grid_steps: int = 5,
) -> dict:
    """Coarse bounded allocator for host validation and reachability checks."""
    if not engines:
        return {"reachable": False, "reason": "no engines"}

    full_thrust = sum(engine.thrust_n for engine in engines)
    min_splayed_thrust = sum(engine.thrust_n * math.cos(math.radians(engine.splay_max_deg)) for engine in engines)
    if desired_thrust_n > full_thrust + 1e-6 or desired_thrust_n < min_splayed_thrust - 1e-6:
        return {
            "reachable": False,
            "reason": "net thrust outside splay envelope",
            "full_thrust_n": full_thrust,
            "min_splayed_thrust_n": min_splayed_thrust,
        }

    best: dict | None = None
    for commands in itertools.product(*(command_grid(engine, grid_steps) for engine in engines)):
        torque = (0.0, 0.0, 0.0)
        thrust = 0.0
        for engine, command in zip(engines, commands):
            pitch, yaw, splay = command
            torque = add(torque, small_angle_torque(engine, pitch, yaw, splay))
            thrust += engine.thrust_n * math.cos(math.radians(splay))

        torque_error = norm((
            torque[0] - desired_torque_nm[0],
            torque[1] - desired_torque_nm[1],
            torque[2] - desired_torque_nm[2],
        ))
        thrust_error = abs(thrust - desired_thrust_n)
        score = torque_error + thrust_error * 0.02

        if best is None or score < best["score"]:
            best = {
                "score": score,
                "torque_error_nm": torque_error,
                "thrust_error_n": thrust_error,
                "achieved_torque_nm": torque,
                "achieved_thrust_n": thrust,
                "commands": commands,
            }

    assert best is not None
    best["reachable"] = best["torque_error_nm"] <= 0.5 and best["thrust_error_n"] <= max(1.0, desired_thrust_n * 0.05)
    return best


def command_grid(engine: EngineGeometry, steps: int) -> list[tuple[float, float, float]]:
    def values(limit: float) -> list[float]:
        if steps <= 1 or limit <= 0:
            return [0.0]
        return [-limit, 0.0, limit]

    splay_values = [0.0, engine.splay_max_deg]
    return [(pitch, yaw, splay) for pitch in values(engine.pitch_max_deg) for yaw in values(engine.yaw_max_deg) for splay in splay_values]


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
                "gimbal": {"pitch_max_deg": body["tvc_max_deg"], "yaw_max_deg": body["tvc_max_deg"], "splay_max_deg": body["tvc_max_deg"]},
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
        )
        for engine in engines
    ]


def main() -> None:
    parser = argparse.ArgumentParser(description="Coarse TV3 rocket allocator reachability check")
    parser.add_argument("--vehicle", type=Path, required=True)
    parser.add_argument("--torque", nargs=3, type=float, default=(0.0, 0.0, 0.0))
    parser.add_argument("--thrust", type=float, required=True)
    args = parser.parse_args()

    vehicle = yaml.safe_load(args.vehicle.read_text())
    result = allocate(engines_from_vehicle(vehicle), tuple(args.torque), args.thrust)
    print(json.dumps(result, indent=2, default=list))


if __name__ == "__main__":
    main()
