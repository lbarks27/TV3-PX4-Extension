"""Incremental engine-frame axis construction for TV3 lander manifests.

Build order (one axis at a time):
  1. Thrust reference (+X body forward at zero gimbal)
  2. Primary axis (mount -> origin, +/-90 deg limits, manifest roll_axis)
  3. Secondary axis (perpendicular to thrust and primary, 0-135 deg limits, manifest yaw_axis)

Kinematics (in plant_thrust_direction): roll acts about the fixed primary axis; yaw acts
about the secondary axis after it has been carried through the current roll angle.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Sequence

from tools.tv3_control_allocator import (
    BODY_FORWARD_AXIS,
    mount_to_origin_axis,
    roll_axis_perpendicular,
)

MAX_BUILD_STAGE = 3


@dataclass(frozen=True)
class EngineFrameAxes:
    thrust_axis: tuple[float, float, float]
    primary_axis: tuple[float, float, float]
    secondary_axis: tuple[float, float, float]
    build_stage: int

    @property
    def roll_axis(self) -> tuple[float, float, float]:
        return self.primary_axis

    @property
    def yaw_axis(self) -> tuple[float, float, float]:
        return self.secondary_axis


def build_thrust_axis() -> tuple[float, float, float]:
    """Stage 1: nominal thrust direction at zero gimbal."""
    return BODY_FORWARD_AXIS


def build_primary_axis(position_m: Sequence[float]) -> tuple[float, float, float]:
    """Stage 2: primary gimbal hinge (+/-90 deg), mount toward origin."""
    return mount_to_origin_axis(position_m)


def build_secondary_axis(
    thrust_axis: Sequence[float],
    primary_axis: Sequence[float],
) -> tuple[float, float, float]:
    """Stage 3: secondary gimbal hinge (0-135 deg), perpendicular companion axis."""
    return roll_axis_perpendicular(thrust_axis, primary_axis)


def build_engine_frame_axes(position_m: Sequence[float], *, build_stage: int = MAX_BUILD_STAGE) -> EngineFrameAxes:
    stage = max(1, min(int(build_stage), MAX_BUILD_STAGE))
    thrust_axis = build_thrust_axis()
    primary_axis = build_primary_axis(position_m) if stage >= 2 else (0.0, -1.0, 0.0)
    secondary_axis = (
        build_secondary_axis(thrust_axis, primary_axis) if stage >= 3 else (0.0, 0.0, -1.0)
    )
    return EngineFrameAxes(
        thrust_axis=thrust_axis,
        primary_axis=primary_axis,
        secondary_axis=secondary_axis,
        build_stage=stage,
    )