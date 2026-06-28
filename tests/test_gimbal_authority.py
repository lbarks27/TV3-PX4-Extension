#!/usr/bin/env python3

import math
import unittest

from tools.tv3_control_allocator import (
    engines_from_vehicle,
    load_manifest,
    plant_total_wrench,
    scaled_engines,
)


def scale_torque_preserve_direction(demand_nm, positive_limit_nm, negative_limit_nm):
    scaled = [0.0, 0.0, 0.0]
    min_axis_scale = 1.0
    for axis, demand in enumerate(demand_nm):
        if abs(demand) < 1e-4:
            continue
        limit = positive_limit_nm[axis] if demand >= 0.0 else negative_limit_nm[axis]
        if limit <= 1e-4:
            min_axis_scale = 0.0
            continue
        if abs(demand) > limit:
            scaled[axis] = math.copysign(limit, demand)
            min_axis_scale = min(min_axis_scale, limit / abs(demand))
        else:
            scaled[axis] = demand
    min_axis_scale = max(0.0, min(1.0, min_axis_scale))
    return scaled, min_axis_scale


def torque_wrench_aligned(desired_nm, achieved_nm, min_demand_nm=0.05):
    desired_norm = math.sqrt(sum(value * value for value in desired_nm))
    achieved_norm = math.sqrt(sum(value * value for value in achieved_nm))
    if desired_norm < min_demand_nm:
        return True
    if achieved_norm < min_demand_nm:
        return False
    if sum(d * a for d, a in zip(desired_nm, achieved_nm)) < 0.0:
        return False
    for demand, achieved in zip(desired_nm, achieved_nm):
        if abs(demand) < min_demand_nm:
            continue
        if abs(achieved) < min_demand_nm or demand * achieved < 0.0:
            return False
    return True


class GimbalAuthorityTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.vehicle = load_manifest("config/vehicles/tv3_lander_v1.json")
        cls.engines = engines_from_vehicle(cls.vehicle)[:3]

    def test_scale_preserves_direction(self) -> None:
        demand = (6.0, -4.0, 0.0)
        positive = (2.0, 1.5, 0.5)
        negative = (2.0, 1.0, 0.5)
        scaled, scale = scale_torque_preserve_direction(demand, positive, negative)
        self.assertAlmostEqual(scale, 0.25)
        self.assertAlmostEqual(scaled[0], 2.0)
        self.assertAlmostEqual(scaled[1], -1.0)
        self.assertAlmostEqual(scaled[2], 0.0)

    def test_dead_axis_does_not_zero_other_axes(self) -> None:
        demand = (6.0, 2.0, 0.0)
        positive = (2.0, 0.0, 0.0)
        negative = (2.0, 1.0, 0.0)
        scaled, _scale = scale_torque_preserve_direction(demand, positive, negative)
        self.assertAlmostEqual(scaled[0], 2.0)
        self.assertAlmostEqual(scaled[1], 0.0)
        self.assertAlmostEqual(scaled[2], 0.0)

    def test_infeasible_demand_scales_down_not_across(self) -> None:
        demand = (8.0, 8.0, 0.0)
        positive = (1.0, 1.0, 0.0)
        negative = (1.0, 1.0, 0.0)
        scaled, scale = scale_torque_preserve_direction(demand, positive, negative)
        self.assertAlmostEqual(scale, 0.125)
        self.assertAlmostEqual(scaled[0], 1.0)
        self.assertAlmostEqual(scaled[1], 1.0)

    def test_direction_check_rejects_opposing_torque(self) -> None:
        self.assertFalse(torque_wrench_aligned((2.0, 0.0, 0.0), (-1.0, 0.0, 0.0)))
        self.assertTrue(torque_wrench_aligned((2.0, 0.0, 0.0), (1.5, 0.0, 0.0)))
        self.assertTrue(torque_wrench_aligned((0.0, 0.0, 0.0), (-1.0, 0.0, 0.0)))

    def test_boost_envelope_is_much_smaller_than_rk_tq_limits(self) -> None:
        thrusts = [24.0, 25.0, 24.0]
        engines = [
            engine.__class__(**{**engine.__dict__})
            for engine in self.engines
        ]
        for engine in engines:
            object.__setattr__(engine, "roll_max_deg", 8.0)
            object.__setattr__(engine, "roll_min_deg", -8.0)
            object.__setattr__(engine, "yaw_max_deg", 8.0)
            object.__setattr__(engine, "yaw_min_deg", -8.0)
        scaled = scaled_engines(
            engines,
            thrust_scales=[thrust / engine.thrust_n for engine, thrust in zip(engines, thrusts)],
        )
        max_roll = 0.0
        for pitch in (-8.0, 0.0, 8.0):
            for yaw in (-8.0, 0.0, 8.0):
                torque, _ = plant_total_wrench(scaled, [(pitch, yaw), (0.0, 0.0), (0.0, 0.0)])
                max_roll = max(max_roll, abs(torque[0]))
        limits = self.vehicle["vehicle"]["torque_limits_nm"]
        self.assertLess(max_roll, limits["roll"])


if __name__ == "__main__":
    unittest.main()