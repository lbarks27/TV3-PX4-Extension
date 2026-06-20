from __future__ import annotations

import importlib.util
import math
import sys
import unittest
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]


def load_module(path: Path):
    spec = importlib.util.spec_from_file_location(path.stem, path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


allocator = load_module(REPO_ROOT / "tools/tv3_control_allocator.py")
LANDER = REPO_ROOT / "config/vehicles/tv3_lander_v1.json"
ASCENT = REPO_ROOT / "config/vehicles/tv3_v1.json"


def full_thrust_n(engines) -> float:
    return sum(engine.thrust_n for engine in engines)


class ControlAllocatorTests(unittest.TestCase):
    def setUp(self) -> None:
        self.lander = allocator.load_manifest(LANDER)
        self.ascent = allocator.load_manifest(ASCENT)
        self.lander_engines = allocator.engines_from_vehicle(self.lander)
        self.ascent_engines = allocator.engines_from_vehicle(self.ascent)
        self.lander_hover_thrust_n = full_thrust_n(self.lander_engines)
        self.ascent_nominal_thrust_n = self.ascent_engines[0].thrust_n

    def test_collective_throttle_yaw_solver_matches_plant(self) -> None:
        full = sum(allocator.plant_axial_thrust(engine, 0.0, 0.0) for engine in self.lander_engines)
        for desired_frac in (1.0, 0.75, 0.5, 0.3):
            desired = full * desired_frac
            yaw = allocator.collective_throttle_yaw_deg(desired, self.lander_engines)
            achieved = sum(
                allocator.plant_axial_thrust(engine, 0.0, yaw) for engine in self.lander_engines
            )
            self.assertAlmostEqual(desired, achieved, delta=0.6)

    def test_nominal_hover_is_reachable(self) -> None:
        result = allocator.allocate(self.lander_engines, (0.0, 0.0, 0.0), self.lander_hover_thrust_n)
        self.assertTrue(result.reachable, result)
        self.assertEqual(allocator.REASON_NONE, result.reason)

    def test_partial_thrust_via_secondary_axis_is_reachable(self) -> None:
        yaw = allocator.collective_throttle_yaw_deg(40.0, self.lander_engines)
        achieved = sum(
            allocator.plant_axial_thrust(engine, 0.0, yaw) for engine in self.lander_engines
        )
        self.assertAlmostEqual(40.0, achieved, delta=0.6)
        self.assertGreater(yaw, 0.0)

    def test_high_thrust_above_full_envelope_is_unreachable(self) -> None:
        result = allocator.allocate(
            self.lander_engines,
            (0.0, 0.0, 0.0),
            full_thrust_n(self.lander_engines) * 1.5,
        )
        self.assertFalse(result.reachable)
        self.assertEqual(allocator.REASON_THRUST_ENVELOPE, result.reason)

    def test_failed_engine_reduces_envelope(self) -> None:
        all_engines = allocator.allocate(
            self.lander_engines,
            (0.0, 0.0, 0.0),
            self.lander_hover_thrust_n,
            active_mask=0b111,
        )
        one_failed = allocator.allocate(
            self.lander_engines,
            (0.0, 0.0, 0.0),
            self.lander_hover_thrust_n,
            active_mask=0b011,
        )
        self.assertTrue(all_engines.reachable, all_engines)
        self.assertFalse(one_failed.reachable, one_failed)
        self.assertEqual(allocator.REASON_THRUST_ENVELOPE, one_failed.reason)

    def test_burnout_scaled_thrust_still_hovers(self) -> None:
        scale = 0.55
        result = allocator.allocate(
            self.lander_engines,
            (0.0, 0.0, 0.0),
            self.lander_hover_thrust_n * scale,
            thrust_scales=[scale, scale, scale],
        )
        self.assertTrue(result.reachable, result)

    def test_burnout_scaled_thrust_cannot_meet_high_demand(self) -> None:
        result = allocator.allocate(
            self.lander_engines,
            (0.0, 0.0, 0.0),
            self.lander_hover_thrust_n,
            thrust_scales=[0.35, 0.35, 0.35],
        )
        self.assertFalse(result.reachable)
        self.assertEqual(allocator.REASON_THRUST_ENVELOPE, result.reason)

    def test_saturated_torque_demand_is_unreachable(self) -> None:
        limits = allocator.torque_limits_from_vehicle(self.lander)
        demand = (0.0, limits.pitch_nm * 2.0, 0.0)
        result = allocator.allocate(self.lander_engines, demand, self.lander_hover_thrust_n)
        self.assertFalse(result.reachable)
        self.assertEqual(allocator.REASON_TORQUE_UNREACHABLE, result.reason)

    def test_single_engine_ascent_nominal_torque(self) -> None:
        result = allocator.allocate(self.ascent_engines, (0.0, 0.5, 0.0), self.ascent_nominal_thrust_n)
        self.assertTrue(result.reachable, result)

    def test_unreachable_results_include_residuals(self) -> None:
        result = allocator.allocate(self.lander_engines, (0.0, 20.0, 0.0), self.lander_hover_thrust_n)
        self.assertFalse(result.reachable)
        self.assertGreater(result.torque_error_nm, 0.5)
        self.assertGreater(allocator.norm(result.unallocated_torque_nm), 0.5)

    def test_flight_and_plant_models_agree_at_small_deflection(self) -> None:
        engine = self.ascent_engines[0]
        angle_deg = 2.0
        flight = allocator.flight_effectiveness_torque(
            engine,
            gimbal_axis=engine.roll_axis,
            max_angle_deg=angle_deg,
            reference_thrust_n=engine.thrust_n,
        )
        plant = allocator.plant_torque(engine, angle_deg, 0.0)
        for axis_index in range(3):
            self.assertAlmostEqual(flight[axis_index], plant[axis_index], places=2)

    def test_guidance_hover_reachability_uses_active_thrust(self) -> None:
        hover_thrust_n = full_thrust_n(self.lander_engines)
        motor_reference = allocator.motor_reference_from_thrust(
            self.lander,
            thrust_n=hover_thrust_n,
            active_mask=0b111,
        )
        result = allocator.guidance_reachability(
            self.lander,
            motor_reference,
            required_thrust_n=hover_thrust_n,
            velocity_sp=(0.0, 0.0, 0.0),
        )
        self.assertTrue(result.reachable, result)

    def test_guidance_rejects_lateral_demand_beyond_torque_limits(self) -> None:
        hover_thrust_n = full_thrust_n(self.lander_engines)
        motor_reference = allocator.motor_reference_from_thrust(
            self.lander,
            thrust_n=hover_thrust_n,
            active_mask=0b111,
        )
        result = allocator.guidance_reachability(
            self.lander,
            motor_reference,
            required_thrust_n=hover_thrust_n,
            velocity_sp=(25.0, 0.0, 0.0),
            position_gain=0.5,
        )
        self.assertFalse(result.reachable)
        self.assertEqual(allocator.REASON_TORQUE_UNREACHABLE, result.reason)


if __name__ == "__main__":
    unittest.main()