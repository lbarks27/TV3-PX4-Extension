from __future__ import annotations

import importlib.util
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
LANDER = REPO_ROOT / "config/vehicles/tv3_lander_v1.yaml"
ASCENT = REPO_ROOT / "config/vehicles/tv3_v1.yaml"


class ControlAllocatorTests(unittest.TestCase):
    def setUp(self) -> None:
        self.lander = allocator.load_manifest(LANDER)
        self.ascent = allocator.load_manifest(ASCENT)
        self.lander_engines = allocator.engines_from_vehicle(self.lander)
        self.ascent_engines = allocator.engines_from_vehicle(self.ascent)

    def test_nominal_hover_is_reachable(self) -> None:
        result = allocator.allocate(self.lander_engines, (0.0, 0.0, 0.0), 620.0)
        self.assertTrue(result.reachable, result)
        self.assertEqual(allocator.REASON_NONE, result.reason)

    def test_low_thrust_outside_splay_envelope_is_unreachable(self) -> None:
        result = allocator.allocate(self.lander_engines, (0.0, 0.0, 0.0), 100.0)
        self.assertFalse(result.reachable)
        self.assertEqual(allocator.REASON_THRUST_ENVELOPE, result.reason)
        self.assertEqual(allocator.CONTROL_THRUST_ENVELOPE, result.control_unreachable_reason)

    def test_high_thrust_above_full_envelope_is_unreachable(self) -> None:
        result = allocator.allocate(self.lander_engines, (0.0, 0.0, 0.0), 900.0)
        self.assertFalse(result.reachable)
        self.assertEqual(allocator.REASON_THRUST_ENVELOPE, result.reason)

    def test_failed_engine_reduces_envelope(self) -> None:
        all_engines = allocator.allocate(self.lander_engines, (0.0, 0.0, 0.0), 620.0, active_mask=0b111)
        one_failed = allocator.allocate(
            self.lander_engines,
            (0.0, 0.0, 0.0),
            620.0,
            active_mask=0b011,
        )
        self.assertTrue(all_engines.reachable, all_engines)
        self.assertFalse(one_failed.reachable, one_failed)
        self.assertEqual(allocator.REASON_THRUST_ENVELOPE, one_failed.reason)

    def test_burnout_scaled_thrust_still_hovers(self) -> None:
        result = allocator.allocate(
            self.lander_engines,
            (0.0, 0.0, 0.0),
            380.0,
            thrust_scales=[0.55, 0.55, 0.55],
        )
        self.assertTrue(result.reachable, result)

    def test_burnout_scaled_thrust_cannot_meet_high_demand(self) -> None:
        result = allocator.allocate(
            self.lander_engines,
            (0.0, 0.0, 0.0),
            620.0,
            thrust_scales=[0.35, 0.35, 0.35],
        )
        self.assertFalse(result.reachable)
        self.assertEqual(allocator.REASON_THRUST_ENVELOPE, result.reason)

    def test_saturated_torque_demand_is_unreachable(self) -> None:
        limits = allocator.torque_limits_from_vehicle(self.lander)
        demand = (0.0, limits.pitch_nm * 2.0, 0.0)
        result = allocator.allocate(self.lander_engines, demand, 620.0)
        self.assertFalse(result.reachable)
        self.assertEqual(allocator.REASON_TORQUE_UNREACHABLE, result.reason)

    def test_single_engine_ascent_nominal_torque(self) -> None:
        result = allocator.allocate(self.ascent_engines, (0.0, 0.5, 0.0), 250.0)
        self.assertTrue(result.reachable, result)

    def test_unreachable_results_include_residuals(self) -> None:
        result = allocator.allocate(self.lander_engines, (0.0, 20.0, 0.0), 620.0)
        self.assertFalse(result.reachable)
        self.assertGreater(result.torque_error_nm, 0.5)
        self.assertGreater(allocator.norm(result.unallocated_torque_nm), 0.5)

    def test_flight_and_plant_models_agree_at_small_deflection(self) -> None:
        engine = self.ascent_engines[0]
        angle_deg = 2.0
        flight = allocator.flight_effectiveness_torque(
            engine,
            gimbal_axis=engine.pitch_axis,
            max_angle_deg=angle_deg,
            reference_thrust_n=engine.thrust_n,
        )
        plant = allocator.plant_torque(engine, angle_deg, 0.0, 0.0)
        for axis_index in range(3):
            self.assertAlmostEqual(flight[axis_index], plant[axis_index], places=2)

    def test_guidance_hover_reachability_uses_active_thrust(self) -> None:
        motor_reference = allocator.motor_reference_from_thrust(
            self.lander,
            thrust_n=620.0,
            active_mask=0b111,
        )
        result = allocator.guidance_reachability(
            self.lander,
            motor_reference,
            required_thrust_n=620.0,
            velocity_sp=(0.0, 0.0, 0.0),
        )
        self.assertTrue(result.reachable, result)

    def test_guidance_rejects_lateral_demand_beyond_torque_limits(self) -> None:
        motor_reference = allocator.motor_reference_from_thrust(
            self.lander,
            thrust_n=620.0,
            active_mask=0b111,
        )
        result = allocator.guidance_reachability(
            self.lander,
            motor_reference,
            required_thrust_n=620.0,
            velocity_sp=(25.0, 0.0, 0.0),
            position_gain=0.5,
        )
        self.assertFalse(result.reachable)
        self.assertEqual(allocator.REASON_TORQUE_UNREACHABLE, result.reason)


if __name__ == "__main__":
    unittest.main()