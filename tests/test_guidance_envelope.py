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


envelope = load_module(REPO_ROOT / "tools/tv3_guidance_envelope.py")

LANDER = REPO_ROOT / "config/vehicles/tv3_lander_v1.yaml"
HOVER_PROFILE = REPO_ROOT / "config/flight_profiles/lander_hover_window.yaml"
IMPOSSIBLE_PROFILE = REPO_ROOT / "config/flight_profiles/lander_impossible_guidance.yaml"


class GuidanceEnvelopeTests(unittest.TestCase):
    def test_hover_profile_has_valid_envelope(self) -> None:
        result = envelope.evaluate_profile_case(
            LANDER,
            HOVER_PROFILE,
            phase=envelope.PHASE_LAUNCH_ASCENT,
            thrust_n=620.0,
            state=envelope.GuidanceVehicleState(
                phase=envelope.PHASE_LAUNCH_ASCENT,
                altitude_m=8.0,
                required_thrust_n=620.0,
            ),
        )
        self.assertTrue(result.solution_valid, result)

    def test_impossible_profile_rejects_solution(self) -> None:
        result = envelope.evaluate_profile_case(
            LANDER,
            IMPOSSIBLE_PROFILE,
            phase=envelope.PHASE_LAUNCH_ASCENT,
            thrust_n=620.0,
        )
        self.assertFalse(result.solution_valid)
        self.assertIn(
            result.guidance_unreachable_reason,
            {envelope.GUIDANCE_IMPULSE, envelope.GUIDANCE_THRUST_MARGIN},
        )

    def test_impossible_profile_rejects_impulse_reserve(self) -> None:
        config = envelope.load_guidance_config(envelope.load_flight_profile(IMPOSSIBLE_PROFILE))
        vehicle = envelope.load_manifest(LANDER)
        motor_reference = envelope.motor_reference_for_state(vehicle, thrust_n=620.0)
        config.min_remaining_impulse_ns = 100.0
        state = envelope.GuidanceVehicleState(
            phase=envelope.PHASE_LAUNCH_ASCENT,
            required_thrust_n=620.0,
            remaining_impulse_ns=50.0,
        )
        result = envelope.evaluate_envelope(vehicle, config, motor_reference, state)
        self.assertFalse(result.solution_valid)
        self.assertEqual(envelope.GUIDANCE_IMPULSE, result.guidance_unreachable_reason)

    def test_landing_reserve_rejects_high_altitude_with_low_delta_v(self) -> None:
        config = envelope.load_guidance_config(envelope.load_flight_profile(HOVER_PROFILE))
        vehicle = envelope.load_manifest(LANDER)
        motor_reference = envelope.motor_reference_for_state(
            vehicle,
            thrust_n=620.0,
            mass_kg=50.0,
        )
        state = envelope.GuidanceVehicleState(
            phase=envelope.PHASE_LANDING_APPROACH,
            altitude_m=120.0,
            mission_started=True,
            required_thrust_n=620.0,
            landing_point_ned=(0.0, 0.0, 0.0),
            remaining_impulse_ns=200.0,
        )
        result = envelope.evaluate_envelope(vehicle, config, motor_reference, state)
        self.assertFalse(result.solution_valid)
        self.assertEqual(envelope.GUIDANCE_LANDING_RESERVE, result.guidance_unreachable_reason)

    def test_abort_corridor_rejects_far_offset_with_low_delta_v(self) -> None:
        config = envelope.load_guidance_config(envelope.load_flight_profile(HOVER_PROFILE))
        vehicle = envelope.load_manifest(LANDER)
        motor_reference = envelope.motor_reference_for_state(vehicle, thrust_n=620.0)
        state = envelope.GuidanceVehicleState(
            phase=envelope.PHASE_LAUNCH_ASCENT,
            altitude_m=5.0,
            position_ned=(500.0, 300.0, -5.0),
            velocity_sp=(2.0, 1.0, -1.0),
            mission_started=True,
            required_thrust_n=620.0,
            landing_point_ned=(0.0, 0.0, 0.0),
            remaining_impulse_ns=15.0,
        )
        result = envelope.evaluate_envelope(vehicle, config, motor_reference, state)
        self.assertFalse(result.solution_valid)
        self.assertEqual(envelope.GUIDANCE_ABORT_CORRIDOR, result.guidance_unreachable_reason)

    def test_excessive_lateral_command_rejects_control_envelope(self) -> None:
        config = envelope.load_guidance_config(envelope.load_flight_profile(HOVER_PROFILE))
        vehicle = envelope.load_manifest(LANDER)
        motor_reference = envelope.motor_reference_for_state(vehicle, thrust_n=620.0)
        state = envelope.GuidanceVehicleState(
            phase=envelope.PHASE_WAYPOINT_TRACK,
            altitude_m=10.0,
            velocity_sp=(30.0, 0.0, 0.0),
            mission_started=True,
            required_thrust_n=620.0,
            landing_point_ned=(0.0, 0.0, 0.0),
        )
        result = envelope.evaluate_envelope(vehicle, config, motor_reference, state)
        self.assertFalse(result.solution_valid)
        self.assertEqual(envelope.GUIDANCE_CONTROL, result.guidance_unreachable_reason)

    def test_monte_carlo_reports_mixed_valid_and_invalid_samples(self) -> None:
        report = envelope.run_monte_carlo(
            LANDER,
            HOVER_PROFILE,
            samples=24,
            seed=11,
            phase=envelope.PHASE_WAYPOINT_TRACK,
            altitude_m=12.0,
            required_thrust_n=620.0,
        )
        self.assertEqual(24, report.samples)
        self.assertGreater(report.valid_count, 0)
        self.assertGreater(report.invalid_count, 0)
        self.assertTrue(report.passed)


if __name__ == "__main__":
    unittest.main()