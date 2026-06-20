from __future__ import annotations

import json
import math
import unittest
from pathlib import Path

from tools.tv3_control_allocator import (
    axes_close,
    coupled_yaw_axis,
    dot,
    engines_from_vehicle,
    mount_to_origin_axis,
    norm,
    plant_thrust_direction,
    rotate_about_axis,
)
from tools.tv3_engine_frame import build_engine_frame_axes


class GimbalAxisTests(unittest.TestCase):
    def test_builder_produces_orthogonal_axes(self) -> None:
        manifest = json.loads(Path("config/vehicles/tv3_lander_v1.json").read_text())
        for engine in manifest["propulsion"]["engines"]:
            frame = build_engine_frame_axes(engine["position_m"])
            self.assertLess(abs(dot(frame.thrust_axis, frame.primary_axis)), 1e-6)
            self.assertLess(abs(dot(frame.thrust_axis, frame.secondary_axis)), 1e-6)
            self.assertLess(abs(dot(frame.primary_axis, frame.secondary_axis)), 1e-6)

    def test_primary_axis_points_toward_origin(self) -> None:
        manifest = json.loads(Path("config/vehicles/tv3_lander_v1.json").read_text())
        for engine in manifest["propulsion"]["engines"]:
            expected = mount_to_origin_axis(engine["position_m"])
            self.assertTrue(axes_close(engine["roll_axis"], expected), engine["id"])

    def test_manifest_matches_builder(self) -> None:
        manifest = json.loads(Path("config/vehicles/tv3_lander_v1.json").read_text())
        for engine in manifest["propulsion"]["engines"]:
            frame = build_engine_frame_axes(engine["position_m"])
            self.assertTrue(axes_close(engine["thrust_axis"], frame.thrust_axis), engine["id"])
            self.assertTrue(axes_close(engine["roll_axis"], frame.primary_axis), engine["id"])
            self.assertTrue(axes_close(engine["yaw_axis"], frame.secondary_axis), engine["id"])

    def test_yaw_deflection_vectors_thrust(self) -> None:
        manifest = json.loads(Path("config/vehicles/tv3_lander_v1.json").read_text())
        engines = engines_from_vehicle(manifest)
        for engine in engines:
            at_trim = plant_thrust_direction(engine, 0.0, 0.0)
            yawed = plant_thrust_direction(engine, 5.0, 20.0)
            self.assertGreater(norm((yawed[0] - at_trim[0], yawed[1] - at_trim[1], yawed[2] - at_trim[2])), 0.05)

    def test_yaw_axis_is_coupled_to_roll_not_vice_versa(self) -> None:
        manifest = json.loads(Path("config/vehicles/tv3_lander_v1.json").read_text())
        engine = engines_from_vehicle(manifest)[0]
        coupled = plant_thrust_direction(engine, 30.0, 20.0)
        decoupled = rotate_about_axis(
            rotate_about_axis(engine.thrust_axis, engine.roll_axis, math.radians(30.0)),
            engine.yaw_axis,
            math.radians(20.0),
        )
        self.assertFalse(axes_close(coupled, decoupled))
        self.assertFalse(axes_close(coupled_yaw_axis(engine, 30.0), engine.yaw_axis))

    def test_engines_from_vehicle_uses_builder_axes(self) -> None:
        manifest = json.loads(Path("config/vehicles/tv3_lander_v1.json").read_text())
        engines = engines_from_vehicle(manifest)
        manifest_engines = manifest["propulsion"]["engines"]
        self.assertEqual(3, len(engines))
        for engine, manifest_engine in zip(engines, manifest_engines):
            frame = build_engine_frame_axes(manifest_engine["position_m"])
            self.assertTrue(axes_close(engine.thrust_axis, frame.thrust_axis), manifest_engine["id"])
            self.assertTrue(axes_close(engine.roll_axis, frame.primary_axis), manifest_engine["id"])
            self.assertTrue(axes_close(engine.yaw_axis, frame.secondary_axis), manifest_engine["id"])


if __name__ == "__main__":
    unittest.main()