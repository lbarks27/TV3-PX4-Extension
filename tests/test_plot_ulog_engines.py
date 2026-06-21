from __future__ import annotations

import unittest

from tests.support import REPO_ROOT, ensure_minimal_ulog, load_module


class PlotUlogEnginesTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.module = load_module(REPO_ROOT / "tools/plot_ulog_engines.py")
        cls.minimal_log = ensure_minimal_ulog()

    def test_rotation_matrix_is_orthogonal(self) -> None:
        matrix = self.module.rotation_matrix_from_quat((0.7071, 0.0, 0.7071, 0.0))
        identity = matrix @ matrix.T
        self.assertAlmostEqual(identity[0, 0], 1.0, places=3)
        self.assertAlmostEqual(identity[1, 1], 1.0, places=3)
        self.assertAlmostEqual(identity[2, 2], 1.0, places=3)

    def test_origin_view_is_tighter_than_full_vehicle_extent(self) -> None:
        manifest = self.module.load_manifest(REPO_ROOT / "config/vehicles/tv3_lander_v1.json")
        origin_half = self.module.origin_view_half_span(manifest, axis_length=0.12)
        self.assertLess(origin_half, 0.22)
        self.assertGreaterEqual(origin_half, 0.11)

    def test_build_replay_frames_from_minimal_log(self) -> None:
        from pyulog import ULog

        manifest = self.module.resolve_manifest(self.minimal_log, None)
        ulog = ULog(str(self.minimal_log))
        frames = self.module.build_replay_frames(ulog, manifest, fps=5.0, stride=2)
        self.assertGreater(len(frames), 10)
        self.assertEqual(3, len(frames[0].engine_thrust_n))
        max_thrust = max(max(frame.engine_thrust_n) for frame in frames)
        self.assertGreater(max_thrust, 0.0)


if __name__ == "__main__":
    unittest.main()
