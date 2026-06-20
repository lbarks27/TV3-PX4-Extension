from __future__ import annotations

import importlib.util
import unittest
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
CATALOG_ALIGN = REPO_ROOT / "logs/sim/2026-06-20/catalog-align-20260620T021609Z/02_16_20.ulg"


def load_module(path: Path):
    spec = importlib.util.spec_from_file_location(path.stem, path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    import sys

    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


class PlotUlogEnginesTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.module = load_module(REPO_ROOT / "tools/plot_ulog_engines.py")

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

    @unittest.skipUnless(CATALOG_ALIGN.exists(), "catalog-align ULog not present locally")
    def test_build_replay_frames_from_passing_log(self) -> None:
        from pyulog import ULog

        manifest = self.module.resolve_manifest(CATALOG_ALIGN, None)
        ulog = ULog(str(CATALOG_ALIGN))
        frames = self.module.build_replay_frames(ulog, manifest, fps=5.0, stride=2)
        self.assertGreater(len(frames), 10)
        self.assertEqual(3, len(frames[0].engine_thrust_n))
        max_thrust = max(max(frame.engine_thrust_n) for frame in frames)
        self.assertGreater(max_thrust, 5.0)
        max_alt = max(-frame.position_ned[2] for frame in frames)
        self.assertGreater(max_alt, 5.0)


if __name__ == "__main__":
    unittest.main()