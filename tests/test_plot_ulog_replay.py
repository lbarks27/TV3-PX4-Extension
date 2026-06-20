from __future__ import annotations

import importlib.util
import unittest
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
SPLAY_LOG = REPO_ROOT / "logs/sim/2026-06-20/20260620T033407Z-splay-secondary/03_34_13.ulg"
ROLL_LOG = REPO_ROOT / "logs/sim/2026-06-20/20260620T030451Z-roll-180-v2/03_04_57.ulg"


def load_module(path: Path):
    spec = importlib.util.spec_from_file_location(path.stem, path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    import sys

    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


class PlotUlogReplayTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.replay = load_module(REPO_ROOT / "tools/plot_ulog_replay.py")
        cls.common = load_module(REPO_ROOT / "tools/ulog_replay_common.py")

    def test_ned_to_plot_xyz_inverts_down_for_altitude(self) -> None:
        north, east, up = self.common.ned_to_plot_xyz(1.0, 2.0, -5.0)
        self.assertEqual((north, east, up), (1.0, 2.0, 5.0))
        self.assertEqual(self.common.altitude_from_ned(-5.0), 5.0)

    def test_mode_and_phase_labels_cover_enums(self) -> None:
        self.assertEqual(self.common.TV3_MODE_LABELS[4], "BOOST")
        self.assertEqual(self.common.GUIDANCE_PHASE_LABELS[3], "WAYPOINT_TRACK")

    @unittest.skipUnless(SPLAY_LOG.exists(), "splay-throttle ULog not present locally")
    def test_build_trajectory_frames_from_splay_log(self) -> None:
        from pyulog import ULog

        ulog = ULog(str(SPLAY_LOG))
        frames = self.replay.build_trajectory_frames(ulog, fps=5.0, stride=2)
        self.assertGreater(len(frames), 10)
        max_alt = max(self.common.altitude_from_ned(frame.position_ned[2]) for frame in frames)
        self.assertGreater(max_alt, 4.0)

    @unittest.skipUnless(SPLAY_LOG.exists(), "splay-throttle ULog not present locally")
    def test_build_guidance_frames_from_splay_log(self) -> None:
        from pyulog import ULog

        ulog = ULog(str(SPLAY_LOG))
        frames = self.replay.build_guidance_frames(ulog, fps=5.0, stride=2)
        self.assertGreater(len(frames), 10)
        margins = [frame.thrust_margin_n for frame in frames]
        self.assertTrue(any(abs(value) > 0.0 for value in margins))
        self.assertTrue(any(frame.mode >= 0 for frame in frames))

    @unittest.skipUnless(ROLL_LOG.exists(), "roll-180 ULog not present locally")
    def test_trajectory_frames_capture_roll_attitude(self) -> None:
        from pyulog import ULog

        ulog = ULog(str(ROLL_LOG))
        frames = self.replay.build_trajectory_frames(ulog, fps=5.0, stride=3)
        rolls = [self.common.euler_angles_deg(frame.quaternion)[0] for frame in frames]
        self.assertGreater(max(abs(roll) for roll in rolls), 30.0)


if __name__ == "__main__":
    unittest.main()