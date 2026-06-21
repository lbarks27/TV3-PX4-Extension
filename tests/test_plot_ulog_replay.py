from __future__ import annotations

import unittest

from tests.support import MINIMAL_ULOG, REPO_ROOT, ensure_minimal_ulog, load_module


class PlotUlogReplayTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.replay = load_module(REPO_ROOT / "tools/tv3_replay.py")
        cls.common = load_module(REPO_ROOT / "tools/ulog_replay_common.py")
        cls.minimal_log = ensure_minimal_ulog()

    def test_ned_to_plot_xyz_inverts_down_for_altitude(self) -> None:
        north, east, up = self.common.ned_to_plot_xyz(1.0, 2.0, -5.0)
        self.assertEqual((north, east, up), (1.0, 2.0, 5.0))
        self.assertEqual(self.common.altitude_from_ned(-5.0), 5.0)

    def test_native_replay_sampling_matches_attitude_rate(self) -> None:
        from pyulog import ULog

        ulog = ULog(str(self.minimal_log))
        native = self.replay.build_trajectory_frames(ulog, fps=0.0, stride=1)
        legacy = self.replay.build_trajectory_frames(ulog, fps=10.0, stride=1)
        self.assertGreater(len(native), len(legacy))
        span = native[-1].time_s - native[0].time_s
        effective_hz = (len(native) - 1) / span
        self.assertGreater(effective_hz, 45.0)
        self.assertLess(effective_hz, 55.0)

    def test_mode_and_phase_labels_cover_enums(self) -> None:
        self.assertEqual(self.common.TV3_MODE_LABELS[4], "BOOST")
        self.assertEqual(self.common.GUIDANCE_PHASE_LABELS[3], "WAYPOINT_TRACK")

    def test_build_trajectory_frames_from_minimal_log(self) -> None:
        from pyulog import ULog

        ulog = ULog(str(self.minimal_log))
        frames = self.replay.build_trajectory_frames(ulog, fps=5.0, stride=2)
        self.assertGreater(len(frames), 10)
        max_alt = max(self.common.altitude_from_ned(frame.position_ned[2]) for frame in frames)
        self.assertGreater(max_alt, 1.0)

    def test_build_guidance_frames_from_minimal_log(self) -> None:
        from pyulog import ULog

        ulog = ULog(str(self.minimal_log))
        frames = self.replay.build_guidance_frames(ulog, fps=5.0, stride=2)
        self.assertGreater(len(frames), 10)
        margins = [frame.thrust_margin_n for frame in frames]
        self.assertTrue(any(abs(value) > 0.0 for value in margins))
        self.assertTrue(any(frame.mode >= 0 for frame in frames))


if __name__ == "__main__":
    unittest.main()
