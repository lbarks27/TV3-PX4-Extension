from __future__ import annotations

import unittest
from pathlib import Path
from tempfile import TemporaryDirectory

from tests.support import REPO_ROOT, ensure_minimal_ulog, load_module


def rerun_available() -> bool:
    try:
        import rerun  # noqa: F401

        return True
    except ImportError:
        return False


class RerunReplayTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.replay_module = load_module(REPO_ROOT / "tools/tv3_replay.py")
        cls.engine_module = load_module(REPO_ROOT / "tools/plot_ulog_engines.py")
        cls.rerun_module = load_module(REPO_ROOT / "tools/rerun_replay.py")
        cls.minimal_log = ensure_minimal_ulog()
        cls.mesh_path = REPO_ROOT / "assets/meshes/tv3_lander_v1.obj"
        if not cls.mesh_path.exists():
            import subprocess

            subprocess.run(
                ["python3", str(REPO_ROOT / "tools/generate_vehicle_mesh.py")],
                check=True,
            )

    def test_vehicle_mesh_exists(self) -> None:
        self.assertTrue(self.mesh_path.exists())
        self.assertTrue(self.mesh_path.with_suffix(".mtl").exists())

    def test_sim_time_timeline_name(self) -> None:
        self.assertEqual(self.rerun_module.SIM_TIME_TIMELINE, "sim_time")

    @unittest.skipUnless(rerun_available(), "rerun-sdk not installed (run ./scripts/setup_viz_env.sh)")
    def test_replay_unified_writes_rrd(self) -> None:
        from pyulog import ULog

        from tools.tv3_control_allocator import engines_from_vehicle

        ulog = ULog(str(self.minimal_log))
        manifest = self.engine_module.resolve_manifest(self.minimal_log, None)
        trajectory_frames = self.replay_module.build_trajectory_frames(ulog, fps=5.0, stride=4)
        guidance_frames = self.replay_module.build_guidance_frames(ulog, fps=5.0, stride=4)
        engine_frames = self.engine_module.build_replay_frames(ulog, manifest, fps=5.0, stride=4)
        engines = engines_from_vehicle(manifest)
        with TemporaryDirectory() as tmp:
            output = Path(tmp) / "unified.rrd"
            self.rerun_module.replay_unified(
                trajectory_frames,
                guidance_frames,
                engine_frames,
                engines,
                manifest,
                self.minimal_log.name,
                axis_length=0.12,
                build_stage=3,
                recording_path=output,
                spawn=False,
            )
            self.assertTrue(output.exists())
            self.assertGreater(output.stat().st_size, 256)


if __name__ == "__main__":
    unittest.main()
