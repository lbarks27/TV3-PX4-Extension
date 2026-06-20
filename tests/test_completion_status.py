from __future__ import annotations

import importlib.util
import json
import sys
import unittest
from pathlib import Path
from tempfile import TemporaryDirectory


def load_module(path: Path):
    spec = importlib.util.spec_from_file_location(path.stem, path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


class CompletionStatusTests(unittest.TestCase):
    def setUp(self) -> None:
        self.module = load_module(Path("tools/report_completion_status.py"))

    def test_default_status_has_all_phases_and_flight_gates(self) -> None:
        status = self.module.default_status()
        self.assertEqual("tv3_completion_status_v1", status["schema"])
        self.assertEqual(
            {phase["id"] for phase in self.module.PHASE_DEFINITIONS},
            set(status["phases"]),
        )
        self.assertEqual(
            {gate["id"] for gate in self.module.FLIGHT_GATE_DEFINITIONS},
            set(status["flight_gates"]),
        )

    def test_manifest_progress_counts_field_provenance(self) -> None:
        progress = self.module.manifest_progress(Path("config/vehicles/tv3_v1.json"))
        self.assertEqual("tv3_v1", progress.name)
        self.assertFalse(progress.flight_ready)
        self.assertGreater(progress.counts.get("measured", 0), 0)
        self.assertGreater(sum(progress.counts.values()), 0)

    def test_derive_status_phase2_uses_bench_evidence(self) -> None:
        manifests = [self.module.manifest_progress(Path("config/vehicles/tv3_v1.json"))]
        status, evidence = self.module.derive_status(
            "2",
            {"evidence": [], "status_override": None},
            "pass",
            manifests,
            [],
            ["logs/ground/bench_capture_example.json"],
            False,
        )
        self.assertEqual("in_progress", status)
        self.assertIn("logs/ground/bench_capture_example.json", evidence)

    def test_update_status_writes_json_and_markdown(self) -> None:
        with TemporaryDirectory() as tmp:
            repo = Path(tmp)
            (repo / "config" / "vehicles").mkdir(parents=True)
            (repo / "docs").mkdir(parents=True)
            (repo / "logs" / "ground").mkdir(parents=True)
            (repo / "config" / "vehicles" / "tv3_v1.json").write_text(
                Path("config/vehicles/tv3_v1.json").read_text()
            )

            original_root = self.module.REPO_ROOT
            original_status = self.module.STATUS_PATH
            original_doc = self.module.STATUS_DOC_PATH
            original_vehicle_dir = self.module.VEHICLE_DIR
            original_ground = self.module.GROUND_LOG_DIR
            original_sim = self.module.SIM_LOG_DIR
            original_px4 = self.module.PX4_SITL_BIN
            try:
                self.module.REPO_ROOT = repo
                self.module.STATUS_PATH = repo / "config/completion_status.json"
                self.module.STATUS_DOC_PATH = repo / "docs/completion_status.md"
                self.module.VEHICLE_DIR = repo / "config/vehicles"
                self.module.GROUND_LOG_DIR = repo / "logs/ground"
                self.module.SIM_LOG_DIR = repo / "logs/sim"
                self.module.PX4_SITL_BIN = repo / "missing/px4"

                status = self.module.update_status("none", 30, write_doc=True)
                self.assertTrue((repo / "config/completion_status.json").is_file())
                self.assertTrue((repo / "docs/completion_status.md").is_file())
                saved = json.loads((repo / "config/completion_status.json").read_text())
                self.assertEqual(status["schema"], saved["schema"])
                doc = (repo / "docs/completion_status.md").read_text()
                self.assertIn("Phase Dashboard", doc)
                self.assertIn("tv3_v1", doc)
            finally:
                self.module.REPO_ROOT = original_root
                self.module.STATUS_PATH = original_status
                self.module.STATUS_DOC_PATH = original_doc
                self.module.VEHICLE_DIR = original_vehicle_dir
                self.module.GROUND_LOG_DIR = original_ground
                self.module.SIM_LOG_DIR = original_sim
                self.module.PX4_SITL_BIN = original_px4


if __name__ == "__main__":
    unittest.main()