from __future__ import annotations

import json
import tempfile
import unittest
from pathlib import Path

from tests.support import REPO_ROOT, load_module


capture = load_module(REPO_ROOT / "tools/capture_bench_manifest.py")
VEHICLE = REPO_ROOT / "config/vehicles/tv3_v1.json"


class CaptureBenchManifestTests(unittest.TestCase):
    def test_update_manifest_promotes_hardware_and_motor_fields(self) -> None:
        manifest = json.loads(VEHICLE.read_text())
        bench_capture = capture.BenchCapture(
            captured_utc="2026-06-18T00:00:00+00:00",
            connect="test",
            vehicle=str(VEHICLE),
            target_system=1,
            target_component=1,
            params={
                "PWM_MAIN_FUNC1": 201.0,
                "PWM_MAIN_FUNC2": 202.0,
                "RK_LC_TARE": 1200.0,
                "RK_LC_SCALE": 0.05,
                "RK_LC_KG_SC": 0.0051,
                "RK_LC_ALPHA": 0.25,
                "RK_LC_TO_MS": 200.0,
            },
            load_cell_samples=[
                capture.LoadCellSample(raw_count=1201.0, mass_kg=0.0, thrust_n=0.0),
                capture.LoadCellSample(raw_count=1199.5, mass_kg=0.0, thrust_n=0.0),
            ],
        )

        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "tv3_v1.json"
            path.write_text(json.dumps(manifest, indent=2))
            capture.update_manifest_from_capture(
                manifest,
                bench_capture,
                body_mass_kg=1.03,
                known_mass_kg=None,
                tvc_max_deg=4.8,
                tvc_slew_dps=210.0,
                promote_flight_ready=False,
            )
            capture.save_manifest(path, manifest)

            fields = manifest["data_status"]["fields"]
            self.assertEqual("measured", fields["hardware.load_cell.calibration"])
            self.assertEqual("measured", fields["vehicle.body_mass_kg"])
            self.assertEqual("measured", fields["vehicle.motor_loaded_mass_kg"])
            self.assertAlmostEqual(manifest["vehicle"]["motor_loaded_mass_kg"], 0.15, places=3)
            self.assertAlmostEqual(manifest["vehicle"]["ca_reference_thrust_n"], 30.88, places=2)
            self.assertEqual(2, manifest["hardware"]["load_cell"]["i2c_bus"])
            self.assertFalse(manifest["data_status"]["flight_ready"])


if __name__ == "__main__":
    unittest.main()