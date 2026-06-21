from __future__ import annotations

import unittest
from pathlib import Path

from tests.support import load_module


class VehicleManifestValidationTests(unittest.TestCase):
    def setUp(self) -> None:
        self.validator = load_module(Path("tools/validate_vehicle_manifest.py"))

    def test_both_vehicle_manifests_validate(self) -> None:
        reports = self.validator.validate_all()
        self.assertEqual(2, len(reports))
        for report in reports:
            with self.subTest(manifest=report.manifest):
                self.assertTrue(report.passed, [check for check in report.checks if not check.passed])

    def test_manifests_are_not_flight_ready(self) -> None:
        reports = self.validator.validate_all()
        for report in reports:
            with self.subTest(manifest=report.manifest):
                self.assertFalse(report.metrics["flight_ready"])

    def test_placeholder_fields_are_tracked(self) -> None:
        reports = self.validator.validate_all()
        for report in reports:
            with self.subTest(manifest=report.manifest):
                self.assertGreaterEqual(report.metrics["tracked_fields"], 8)
                self.assertGreater(report.metrics["non_measured_fields"], 0)

    def test_param_parity_checks_pass(self) -> None:
        reports = self.validator.validate_all()
        for report in reports:
            parity_checks = [check for check in report.checks if check.name.startswith("parity.")]
            with self.subTest(manifest=report.manifest):
                self.assertTrue(parity_checks)
                self.assertTrue(all(check.passed for check in parity_checks), parity_checks)

    def test_unit_vector_validation_rejects_bad_axis(self) -> None:
        manifest = self.validator.load_json(Path("config/vehicles/tv3_v1.json"))
        manifest["propulsion"]["engines"][0]["thrust_axis"] = [2.0, 0.0, 0.0]
        schema = self.validator.load_json(self.validator.DEFAULT_SCHEMA)
        report = self.validator.validate_manifest(manifest, Path("bad.json"), schema)
        failed = [check.name for check in report.checks if not check.passed]
        self.assertIn("engine_0.thrust_axis", failed)

    def test_generator_rejects_invalid_manifest(self) -> None:
        generator = load_module(Path("tools/generate_vehicle_assets.py"))
        manifest = generator.load_vehicle(Path("config/vehicles/tv3_v1.json"))
        manifest.pop("data_status")
        with self.assertRaises(ValueError):
            generator.validate_vehicle(manifest, Path("config/vehicles/tv3_v1.json"))


if __name__ == "__main__":
    unittest.main()