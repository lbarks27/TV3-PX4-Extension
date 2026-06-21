from __future__ import annotations

import csv
import json
import sys
import unittest
from pathlib import Path
from tempfile import TemporaryDirectory

from tests.support import REPO_ROOT, load_module


def write_csv(path: Path, fieldnames: list[str], rows: list[dict[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


class MotorCatalogUnitTests(unittest.TestCase):
    def test_generate_catalog_rejects_invalid_motor(self) -> None:
        module = load_module(Path("tools/generate_motor_catalog.py"))

        with TemporaryDirectory() as tmp:
            root = Path(tmp)
            source = root / "source"
            output = root / "output"

            write_csv(
                source / "motor_inventory.csv",
                [
                    "manufacturer",
                    "designation",
                    "dynamics_file",
                    "specs_file",
                ],
                [
                    {
                        "manufacturer": "GoodCo",
                        "designation": "G100",
                        "dynamics_file": "GoodCo/G100/curve.csv",
                        "specs_file": "GoodCo/G100/specs.csv",
                    },
                    {
                        "manufacturer": "BadCo",
                        "designation": "B200",
                        "dynamics_file": "BadCo/B200/curve.csv",
                        "specs_file": "BadCo/B200/specs.csv",
                    },
                ],
            )

            write_csv(
                source / "GoodCo/G100/specs.csv",
                ["initial_mass_g", "dry_mass_g", "diameter_mm", "length_mm"],
                [{"initial_mass_g": 100.0, "dry_mass_g": 60.0, "diameter_mm": 54.0, "length_mm": 240.0}],
            )
            write_csv(
                source / "GoodCo/G100/curve.csv",
                ["time_s", "thrust_N", "motor_mass_kg"],
                [
                    {"time_s": 0.0, "thrust_N": 0.0, "motor_mass_kg": 0.10},
                    {"time_s": 0.5, "thrust_N": 150.0, "motor_mass_kg": 0.08},
                    {"time_s": 1.0, "thrust_N": 0.0, "motor_mass_kg": 0.06},
                ],
            )

            write_csv(
                source / "BadCo/B200/specs.csv",
                ["initial_mass_g", "dry_mass_g", "diameter_mm", "length_mm"],
                [{"initial_mass_g": 90.0, "dry_mass_g": -10.0, "diameter_mm": 54.0, "length_mm": 220.0}],
            )
            write_csv(
                source / "BadCo/B200/curve.csv",
                ["time_s", "thrust_N", "motor_mass_kg"],
                [
                    {"time_s": 0.0, "thrust_N": 0.0, "motor_mass_kg": 0.09},
                    {"time_s": 0.3, "thrust_N": 120.0, "motor_mass_kg": -0.01},
                ],
            )

            report = module.generate_catalog(source, output)

            self.assertEqual(report["summary"]["motors_total"], 2)
            self.assertEqual(report["summary"]["motors_active"], 1)
            self.assertEqual(report["summary"]["motors_rejected"], 1)

            with (output / "catalog.csv").open() as stream:
                catalog_rows = list(csv.DictReader(stream))
            self.assertEqual(catalog_rows[0]["active"], "1")
            self.assertEqual(catalog_rows[1]["active"], "0")
            self.assertTrue("dry mass" in catalog_rows[1]["errors"] or "negative motor mass" in catalog_rows[1]["errors"])

            with (output / "goodco-g100" / "curve.csv").open() as stream:
                curve_rows = list(csv.DictReader(stream))
            self.assertEqual(curve_rows[-1]["thrust_n"], "0.000000")

            report_json = json.loads((output / "validation_report.json").read_text())
            self.assertEqual(report_json["summary"]["motors_active"], 1)

    def test_config_thrust_curves_are_valid(self) -> None:
        module = load_module(Path("tools/generate_motor_catalog.py"))

        source = Path(__file__).resolve().parents[1] / "config" / "thrust_curves"
        self.assertTrue(source.exists())

        with TemporaryDirectory() as tmp:
            output = Path(tmp) / "output"
            report = module.generate_catalog(source, output)

            self.assertEqual(report["summary"]["motors_total"], 3)
            self.assertEqual(report["summary"]["motors_active"], 3)
            self.assertEqual(report["summary"]["motors_rejected"], 0)

            with (output / "catalog.csv").open() as stream:
                catalog_rows = list(csv.DictReader(stream))

            expected = {
                ("AeroTech", "G8"),
                ("AeroTech", "G12"),
                ("Apogee", "F10"),
            }
            active_rows = {
                (row["manufacturer"], row["designation"])
                for row in catalog_rows
                if row["active"] == "1"
            }
            self.assertEqual(active_rows, expected)
            self.assertTrue(all((output / row["motor_id"] / "curve.csv").exists() for row in catalog_rows if row["active"] == "1"))
            self.assertTrue(all((output / row["motor_id"] / "specs.csv").exists() for row in catalog_rows if row["active"] == "1"))


motor_catalog = load_module(REPO_ROOT / "tools/tv3_motor_catalog.py")
allocator = load_module(REPO_ROOT / "tools/tv3_control_allocator.py")
LANDER = REPO_ROOT / "config/vehicles/tv3_lander_v1.json"
ASCENT = REPO_ROOT / "config/vehicles/tv3_v1.json"


class MotorCatalogIntegrationTests(unittest.TestCase):
    def setUp(self) -> None:
        self.catalog = motor_catalog.load_motor_catalog("config/thrust_curves")
        self.lander = allocator.load_manifest(LANDER)
        self.ascent = allocator.load_manifest(ASCENT)

    def test_repo_motors_load(self) -> None:
        self.assertEqual(
            set(self.catalog.keys()),
            {"aerotech-g8", "aerotech-g12", "apogee-f10"},
        )

    def test_g12_max_thrust_matches_certified_spec(self) -> None:
        entry = self.catalog["aerotech-g12"]
        self.assertAlmostEqual(entry.max_thrust_n, 30.9, places=1)
        self.assertGreater(entry.curve_peak_thrust_n, entry.average_thrust_n)

    def test_lander_engines_use_catalog_thrust_not_allocator_reference(self) -> None:
        engines = allocator.engines_from_vehicle(self.lander)
        expected_total = sum(engine.thrust_n for engine in engines)
        reference = self.lander["vehicle"]["ca_reference_thrust_n"]
        for engine in engines:
            self.assertAlmostEqual(engine.thrust_n, self.catalog["aerotech-g12"].max_thrust_n, places=1)
            self.assertLess(engine.thrust_n, reference)
        self.assertAlmostEqual(reference, expected_total, places=1)

    def test_motor_reference_uses_catalog_thrust(self) -> None:
        engines = allocator.engines_from_vehicle(self.lander)
        expected_total = sum(engine.thrust_n for engine in engines)
        reference = allocator.motor_reference_from_thrust(self.lander, thrust_n=expected_total)
        self.assertAlmostEqual(reference.expected_thrust_n, expected_total, places=2)
        self.assertAlmostEqual(sum(reference.expected_thrust_n_engine), expected_total, places=2)

    def test_changing_default_motor_updates_engine_thrust(self) -> None:
        manifest = allocator.load_manifest(LANDER)
        manifest["motor_selection"]["default_motor_id"] = "aerotech-g8"
        for engine in manifest["propulsion"]["engines"]:
            engine["motor_id"] = "aerotech-g8"

        engines = allocator.engines_from_vehicle(manifest)
        self.assertAlmostEqual(engines[0].thrust_n, self.catalog["aerotech-g8"].max_thrust_n, places=1)

    def test_ascent_single_engine_uses_catalog_thrust(self) -> None:
        engines = allocator.engines_from_vehicle(self.ascent)
        self.assertEqual(len(engines), 1)
        self.assertAlmostEqual(engines[0].thrust_n, self.catalog["aerotech-g12"].max_thrust_n, places=1)


if __name__ == "__main__":
    unittest.main()
