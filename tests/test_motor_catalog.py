from __future__ import annotations

import csv
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


def write_csv(path: Path, fieldnames: list[str], rows: list[dict[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


class MotorCatalogTests(unittest.TestCase):
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


if __name__ == "__main__":
    unittest.main()
