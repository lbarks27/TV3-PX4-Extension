from __future__ import annotations

import importlib.util
import sys
import unittest
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]


def load_module(path: Path):
    spec = importlib.util.spec_from_file_location(path.stem, path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


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