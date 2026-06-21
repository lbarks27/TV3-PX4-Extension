from __future__ import annotations

import json
import unittest
from pathlib import Path
from tempfile import TemporaryDirectory

from tests.support import REPO_ROOT, load_module


class GenerateVehicleMeshTests(unittest.TestCase):
    def test_build_manifest_mesh_has_body_and_engines(self) -> None:
        module = load_module(REPO_ROOT / "tools/generate_vehicle_mesh.py")
        manifest = json.loads((REPO_ROOT / "config/vehicles/tv3_lander_v1.json").read_text())
        vertices, faces = module.build_manifest_mesh(manifest)
        self.assertGreater(len(vertices), 20)
        self.assertGreater(len(faces), 20)
        engine_count = len(manifest["propulsion"]["engines"])
        self.assertEqual(3, engine_count)

    def test_write_obj_round_trip(self) -> None:
        module = load_module(REPO_ROOT / "tools/generate_vehicle_mesh.py")
        mesh_module = load_module(REPO_ROOT / "tools/vehicle_mesh.py")
        manifest = json.loads((REPO_ROOT / "config/vehicles/tv3_lander_v1.json").read_text())
        with TemporaryDirectory() as tmp:
            output = Path(tmp) / "vehicle.obj"
            vertices, faces = module.build_manifest_mesh(manifest)
            module.write_obj(output, vertices, faces)
            loaded_vertices, loaded_faces = mesh_module.load_obj_mesh(output)
            self.assertEqual(len(vertices), len(loaded_vertices))
            self.assertEqual(len(faces), len(loaded_faces))


if __name__ == "__main__":
    unittest.main()