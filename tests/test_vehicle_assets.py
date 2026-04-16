from __future__ import annotations

import importlib.util
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


class VehicleAssetTests(unittest.TestCase):
    def test_generate_vehicle_assets(self) -> None:
        module = load_module(Path("tools/generate_vehicle_assets.py"))
        vehicle = Path("config/vehicles/tv3_v1.yaml")

        with TemporaryDirectory() as tmp:
            output = Path(tmp) / "generated"
            module.generate_assets(vehicle, output)

            params = (output / "runtime" / "tv3_v1.params").read_text()
            self.assertIn("RK_LAUNCH_THR_N", params)
            self.assertIn("CA_RK_REF_THR", params)
            self.assertIn("RK_GD_ENABLE", params)
            self.assertIn("RK_GD_WP1_N_M", params)

            model_config = (output / "gazebo" / "tv3_v1" / "model.config").read_text()
            self.assertIn("<name>tv3_v1</name>", model_config)

            jsbsim = (output / "jsbsim" / "tv3_v1" / "propulsion_motor.xml").read_text()
            self.assertIn("<motor_index>0</motor_index>", jsbsim)

    def test_sitl_airframes_share_common_defaults(self) -> None:
        common = Path("overlay/ROMFS/init.d-posix/airframes/tv3_rocket_common.inc").read_text()
        self.assertIn("param set-default CA_AIRFRAME 16", common)
        self.assertIn("param set-default RK_LC_SRC 1", common)

        gz = Path("overlay/ROMFS/init.d-posix/airframes/11000_gz_tv3_rocket").read_text()
        self.assertIn(". ${R}etc/init.d-posix/airframes/tv3_rocket_common.inc", gz)
        self.assertIn("PX4_SIMULATOR=${PX4_SIMULATOR:=gz}", gz)

        jsbsim = Path("overlay/ROMFS/init.d-posix/airframes/11001_jsbsim_tv3_rocket").read_text()
        self.assertIn(". ${R}etc/init.d-posix/airframes/tv3_rocket_common.inc", jsbsim)
        self.assertIn("param set-default SYS_HITL 1", jsbsim)

        post = Path("overlay/ROMFS/init.d-posix/airframes/tv3_rocket_common.post").read_text()
        self.assertIn("rocket_mode_manager start", post)
        self.assertIn("rocket_att_control start", post)


if __name__ == "__main__":
    unittest.main()
