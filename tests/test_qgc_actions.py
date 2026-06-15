from __future__ import annotations

import json
import re
import unittest
from pathlib import Path


class QGroundControlActionTests(unittest.TestCase):
    def test_tv3_qgc_actions_match_firmware_command_contract(self) -> None:
        actions_path = Path("config/qgc/TV3Actions.json")
        source_path = Path("src/modules/flight_modes/tv3_mode_manager.cpp")
        command_msg_path = Path("msg/Tv3Command.msg")

        actions_doc = json.loads(actions_path.read_text())
        source = source_path.read_text()
        command_msg = command_msg_path.read_text()

        command_id_match = re.search(r"constexpr uint32_t kTV3VehicleCommand = (\d+);", source)
        self.assertIsNotNone(command_id_match)
        command_id = int(command_id_match.group(1))

        command_values = {
            name.lower(): int(value)
            for name, value in re.findall(r"uint8 COMMAND_(LAUNCH|ABORT|RESET) = (\d+)", command_msg)
        }

        self.assertEqual(31010, command_id)
        self.assertEqual({"launch": 1, "abort": 2, "reset": 3}, command_values)
        self.assertEqual("MavlinkActions", actions_doc["fileType"])
        self.assertEqual(1, actions_doc["version"])

        expected_actions = {
            "TV3 Launch": command_values["launch"],
            "TV3 Abort": command_values["abort"],
            "TV3 Reset": command_values["reset"],
        }

        actual_actions = {action["label"]: action for action in actions_doc["actions"]}
        self.assertEqual(set(expected_actions), set(actual_actions))

        for label, param1 in expected_actions.items():
            action = actual_actions[label]
            self.assertEqual(command_id, action["mavCmd"])
            self.assertEqual(1, action["compId"])
            self.assertEqual(param1, action["param1"])

        self.assertIn("VEHICLE_CMD_RESULT_DENIED", source)
        self.assertIn("TV3 state %s", source)


if __name__ == "__main__":
    unittest.main()
