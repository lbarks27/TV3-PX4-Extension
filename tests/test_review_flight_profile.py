from __future__ import annotations

import unittest
from pathlib import Path
from tempfile import TemporaryDirectory

import json

from tests.support import load_module


class ReviewFlightProfileTests(unittest.TestCase):
    def test_hover_window_profile_declares_review_criteria(self) -> None:
        profile = json.loads(Path("config/flight_profiles/lander_hover_window.json").read_text())
        review = profile["review"]
        self.assertEqual(3.0, review["min_hover_s"])
        self.assertIn("tv3_guidance_status", review["required_topics"])
        self.assertIn("control_allocator_status", review["required_topics"])

    def test_review_script_imports(self) -> None:
        module = load_module(Path("tools/review_flight_profile.py"))
        self.assertTrue(callable(module.review_ulog))
        self.assertTrue(callable(module.longest_true_run))
        self.assertEqual(["tv3_engine_state", "rocket_engine_state"], module.topic_aliases("tv3_engine_state"))


if __name__ == "__main__":
    unittest.main()