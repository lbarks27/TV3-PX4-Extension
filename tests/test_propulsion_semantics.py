from __future__ import annotations

import csv
import unittest

from tests.support import REPO_ROOT, load_module

FIXTURES = REPO_ROOT / "tests" / "fixtures" / "load_cell_adc"


propulsion = load_module(REPO_ROOT / "tools/tv3_propulsion_model.py")
replay = load_module(REPO_ROOT / "tools/replay_load_cell_adc.py")


def load_adc_csv(path: Path) -> list[tuple[float, int | None]]:
    samples: list[tuple[float, int | None]] = []
    with path.open(newline="") as handle:
        reader = csv.DictReader(handle)
        for row in reader:
            time_s = float(row["time_s"])
            raw_value = row.get("raw_count", "").strip()
            raw_count = None if raw_value == "" else int(float(raw_value))
            samples.append((time_s, raw_count))
    return samples


def run_trace(
    fixture_name: str,
    *,
    vehicle: str = "config/vehicles/tv3_v1.json",
    launch_at_s: float = 0.0,
    source: int | None = None,
):
    vehicle_config = REPO_ROOT / vehicle
    samples = load_adc_csv(FIXTURES / fixture_name)
    load_cell, mode_manager, manifest = replay.build_models(vehicle_config, source=source)

    def reference_builder(time_s: float, thrust_n: float):
        ignition_mask = 0
        if time_s >= launch_at_s:
            active_slot = mode_manager.active_sequence_slot
            for slot in range(active_slot + 1):
                ignition_mask |= 1 << mode_manager.config.ignition_sequence[slot]
        return propulsion.motor_reference_from_manifest(
            manifest,
            thrust_n=thrust_n,
            ignition_mask=ignition_mask,
        )

    return propulsion.replay_adc_trace(
        samples,
        load_cell=load_cell,
        mode_manager=mode_manager,
        reference_builder=reference_builder,
        launch_at_s=launch_at_s,
    )


class PropulsionSemanticsTests(unittest.TestCase):
    def test_reference_and_adc_sources_share_ignition_confirmation(self) -> None:
        manifest = propulsion.load_manifest(REPO_ROOT / "config/vehicles/tv3_v1.json")
        thrust_curve = [0.0, 0.0, 20.0, 40.0, 50.0, 50.0]

        def run_with_source(source: int) -> list[bool]:
            load_cell_config = propulsion.load_cell_config_from_manifest(manifest)
            load_cell_config.source = source
            load_cell = propulsion.LoadCellModel(config=load_cell_config)
            confirmations: list[bool] = []

            for index, thrust_n in enumerate(thrust_curve):
                timestamp_us = propulsion.TIME_OFFSET_US + index * 200_000
                reference = propulsion.motor_reference_from_manifest(manifest, thrust_n=thrust_n)
                load_cell.set_reference(reference)
                raw = propulsion.adc_raw_from_thrust_n(thrust_n, load_cell_config)
                thrust, _engine_state = load_cell.step(timestamp_us, adc_raw=raw if source == propulsion.SOURCE_ADC else None)
                confirmations.append(thrust.ignition_confirmed)

            return confirmations

        self.assertEqual(run_with_source(propulsion.SOURCE_ADC), run_with_source(propulsion.SOURCE_REFERENCE))

    def test_delayed_ignition_reaches_boost(self) -> None:
        timeline = run_trace("delayed_ignition.csv")
        summary = replay.summarize_timeline(timeline)
        self.assertTrue(summary["saw_ignition_pending"])
        self.assertTrue(summary["saw_boost"])
        self.assertFalse(summary["ignition_timeout"])

    def test_failed_ignition_times_out(self) -> None:
        timeline = run_trace("failed_ignition.csv")
        summary = replay.summarize_timeline(timeline)
        self.assertTrue(summary["saw_abort"])
        self.assertTrue(summary["ignition_timeout"])
        self.assertFalse(summary["saw_boost"])

    def test_false_positive_spike_is_rejected(self) -> None:
        timeline = run_trace("false_positive_spike.csv")
        summary = replay.summarize_timeline(timeline)
        self.assertFalse(summary["saw_boost"])
        self.assertTrue(summary["saw_abort"])
        self.assertTrue(summary["ignition_timeout"])

    def test_burnout_transitions_to_coast(self) -> None:
        timeline = run_trace("burnout.csv")
        summary = replay.summarize_timeline(timeline)
        self.assertTrue(summary["saw_boost"])
        self.assertTrue(summary["saw_coast"])
        self.assertFalse(summary["saw_abort"])

    def test_stale_adc_during_boost_aborts(self) -> None:
        timeline = run_trace("stale_mid_burn.csv")
        summary = replay.summarize_timeline(timeline)
        self.assertTrue(summary["saw_boost"])
        self.assertTrue(summary["sensor_stale"])
        self.assertTrue(summary["saw_abort"])

    def test_lander_three_engine_sequence_confirms_all_engines(self) -> None:
        timeline = run_trace(
            "lander_three_engine_sequence.csv",
            vehicle="config/vehicles/tv3_lander_v1.json",
        )
        summary = replay.summarize_timeline(timeline)
        self.assertTrue(summary["saw_boost"])
        self.assertTrue(summary["final_sequence_complete"])
        self.assertEqual(0b111, summary["max_confirmed_mask"])

    def test_adc_replay_fixture_reproduces_aggregate_thrust(self) -> None:
        timeline = run_trace("delayed_ignition.csv")
        peak = max(entry["thrust"].filtered_thrust_n for entry in timeline)
        self.assertGreaterEqual(peak, 43.0)
        final_engine_state = timeline[-1]["engine_state"]
        self.assertGreater(final_engine_state.filtered_thrust_n[0], 40.0)

    def test_bad_scale_flags_invalid_thrust(self) -> None:
        config = propulsion.LoadCellConfig(scale=0.0)
        model = propulsion.LoadCellModel(config=config)
        reference = propulsion.MotorReference(loaded=True, expected_thrust_n=0.0)
        model.set_reference(reference)
        thrust, _engine_state = model.step(1_000_000, adc_raw=100)
        self.assertFalse(thrust.valid)
        self.assertTrue(thrust.fault_flags & propulsion.THRUST_FAULT_BAD_SCALE)

    def test_bench_calibration_template_exists(self) -> None:
        template = REPO_ROOT / "docs/templates/bench_calibration_report.md"
        self.assertTrue(template.exists())
        text = template.read_text()
        self.assertIn("tare", text)
        self.assertIn("scale", text)
        self.assertIn("kg_per_count", text)


if __name__ == "__main__":
    unittest.main()
