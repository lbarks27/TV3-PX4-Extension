from __future__ import annotations

import importlib.util
import math
import sys
import unittest
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
MIN_CONVERGENCE_RATE = 0.90


def load_module(path: Path):
    spec = importlib.util.spec_from_file_location(path.stem, path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


allocator = load_module(REPO_ROOT / "tools/tv3_control_allocator.py")
LANDER = REPO_ROOT / "config/vehicles/tv3_lander_v1.json"


class GimbalLmConvergenceTests(unittest.TestCase):
    def setUp(self) -> None:
        self.lander = allocator.load_manifest(LANDER)
        self.engines = allocator.engines_from_vehicle(self.lander)
        self.torque_limits = allocator.torque_limits_from_vehicle(self.lander)
        self.full_thrust_n, _ = allocator.thrust_envelope(self.engines)

    def test_envelope_convergence_rate(self) -> None:
        summary, results = allocator.run_lm_sweep(
            self.engines,
            torque_limits=self.torque_limits,
        )
        self.assertGreater(
            summary.reachable_count,
            0,
            "expected at least one reachable demand in the default sweep",
        )
        self.assertGreaterEqual(
            summary.convergence_rate,
            MIN_CONVERGENCE_RATE,
            msg=(
                f"reachable convergence {summary.convergence_rate:.1%} "
                f"below {MIN_CONVERGENCE_RATE:.0%}; "
                f"failed={summary.reachable_failed_count}"
            ),
        )

        for entry in results:
            if entry.reachable:
                self.assertTrue(math.isfinite(entry.residual_torque_nm))
                self.assertTrue(math.isfinite(entry.residual_thrust_n))

        failed = [entry for entry in results if entry.reachable and not entry.lm_converged]
        if failed and summary.convergence_rate < MIN_CONVERGENCE_RATE:
            lines = ["Top LM failures:"]
            for entry in failed[:10]:
                lines.append(
                    "  torque="
                    f"{entry.case.desired_torque_nm} thrust={entry.case.desired_thrust_n:.2f} "
                    f"tau={entry.residual_torque_nm:.3f} thr={entry.residual_thrust_n:.3f} "
                    f"iters={entry.iterations_used}"
                )
            print("\n".join(lines), file=sys.stderr)

    def test_hover_case_converges(self) -> None:
        warm = allocator.firmware_warm_start(
            self.engines,
            (0.0, 0.0, 0.0),
            self.full_thrust_n,
        )
        result = allocator.solve_gimbal_lm(
            self.engines,
            (0.0, 0.0, 0.0),
            self.full_thrust_n,
            warm,
        )
        self.assertTrue(
            result.converged
            or allocator.lm_converged(
                result.residual_torque_nm,
                result.residual_thrust_n,
                self.full_thrust_n,
            )
        )

    def test_reduced_thrust_case_converges(self) -> None:
        desired_thrust = 80.0
        warm = allocator.firmware_warm_start(
            self.engines,
            (0.0, 0.0, 0.0),
            desired_thrust,
        )
        result = allocator.solve_gimbal_lm(
            self.engines,
            (0.0, 0.0, 0.0),
            desired_thrust,
            warm,
        )
        self.assertTrue(
            result.converged
            or allocator.lm_converged(
                result.residual_torque_nm,
                result.residual_thrust_n,
                desired_thrust,
            )
        )


if __name__ == "__main__":
    unittest.main()
