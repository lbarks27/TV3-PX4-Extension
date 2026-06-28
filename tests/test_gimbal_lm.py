from __future__ import annotations

import importlib.util
import math
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


allocator = load_module(REPO_ROOT / "tools/tv3_control_allocator.py")
LANDER = REPO_ROOT / "config/vehicles/tv3_lander_v1.json"


def full_thrust_n(engines) -> float:
    return sum(engine.thrust_n for engine in engines)


def allocator_like_initial(engine_count: int) -> tuple[tuple[float, float], ...]:
    return tuple((0.0, 0.0) for _ in range(engine_count))


def pitch_hint_initial(engine_count: int) -> tuple[tuple[float, float], ...]:
    if engine_count < 3:
        return allocator_like_initial(engine_count)
    return ((3.0, 0.0), (-1.5, 0.0), (-1.5, 0.0))


class GimbalLmTests(unittest.TestCase):
    def setUp(self) -> None:
        self.lander = allocator.load_manifest(LANDER)
        self.engines = allocator.engines_from_vehicle(self.lander)
        self.hover_thrust_n = full_thrust_n(self.engines)

    def test_hover_converges_quickly(self) -> None:
        result = allocator.solve_gimbal_lm(
            self.engines,
            (0.0, 0.0, 0.0),
            self.hover_thrust_n,
            allocator_like_initial(len(self.engines)),
        )
        self.assertLessEqual(result.iterations_used, 12)
        self.assertLess(result.residual_torque_nm, 0.15)
        self.assertLess(abs(result.residual_thrust_n), max(1.0, self.hover_thrust_n * 0.05))

    def test_hover_matches_grid_allocator(self) -> None:
        grid = allocator.allocate(self.engines, (0.0, 0.0, 0.0), self.hover_thrust_n)
        lm = allocator.solve_gimbal_lm(
            self.engines,
            (0.0, 0.0, 0.0),
            self.hover_thrust_n,
            allocator_like_initial(len(self.engines)),
        )
        grid_torque, grid_thrust = allocator.plant_total_wrench(self.engines, grid.commands)
        lm_torque, lm_thrust = allocator.plant_total_wrench(self.engines, lm.commands)
        self.assertLess(allocator.norm(grid_torque), 0.5)
        self.assertLess(allocator.norm(lm_torque), 0.15)
        self.assertAlmostEqual(self.hover_thrust_n, grid_thrust, delta=1.0)
        self.assertAlmostEqual(self.hover_thrust_n, lm_thrust, delta=1.0)

    def test_reduced_thrust_without_acos_preset(self) -> None:
        desired = 80.0
        result = allocator.solve_gimbal_lm(
            self.engines,
            (0.0, 0.0, 0.0),
            desired,
            allocator_like_initial(len(self.engines)),
        )
        _, achieved = allocator.plant_total_wrench(self.engines, result.commands)
        self.assertAlmostEqual(desired, achieved, delta=1.5)
        self.assertGreater(max(command[1] for command in result.commands), 0.0)

    def test_pitch_torque_demand_from_allocator_seed(self) -> None:
        demand = (0.0, 1.0, 0.0)
        result = allocator.solve_gimbal_lm(
            self.engines,
            demand,
            self.hover_thrust_n,
            pitch_hint_initial(len(self.engines)),
        )
        torque, _thrust = allocator.plant_total_wrench(self.engines, result.commands)
        self.assertLess(result.residual_torque_nm, 0.15)
        self.assertLess(abs(torque[1] - demand[1]), 0.2)

    def test_lm_improves_grid_seed_at_reduced_thrust(self) -> None:
        demand = (0.0, 2.0, 1.0)
        thrust = 80.0
        grid = allocator.allocate(self.engines, demand, thrust)
        result = allocator.solve_gimbal_lm(self.engines, demand, thrust, grid.commands)
        self.assertLess(result.residual_torque_nm, grid.torque_error_nm)

    def test_saturated_demand_does_not_converge(self) -> None:
        result = allocator.solve_gimbal_lm(
            self.engines,
            (0.0, 20.0, 0.0),
            self.hover_thrust_n,
            pitch_hint_initial(len(self.engines)),
            torque_limits=allocator.torque_limits_from_vehicle(self.lander),
        )
        self.assertLessEqual(result.iterations_used, 12)
        self.assertTrue(math.isfinite(result.cost))
        self.assertFalse(result.converged)
        self.assertGreater(result.residual_torque_nm, 0.15)
        self.assertTrue(result.demand_saturated)

    def test_infeasible_thrust_demand_is_rejected(self) -> None:
        result = allocator.solve_gimbal_lm(
            self.engines,
            (0.0, 0.0, 0.0),
            40.0,
            allocator_like_initial(len(self.engines)),
            torque_limits=allocator.torque_limits_from_vehicle(self.lander),
        )
        self.assertFalse(result.converged)
        self.assertTrue(result.demand_saturated)

    def test_warm_start_converges_faster(self) -> None:
        seed = allocator.allocate(self.engines, (0.0, 2.0, 1.0), self.hover_thrust_n).commands
        cold = allocator.solve_gimbal_lm(
            self.engines,
            (0.0, 2.0, 1.0),
            self.hover_thrust_n,
            pitch_hint_initial(len(self.engines)),
        )
        warm = allocator.solve_gimbal_lm(
            self.engines,
            (0.0, 2.5, 1.2),
            self.hover_thrust_n * 0.95,
            cold.commands if cold.commands else seed,
        )
        self.assertLessEqual(warm.iterations_used, cold.iterations_used)


if __name__ == "__main__":
    unittest.main()
