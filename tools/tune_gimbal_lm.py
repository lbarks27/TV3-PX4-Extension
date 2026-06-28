#!/usr/bin/env python3

from __future__ import annotations

import argparse
import json
import sys
from dataclasses import asdict
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from tools.tv3_control_allocator import (  # noqa: E402
    LmConfig,
    engines_from_vehicle,
    load_manifest,
    run_lm_sweep,
    torque_limits_from_vehicle,
    tune_lm_config,
)


def _config_from_args(args: argparse.Namespace) -> LmConfig | None:
    overrides = {
        "max_iter": args.max_iter,
        "torque_tol_nm": args.torque_tol,
        "lambda0": args.lambda0,
        "thrust_weight": args.thrust_weight,
        "splay_weight": args.splay_weight,
        "fd_eps": args.fd_eps,
    }
    if all(value is None for value in overrides.values()):
        return None
    base = LmConfig()
    return LmConfig(
        max_iter=overrides["max_iter"] if overrides["max_iter"] is not None else base.max_iter,
        torque_tol_nm=overrides["torque_tol"] if overrides["torque_tol"] is not None else base.torque_tol_nm,
        lambda0=overrides["lambda0"] if overrides["lambda0"] is not None else base.lambda0,
        thrust_weight=overrides["thrust_weight"] if overrides["thrust_weight"] is not None else base.thrust_weight,
        splay_weight=overrides["splay_weight"] if overrides["splay_weight"] is not None else base.splay_weight,
        fd_eps=overrides["fd_eps"] if overrides["fd_eps"] is not None else base.fd_eps,
    )


def _summary_payload(summary, results) -> dict:
    return {
        "summary": {
            "total_cases": summary.total_cases,
            "reachable_count": summary.reachable_count,
            "unreachable_count": summary.unreachable_count,
            "lm_converged_count": summary.lm_converged_count,
            "reachable_failed_count": summary.reachable_failed_count,
            "convergence_rate": summary.convergence_rate,
            "torque_residual_p50": summary.torque_residual_p50,
            "torque_residual_p95": summary.torque_residual_p95,
            "thrust_residual_p50": summary.thrust_residual_p50,
            "iterations_p50": summary.iterations_p50,
            "iterations_max": summary.iterations_max,
            "config": asdict(summary.config),
            "worst_failures": summary.worst_failures,
        },
        "cases": [
            {
                "torque_nm": list(entry.case.desired_torque_nm),
                "thrust_n": entry.case.desired_thrust_n,
                "reachable": entry.reachable,
                "oracle_reason": entry.oracle_reason,
                "oracle_torque_error_nm": entry.oracle_torque_error_nm,
                "oracle_thrust_error_n": entry.oracle_thrust_error_n,
                "lm_converged": entry.lm_converged,
                "residual_torque_nm": entry.residual_torque_nm,
                "residual_thrust_n": entry.residual_thrust_n,
                "iterations_used": entry.iterations_used,
                "warm_start": [list(command) for command in entry.warm_start],
            }
            for entry in results
        ],
    }


def _print_summary(summary) -> None:
    print("LM sweep summary")
    print(f"  cases:              {summary.total_cases}")
    print(f"  reachable:          {summary.reachable_count}")
    print(f"  unreachable:        {summary.unreachable_count}")
    print(f"  lm converged:       {summary.lm_converged_count}")
    print(f"  reachable failed:   {summary.reachable_failed_count}")
    print(f"  convergence rate:   {summary.convergence_rate:.1%}")
    print(f"  torque residual p50:{summary.torque_residual_p50:.4f} Nm")
    print(f"  torque residual p95:{summary.torque_residual_p95:.4f} Nm")
    print(f"  thrust residual p50:{summary.thrust_residual_p50:.4f} N")
    print(f"  iterations p50/max: {summary.iterations_p50:.1f} / {summary.iterations_max}")


def main() -> int:
    parser = argparse.ArgumentParser(description="Sweep and tune the TV3 gimbal LM allocator")
    parser.add_argument("--vehicle", type=Path, required=True)
    parser.add_argument("--output", type=Path, default=None, help="Write JSON report to this path")
    parser.add_argument("--full", action="store_true", help="Use denser thrust/torque grid")
    parser.add_argument("--tune", action="store_true", help="Search for a better LmConfig")
    parser.add_argument("--max-tune-trials", type=int, default=50)
    parser.add_argument("--grid-steps", type=int, default=5, help="Grid allocator steps for oracle")
    parser.add_argument("--quiet", action="store_true", help="Only print summary table")
    parser.add_argument("--max-iter", type=int, default=None)
    parser.add_argument("--torque-tol", type=float, default=None)
    parser.add_argument("--lambda0", type=float, default=None)
    parser.add_argument("--thrust-weight", type=float, default=None)
    parser.add_argument("--splay-weight", type=float, default=None)
    parser.add_argument("--fd-eps", type=float, default=None)
    args = parser.parse_args()

    manifest = load_manifest(args.vehicle)
    engines = engines_from_vehicle(manifest)
    torque_limits = torque_limits_from_vehicle(manifest)
    config = _config_from_args(args)

    if args.tune:
        best_config, summary = tune_lm_config(
            engines,
            torque_limits=torque_limits,
            max_trials=args.max_tune_trials,
            grid_steps=args.grid_steps,
            full=args.full,
        )
        _, results = run_lm_sweep(
            engines,
            torque_limits=torque_limits,
            config=best_config,
            grid_steps=args.grid_steps,
            full=args.full,
        )
        if not args.quiet:
            print("Best LmConfig:")
            print(json.dumps(asdict(best_config), indent=2))
    else:
        summary, results = run_lm_sweep(
            engines,
            torque_limits=torque_limits,
            config=config,
            grid_steps=args.grid_steps,
            full=args.full,
        )

    if not args.quiet or args.output is None:
        _print_summary(summary)

    if args.output is not None:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(_summary_payload(summary, results), indent=2), encoding="utf-8")
        if not args.quiet:
            print(f"Wrote report to {args.output}")

    if summary.reachable_count == 0:
        return 1
    return 0 if summary.convergence_rate >= 0.90 else 1


if __name__ == "__main__":
    raise SystemExit(main())
