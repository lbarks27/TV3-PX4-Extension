#!/usr/bin/env python3
"""Run a lightweight guidance-envelope Monte Carlo sweep for Phase 5 gates."""

from __future__ import annotations

import argparse
import json
import sys
from dataclasses import asdict
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from tools.tv3_control_allocator import load_manifest, vehicle_full_thrust_n  # noqa: E402
from tools.tv3_guidance_envelope import (  # noqa: E402
    PHASE_WAYPOINT_TRACK,
    evaluate_profile_case,
    run_monte_carlo,
)
# The envelope checks (and thus MC) use the projected GD joint allocator for control reachability.


def main() -> None:
    parser = argparse.ArgumentParser(description="TV3 guidance envelope Monte Carlo sweep")
    parser.add_argument(
        "--vehicle",
        type=Path,
        default=REPO_ROOT / "config/vehicles/tv3_lander_v1.json",
    )
    parser.add_argument(
        "--profile",
        type=Path,
        default=REPO_ROOT / "config/flight_profiles/lander_hover_window.json",
    )
    parser.add_argument(
        "--impossible-profile",
        type=Path,
        default=REPO_ROOT / "config/flight_profiles/lander_impossible_guidance.json",
    )
    parser.add_argument("--samples", type=int, default=64)
    parser.add_argument("--seed", type=int, default=5)
    args = parser.parse_args()

    report = run_monte_carlo(
        args.vehicle,
        args.profile,
        samples=args.samples,
        seed=args.seed,
        phase=PHASE_WAYPOINT_TRACK,
    )
    hover_thrust_n = vehicle_full_thrust_n(load_manifest(args.vehicle))
    impossible = evaluate_profile_case(
        args.vehicle,
        args.impossible_profile,
        phase=PHASE_WAYPOINT_TRACK,
        thrust_n=hover_thrust_n,
        state=None,
    )
    payload = {
        "monte_carlo": asdict(report),
        "impossible_profile_rejected": not impossible.solution_valid,
        "impossible_reason": impossible.reason,
    }
    print(json.dumps(payload, indent=2))

    if not report.passed or impossible.solution_valid:
        raise SystemExit(1)


if __name__ == "__main__":
    main()