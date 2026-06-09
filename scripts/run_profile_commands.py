#!/usr/bin/env python3

from __future__ import annotations

import argparse
import os
import sys
import time
from pathlib import Path

import yaml
from pymavlink import mavutil


REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_PROFILE = REPO_ROOT / "config/flight_profiles/lander_hover_window.yaml"
ROCKET_COMMAND = 31010
ROCKET_ACTIONS = {
    "launch": 1.0,
    "abort": 2.0,
    "reset": 3.0,
}


def resolve_repo_path(value: str | None, default: Path) -> Path:
    path = Path(value) if value else default
    if not path.is_absolute():
        path = REPO_ROOT / path
    return path


def load_commands(profile_path: Path) -> list[dict]:
    profile = yaml.safe_load(profile_path.read_text())
    commands = profile.get("commands", [])
    if not isinstance(commands, list):
        raise ValueError(f"profile commands must be a list: {profile_path}")
    return sorted(commands, key=lambda command: float(command.get("at_s", 0.0)))


def profile_allows_force_arm(profile_path: Path) -> bool:
    profile = yaml.safe_load(profile_path.read_text())
    scenario = profile.get("scenario", {})
    return bool(scenario.get("simulated_only")) and scenario.get("simulator") == "sih"


def send_command(master, command_id: int, params: list[float]) -> None:
    padded = params + [0.0] * (7 - len(params))
    master.mav.command_long_send(
        master.target_system,
        master.target_component or mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1,
        command_id,
        0,
        *padded[:7],
    )


def wait_ack(master, command_id: int, timeout_s: float) -> str:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        ack = master.recv_match(type="COMMAND_ACK", blocking=True, timeout=0.25)
        if ack is None:
            continue
        if int(ack.command) == command_id:
            return mavutil.mavlink.enums["MAV_RESULT"].get(int(ack.result), ack.result).name
    return "timeout"


def run_command(
    master,
    command: dict,
    ack_timeout_s: float,
    arm_ready_timeout_s: float,
    dry_run: bool,
    force_arm: bool,
) -> None:
    action = command.get("action")
    if not action:
        raise ValueError(f"profile command missing action: {command}")

    if action in {"arm", "arm_and_ready"}:
        print(f"profile command: {action} -> MAV_CMD_COMPONENT_ARM_DISARM")
        if not dry_run:
            params = [1.0, 21196.0] if force_arm else [1.0]
            deadline = time.monotonic() + arm_ready_timeout_s

            while True:
                send_command(master, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, params)
                ack = wait_ack(master, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, ack_timeout_s)
                print(f"  ack: {ack}")

                if ack == "MAV_RESULT_ACCEPTED":
                    break

                if time.monotonic() >= deadline:
                    break

                time.sleep(1.0)
        return

    if action in ROCKET_ACTIONS:
        param1 = float(command.get("param1", ROCKET_ACTIONS[action]))
        print(f"profile command: {action} -> {ROCKET_COMMAND} param1={param1:g}")
        if not dry_run:
            send_command(master, ROCKET_COMMAND, [param1])
            print(f"  ack: {wait_ack(master, ROCKET_COMMAND, ack_timeout_s)}")
        return

    raise ValueError(f"unsupported profile action: {action}")


def main() -> int:
    parser = argparse.ArgumentParser(description="Execute a TV3 flight profile command timeline over MAVLink.")
    parser.add_argument(
        "--flight-profile",
        default=os.environ.get("TV3_FLIGHT_PROFILE"),
        help="TV3 flight profile YAML. Defaults to the lander hover-window profile.",
    )
    parser.add_argument(
        "--connect",
        default=os.environ.get("TV3_MAVLINK_URL", "udpin:0.0.0.0:14540"),
        help="pymavlink connection URL. Defaults to the PX4 SITL offboard UDP receive port.",
    )
    parser.add_argument("--heartbeat-timeout", type=float, default=60.0)
    parser.add_argument("--ack-timeout", type=float, default=3.0)
    parser.add_argument(
        "--arm-ready-timeout",
        type=float,
        default=float(os.environ.get("TV3_ARM_READY_TIMEOUT", "30.0")),
        help="Retry arm commands for this many seconds while SIH estimator health settles.",
    )
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()

    profile_path = resolve_repo_path(args.flight_profile, DEFAULT_PROFILE)
    commands = load_commands(profile_path)
    if not commands:
        print(f"no profile commands in {profile_path}")
        return 0
    force_arm = profile_allows_force_arm(profile_path)

    print(f"profile: {profile_path}")
    print(f"mavlink: {args.connect}")
    if force_arm:
        print("force-arm: enabled for simulated-only SIH profile")
    master = mavutil.mavlink_connection(args.connect)
    print("waiting for PX4 heartbeat...")
    heartbeat = master.wait_heartbeat(timeout=args.heartbeat_timeout)
    if heartbeat is None:
        print("timed out waiting for heartbeat", file=sys.stderr)
        return 1

    print(f"heartbeat: system={master.target_system} component={master.target_component}")
    start = time.monotonic()
    for command in commands:
        at_s = float(command.get("at_s", 0.0))
        remaining_s = start + at_s - time.monotonic()
        if remaining_s > 0:
            time.sleep(remaining_s)
        run_command(master, command, args.ack_timeout, args.arm_ready_timeout, args.dry_run, force_arm)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
