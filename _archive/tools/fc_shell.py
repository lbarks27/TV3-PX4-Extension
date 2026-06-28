#!/usr/bin/env python3
"""Run non-interactive PX4 NSH commands over MAVLink SERIAL_CONTROL."""

from __future__ import annotations

import argparse
import os
import sys
import time

DEFAULT_CONNECT = os.environ.get("TV3_MAVLINK_CONNECT", "/dev/cu.usbmodem01")


class MavlinkShell:
    def __init__(self, connect: str, baud: int = 57600) -> None:
        from pymavlink import mavutil

        self._mavutil = mavutil
        if connect.startswith("/dev/"):
            self.mav = mavutil.mavlink_connection(connect, baud=baud)
        else:
            self.mav = mavutil.mavlink_connection(connect)
        self.mav.wait_heartbeat(timeout=10)
        self.mav.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_GENERIC,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID,
            0,
            0,
            0,
        )

    def run(self, command: str, timeout_s: float = 4.0) -> str:
        command = command.strip()
        if not command:
            return ""

        payload = (command + "\n").encode()
        while payload:
            chunk = payload[:70]
            payload = payload[70:]
            buf = list(chunk) + [0] * (70 - len(chunk))
            self.mav.mav.serial_control_send(
                self._mavutil.mavlink.SERIAL_CONTROL_DEV_SHELL,
                self._mavutil.mavlink.SERIAL_CONTROL_FLAG_EXCLUSIVE
                | self._mavutil.mavlink.SERIAL_CONTROL_FLAG_RESPOND,
                0,
                0,
                len(chunk),
                buf,
            )

        deadline = time.time() + timeout_s
        chunks: list[str] = []
        while time.time() < deadline:
            message = self.mav.recv_match(
                condition="SERIAL_CONTROL.count!=0",
                type="SERIAL_CONTROL",
                blocking=True,
                timeout=0.2,
            )
            if message is None:
                continue
            data = bytes(message.data[: message.count]).decode(errors="replace")
            if data:
                chunks.append(data)
            if "nsh>" in data or "pxh>" in data:
                break

        return "".join(chunks)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--connect", default=DEFAULT_CONNECT)
    parser.add_argument("--baud", type=int, default=57600)
    parser.add_argument("--timeout", type=float, default=4.0)
    parser.add_argument("command", nargs="+", help="NSH command to run")
    args = parser.parse_args()

    try:
        shell = MavlinkShell(args.connect, baud=args.baud)
    except Exception as exc:
        message = str(exc).lower()
        if "busy" in message:
            print(
                f"serial port busy: {args.connect}\n"
                "Close QGroundControl and retry.",
                file=sys.stderr,
            )
            return 1
        print(f"connect failed: {exc}", file=sys.stderr)
        return 1

    output = shell.run(" ".join(args.command), timeout_s=args.timeout)
    if output:
        sys.stdout.write(output)
        if not output.endswith("\n"):
            sys.stdout.write("\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())