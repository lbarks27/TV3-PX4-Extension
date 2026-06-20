#!/usr/bin/env python3
"""Diagnose ADS1115 path and run tv3_load_cell_telemetry tare on the flight controller."""

from __future__ import annotations

import argparse
import os
import re
import sys
import time
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from tools.fc_shell import MavlinkShell  # noqa: E402
from tools.load_cell_stats import parse_status, robust_spread  # noqa: E402

DEFAULT_CONNECT = os.environ.get("TV3_MAVLINK_CONNECT", "/dev/cu.usbmodem01")


def run_tare_workflow(shell: MavlinkShell, *, attempts: int = 3) -> bool:
    startup = "\n".join(
        [
            "param set RK_LC_ADC_INST 1",
            "param set RK_LC_MODE 0",
            "param set RK_LC_CH 0",
            "param set RK_LC_ALPHA 0.45",

            "param set RK_LC_DB 2",

            "ads1115 stop",
            "ads1115 start -X -b 2 -a 0x48",
            "tv3_load_cell start",
            "tv3_load_cell_telemetry stop",
            "tv3_load_cell_telemetry start",
        ]
    )
    for command in startup.splitlines():
        print(f">>> {command}")
        print(shell.run(command, timeout_s=5.0))

    time.sleep(2.0)
    for attempt in range(attempts):
        status = shell.run("tv3_load_cell_telemetry status", timeout_s=4.0)
        print(status)
        parsed = parse_status(status)
        age = parsed.sample_age_us
        filt_raw = parsed.filt_raw
        if filt_raw is not None and (age is None or age < 5_000_000):
            spread_samples: list[float] = []
            for _ in range(6):
                sample_status = shell.run("tv3_load_cell_telemetry status", timeout_s=4.0)
                sample = parse_status(sample_status)
                if sample.filt_raw is not None:
                    spread_samples.append(sample.filt_raw)
                time.sleep(0.25)
            spread = robust_spread(spread_samples) if spread_samples else 0.0
            print(f">>> tare attempt {attempt + 1}: filt_raw={filt_raw:.1f} spread={spread:.1f}")
            if spread > 80.0:
                print("waiting for a steadier window before tare")
                time.sleep(1.0)
                continue

            tare_out = shell.run("tv3_load_cell_telemetry tare", timeout_s=4.0)
            print(tare_out)
            if "set RK_LC_TARE" in tare_out:
                save_out = shell.run("param save", timeout_s=6.0)
                print(save_out)
                verify = shell.run("tv3_load_cell_telemetry status", timeout_s=4.0)
                print(verify)
                return True

            if "no adc sample yet" in tare_out.lower() and filt_raw is not None:
                print(">>> fallback: param set RK_LC_TARE from live filt_raw")
                for command in (
                    f"param set RK_LC_TARE {filt_raw:.3f}",
                    "param save",
                ):
                    print(f">>> {command}")
                    print(shell.run(command, timeout_s=6.0))
                verify = shell.run("tv3_load_cell_telemetry status", timeout_s=4.0)
                print(verify)
                tare_match = re.search(r"tare:\s*([-+0-9.]+)", verify)
                if tare_match and abs(float(tare_match.group(1)) - filt_raw) < 0.51:
                    return True
        time.sleep(1.0)

    return False


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--connect", default=DEFAULT_CONNECT)
    parser.add_argument("--baud", type=int, default=57600)
    args = parser.parse_args()

    print(f"connecting: {args.connect}")
    try:
        shell = MavlinkShell(args.connect, baud=args.baud)
    except Exception as exc:
        message = str(exc).lower()
        if "busy" in message:
            print("Close QGroundControl, then retry.", file=sys.stderr)
        else:
            print(f"connect failed: {exc}", file=sys.stderr)
        return 1

    print(">>> ads1115 status")
    print(shell.run("ads1115 status", timeout_s=4.0))

    if run_tare_workflow(shell):
        print("tare succeeded")
        return 0

    print("tare failed: no fresh ADS1115 sample on adc_report instance 1", file=sys.stderr)
    print("check I2C2 wiring, 0x48 address, and ads1115 start output above", file=sys.stderr)
    return 1


if __name__ == "__main__":
    raise SystemExit(main())