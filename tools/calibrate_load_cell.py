#!/usr/bin/env python3
"""Set load-cell scale from a known mass using stable filtered raw counts on the flight controller."""

from __future__ import annotations

import argparse
import os
import sys
import time
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from tools.fc_shell import MavlinkShell  # noqa: E402
from tools.load_cell_stats import LoadCellStatus, parse_status, robust_median, robust_spread  # noqa: E402

DEFAULT_CONNECT = os.environ.get("TV3_MAVLINK_CONNECT", "/dev/cu.usbmodem01")


def ensure_telemetry(shell: MavlinkShell) -> None:
    print(">>> ensuring ADS1115 hardware differential + telemetry on adc instance 1")
    for command in (
        "param set RK_LC_ADC_INST 1",
        "param set RK_LC_MODE 0",
        "param set RK_LC_CH 0",
        "param set RK_LC_ALPHA 0.45",

        "param set RK_LC_DB 2",

        "ads1115 stop",
        "ads1115 start -X -b 2 -a 0x48",
        "tv3_load_cell_telemetry stop",
        "tv3_load_cell_telemetry start",
    ):
        print(shell.run(command, timeout_s=5))


def collect_filtered_raw(shell: MavlinkShell, *, samples: int, settle_s: float) -> tuple[list[float], LoadCellStatus]:
    time.sleep(settle_s)
    values: list[float] = []
    latest = LoadCellStatus()
    for _ in range(samples):
        status = shell.run("tv3_load_cell_telemetry status", timeout_s=4)
        parsed = parse_status(status)
        latest = parsed
        if parsed.filt_raw is not None:
            values.append(parsed.filt_raw)
        time.sleep(0.25)
    return values, latest


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("known_mass_kg", type=float, help="Known mass currently on the load cell (kg)")
    parser.add_argument("--connect", default=DEFAULT_CONNECT)
    parser.add_argument("--baud", type=int, default=57600)
    parser.add_argument("--samples", type=int, default=24, help="Filtered status samples for median delta")
    parser.add_argument("--settle-s", type=float, default=2.0, help="Seconds to wait before sampling")
    args = parser.parse_args()

    if args.known_mass_kg <= 0:
        print("known_mass_kg must be > 0", file=sys.stderr)
        return 1

    try:
        shell = MavlinkShell(args.connect, baud=args.baud)
    except Exception as exc:
        print(f"connect failed: {exc}", file=sys.stderr)
        if "busy" in str(exc).lower():
            print("Close ./scripts/watch_load_cell.sh or QGroundControl first.", file=sys.stderr)
        return 1

    ensure_telemetry(shell)

    raw_values, status = collect_filtered_raw(shell, samples=args.samples, settle_s=args.settle_s)
    if not raw_values or status.tare is None:
        print("could not read filt_raw/tare from tv3_load_cell_telemetry status", file=sys.stderr)
        print(shell.run("tv3_load_cell_telemetry status", timeout_s=4), file=sys.stderr)
        return 1

    spread = robust_spread(raw_values)
    max_dev = max(25.0, spread * 4.0)
    filt_median = robust_median(raw_values, max_abs_dev=max_dev)
    delta = filt_median - status.tare

    print(
        f"filt_raw median={filt_median:.2f} spread={spread:.2f} tare={status.tare:.2f} "
        f"delta={delta:.2f} spikes_rejected={status.rejected_spikes}"
    )

    if abs(delta) < 20.0:
        print(
            f"load delta too small ({delta:.1f} counts).\n"
            "Place a known mass on the cell, wait for it to settle, then re-run.\n"
            "If the cell is empty, run tools/tare_load_cell.py first.",
            file=sys.stderr,
        )
        return 1

    if spread > abs(delta) * 0.15:
        print(
            f"warning: noise spread ({spread:.1f} counts) is high relative to load delta ({delta:.1f}).\n"
            "Hold the mass steady and re-run, or check ADS1115 wiring/excitation.",
            file=sys.stderr,
        )

    kg_per_count = args.known_mass_kg / delta
    print(f"RK_LC_KG_SC={kg_per_count:.9f}")

    cal_out = shell.run(f"tv3_load_cell_telemetry calibrate {args.known_mass_kg:.3f}", timeout_s=5)
    print(cal_out)
    if "no adc sample yet" in cal_out.lower() or "load delta too small" in cal_out.lower():
        print(">>> fallback: param set RK_LC_KG_SC from host median")
        for command in (
            f"param set RK_LC_KG_SC {kg_per_count:.9f}",
            "param save",
        ):
            print(f">>> {command}")
            print(shell.run(command, timeout_s=6))

    print(shell.run("tv3_load_cell_telemetry status", timeout_s=4))
    print("Re-run ./scripts/watch_load_cell.sh and verify mass_kg tracks the known load.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())