#!/usr/bin/env python3
"""Print live load-cell samples from MAVLink DEBUG_VECT lc_data messages."""

from __future__ import annotations

import argparse
import os
import statistics
import sys
import time
from typing import Any

DEFAULT_CONNECT = os.environ.get("TV3_MAVLINK_CONNECT", "/dev/cu.usbmodem01")
ADC_FULL_SCALE_V = float(os.environ.get("TV3_LC_ADC_FS_V", "0.256"))
ADC_RESOLUTION = 32768.0


def mavlink_string(value: Any) -> str:
    if isinstance(value, bytes):
        return value.decode(errors="ignore").strip("\x00")
    return str(value).strip("\x00")


def decode_param_value(message: Any) -> float | int:
    value = float(message.param_value)
    param_type = int(message.param_type)
    if param_type in (5, 6, 7, 8):
        return int(round(value))
    return value


def connect_mavlink(connect: str, baud: int):
    try:
        from pymavlink import mavutil
    except ImportError as exc:
        raise SystemExit(
            "missing pymavlink: run scripts/complete_phase2_bench.sh once or "
            "`python3 -m pip install pymavlink pyserial`"
        ) from exc

    try:
        if connect.startswith("/dev/"):
            master = mavutil.mavlink_connection(connect, baud=baud)
        else:
            master = mavutil.mavlink_connection(connect)
    except Exception as exc:
        message = str(exc).lower()
        if "busy" in message or "errno 16" in message:
            raise SystemExit(
                f"serial port busy: {connect}\n"
                "Close QGroundControl (or any app using the Cube USB port), then retry.\n"
                "Check with: lsof " + connect
            ) from exc
        raise SystemExit(f"could not open MAVLink link {connect}: {exc}") from exc

    heartbeat = master.wait_heartbeat(timeout=8)
    if heartbeat is None or master.target_system == 0:
        raise SystemExit(
            f"no MAVLink heartbeat on {connect}.\n"
            "Confirm the Cube is powered, TV3 firmware is flashed, and the link is correct."
        )
    return master


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--connect",
        default=DEFAULT_CONNECT,
        help=f"MAVLink device or URL (default: {DEFAULT_CONNECT})",
    )
    parser.add_argument("--baud", type=int, default=57600, help="Serial baud when using /dev/*")
    parser.add_argument(
        "--duration",
        type=float,
        default=0.0,
        help="Stop after N seconds (default: 0 = run until Ctrl+C)",
    )
    parser.add_argument(
        "--stats-every",
        type=float,
        default=5.0,
        help="Print rolling noise stats every N seconds (0 disables)",
    )
    parser.add_argument(
        "--counts-only",
        action="store_true",
        help="Show filtered ADC counts only (best for push/tare tests before calibration)",
    )
    args = parser.parse_args()

    master = connect_mavlink(args.connect, args.baud)
    print(f"connected: {args.connect}")
    kg_sc = None
    tare_counts: float | None = None
    for name in ("RK_LC_ADC_INST", "RK_LC_TARE", "RK_LC_KG_SC", "RK_LC_MAX_JMP", "RK_LC_ALPHA"):
        master.mav.param_request_read_send(
            master.target_system, master.target_component, name.encode(), -1
        )
        message = master.recv_match(type="PARAM_VALUE", blocking=True, timeout=2.0)
        if message is not None:
            value = decode_param_value(message)
            print(f"  {name} = {value}")
            if name == "RK_LC_KG_SC":
                kg_sc = float(value)
            if name == "RK_LC_TARE":
                tare_counts = float(value)
    if args.counts_only:
        print(f"  tare: {tare_counts if tare_counts is not None else 'unknown'} counts")
        print(f"  adc full-scale: +/-{ADC_FULL_SCALE_V:.3f} V (PGA)")
        print("columns: filtered_counts  delta_from_tare  delta_mV")
        print("push on the load cell and watch counts move; Ctrl+C to stop")
    else:
        if kg_sc is None or abs(kg_sc) < 1e-12:
            print("  note: RK_LC_KG_SC=0 keeps mass_kg at 0 until calibration")
        print("lc_data columns: raw_counts  mass_kg  thrust_N")
        print("Ctrl+C to stop")

    end = time.time() + args.duration if args.duration > 0 else None
    last_warn = 0.0
    samples = 0
    raw_window: list[float] = []
    mass_window: list[float] = []
    stats_deadline = time.time() + args.stats_every if args.stats_every > 0 else None

    while end is None or time.time() < end:
        message = master.recv_match(blocking=True, timeout=1.0)
        if message is None:
            if samples == 0 and time.time() - last_warn > 5.0:
                print(
                    "waiting for lc_data... ensure tv3_load_cell_telemetry is running "
                    "and extras.txt streams DEBUG_VECT on the USB link",
                    file=sys.stderr,
                )
                last_warn = time.time()
            continue

        if message.get_type() != "DEBUG_VECT":
            continue
        if mavlink_string(message.name) != "lc_data":
            continue

        samples += 1
        filtered_counts = float(message.x)
        raw_window.append(filtered_counts)
        mass_window.append(float(message.y))
        if len(raw_window) > 40:
            raw_window.pop(0)
            mass_window.pop(0)
        if args.counts_only:
            delta = filtered_counts - tare_counts if tare_counts is not None else 0.0
            delta_mv = delta * ADC_FULL_SCALE_V / ADC_RESOLUTION * 1000.0
            print(f"{filtered_counts:8.1f}  {delta:+8.1f}  {delta_mv:+8.3f}", flush=True)
        else:
            print(f"{message.x:8.1f}  {message.y:7.4f}  {message.z:7.2f}", flush=True)

        if stats_deadline is not None and time.time() >= stats_deadline and len(raw_window) >= 5:
            raw_median = statistics.median(raw_window)
            raw_spread = statistics.median([abs(v - raw_median) for v in raw_window])
            if args.counts_only:
                delta_median = raw_median - tare_counts if tare_counts is not None else raw_median
                print(
                    f"  stats: median={raw_median:.1f} spread={raw_spread:.1f} delta={delta_median:+.1f}",
                    flush=True,
                )
            else:
                mass_spread = statistics.median([abs(v - statistics.median(mass_window)) for v in mass_window])
                print(
                    f"  stats: raw_median={raw_median:.1f} raw_spread={raw_spread:.1f} "
                    f"mass_spread={mass_spread:.4f} kg",
                    flush=True,
                )
            stats_deadline = time.time() + args.stats_every

    if samples == 0:
        print("no lc_data samples received", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())