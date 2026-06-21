#!/usr/bin/env python3
"""Build a small checked-in ULog fixture for replay unit tests."""

from __future__ import annotations

import argparse
import struct
from pathlib import Path

HEADER = b"ULog\x01\x12"
MSG_HEADER = struct.Struct("<HB")


def read_messages(data: bytes) -> list[tuple[int, bytes]]:
    if not data.startswith(HEADER):
        raise ValueError("not a ULog file")
    offset = len(HEADER)
    messages: list[tuple[int, bytes]] = []
    while offset + MSG_HEADER.size <= len(data):
        size, msg_type = MSG_HEADER.unpack_from(data, offset)
        total = MSG_HEADER.size + size
        if offset + total > len(data):
            break
        messages.append((msg_type, data[offset : offset + total]))
        offset += total
    return messages


def message_timestamp_us(payload: bytes) -> int | None:
    if len(payload) < MSG_HEADER.size + 8:
        return None
    return struct.unpack_from("<Q", payload, MSG_HEADER.size)[0]


def trim_ulog(source: Path, destination: Path, *, max_time_us: int) -> None:
    data = source.read_bytes()
    messages = read_messages(data)
    if not messages:
        raise ValueError(f"no messages in {source}")

    start_us: int | None = None
    kept: list[bytes] = [HEADER]
    for msg_type, raw in messages:
        if msg_type == ord("D"):
            timestamp = message_timestamp_us(raw)
            if timestamp is None:
                continue
            if start_us is None:
                start_us = timestamp
            if timestamp - start_us > max_time_us:
                continue
        kept.append(raw)

    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_bytes(b"".join(kept))
    print(f"wrote {destination} ({destination.stat().st_size} bytes from {source.name})")


def main() -> None:
    repo_root = Path(__file__).resolve().parents[2]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--source",
        type=Path,
        default=repo_root
        / "logs/sim/2026-06-20/20260620T033407Z-splay-secondary/03_34_13.ulg",
        help="Source ULog to trim",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=repo_root / "tests/fixtures/minimal_lander_hover.ulg",
    )
    parser.add_argument("--seconds", type=float, default=8.0, help="Keep this many seconds of DATA messages")
    args = parser.parse_args()

    if not args.source.exists():
        raise SystemExit(
            f"source log not found: {args.source}\n"
            "Run a SIH log archive first or pass --source to an existing .ulg"
        )

    trim_ulog(args.source, args.output, max_time_us=int(args.seconds * 1e6))


if __name__ == "__main__":
    main()
