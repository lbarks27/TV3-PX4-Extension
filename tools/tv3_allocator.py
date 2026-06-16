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

from tools.tv3_control_allocator import allocate_from_vehicle  # noqa: E402


def main() -> None:
    parser = argparse.ArgumentParser(description="TV3 allocator reachability check")
    parser.add_argument("--vehicle", type=Path, required=True)
    parser.add_argument("--torque", nargs=3, type=float, default=(0.0, 0.0, 0.0))
    parser.add_argument("--thrust", type=float, required=True)
    args = parser.parse_args()

    result = allocate_from_vehicle(args.vehicle, tuple(args.torque), args.thrust)
    payload = asdict(result)
    print(json.dumps(payload, indent=2, default=list))


if __name__ == "__main__":
    main()