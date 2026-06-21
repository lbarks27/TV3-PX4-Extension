#!/usr/bin/env python3
"""Deprecated shim — use tools.tv3_replay instead."""

from __future__ import annotations

import sys
import warnings
from typing import Sequence

from tools.tv3_replay import (  # noqa: F401
    GuidanceFrame,
    TrajectoryFrame,
    build_guidance_frames,
    build_parser,
    build_trajectory_frames,
    default_output_path,
    guidance_summary_text,
    main as replay_main,
    trajectory_summary_text,
)


def main(argv: Sequence[str] | None = None) -> int:
    warnings.warn("plot_ulog_replay is deprecated; use tv3_replay", DeprecationWarning, stacklevel=2)
    print("note: plot_ulog_replay is deprecated; use ./scripts/tv3_replay.sh", file=sys.stderr)
    return replay_main(argv)


if __name__ == "__main__":
    raise SystemExit(main())
