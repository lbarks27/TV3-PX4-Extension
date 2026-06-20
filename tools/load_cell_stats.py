#!/usr/bin/env python3
"""Shared helpers for parsing tv3_load_cell_telemetry status and robust averaging."""

from __future__ import annotations

import re
import statistics
from dataclasses import dataclass


@dataclass(frozen=True)
class LoadCellStatus:
    raw: float | None = None
    filt_raw: float | None = None
    tare: float | None = None
    kg_per_count: float | None = None
    mass_kg: float | None = None
    sample_age_us: int | None = None
    rejected_spikes: int | None = None
    window_count: int | None = None


def parse_status(status_text: str) -> LoadCellStatus:
    def _float(pattern: str) -> float | None:
        match = re.search(pattern, status_text)
        return float(match.group(1)) if match else None

    def _int(pattern: str) -> int | None:
        match = re.search(pattern, status_text)
        return int(match.group(1)) if match else None

    raw = _float(r"raw:\s*([-+0-9.]+)")
    filt_raw = _float(r"filt_raw:\s*([-+0-9.]+)")
    return LoadCellStatus(
        raw=raw,
        filt_raw=filt_raw if filt_raw is not None else raw,
        tare=_float(r"tare:\s*([-+0-9.]+)"),
        kg_per_count=_float(r"kg/count:\s*([-+0-9.eE]+)"),
        mass_kg=_float(r"kg:\s*([-+0-9.]+)"),
        sample_age_us=_int(r"sample age us:\s*(\d+)"),
        rejected_spikes=_int(r"spikes rejected:\s*(\d+)"),
        window_count=_int(r"window:\s*(\d+)"),
    )


def robust_median(values: list[float], *, max_abs_dev: float | None = None) -> float:
    if not values:
        raise ValueError("no values")

    median = statistics.median(values)
    if max_abs_dev is None or len(values) < 4:
        return median

    filtered = [value for value in values if abs(value - median) <= max_abs_dev]
    if len(filtered) < max(3, len(values) // 2):
        return median
    return statistics.median(filtered)


def robust_spread(values: list[float]) -> float:
    if len(values) < 2:
        return 0.0
    median = statistics.median(values)
    deviations = [abs(value - median) for value in values]
    return statistics.median(deviations)