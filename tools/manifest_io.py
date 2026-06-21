"""Shared vehicle manifest loading and engine list resolution."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any


def deep_merge(base: dict[str, Any], overlay: dict[str, Any]) -> dict[str, Any]:
    merged = dict(base)
    for key, value in overlay.items():
        if key in merged and isinstance(merged[key], dict) and isinstance(value, dict):
            merged[key] = deep_merge(merged[key], value)
        else:
            merged[key] = value
    return merged


def default_overlays(path: Path) -> list[Path]:
    overlays: list[Path] = []
    for suffix in ("_measured", "_physical_model"):
        candidate = path.with_name(f"{path.stem}{suffix}.json")
        if candidate.exists():
            overlays.append(candidate)
    return overlays


def load_manifest(path: Path | str, *, overlays: list[Path | str] | None = None) -> dict:
    manifest_path = Path(path)
    manifest = json.loads(manifest_path.read_text())
    if not isinstance(manifest, dict):
        raise ValueError(f"vehicle manifest must be a JSON object: {manifest_path}")

    overlay_paths = [Path(item) for item in overlays] if overlays is not None else default_overlays(manifest_path)
    for overlay_path in overlay_paths:
        if not overlay_path.exists():
            continue
        overlay = json.loads(overlay_path.read_text())
        if not isinstance(overlay, dict):
            raise ValueError(f"manifest overlay must be a JSON object: {overlay_path}")
        manifest = deep_merge(manifest, overlay)
    return manifest


def engines_from_manifest(manifest: dict) -> list[dict]:
    """Return propulsion engine dicts, with single-engine fallback from vehicle body fields."""
    propulsion = manifest.get("propulsion", {})
    engines = propulsion.get("engines")
    if engines:
        return engines

    body = manifest["vehicle"]
    motor = manifest.get("motor_selection", {})
    load_cell = manifest.get("hardware", {}).get("load_cell", {})
    return [
        {
            "id": "engine_0",
            "motor_index": motor.get("index", 0),
            "load_cell_channel": load_cell.get("adc_channel", 0),
            "position_m": [body["motor_com_x_m"], 0.0, 0.0],
            "thrust_axis": [1.0, 0.0, 0.0],
            "roll_axis": [0.0, -1.0, 0.0],
            "yaw_axis": [0.0, 0.0, -1.0],
            "thrust_fraction": 1.0,
            "gimbal": {
                "roll_max_deg": body["tvc_max_deg"],
                "yaw_max_deg": body["tvc_max_deg"],
                "splay_max_deg": body["tvc_max_deg"],
                "slew_dps": body["tvc_slew_dps"],
                "roll_trim": 0.0,
                "yaw_trim": 0.0,
            },
        }
    ]
