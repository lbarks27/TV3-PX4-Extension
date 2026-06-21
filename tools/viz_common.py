"""Shared constants and helpers for TV3 visualization tools."""

from __future__ import annotations

from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_MESH = REPO_ROOT / "assets/meshes/tv3_lander_v1.obj"

ENGINE_COLORS = ("#ff7f0e", "#e377c2", "#17becf", "#bcbd22")

CAMERA_PRESETS = {
    "iso": (24, -58),
    "top": (90, -90),
    "side": (0, -90),
    "front": (0, 180),
    "forward_up": (-90, 0),
    "overview": (25, -55),
    "track": (20, -70),
}


def resolve_mesh_path(manifest: dict | None = None, override: Path | None = None) -> Path:
    if override is not None:
        path = override if override.is_absolute() else REPO_ROOT / override
        if path.exists():
            return path
        raise SystemExit(f"vehicle mesh not found: {path}")
    if manifest is not None:
        mesh_ref = manifest.get("visualization", {}).get("mesh")
        if mesh_ref:
            path = REPO_ROOT / mesh_ref if not Path(mesh_ref).is_absolute() else Path(mesh_ref)
            if path.exists():
                return path
    if DEFAULT_MESH.exists():
        return DEFAULT_MESH
    raise SystemExit(
        f"vehicle mesh not found at {DEFAULT_MESH}. "
        "Run: python3 tools/generate_vehicle_mesh.py"
    )