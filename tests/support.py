"""Shared test utilities for TV3 host tests."""

from __future__ import annotations

import importlib.util
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
MINIMAL_ULOG = REPO_ROOT / "tests/fixtures/minimal_lander_hover.ulg"


def load_module(path: Path):
    path = path if path.is_absolute() else REPO_ROOT / path
    spec = importlib.util.spec_from_file_location(path.stem, path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def ensure_minimal_ulog() -> Path:
    if not MINIMAL_ULOG.exists():
        generator = REPO_ROOT / "tests/fixtures/generate_minimal_ulog.py"
        if not generator.exists():
            raise FileNotFoundError(f"missing ULog fixture and generator: {MINIMAL_ULOG}")
        import subprocess

        subprocess.run([sys.executable, str(generator)], check=True)
    return MINIMAL_ULOG
