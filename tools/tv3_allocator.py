#!/usr/bin/env python3
"""Deprecated shim — use `python3 -m tools.tv3_control_allocator` instead."""

from tools.tv3_control_allocator import _allocator_cli

if __name__ == "__main__":
    _allocator_cli()
