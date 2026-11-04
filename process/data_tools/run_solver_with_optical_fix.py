#!/usr/bin/env python3
"""Diagnostic-only runner using the optical-to-camera-link rotation direction."""

from __future__ import annotations

import importlib.util
from pathlib import Path
import sys


def main() -> int:
    if len(sys.argv) < 2:
        raise SystemExit("usage: run_solver_with_optical_fix.py SOLVER [solver args]")
    solver_path = Path(sys.argv[1])
    spec = importlib.util.spec_from_file_location("joint_solver_fixed", solver_path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    module.CAMERA_LINK_TO_OPTICAL = module.CAMERA_LINK_TO_OPTICAL.T
    sys.argv = [str(solver_path), *sys.argv[2:]]
    return int(module.main())


if __name__ == "__main__":
    raise SystemExit(main())
