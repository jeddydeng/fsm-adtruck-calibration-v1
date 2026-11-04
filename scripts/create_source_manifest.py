#!/usr/bin/env python3
"""Hash tracked files for a reviewable source/config snapshot, excluding this manifest."""

from __future__ import annotations

import hashlib
from pathlib import Path
import subprocess


def main() -> int:
    root = Path(__file__).resolve().parents[1]
    names = subprocess.check_output(["git", "ls-files", "-z"], cwd=root).decode().split("\0")
    lines = []
    for name in sorted(filter(None, names)):
        if name == "SOURCE_MANIFEST.sha256":
            continue
        digest = hashlib.sha256((root / name).read_bytes()).hexdigest()
        lines.append(f"{digest}  {name}\n")
    (root / "SOURCE_MANIFEST.sha256").write_text("".join(lines))
    print(f"Hashed {len(lines)} source/config artifacts")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
