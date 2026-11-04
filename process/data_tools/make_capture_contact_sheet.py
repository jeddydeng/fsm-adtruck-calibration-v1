#!/usr/bin/env python3
"""Create labelled camera contact sheets from joint capture manifests."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import cv2
import numpy as np


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("capture", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    manifest = json.loads((args.capture / "manifest.json").read_text())
    cameras = [
        (name, item)
        for name, item in manifest["sensors"].items()
        if "camera" in str(item.get("kind", ""))
    ]
    cell_w, cell_h, label_h = 400, 300, 40
    columns = 3
    rows = (len(cameras) + columns - 1) // columns
    sheet = np.full((rows * (cell_h + label_h), columns * cell_w, 3), 28, dtype=np.uint8)
    for index, (name, item) in enumerate(cameras):
        image = cv2.imread(str(args.capture / item["file"]))
        row, column = divmod(index, columns)
        x, y = column * cell_w, row * (cell_h + label_h)
        if image is not None:
            scale = min(cell_w / image.shape[1], cell_h / image.shape[0])
            resized = cv2.resize(image, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
            ox = x + (cell_w - resized.shape[1]) // 2
            oy = y + label_h + (cell_h - resized.shape[0]) // 2
            sheet[oy : oy + resized.shape[0], ox : ox + resized.shape[1]] = resized
        count = int(item.get("detection", {}).get("tag_count", 0))
        color = (80, 220, 80) if count else (80, 80, 255)
        cv2.putText(
            sheet,
            f"{name}  tags={count}",
            (x + 8, y + 27),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.64,
            color,
            2,
            cv2.LINE_AA,
        )
    cv2.putText(
        sheet,
        manifest.get("capture_id", args.capture.name),
        (12, sheet.shape[0] - 12),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        (230, 230, 230),
        1,
        cv2.LINE_AA,
    )
    if not cv2.imwrite(str(args.output), sheet):
        raise SystemExit(f"failed to write {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
