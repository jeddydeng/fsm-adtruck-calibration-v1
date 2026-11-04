#!/usr/bin/env python3
"""Detect stale/frozen camera imagery even when ROS timestamps keep changing."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
from typing import Any

import cv2
import numpy as np


def compact_image(path: Path, width: int = 80, height: int = 60) -> np.ndarray:
    image = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
    if image is None:
        raise ValueError(f"cannot decode image: {path}")
    return cv2.resize(image, (width, height), interpolation=cv2.INTER_AREA).astype(np.float32)


def longest_true_run(values: list[bool]) -> tuple[int, int]:
    best = (0, 0)
    start = None
    for index, value in enumerate(values + [False]):
        if value and start is None:
            start = index
        elif not value and start is not None:
            if index - start > best[1] - best[0]:
                best = (start, index)
            start = None
    return best


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("root", type=Path)
    parser.add_argument("--camera", default="traffic_light_front")
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--mae-threshold", type=float, default=0.8)
    parser.add_argument("--changed-fraction-threshold", type=float, default=0.01)
    args = parser.parse_args()
    rows: list[dict[str, Any]] = []
    previous = None
    for manifest_path in sorted(args.root.glob("*/manifest.json")):
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        sensor = manifest.get("sensors", {}).get(args.camera, {})
        relative = sensor.get("file")
        path = manifest_path.parent / relative if relative else None
        if path is None or not path.is_file():
            continue
        current = compact_image(path)
        if previous is None:
            mae = changed = p99 = None
            near_same = False
        else:
            difference = np.abs(current - previous)
            mae = float(difference.mean())
            changed = float(np.mean(difference > 2.0))
            p99 = float(np.percentile(difference, 99))
            near_same = mae <= args.mae_threshold and changed <= args.changed_fraction_threshold
        rows.append(
            {
                "capture_id": manifest.get("capture_id", manifest_path.parent.name),
                "file": str(path),
                "mae_from_previous": mae,
                "fraction_abs_change_gt_2": changed,
                "p99_abs_change": p99,
                "near_same_as_previous": near_same,
                "trigger_offset_ms": sensor.get("trigger_offset_ms"),
                "received_unix_ns": sensor.get("received_unix_ns"),
            }
        )
        previous = current
    if not rows:
        raise SystemExit("no complete camera files found")
    flags = [bool(row["near_same_as_previous"]) for row in rows]
    start, end = longest_true_run(flags)
    # A true flag at index i means frame i repeats i-1, so include its predecessor.
    frame_start = max(0, start - 1) if end > start else 0
    summary = {
        "camera": args.camera,
        "available_frames": len(rows),
        "near_same_transitions": sum(flags),
        "longest_near_same_frame_run": end - frame_start if end > start else 1,
        "longest_run_start": rows[frame_start]["capture_id"],
        "longest_run_end": rows[end - 1]["capture_id"] if end > start else rows[0]["capture_id"],
        "mae_threshold": args.mae_threshold,
        "changed_fraction_threshold": args.changed_fraction_threshold,
    }
    args.output_dir.mkdir(parents=True, exist_ok=True)
    with (args.output_dir / f"{args.camera}_frame_changes.csv").open(
        "w", newline="", encoding="utf-8"
    ) as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    (args.output_dir / f"{args.camera}_freeze_summary.json").write_text(
        json.dumps(summary, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
