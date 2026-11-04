#!/usr/bin/env python3
"""Summarize capture chronology, board motion, stale streams, and repeat clicks."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import statistics
from collections import Counter, defaultdict
from pathlib import Path
from typing import Any

import numpy as np


CAMERAS = (
    "video_front_left",
    "video_front_mid",
    "video_front_right",
    "video_rear_left",
    "video_rear_mid",
    "video_rear_right",
    "fisheye_front",
    "fisheye_left",
    "fisheye_right",
    "fisheye_rear",
    "traffic_light_front",
)
DIRECTIONS = ("front", "left", "right", "rear")


def load(root: Path) -> list[tuple[Path, dict[str, Any]]]:
    rows = []
    for path in sorted(root.glob("*/manifest.json")):
        try:
            rows.append((path.parent, json.loads(path.read_text(encoding="utf-8"))))
        except (OSError, ValueError):
            pass
    return rows


def targets(manifest: dict[str, Any]) -> dict[str, dict[str, Any]]:
    values = (
        manifest.get("sensors", {}).get("lidar_top", {}).get("detection", {}).get("targets", [])
    )
    by_direction: dict[str, dict[str, Any]] = {}
    for item in values:
        direction = str(item.get("direction"))
        previous = by_direction.get(direction)
        if previous is None or float(item.get("confidence", 0)) > float(
            previous.get("confidence", 0)
        ):
            by_direction[direction] = item
    return by_direction


def densest_center(points: list[list[float]], radius: float = 0.35) -> tuple[np.ndarray, int]:
    if not points:
        return np.full(3, np.nan), 0
    array = np.asarray(points, dtype=float)
    counts = np.asarray(
        [np.count_nonzero(np.linalg.norm(array - point, axis=1) <= radius) for point in array]
    )
    seed = array[int(np.argmax(counts))]
    mask = np.linalg.norm(array - seed, axis=1) <= radius
    return np.median(array[mask], axis=0), int(mask.sum())


def longest_constant_run(values: list[Any]) -> dict[str, Any]:
    best_start = best_end = start = 0
    for index in range(1, len(values) + 1):
        if index == len(values) or values[index] != values[start]:
            if index - start > best_end - best_start:
                best_start, best_end = start, index
            start = index
    return {
        "start_index": best_start,
        "end_index_exclusive": best_end,
        "length": best_end - best_start,
        "value": values[best_start] if values else None,
    }


def sample_identity(manifest: dict[str, Any]) -> str:
    fields = []
    for name, item in sorted(manifest.get("sensors", {}).items()):
        fields.append(
            (
                name,
                item.get("source_stamp_ns"),
                item.get("received_unix_ns"),
                item.get("detection", {}).get("sample_sequence"),
            )
        )
    return hashlib.sha256(repr(fields).encode()).hexdigest()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("root", type=Path)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--window", type=int, default=10)
    args = parser.parse_args()
    captures = load(args.root)
    if not captures:
        raise SystemExit("no capture manifests found")
    args.output_dir.mkdir(parents=True, exist_ok=True)

    direction_points: dict[str, list[list[float]]] = defaultdict(list)
    target_rows = []
    for _directory, manifest in captures:
        current = targets(manifest)
        target_rows.append(current)
        for direction, item in current.items():
            direction_points[direction].append(item["center_top_m"])
    baselines = {direction: densest_center(direction_points[direction]) for direction in DIRECTIONS}

    traffic_received = []
    traffic_offsets = []
    identities = []
    table = []
    previous_time_ns = None
    for index, ((_directory, manifest), current) in enumerate(zip(captures, target_rows)):
        sensors = manifest.get("sensors", {})
        capture_ns = int(manifest.get("capture_unix_ns", 0))
        gap_s = None if previous_time_ns is None else (capture_ns - previous_time_ns) / 1e9
        previous_time_ns = capture_ns
        traffic = sensors.get("traffic_light_front", {})
        received = traffic.get("received_unix_ns")
        traffic_received.append(received)
        offset = traffic.get("trigger_offset_ms")
        traffic_offsets.append(offset)
        identities.append(sample_identity(manifest))
        row: dict[str, Any] = {
            "index": index,
            "capture_id": manifest.get("capture_id", captures[index][0].name),
            "gap_s": "" if gap_s is None else round(gap_s, 3),
            "spread_ms": manifest.get("software_receive_spread_ms"),
            "traffic_received_ns": received,
            "traffic_offset_ms": offset,
        }
        moved = []
        for direction in DIRECTIONS:
            item = current.get(direction)
            center = None if item is None else np.asarray(item["center_top_m"], dtype=float)
            if center is None:
                distance = None
            else:
                distance = float(np.linalg.norm(center - baselines[direction][0]))
                row[f"{direction}_x"] = round(float(center[0]), 4)
                row[f"{direction}_y"] = round(float(center[1]), 4)
                row[f"{direction}_z"] = round(float(center[2]), 4)
                if distance > 0.45:
                    moved.append(direction)
            row[f"{direction}_baseline_delta_m"] = "" if distance is None else round(distance, 3)
        row["motion_signals"] = ",".join(moved)
        for camera in CAMERAS:
            row[f"{camera}_tags"] = int(
                sensors.get(camera, {}).get("detection", {}).get("tag_count", 0)
            )
        for lidar in ("lidar_front_left", "lidar_front_right", "lidar_rear"):
            row[f"{lidar}_directions"] = ",".join(
                sensors.get(lidar, {}).get("detection", {}).get("directions", [])
            )
        table.append(row)

    csv_path = args.output_dir / "capture_sequence.csv"
    with csv_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(table[0]))
        writer.writeheader()
        writer.writerows(table)

    windows = []
    for start in range(0, len(table), args.window):
        block = table[start : start + args.window]
        item: dict[str, Any] = {
            "start_index": start,
            "end_index": start + len(block) - 1,
            "start_capture": block[0]["capture_id"],
            "end_capture": block[-1]["capture_id"],
            "motion_signal_counts": dict(
                Counter(
                    token for row in block for token in row["motion_signals"].split(",") if token
                )
            ),
        }
        for camera in CAMERAS:
            values = [int(row[f"{camera}_tags"]) for row in block]
            item[f"{camera}_tag_median"] = float(statistics.median(values))
            item[f"{camera}_positive"] = sum(value > 0 for value in values)
        windows.append(item)

    duplicate_groups: dict[str, list[str]] = defaultdict(list)
    for identity, row in zip(identities, table):
        duplicate_groups[identity].append(row["capture_id"])
    exact_duplicates = [values for values in duplicate_groups.values() if len(values) > 1]
    traffic_run = longest_constant_run(traffic_received)
    if traffic_run["length"]:
        traffic_run["start_capture"] = table[traffic_run["start_index"]]["capture_id"]
        traffic_run["end_capture"] = table[traffic_run["end_index_exclusive"] - 1]["capture_id"]
    stale_by_offset = [
        row["capture_id"]
        for row in table
        if isinstance(row["traffic_offset_ms"], (int, float))
        and abs(float(row["traffic_offset_ms"])) > 2000.0
    ]
    summary = {
        "capture_count": len(table),
        "first_capture": table[0]["capture_id"],
        "last_capture": table[-1]["capture_id"],
        "baseline_targets": {
            direction: {
                "center_top_m": np.round(value[0], 4).tolist(),
                "support_count": value[1],
            }
            for direction, value in baselines.items()
        },
        "traffic_unique_received_frames": len(set(traffic_received)),
        "traffic_longest_constant_run": traffic_run,
        "traffic_stale_by_2s_offset_count": len(stale_by_offset),
        "traffic_stale_by_2s_offset_first": stale_by_offset[0] if stale_by_offset else None,
        "exact_duplicate_sample_groups": exact_duplicates,
        "exact_duplicate_capture_count": sum(len(values) - 1 for values in exact_duplicates),
        "windows": windows,
    }
    summary_path = args.output_dir / "capture_sequence_summary.json"
    summary_path.write_text(
        json.dumps(summary, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
