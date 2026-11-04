#!/usr/bin/env python3
"""Read-only quality audit for joint camera/lidar capture station datasets."""

from __future__ import annotations

import argparse
import json
import math
import statistics
from collections import Counter, defaultdict
from datetime import datetime
from pathlib import Path
from typing import Any

import numpy as np


def percentile(values: list[float], value: float) -> float | None:
    if not values:
        return None
    return round(float(np.percentile(np.asarray(values, dtype=float), value)), 3)


def distribution(values: list[float]) -> dict[str, float | None]:
    if not values:
        return {"min": None, "median": None, "p95": None, "max": None}
    return {
        "min": round(min(values), 3),
        "median": round(statistics.median(values), 3),
        "p95": percentile(values, 95),
        "max": round(max(values), 3),
    }


def center_geometry(points: list[list[float]], cluster_radius: float) -> dict[str, Any]:
    if not points:
        return {"count": 0, "rank_tol_0_08": 0, "cluster_count": 0}
    array = np.asarray(points, dtype=float)
    centered = array - array.mean(axis=0)
    singular = np.linalg.svd(centered, compute_uv=False)
    clusters: list[np.ndarray] = []
    counts: list[int] = []
    for point in array:
        if not clusters:
            clusters.append(point.copy())
            counts.append(1)
            continue
        distances = np.linalg.norm(np.asarray(clusters) - point, axis=1)
        nearest = int(np.argmin(distances))
        if float(distances[nearest]) <= cluster_radius:
            counts[nearest] += 1
            clusters[nearest] += (point - clusters[nearest]) / counts[nearest]
        else:
            clusters.append(point.copy())
            counts.append(1)
    return {
        "count": len(points),
        "mean_xyz_m": np.round(array.mean(axis=0), 4).tolist(),
        "range_xyz_m": np.round(np.ptp(array, axis=0), 4).tolist(),
        "singular_values_m": np.round(singular, 4).tolist(),
        "rank_tol_0_08": int(np.linalg.matrix_rank(centered, tol=0.08)),
        "cluster_radius_m": cluster_radius,
        "cluster_count": len(clusters),
        "cluster_centers_xyz_m": np.round(np.asarray(clusters), 3).tolist(),
        "cluster_sizes": counts,
    }


def angular_spread_deg(normals: list[list[float]]) -> dict[str, float | None]:
    if not normals:
        return {"median_from_mean": None, "max_from_mean": None}
    vectors = np.asarray(normals, dtype=float)
    lengths = np.linalg.norm(vectors, axis=1)
    vectors = vectors[lengths > 1e-8]
    if not len(vectors):
        return {"median_from_mean": None, "max_from_mean": None}
    vectors /= np.linalg.norm(vectors, axis=1, keepdims=True)
    mean = vectors.mean(axis=0)
    mean /= np.linalg.norm(mean)
    angles = [math.degrees(math.acos(float(np.clip(abs(item @ mean), -1, 1)))) for item in vectors]
    return {
        "median_from_mean": round(statistics.median(angles), 3),
        "max_from_mean": round(max(angles), 3),
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("root", type=Path)
    parser.add_argument("--prefix", default="")
    parser.add_argument("--cluster-radius", type=float, default=0.15)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()

    manifests: list[tuple[Path, dict[str, Any]]] = []
    for path in sorted(args.root.glob(f"{args.prefix}*/manifest.json")):
        try:
            manifests.append((path.parent, json.loads(path.read_text(encoding="utf-8"))))
        except (OSError, ValueError):
            pass
    if not manifests:
        raise SystemExit("no manifests found")

    spreads: list[float] = []
    offsets: list[float] = []
    capture_times: list[datetime] = []
    saved_counts: list[int] = []
    complete = 0
    requirements_met = 0
    missing_files: list[str] = []
    sensor_saved: Counter[str] = Counter()
    camera_tags: dict[str, list[int]] = defaultdict(list)
    lidar_direction_counts: dict[str, Counter[str]] = defaultdict(Counter)
    reference_centers: dict[str, list[list[float]]] = defaultdict(list)
    reference_normals: dict[str, list[list[float]]] = defaultdict(list)
    lidar_reference_centers: dict[str, list[list[float]]] = defaultdict(list)
    front_positions: list[dict[str, Any]] = []

    for directory, manifest in manifests:
        value = manifest.get("software_receive_spread_ms")
        if isinstance(value, (int, float)):
            spreads.append(float(value))
        stamp = manifest.get("capture_time")
        if stamp:
            capture_times.append(datetime.fromisoformat(stamp))
        saved_counts.append(int(manifest.get("saved_sensor_count", 0)))
        complete += bool(manifest.get("complete_sensor_set"))
        requirements_met += bool(manifest.get("requirements", {}).get("met"))
        sensors = manifest.get("sensors", {})
        top_targets: dict[str, dict[str, Any]] = {}
        for target in sensors.get("lidar_top", {}).get("detection", {}).get("targets", []):
            direction = str(target.get("direction"))
            top_targets[direction] = target
            reference_centers[direction].append(target["center_top_m"])
            reference_normals[direction].append(target["normal_top"])
        if "front" in top_targets:
            front_positions.append(
                {
                    "capture_id": manifest.get("capture_id", directory.name),
                    "center_top_m": top_targets["front"]["center_top_m"],
                    "normal_top": top_targets["front"]["normal_top"],
                }
            )
        for name, sensor in sensors.items():
            if sensor.get("saved", True):
                sensor_saved[name] += 1
            relative = sensor.get("file")
            if not relative or not (directory / relative).is_file():
                missing_files.append(f"{directory.name}:{name}:{relative}")
            offset = sensor.get("trigger_offset_ms")
            if isinstance(offset, (int, float)):
                offsets.append(abs(float(offset)))
            kind = str(sensor.get("kind", ""))
            detection = sensor.get("detection", {})
            if "camera" in kind:
                camera_tags[name].append(int(detection.get("tag_count", 0)))
            elif kind == "lidar":
                for target in detection.get("targets", []):
                    direction = str(target.get("direction"))
                    lidar_direction_counts[name][direction] += 1
                    if direction in top_targets:
                        lidar_reference_centers[name].append(top_targets[direction]["center_top_m"])

    gaps = [(right - left).total_seconds() for left, right in zip(capture_times, capture_times[1:])]
    camera_summary = {}
    for name, counts in sorted(camera_tags.items()):
        positive = [value for value in counts if value > 0]
        camera_summary[name] = {
            "frames": len(counts),
            "tag_positive_frames": len(positive),
            "positive_rate": round(len(positive) / len(counts), 3) if counts else 0,
            "tag_count_all": distribution([float(value) for value in counts]),
            "tag_count_positive": distribution([float(value) for value in positive]),
        }
    lidar_summary = {}
    for name in sorted(lidar_direction_counts):
        lidar_summary[name] = {
            "directions": dict(lidar_direction_counts[name]),
            "reference_center_geometry": center_geometry(
                lidar_reference_centers[name], args.cluster_radius
            ),
        }

    report = {
        "root": str(args.root),
        "prefix": args.prefix,
        "capture_count": len(manifests),
        "first_capture": manifests[0][1].get("capture_id", manifests[0][0].name),
        "last_capture": manifests[-1][1].get("capture_id", manifests[-1][0].name),
        "duration_s": round((capture_times[-1] - capture_times[0]).total_seconds(), 3)
        if len(capture_times) >= 2
        else 0,
        "complete_sensor_sets": complete,
        "requirements_met_sets": requirements_met,
        "saved_sensor_count_distribution": dict(Counter(saved_counts)),
        "missing_file_count": len(missing_files),
        "missing_files": missing_files,
        "software_receive_spread_ms": distribution(spreads),
        "absolute_trigger_offset_ms": distribution(offsets),
        "capture_gap_s": distribution(gaps),
        "capture_gap_under_3s": sum(value < 3 for value in gaps),
        "saved_per_sensor": dict(sorted(sensor_saved.items())),
        "reference_target_geometry": {
            direction: {
                "centers": center_geometry(points, args.cluster_radius),
                "normal_spread_deg": angular_spread_deg(reference_normals[direction]),
            }
            for direction, points in sorted(reference_centers.items())
        },
        "front_target_positions": front_positions,
        "lidars": lidar_summary,
        "cameras": camera_summary,
    }
    rendered = json.dumps(report, ensure_ascii=False, indent=2)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(rendered + "\n", encoding="utf-8")
    print(rendered)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
