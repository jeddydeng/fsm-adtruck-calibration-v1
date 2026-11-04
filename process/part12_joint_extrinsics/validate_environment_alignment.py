#!/usr/bin/env python3
"""Independent wall/column/floor alignment check for Part 1/2 lidar poses."""

from __future__ import annotations

import argparse
from collections import defaultdict
import json
from pathlib import Path
import sys

import numpy as np
from scipy.spatial import cKDTree
import yaml


HERE = Path(__file__).resolve().parent
JOINT = HERE.parent / "joint_extrinsic_station"
sys.path.insert(0, str(JOINT))
from calibrate_lidar_heights import read_binary_pcd_xyz  # noqa: E402
from refine_camera_extrinsics import pcd_for, voxel  # noqa: E402
from refine_lidar_environment import apply, matrix_from_item  # noqa: E402


FRAMES = {
    "lidar_front_left": "velodyne_front_left_base_link",
    "lidar_front_right": "velodyne_front_right_base_link",
    "lidar_rear": "velodyne_rear_base_link",
}


def cloud(directory: Path, sensor: str, transform: np.ndarray) -> np.ndarray:
    points = apply(transform, read_binary_pcd_xyz(pcd_for(directory, sensor)))
    radius = np.linalg.norm(points[:, :2], axis=1)
    points = points[
        (radius >= 0.8) & (radius <= 11.0) & (points[:, 2] >= -1.65) & (points[:, 2] <= 0.8)
    ]
    return voxel(points, 0.070)


def common_side_clouds(
    directory: Path, sensor: str, previous: np.ndarray, candidate: np.ndarray
) -> tuple[np.ndarray, np.ndarray]:
    raw = read_binary_pcd_xyz(pcd_for(directory, sensor))
    before, after = apply(previous, raw), apply(candidate, raw)
    before_radius = np.linalg.norm(before[:, :2], axis=1)
    after_radius = np.linalg.norm(after[:, :2], axis=1)
    mask = (
        (before_radius >= 0.8)
        & (before_radius <= 11.0)
        & (after_radius >= 0.8)
        & (after_radius <= 11.0)
        & (before[:, 2] >= -1.65)
        & (before[:, 2] <= 0.8)
        & (after[:, 2] >= -1.65)
        & (after[:, 2] <= 0.8)
    )
    before, after = before[mask], after[mask]
    # Evaluate identical raw returns under both transforms.
    if len(before) > 7000:
        take = np.linspace(0, len(before) - 1, 7000).round().astype(int)
        before, after = before[take], after[take]
    return before, after


def summarize(values: list[float]) -> dict[str, float | int]:
    array = np.asarray(values)
    clipped = array[array <= 0.30]
    return {
        "sample_points": int(len(array)),
        "overlap_points_le_0p30m": int(len(clipped)),
        "overlap_fraction": float(len(clipped) / max(len(array), 1)),
        "fraction_le_0p10m": float(np.mean(array <= 0.10)),
        "overlap_median_m": float(np.median(clipped)) if len(clipped) else None,
        "overlap_p95_m": float(np.percentile(clipped, 95)) if len(clipped) else None,
    }


def main() -> int:
    base = Path(__file__).resolve().parents[2]
    parser = argparse.ArgumentParser()
    parser.add_argument("--base", type=Path, default=base)
    parser.add_argument(
        "--old",
        type=Path,
        default=base / "result/extrinsics/all_sensors_relative_to_roof_lidar.yaml",
    )
    parser.add_argument(
        "--candidate",
        type=Path,
        default=base / "result/part12_20260903/part12_joint_extrinsics.yaml",
    )
    parser.add_argument(
        "--output", type=Path, default=base / "result/part12_20260903/environment_alignment.json"
    )
    parser.add_argument("--captures-per-part", type=int, default=12)
    args = parser.parse_args()

    old = yaml.safe_load(args.old.read_text(encoding="utf-8"))
    candidate = yaml.safe_load(args.candidate.read_text(encoding="utf-8"))
    transforms = {
        "previous": {
            sensor: matrix_from_item(old["transforms"][frame]) for sensor, frame in FRAMES.items()
        },
        "candidate": {
            sensor: matrix_from_item(candidate["transforms"][frame])
            for sensor, frame in FRAMES.items()
        },
    }
    distances: dict[tuple[str, int, str], list[float]] = defaultdict(list)
    used = []
    manifest_root = args.base / "data/backup_manifests/20260903"
    for part in (1, 2):
        ids = (manifest_root / f"part{part}_capture_groups.txt").read_text(encoding="utf-8").split()
        indices = (
            np.linspace(0, len(ids) - 1, min(args.captures_per_part, len(ids))).round().astype(int)
        )
        for index in sorted(set(indices.tolist())):
            capture_id = ids[index]
            directory = args.base / "data/raw_extrinsic_data" / capture_id
            top = cloud(directory, "lidar_top", np.eye(4))
            tree = cKDTree(top)
            for sensor in FRAMES:
                before, after = common_side_clouds(
                    directory,
                    sensor,
                    transforms["previous"][sensor],
                    transforms["candidate"][sensor],
                )
                for label, side in (("previous", before), ("candidate", after)):
                    nearest = tree.query(side, k=1, workers=-1)[0]
                    distances[(label, part, sensor)].extend(nearest.tolist())
            used.append({"part": part, "capture_id": capture_id})
            print(json.dumps({"part": part, "capture": capture_id}, ensure_ascii=False), flush=True)

    result: dict[str, object] = {
        "schema_version": 1,
        "method": "directed_nearest_roof_lidar_surface_distance",
        "note": "Only distances <=0.30 m are treated as overlapping static surfaces; boards are a negligible fraction of the garage cloud.",
        "captures": used,
        "per_part": {},
        "combined": {},
    }
    for part in (1, 2):
        result["per_part"][f"part{part}"] = {}
        for sensor in FRAMES:
            result["per_part"][f"part{part}"][sensor] = {
                label: summarize(distances[(label, part, sensor)])
                for label in ("previous", "candidate")
            }
    for sensor in FRAMES:
        result["combined"][sensor] = {}
        for label in ("previous", "candidate"):
            values = distances[(label, 1, sensor)] + distances[(label, 2, sensor)]
            result["combined"][sensor][label] = summarize(values)
        before = result["combined"][sensor]["previous"]
        after = result["combined"][sensor]["candidate"]
        result["combined"][sensor]["candidate_improves_median"] = (
            after["overlap_median_m"] < before["overlap_median_m"]
        )
        result["combined"][sensor]["candidate_improves_fraction_le_0p10m"] = (
            after["fraction_le_0p10m"] > before["fraction_le_0p10m"]
        )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps(result["combined"], ensure_ascii=False), flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
