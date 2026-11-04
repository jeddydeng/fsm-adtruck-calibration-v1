#!/usr/bin/env python3
"""Refine lower-lidar extrinsics against the stationary garage structure.

This intentionally ignores capture-station board labels.  The vehicle stayed
fixed, so walls, columns and the garage floor provide far more independent
geometry than a single 900 mm board.  Moving boards are sparse robust outliers.
The script writes a review-only result and never edits the live Autoware TF.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
from typing import Any

import numpy as np
import yaml
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation

from calibrate_lidar_heights import read_binary_pcd_xyz


LOWER_LIDARS = {
    "lidar_front_left": "velodyne_front_left_base_link",
    "lidar_front_right": "velodyne_front_right_base_link",
    "lidar_rear": "velodyne_rear_base_link",
}


def matrix_from_item(item: dict[str, Any]) -> np.ndarray:
    matrix = np.eye(4)
    matrix[:3, :3] = Rotation.from_euler("xyz", item["rpy_deg"], degrees=True).as_matrix()
    matrix[:3, 3] = np.asarray(item["xyz_m"], dtype=float)
    return matrix


def apply(matrix: np.ndarray, points: np.ndarray) -> np.ndarray:
    return points @ matrix[:3, :3].T + matrix[:3, 3]


def voxel(points: np.ndarray, size: float) -> np.ndarray:
    if not len(points):
        return points
    cells = np.floor(points / size).astype(np.int32)
    _, index = np.unique(cells, axis=0, return_index=True)
    return points[np.sort(index)]


def crop_top(points: np.ndarray) -> np.ndarray:
    radius = np.linalg.norm(points[:, :2], axis=1)
    return points[
        (radius >= 1.0) & (radius <= 13.0) & (points[:, 2] >= -1.90) & (points[:, 2] <= 0.80)
    ]


def pcd_for(capture: Path, sensor: str) -> Path:
    matches = list((capture / "lidars").glob(f"*_{sensor}.pcd"))
    if len(matches) != 1:
        raise RuntimeError(f"{capture.name}: expected one {sensor} PCD, got {len(matches)}")
    return matches[0]


def selected_captures(root: Path, annotations: Path | None, limit: int) -> list[Path]:
    available = {path.parent.name: path.parent for path in root.glob("*/manifest.json")}
    names: list[str]
    if annotations:
        with annotations.open(newline="", encoding="utf-8") as stream:
            rows = list(csv.DictReader(stream))
        names = [
            row["capture_id"]
            for row in rows
            if row.get("include_nonduplicate", "").lower() == "true"
        ]
    else:
        names = sorted(available)
    names = [name for name in names if name in available]
    if len(names) > limit:
        indices = np.linspace(0, len(names) - 1, limit).round().astype(int)
        names = [names[index] for index in indices]
    return [available[name] for name in names]


def aggregate(
    captures: list[Path], sensor: str, local_voxel: float, initial: np.ndarray | None = None
) -> np.ndarray:
    clouds = []
    for capture in captures:
        points = read_binary_pcd_xyz(pcd_for(capture, sensor))
        distance = np.linalg.norm(points, axis=1)
        points = points[(distance >= 0.7) & (distance <= 14.0)]
        if initial is not None:
            points = crop_top(apply(initial, points))
            points = apply(np.linalg.inv(initial), points)
        clouds.append(voxel(points, local_voxel))
    return voxel(np.concatenate(clouds), local_voxel)


def rigid_fit(source: np.ndarray, target: np.ndarray, weights: np.ndarray) -> np.ndarray:
    weights = weights / weights.sum()
    source_center = np.sum(source * weights[:, None], axis=0)
    target_center = np.sum(target * weights[:, None], axis=0)
    covariance = (source - source_center).T @ ((target - target_center) * weights[:, None])
    left, _, right_t = np.linalg.svd(covariance)
    rotation = right_t.T @ left.T
    if np.linalg.det(rotation) < 0:
        right_t[-1] *= -1
        rotation = right_t.T @ left.T
    result = np.eye(4)
    result[:3, :3] = rotation
    result[:3, 3] = target_center - rotation @ source_center
    return result


def delta_size(matrix: np.ndarray) -> tuple[float, float]:
    angle = math.degrees(Rotation.from_matrix(matrix[:3, :3]).magnitude())
    return float(np.linalg.norm(matrix[:3, 3])), float(angle)


def correspondences(
    source_top: np.ndarray,
    target: np.ndarray,
    target_tree: cKDTree,
    threshold: float,
    max_points: int = 45000,
    minimum_points: int = 250,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    source_top = crop_top(source_top)
    distances, indices = target_tree.query(source_top, k=1, workers=-1)
    mask = distances <= threshold
    source_top = source_top[mask]
    matched = target[indices[mask]]
    distances = distances[mask]
    if len(distances) < minimum_points:
        raise RuntimeError(f"only {len(distances)} correspondences within {threshold:.2f} m")
    # Trim sliding/unobserved surfaces while retaining multiple orthogonal walls.
    limit = min(threshold, float(np.percentile(distances, 72.0)))
    keep = distances <= limit
    source_top, matched, distances = source_top[keep], matched[keep], distances[keep]
    if len(distances) > max_points:
        indices = np.linspace(0, len(distances) - 1, max_points).astype(int)
        source_top, matched, distances = source_top[indices], matched[indices], distances[indices]
    return source_top, matched, distances


def icp(
    source_local: np.ndarray, target: np.ndarray, initial: np.ndarray
) -> tuple[np.ndarray, list[dict[str, Any]]]:
    tree = cKDTree(target)
    current = initial.copy()
    history = []
    for threshold, iterations in ((0.60, 5), (0.40, 6), (0.28, 7), (0.20, 8), (0.14, 10)):
        for _ in range(iterations):
            transformed = apply(current, source_local)
            try:
                source_match, target_match, distances = correspondences(
                    transformed, target, tree, threshold
                )
            except RuntimeError:
                # A blind-spot lidar can have little overlap with the roof
                # lidar at the finest scale.  Keep the last valid coarse result
                # instead of discarding the whole sensor solution.
                if history and threshold <= 0.20:
                    break
                raise
            scale = max(0.035, float(np.median(distances)) * 2.5)
            weights = 1.0 / (1.0 + np.square(distances / scale))
            update = rigid_fit(source_match, target_match, weights)
            translation_step, rotation_step = delta_size(update)
            if translation_step > 0.08 or rotation_step > 1.2:
                fraction = min(0.08 / max(translation_step, 1e-12), 1.2 / max(rotation_step, 1e-12))
                update[:3, :3] = Rotation.from_rotvec(
                    Rotation.from_matrix(update[:3, :3]).as_rotvec() * fraction
                ).as_matrix()
                update[:3, 3] *= fraction
            current = update @ current
            history.append(
                {
                    "threshold_m": threshold,
                    "correspondence_count": len(distances),
                    "median_m": round(float(np.median(distances)), 5),
                    "p90_m": round(float(np.percentile(distances, 90)), 5),
                    "step_translation_m": round(translation_step, 6),
                    "step_rotation_deg": round(rotation_step, 5),
                }
            )
            if translation_step < 0.0002 and rotation_step < 0.005:
                break
    correction = current @ np.linalg.inv(initial)
    translation, angle = delta_size(correction)
    if translation > 0.35 or angle > 6.0:
        raise RuntimeError(
            f"ICP correction exceeds safety bounds: {translation:.3f} m, {angle:.2f} deg"
        )
    return current, history


def validation(source: np.ndarray, target: np.ndarray, transform: np.ndarray) -> dict[str, Any]:
    transformed = crop_top(apply(transform, source))
    distances = cKDTree(target).query(transformed, k=1, workers=-1)[0]
    return {
        "point_count": len(distances),
        "median_m": round(float(np.median(distances)), 5),
        "p75_m": round(float(np.percentile(distances, 75)), 5),
        "p90_m": round(float(np.percentile(distances, 90)), 5),
        "overlap_0_10": round(float(np.mean(distances <= 0.10)), 4),
        "overlap_0_20": round(float(np.mean(distances <= 0.20)), 4),
    }


def item_from_matrix(matrix: np.ndarray) -> dict[str, Any]:
    return {
        "parent": "velodyne_top_base_link",
        "xyz_m": np.round(matrix[:3, 3], 7).tolist(),
        "rpy_deg": np.round(
            Rotation.from_matrix(matrix[:3, :3]).as_euler("xyz", degrees=True), 7
        ).tolist(),
        "source": "robust_stationary_garage_icp",
    }


def main() -> int:
    base = Path(__file__).resolve().parent
    parser = argparse.ArgumentParser()
    parser.add_argument("--data-root", type=Path, required=True)
    parser.add_argument("--annotations", type=Path)
    parser.add_argument("--initial", type=Path, default=base / "initial_extrinsics.yaml")
    parser.add_argument("--output", type=Path, default=base / "lidar_environment_refined.yaml")
    parser.add_argument("--max-captures", type=int, default=24)
    parser.add_argument("--voxel", type=float, default=0.11)
    args = parser.parse_args()
    initial = yaml.safe_load(args.initial.read_text(encoding="utf-8"))
    captures = selected_captures(args.data_root, args.annotations, args.max_captures)
    if len(captures) < 4:
        raise SystemExit("need at least four complete captures")
    print(f"using {len(captures)} captures")

    identity = np.eye(4)
    roof = crop_top(aggregate(captures, "lidar_top", args.voxel, identity))
    roof = voxel(roof, args.voxel)
    result: dict[str, Any] = {
        "schema_version": 1,
        "reference_frame": "velodyne_top_base_link",
        "capture_ids": [path.name for path in captures],
        "warning": "Review held-out metrics and RViz overlay before applying.",
        "transforms": {},
        "rejected_candidates": {},
        "quality": {},
    }
    for sensor, frame in LOWER_LIDARS.items():
        seed = matrix_from_item(initial["transforms"][frame])
        source = aggregate(captures, sensor, args.voxel, seed)
        before = validation(source, roof, seed)
        refined, history = icp(source, roof, seed)
        after = validation(source, roof, refined)
        correction = refined @ np.linalg.inv(seed)
        correction_translation, correction_angle = delta_size(correction)
        median_gain = before["median_m"] - after["median_m"]
        overlap_gain = after["overlap_0_20"] - before["overlap_0_20"]
        p75_regression = after["p75_m"] - before["p75_m"]
        accepted = bool(median_gain >= 0.01 and overlap_gain >= 0.02 and p75_regression <= 0.03)
        candidate = item_from_matrix(refined)
        if accepted:
            result["transforms"][frame] = candidate
        else:
            result["rejected_candidates"][frame] = candidate
        result["quality"][frame] = {
            "accepted": accepted,
            "acceptance_rule": (
                "median_gain>=0.01m and overlap_0_20_gain>=0.02 and p75_regression<=0.03m"
            ),
            "before": before,
            "after": after,
            "correction_translation_m": round(correction_translation, 6),
            "correction_rotation_deg": round(correction_angle, 5),
            "iterations": history,
        }
        status = "accepted" if accepted else "REJECTED"
        print(
            f"{sensor}: {status}, median {before['median_m']:.3f}->"
            f"{after['median_m']:.3f} m, overlap20 "
            f"{before['overlap_0_20']:.1%}->{after['overlap_0_20']:.1%}"
        )
    args.output.write_text(
        yaml.safe_dump(result, allow_unicode=True, sort_keys=False), encoding="utf-8"
    )
    print(f"review-only result: {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
