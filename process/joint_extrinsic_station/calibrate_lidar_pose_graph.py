#!/usr/bin/env python3
"""Robust offline lower-lidar calibration with floor and garage constraints.

The vehicle stayed stationary for the whole capture session.  The roof lidar is
therefore the fixed reference, while the garage floor fixes height/roll/pitch
and walls/columns fix horizontal translation/yaw.  Five disjoint bootstrap
groups are solved independently.  A result is reported as consensus only when
at least 60% of the groups agree within 10 cm and 1 degree.

This is an offline, review-only tool.  It never edits the vehicle configuration.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
import re
from typing import Any

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation
import yaml

from refine_lidar_environment import (
    LOWER_LIDARS,
    aggregate,
    apply,
    crop_top,
    matrix_from_item,
    pcd_for,
    validation,
    voxel,
)
from calibrate_lidar_heights import read_binary_pcd_xyz


VECTOR_RE = re.compile(r"\[([^]]+)\]")


def parse_vector(text: str) -> np.ndarray:
    match = VECTOR_RE.fullmatch(text.strip())
    if not match:
        raise ValueError(f"invalid vector: {text!r}")
    return np.asarray([float(value) for value in match.group(1).split()])


def load_floor_evidence(path: Path, initial: dict[str, Any]) -> dict[str, Any]:
    roof_normals: list[np.ndarray] = []
    values: dict[str, dict[str, list[Any]]] = {
        name: {"normal": [], "z": [], "height": [], "rms": []} for name in LOWER_LIDARS
    }
    with path.open(newline="", encoding="utf-8") as stream:
        for row in csv.DictReader(stream):
            sensor = row.get("sensor", "")
            if sensor == "lidar_top" and row.get("normal_in_roof_seed"):
                roof_normals.append(parse_vector(row["normal_in_roof_seed"]))
            elif sensor in values and row.get("normal_in_roof_seed"):
                values[sensor]["normal"].append(parse_vector(row["normal_in_roof_seed"]))
                values[sensor]["z"].append(float(row["new_translation_z_m"]))
                values[sensor]["height"].append(float(row["height_or_ground_z_m"]))
                values[sensor]["rms"].append(float(row["rms_m"]))
    if not roof_normals:
        raise ValueError("floor evidence contains no roof normal")
    roof = np.mean(roof_normals, axis=0)
    roof /= np.linalg.norm(roof)
    result: dict[str, Any] = {"roof_normal": roof}
    for sensor, frame in LOWER_LIDARS.items():
        if not values[sensor]["normal"]:
            raise ValueError(f"floor evidence contains no {sensor} measurements")
        current = np.mean(values[sensor]["normal"], axis=0)
        current /= np.linalg.norm(current)
        # Minimal rotation taking the currently transformed floor normal onto
        # the roof-lidar floor normal.  This changes only the two observable
        # tilt degrees of freedom; wall/column alignment handles yaw later.
        correction, _ = Rotation.align_vectors(roof.reshape(1, 3), current.reshape(1, 3))
        seed = matrix_from_item(initial["transforms"][frame])
        floor_seed = seed.copy()
        floor_seed[:3, :3] = correction.as_matrix() @ seed[:3, :3]
        floor_seed[2, 3] = float(np.median(values[sensor]["z"]))
        result[sensor] = {
            "transform": floor_seed,
            "roof_normal": roof,
            "normal_before": current,
            "normal_after": correction.apply(current),
            "tilt_correction_deg": math.degrees(correction.magnitude()),
            "translation_z_m": float(floor_seed[2, 3]),
            "translation_z_max_deviation_m": float(
                np.max(np.abs(np.asarray(values[sensor]["z"]) - floor_seed[2, 3]))
            ),
            "physical_height_m": float(np.median(values[sensor]["height"])),
            "floor_plane_rms_m": float(np.median(values[sensor]["rms"])),
            "sample_count": len(values[sensor]["z"]),
        }
    return result


def read_annotations(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as stream:
        return [
            row
            for row in csv.DictReader(stream)
            if row.get("include_nonduplicate", "").lower() == "true"
        ]


def bootstrap_groups(
    rows: list[dict[str, str]], root: Path, group_count: int, captures_per_group: int
) -> list[list[Path]]:
    groups: list[list[Path]] = []
    phases = sorted({row["phase"] for row in rows})
    by_phase = {phase: [row for row in rows if row["phase"] == phase] for phase in phases}
    for group_index in range(group_count):
        chosen: list[str] = []
        # Every bootstrap receives all acquisition phases; offsets differ so
        # the five solutions are based on independent timestamps.
        for phase in phases:
            candidates = by_phase[phase][group_index::group_count]
            quota = max(1, round(captures_per_group * len(by_phase[phase]) / max(len(rows), 1)))
            if len(candidates) > quota:
                indices = np.linspace(0, len(candidates) - 1, quota).round().astype(int)
                candidates = [candidates[index] for index in indices]
            chosen.extend(row["capture_id"] for row in candidates)
        chosen = sorted(dict.fromkeys(chosen))
        if len(chosen) > captures_per_group:
            indices = np.linspace(0, len(chosen) - 1, captures_per_group).round().astype(int)
            chosen = [chosen[index] for index in indices]
        groups.append([root / capture_id for capture_id in chosen])
    return groups


def target_normals(points: np.ndarray, neighbours: int = 14) -> tuple[np.ndarray, np.ndarray]:
    tree = cKDTree(points)
    indices = tree.query(points, k=min(neighbours, len(points)), workers=-1)[1]
    local = points[indices]
    centered = local - np.mean(local, axis=1, keepdims=True)
    covariance = np.einsum("nki,nkj->nij", centered, centered) / max(centered.shape[1], 1)
    eigenvalues, eigenvectors = np.linalg.eigh(covariance)
    normals = eigenvectors[:, :, 0]
    curvature = eigenvalues[:, 0] / np.maximum(np.sum(eigenvalues, axis=1), 1e-12)
    return normals, curvature


def planar_matrix(base: np.ndarray, values: np.ndarray) -> np.ndarray:
    result = base.copy()
    yaw = Rotation.from_euler("z", float(values[2])).as_matrix()
    result[:3, :3] = yaw @ base[:3, :3]
    result[0, 3] = base[0, 3] + float(values[0])
    result[1, 3] = base[1, 3] + float(values[1])
    return result


def point_to_plane_icp(
    source_local: np.ndarray, target: np.ndarray, base: np.ndarray
) -> tuple[np.ndarray, list[dict[str, Any]]]:
    tree = cKDTree(target)
    normals, curvature = target_normals(target)
    current = np.zeros(3)
    history: list[dict[str, Any]] = []
    for threshold, count in ((0.55, 4), (0.35, 5), (0.22, 6), (0.14, 8)):
        for _ in range(count):
            transform = planar_matrix(base, current)
            transformed = crop_top(apply(transform, source_local))
            distances, indices = tree.query(transformed, k=1, workers=-1)
            matched_normals = normals[indices]
            mask = (
                (distances <= threshold)
                & (curvature[indices] <= 0.055)
                & (np.abs(matched_normals[:, 2]) <= 0.78)
            )
            if np.count_nonzero(mask) < 180:
                if history and threshold <= 0.22:
                    break
                raise RuntimeError(f"only {np.count_nonzero(mask)} planar matches")
            source_match = transformed[mask]
            target_match = target[indices[mask]]
            normal_match = matched_normals[mask]
            distance_match = distances[mask]
            trim = min(threshold, float(np.percentile(distance_match, 68.0)))
            keep = distance_match <= trim
            source_match = source_match[keep]
            target_match = target_match[keep]
            normal_match = normal_match[keep]
            if len(source_match) > 24000:
                choice = np.linspace(0, len(source_match) - 1, 24000).astype(int)
                source_match = source_match[choice]
                target_match = target_match[choice]
                normal_match = normal_match[choice]

            # Optimize an incremental planar update around the current pose.
            current_transform = transform
            source_in_local = apply(np.linalg.inv(current_transform), source_match)

            def residual(delta: np.ndarray) -> np.ndarray:
                candidate = planar_matrix(current_transform, delta)
                moved = apply(candidate, source_in_local)
                plane = np.sum((moved - target_match) * normal_match, axis=1)
                # Weak mechanical prior prevents sliding along a single wall.
                prior = np.asarray([delta[0] / 0.08, delta[1] / 0.08, delta[2] / math.radians(1.5)])
                return np.r_[plane / 0.035, prior]

            solved = least_squares(residual, np.zeros(3), loss="huber", f_scale=1.0, max_nfev=80)
            step = solved.x
            # Convert the incremental matrix back into base-relative planar
            # parameters without using Euler angles near the sensor's -90° pitch.
            updated = planar_matrix(current_transform, step)
            current[0] = updated[0, 3] - base[0, 3]
            current[1] = updated[1, 3] - base[1, 3]
            relative_rotation = updated[:3, :3] @ base[:3, :3].T
            current[2] = Rotation.from_matrix(relative_rotation).as_rotvec()[2]
            history.append(
                {
                    "threshold_m": threshold,
                    "matches": int(len(source_match)),
                    "median_euclidean_m": round(float(np.median(distance_match[keep])), 6),
                    "step_xy_m": round(float(np.linalg.norm(step[:2])), 7),
                    "step_yaw_deg": round(math.degrees(float(step[2])), 7),
                }
            )
            if np.linalg.norm(step[:2]) < 2e-5 and abs(step[2]) < math.radians(0.002):
                break
    return planar_matrix(base, current), history


def transform_distance(left: np.ndarray, right: np.ndarray) -> tuple[float, float]:
    translation = float(np.linalg.norm(left[:3, 3] - right[:3, 3]))
    rotation = math.degrees(Rotation.from_matrix(left[:3, :3] @ right[:3, :3].T).magnitude())
    return translation, rotation


def consensus(transforms: list[np.ndarray]) -> tuple[np.ndarray, list[int], dict[str, Any]]:
    neighbourhoods = []
    for left, candidate in enumerate(transforms):
        members = []
        for right, other in enumerate(transforms):
            distance, angle = transform_distance(candidate, other)
            if distance <= 0.10 and angle <= 1.0:
                members.append(right)
        neighbourhoods.append(members)
    members = max(neighbourhoods, key=len)
    if len(members) / len(transforms) < 0.60:
        raise RuntimeError(f"dominant bootstrap cluster is only {len(members)}/{len(transforms)}")
    selected = [transforms[index] for index in members]
    result = np.eye(4)
    result[:3, 3] = np.median([value[:3, 3] for value in selected], axis=0)
    result[:3, :3] = Rotation.from_matrix([value[:3, :3] for value in selected]).mean().as_matrix()
    distances = [transform_distance(result, value) for value in selected]
    return (
        result,
        members,
        {
            "dominant_count": len(members),
            "total_count": len(transforms),
            "dominant_fraction": len(members) / len(transforms),
            "translation_p95_m": float(np.percentile([item[0] for item in distances], 95)),
            "rotation_p95_deg": float(np.percentile([item[1] for item in distances], 95)),
            "translation_max_m": max(item[0] for item in distances),
            "rotation_max_deg": max(item[1] for item in distances),
        },
    )


def matrix_item(matrix: np.ndarray, source: str) -> dict[str, Any]:
    return {
        "parent": "velodyne_top_base_link",
        "xyz_m": np.round(matrix[:3, 3], 8).tolist(),
        "rpy_deg": np.round(
            Rotation.from_matrix(matrix[:3, :3]).as_euler("xyz", degrees=True), 8
        ).tolist(),
        "source": source,
    }


def main() -> int:
    base = Path(__file__).resolve().parent
    parser = argparse.ArgumentParser()
    parser.add_argument("--data-root", type=Path, required=True)
    parser.add_argument("--annotations", type=Path, required=True)
    parser.add_argument("--initial", type=Path, default=base / "initial_extrinsics.yaml")
    parser.add_argument("--floor-evidence", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--bootstrap-groups", type=int, default=5)
    parser.add_argument("--captures-per-group", type=int, default=18)
    parser.add_argument("--voxel", type=float, default=0.11)
    args = parser.parse_args()

    initial = yaml.safe_load(args.initial.read_text(encoding="utf-8"))
    floor = load_floor_evidence(args.floor_evidence, initial)
    rows = read_annotations(args.annotations)
    groups = bootstrap_groups(rows, args.data_root, args.bootstrap_groups, args.captures_per_group)
    validation_names = [row["capture_id"] for row in rows[2 :: max(1, len(rows) // 18)]][:18]
    validation_captures = [args.data_root / name for name in validation_names]

    bootstraps: dict[str, list[dict[str, Any]]] = {name: [] for name in LOWER_LIDARS}
    matrices: dict[str, list[np.ndarray]] = {name: [] for name in LOWER_LIDARS}
    for group_index, captures in enumerate(groups, start=1):
        print(f"bootstrap {group_index}/{len(groups)}: {len(captures)} captures", flush=True)
        roof = voxel(crop_top(aggregate(captures, "lidar_top", args.voxel, np.eye(4))), args.voxel)
        for sensor in LOWER_LIDARS:
            base_transform = floor[sensor]["transform"]
            source = aggregate(captures, sensor, args.voxel, base_transform)
            before = validation(source, roof, base_transform)
            fitted, history = point_to_plane_icp(source, roof, base_transform)
            # Preserve the independently measured floor height exactly.
            fitted[2, 3] = base_transform[2, 3]
            after = validation(source, roof, fitted)
            matrices[sensor].append(fitted)
            bootstraps[sensor].append(
                {
                    "group": group_index,
                    "capture_ids": [path.name for path in captures],
                    "transform": matrix_item(fitted, "bootstrap_floor_wall_column"),
                    "before": before,
                    "after": after,
                    "iterations": history,
                }
            )
            print(
                f"  {sensor}: median {before['median_m']:.3f}->"
                f"{after['median_m']:.3f} m, overlap20 "
                f"{before['overlap_0_20']:.1%}->{after['overlap_0_20']:.1%}",
                flush=True,
            )

    result: dict[str, Any] = {
        "schema_version": 1,
        "reference_frame": "velodyne_top_base_link",
        "method": "floor_normal_and_height_plus_bootstrap_point_to_plane_garage_pose_graph",
        "policy": {
            "dominant_cluster_min_fraction": 0.60,
            "cluster_translation_m": 0.10,
            "cluster_rotation_deg": 1.0,
            "target_translation_repeatability_m": 0.001,
            "target_rotation_repeatability_deg": 0.1,
            "fallback_translation_repeatability_m": 0.01,
            "fallback_rotation_repeatability_deg": 1.0,
            "note": "repeatability is not the same as absolute accuracy",
        },
        "floor_evidence": {},
        "transforms": {},
        "quality": {},
        "bootstraps": bootstraps,
    }
    validation_roof = voxel(
        crop_top(aggregate(validation_captures, "lidar_top", args.voxel, np.eye(4))), args.voxel
    )
    for sensor, frame in LOWER_LIDARS.items():
        floor_item = {key: value for key, value in floor[sensor].items() if key != "transform"}
        floor_item = {
            key: (value.tolist() if isinstance(value, np.ndarray) else value)
            for key, value in floor_item.items()
        }
        result["floor_evidence"][frame] = floor_item
        try:
            agreed, members, stability = consensus(matrices[sensor])
            agreed[2, 3] = floor[sensor]["translation_z_m"]
            source = aggregate(validation_captures, sensor, args.voxel, floor[sensor]["transform"])
            initial_validation = validation(source, validation_roof, floor[sensor]["transform"])
            final_validation = validation(source, validation_roof, agreed)
            target_met = (
                stability["translation_p95_m"] <= 0.001 and stability["rotation_p95_deg"] <= 0.1
            )
            fallback_met = (
                stability["translation_p95_m"] <= 0.01 and stability["rotation_p95_deg"] <= 1.0
            )
            result["transforms"][frame] = matrix_item(
                agreed, "bootstrap_consensus_floor_wall_column"
            )
            result["quality"][frame] = {
                "status": "target_met"
                if target_met
                else ("fallback_met" if fallback_met else "continue_refinement"),
                "target_met": target_met,
                "fallback_met": fallback_met,
                "dominant_bootstrap_indices": members,
                "bootstrap_stability": stability,
                "heldout_before": initial_validation,
                "heldout_after": final_validation,
            }
        except RuntimeError as error:
            result["quality"][frame] = {"status": "no_consensus", "error": str(error)}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        yaml.safe_dump(result, allow_unicode=True, sort_keys=False), encoding="utf-8"
    )
    print(f"review-only result: {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
