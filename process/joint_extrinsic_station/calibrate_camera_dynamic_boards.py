#!/usr/bin/env python3
"""Calibrate cameras to the roof lidar using temporally moving boards.

Only one 900 mm board moved during each acquisition phase.  Static garage
walls, columns and the three untouched boards are removed by comparing every
roof-lidar scan with several scans from the same phase.  The image AprilGrid
pose supplies a local search volume and orientation; raw PCD points supply an
independent plane centre/normal.  Repeated reference scans must agree before an
observation can enter the optimizer.
"""

from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path
from typing import Any

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation
import yaml

from calibrate_lidar_heights import read_binary_pcd_xyz
from extract_camera_pair_constraints import (
    BoardPose,
    CAMERAS,
    matrix_summary,
    observations_for_capture,
    transform_distance,
)
from refine_camera_extrinsics import (
    BoardObservation,
    fit_board_near_prediction,
    metrics,
    normalize,
    pcd_for,
    unique_board_poses,
    voxel,
)
from solve_joint_extrinsics import (
    CAMERA_OPTICAL_TO_LINK,
    absolute_transforms,
    load_intrinsics,
)


@dataclass(frozen=True)
class PhaseRow:
    capture_id: str
    phase: str
    moved_board: str


def read_rows(path: Path) -> list[PhaseRow]:
    with path.open(newline="", encoding="utf-8") as stream:
        return [
            PhaseRow(row["capture_id"], row["phase"], row["moved_board"])
            for row in csv.DictReader(stream)
            if row.get("include_nonduplicate", "").lower() == "true"
        ]


def phase_references(rows: list[PhaseRow], root: Path, count: int = 5) -> dict[str, list[Path]]:
    result: dict[str, list[Path]] = {}
    for phase in sorted({row.phase for row in rows}):
        names = [row.capture_id for row in rows if row.phase == phase]
        indices = np.linspace(0, len(names) - 1, min(count, len(names))).round().astype(int)
        result[phase] = [root / names[index] for index in sorted(set(indices))]
    return result


def load_roof(path: Path) -> np.ndarray:
    points = read_binary_pcd_xyz(pcd_for(path, "lidar_top"))
    radius = np.linalg.norm(points[:, :2], axis=1)
    points = points[
        (radius >= 0.7) & (radius <= 14.0) & (points[:, 2] >= -2.0) & (points[:, 2] <= 1.3)
    ]
    return voxel(points, 0.025)


def matches_phase_direction(center: np.ndarray, direction: str) -> bool:
    x, y, _z = center
    if direction == "front":
        return x >= 0.8
    if direction == "left":
        return y >= 0.8
    if direction == "right":
        return y <= -0.8
    return x <= -0.8


def angle_deg(left: np.ndarray, right: np.ndarray) -> float:
    return math.degrees(
        math.acos(float(np.clip(abs(normalize(left) @ normalize(right)), -1.0, 1.0)))
    )


def consensus_fit(
    candidates: list[tuple[np.ndarray, np.ndarray, dict[str, Any], str]],
) -> tuple[np.ndarray, np.ndarray, dict[str, Any]] | None:
    if not candidates:
        return None
    neighbourhoods: list[list[int]] = []
    for center, normal, _quality, _reference in candidates:
        members = []
        for index, (other_center, other_normal, _q, _r) in enumerate(candidates):
            if (
                np.linalg.norm(center - other_center) <= 0.10
                and angle_deg(normal, other_normal) <= 3.0
            ):
                members.append(index)
        neighbourhoods.append(members)
    members = max(neighbourhoods, key=len)
    # Two independent temporal backgrounds are enough; a reference captured
    # at the same board position is expected to yield no dynamic target.
    if len(members) < 2:
        return None
    values = [candidates[index] for index in members]
    centers = np.asarray([item[0] for item in values])
    normal_seed = values[0][1]
    normals = np.asarray([item[1] if item[1] @ normal_seed >= 0 else -item[1] for item in values])
    center = np.median(centers, axis=0)
    normal = normalize(np.mean(normals, axis=0))
    return (
        center,
        normal,
        {
            "reference_count": len(candidates),
            "consensus_count": len(members),
            "consensus_fraction": len(members) / len(candidates),
            "center_max_deviation_m": float(
                max(np.linalg.norm(item[0] - center) for item in values)
            ),
            "normal_max_deviation_deg": float(max(angle_deg(item[1], normal) for item in values)),
            "references": [item[3] for item in values],
            "lidar_points_median": int(np.median([item[2]["points"] for item in values])),
            "lidar_rms_median_m": float(np.median([item[2]["rms_m"] for item in values])),
        },
    )


def strict_fit(
    observations: list[BoardObservation], initial: np.ndarray
) -> tuple[np.ndarray, list[BoardObservation], dict[str, Any]]:
    if len(observations) < 8:
        raise RuntimeError(f"need >=8 unique board poses, got {len(observations)}")
    initial_values = np.r_[Rotation.from_matrix(initial[:3, :3]).as_rotvec(), initial[:3, 3]]

    def solve(items: list[BoardObservation], seed: np.ndarray) -> np.ndarray:
        signs = [
            1.0
            if (Rotation.from_rotvec(seed[:3]).apply(item.camera_normal) @ item.lidar_normal) >= 0
            else -1.0
            for item in items
        ]

        def residual(values: np.ndarray) -> np.ndarray:
            rotation = Rotation.from_rotvec(values[:3])
            parts = []
            for item, sign in zip(items, signs):
                camera_normal = item.camera_normal * sign
                camera_offset = -float(camera_normal @ item.camera_center)
                predicted_normal = rotation.apply(camera_normal)
                lidar_normal = item.lidar_normal
                if predicted_normal @ lidar_normal < 0.0:
                    lidar_normal = -lidar_normal
                lidar_offset = -float(lidar_normal @ item.lidar_center)
                predicted_offset = camera_offset - float(predicted_normal @ values[3:])
                # Plane-to-plane calibration is insensitive to partially
                # visible board edges and avoids the tangential centre bias of
                # sparse lidar rings.
                parts.extend((predicted_normal - lidar_normal) / 0.035)
                parts.append((predicted_offset - lidar_offset) / 0.025)
            # Tape-measure geometry constrains plane-tangent directions that a
            # single board orientation cannot observe.
            parts.extend((values[:3] - initial_values[:3]) / math.radians(15.0))
            parts.extend((values[3:] - initial_values[3:]) / 0.06)
            return np.asarray(parts)

        lower = np.r_[np.full(3, -np.inf), initial_values[3:] - 0.10]
        upper = np.r_[np.full(3, np.inf), initial_values[3:] + 0.10]
        seed = np.minimum(np.maximum(seed, lower + 1e-10), upper - 1e-10)
        fitted = least_squares(
            residual, seed, bounds=(lower, upper), loss="soft_l1", f_scale=1.0, max_nfev=1200
        )
        if not fitted.success:
            raise RuntimeError(fitted.message)
        return fitted.x

    first = solve(observations, initial_values)
    first_matrix = np.eye(4)
    first_matrix[:3, :3] = Rotation.from_rotvec(first[:3]).as_matrix()
    first_matrix[:3, 3] = first[3:]
    plane_errors = []
    normal_errors = []
    for item in observations:
        transformed_normal = first_matrix[:3, :3] @ item.camera_normal
        sign = 1.0 if transformed_normal @ item.lidar_normal >= 0 else -1.0
        camera_normal = item.camera_normal * sign
        transformed_normal = first_matrix[:3, :3] @ camera_normal
        lidar_normal = item.lidar_normal
        if transformed_normal @ lidar_normal < 0:
            lidar_normal = -lidar_normal
        camera_offset = -float(camera_normal @ item.camera_center)
        predicted_offset = camera_offset - float(transformed_normal @ first_matrix[:3, 3])
        lidar_offset = -float(lidar_normal @ item.lidar_center)
        plane_errors.append(abs(predicted_offset - lidar_offset))
        normal_errors.append(angle_deg(transformed_normal, lidar_normal))
    # The operator explicitly allows discarding position disagreements over
    # 10 cm.  This hard ceiling is applied before the final robust solve.
    keep = [
        (distance <= 0.10 and angle <= 6.0) for distance, angle in zip(plane_errors, normal_errors)
    ]
    filtered = [item for item, accepted in zip(observations, keep) if accepted]
    if len(filtered) < 8:
        raise RuntimeError(f"only {len(filtered)} plane observations remain after 10 cm gate")
    final = solve(filtered, first)
    matrix = np.eye(4)
    matrix[:3, :3] = Rotation.from_rotvec(final[:3]).as_matrix()
    matrix[:3, 3] = final[3:]
    return (
        matrix,
        filtered,
        {
            "raw_count": len(observations),
            "accepted_count": len(filtered),
            "discarded_over_10cm_or_6deg": len(observations) - len(filtered),
            "plane_distance_median_m": float(
                np.median([value for value, accepted in zip(plane_errors, keep) if accepted])
            ),
            "plane_distance_p95_m": float(
                np.percentile(
                    [value for value, accepted in zip(plane_errors, keep) if accepted], 95
                )
            ),
            "normal_median_deg": float(
                np.median([value for value, accepted in zip(normal_errors, keep) if accepted])
            ),
        },
    )


def bootstrap_camera(
    values: list[BoardObservation], initial: np.ndarray
) -> tuple[np.ndarray, dict[str, Any]]:
    if len(values) < 12:
        fitted, filtered, audit = strict_fit(values, initial)
        audit.update({"bootstrap_count": 1, "position_p95_m": None, "angle_p95_deg": None})
        return fitted, audit
    candidates = []
    audits = []
    for fold in range(5):
        training = [item for index, item in enumerate(values) if index % 5 != fold]
        candidate, _filtered, audit = strict_fit(training, initial)
        candidates.append(candidate)
        audits.append(audit)
    neighbourhoods = []
    for candidate in candidates:
        neighbourhoods.append(
            [
                index
                for index, other in enumerate(candidates)
                if transform_distance(candidate, other)[0] <= 0.10
                and transform_distance(candidate, other)[1] <= 1.0
            ]
        )
    members = max(neighbourhoods, key=len)
    if len(members) / 5.0 < 0.60:
        raise RuntimeError(f"bootstrap oscillation: dominant cluster {len(members)}/5")
    selected = [candidates[index] for index in members]
    result = np.eye(4)
    result[:3, 3] = np.median([item[:3, 3] for item in selected], axis=0)
    result[:3, :3] = Rotation.from_matrix([item[:3, :3] for item in selected]).mean().as_matrix()
    residuals = [transform_distance(result, item) for item in selected]
    return result, {
        "bootstrap_count": 5,
        "dominant_count": len(members),
        "dominant_fraction": len(members) / 5.0,
        "position_p95_m": float(np.percentile([item[0] for item in residuals], 95)),
        "angle_p95_deg": float(np.percentile([item[1] for item in residuals], 95)),
        "fold_audits": audits,
    }


def plane_metrics(values: list[BoardObservation], transform: np.ndarray) -> dict[str, Any]:
    distances = []
    angles = []
    for item in values:
        normal = transform[:3, :3] @ item.camera_normal
        sign = 1.0 if normal @ item.lidar_normal >= 0 else -1.0
        camera_normal = item.camera_normal * sign
        normal = transform[:3, :3] @ camera_normal
        lidar_normal = item.lidar_normal
        if normal @ lidar_normal < 0:
            lidar_normal = -lidar_normal
        camera_offset = -float(camera_normal @ item.camera_center)
        predicted_offset = camera_offset - float(normal @ transform[:3, 3])
        lidar_offset = -float(lidar_normal @ item.lidar_center)
        distances.append(abs(predicted_offset - lidar_offset))
        angles.append(angle_deg(normal, lidar_normal))
    return (
        {
            "count": len(values),
            "plane_distance_median_m": float(np.median(distances)),
            "plane_distance_p95_m": float(np.percentile(distances, 95)),
            "normal_median_deg": float(np.median(angles)),
            "normal_p95_deg": float(np.percentile(angles, 95)),
        }
        if values
        else {"count": 0}
    )


def main() -> int:
    base = Path(__file__).resolve().parent
    parser = argparse.ArgumentParser()
    parser.add_argument("--data-root", type=Path, required=True)
    parser.add_argument("--annotations", type=Path, required=True)
    parser.add_argument("--config", type=Path, default=base / "config.yaml")
    parser.add_argument("--initial", type=Path, default=base / "initial_extrinsics.yaml")
    parser.add_argument("--intrinsics", type=Path, required=True)
    parser.add_argument("--floor-evidence", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--observations-csv", type=Path, required=True)
    args = parser.parse_args()

    config = yaml.safe_load(args.config.read_text(encoding="utf-8"))
    initial = yaml.safe_load(args.initial.read_text(encoding="utf-8"))
    absolute = absolute_transforms(initial)
    top_from_kit = np.linalg.inv(absolute["velodyne_top_base_link"])
    initial_top_optical = {
        name: top_from_kit @ absolute[f"{name}/camera_link"] @ CAMERA_OPTICAL_TO_LINK
        for name in CAMERAS
    }
    intrinsics = {name: load_intrinsics(args.intrinsics / f"{name}.yaml") for name in CAMERAS}
    floor_rows = []
    with args.floor_evidence.open(newline="", encoding="utf-8") as stream:
        floor_rows = [row for row in csv.DictReader(stream) if row.get("sensor") == "lidar_top"]
    ground_z = float(np.median([float(row["height_or_ground_z_m"]) for row in floor_rows]))
    rows = read_rows(args.annotations)
    references = phase_references(rows, args.data_root)
    roof_cache = {path.name: load_roof(path) for values in references.values() for path in values}
    reference_trees = {name: cKDTree(points) for name, points in roof_cache.items()}

    observations: dict[str, list[BoardObservation]] = {name: [] for name in CAMERAS}
    audit_rows: list[dict[str, Any]] = []
    for capture_index, row in enumerate(rows, start=1):
        directory = args.data_root / row.capture_id
        manifest = json.loads((directory / "manifest.json").read_text(encoding="utf-8"))
        camera_values = observations_for_capture(
            directory, manifest, config["tag_board"], intrinsics, initial_top_optical
        )
        camera_values = [
            item
            for item in camera_values
            if matches_phase_direction(item.rough_center_top, row.moved_board)
        ]
        if not camera_values:
            continue
        current = load_roof(directory)
        dynamic_by_reference: dict[str, np.ndarray] = {}
        for reference in references[row.phase]:
            if reference.name == row.capture_id:
                continue
            distances = reference_trees[reference.name].query(current, k=1, workers=-1)[0]
            dynamic_by_reference[reference.name] = current[distances > 0.075]
        for camera_pose in camera_values:
            candidates = []
            for reference_name, dynamic in dynamic_by_reference.items():
                diagnostics: dict[str, Any] = {}
                digest = hashlib.sha256(
                    f"{row.capture_id}:{camera_pose.camera}:{reference_name}".encode()
                ).digest()
                fitted = fit_board_near_prediction(
                    dynamic,
                    camera_pose.rough_center_top,
                    camera_pose.rough_normal_top,
                    int.from_bytes(digest[:4], "little"),
                    diagnostics,
                    ground_z=ground_z,
                )
                if fitted is None:
                    continue
                center, normal, quality = fitted
                candidates.append((center, normal, quality, reference_name))
            agreed = consensus_fit(candidates)
            if agreed is None:
                audit_rows.append(
                    {
                        "capture_id": row.capture_id,
                        "phase": row.phase,
                        "moved_board": row.moved_board,
                        "camera": camera_pose.camera,
                        "status": "no_temporal_consensus",
                        "candidate_count": len(candidates),
                    }
                )
                continue
            center, normal, quality = agreed
            camera_range = float(np.linalg.norm(camera_pose.transform_camera_from_board[:3, 3]))
            normal_gap = angle_deg(camera_pose.rough_normal_top, normal)
            center_gap = float(np.linalg.norm(camera_pose.rough_center_top - center))
            is_fisheye = camera_pose.camera.startswith("fisheye_")
            if (
                camera_range > (5.0 if is_fisheye else 5.8)
                or normal_gap > (18.0 if is_fisheye else 11.0)
                or center_gap > (0.65 if is_fisheye else 0.55)
            ):
                audit_rows.append(
                    {
                        "capture_id": row.capture_id,
                        "phase": row.phase,
                        "moved_board": row.moved_board,
                        "camera": camera_pose.camera,
                        "status": "rejected_geometric_quality_gate",
                        "camera_range_m": camera_range,
                        "initial_normal_gap_deg": normal_gap,
                        "initial_center_gap_m": center_gap,
                    }
                )
                continue
            observations[camera_pose.camera].append(
                BoardObservation(
                    row.capture_id,
                    camera_pose.transform_camera_from_board[:3, 3],
                    camera_pose.transform_camera_from_board[:3, 2],
                    center,
                    normal,
                    int(quality["lidar_points_median"]),
                    float(quality["lidar_rms_median_m"]),
                    np.zeros(2),
                )
            )
            audit_rows.append(
                {
                    "capture_id": row.capture_id,
                    "phase": row.phase,
                    "moved_board": row.moved_board,
                    "camera": camera_pose.camera,
                    "status": "accepted_dynamic_board",
                    "candidate_count": len(candidates),
                    "consensus_count": quality["consensus_count"],
                    "center_max_deviation_m": quality["center_max_deviation_m"],
                    "normal_max_deviation_deg": quality["normal_max_deviation_deg"],
                    "lidar_points": quality["lidar_points_median"],
                    "lidar_rms_m": quality["lidar_rms_median_m"],
                    "lidar_center_top": " ".join(f"{item:.7f}" for item in center),
                    "lidar_normal_top": " ".join(f"{item:.7f}" for item in normal),
                    "camera_reprojection_rms_px": camera_pose.reprojection_rms_px,
                    "camera_range_m": camera_range,
                    "initial_normal_gap_deg": normal_gap,
                    "initial_center_gap_m": center_gap,
                }
            )
        if capture_index % 25 == 0:
            print(f"processed {capture_index}/{len(rows)} captures", flush=True)

    result: dict[str, Any] = {
        "schema_version": 1,
        "reference_frame": "velodyne_top_base_link",
        "method": "camera_aprilgrid_to_temporal_difference_roof_lidar_board",
        "ground_z_top_m": ground_z,
        "excluded": {"traffic_light_front": "frozen complete session"},
        "transforms": {},
        "quality": {},
        "rejected": {},
    }
    for name in CAMERAS:
        unique = unique_board_poses(observations[name])
        frame = f"{name}/camera_link"
        try:
            fitted_optical, stability = bootstrap_camera(unique, initial_top_optical[name])
            fitted_link = fitted_optical @ np.linalg.inv(CAMERA_OPTICAL_TO_LINK)
            result["transforms"][frame] = {
                "parent": "velodyne_top_base_link",
                **matrix_summary(fitted_link),
                "source": "temporal_dynamic_board_bootstrap",
            }
            result["quality"][frame] = {
                "raw_observations": len(observations[name]),
                "unique_board_poses": len(unique),
                "stability": stability,
                "plane_fit": plane_metrics(unique, fitted_optical),
                "board_center_diagnostic_only": metrics(unique, fitted_optical),
                "difference_from_rough": {
                    "position_m": transform_distance(fitted_optical, initial_top_optical[name])[0],
                    "angle_deg": transform_distance(fitted_optical, initial_top_optical[name])[1],
                },
            }
            print(f"{name}: candidate from {len(unique)} unique dynamic poses", flush=True)
        except RuntimeError as error:
            result["rejected"][frame] = {
                "raw_observations": len(observations[name]),
                "unique_board_poses": len(unique),
                "error": str(error),
            }
            print(f"{name}: no candidate ({error})", flush=True)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        yaml.safe_dump(result, allow_unicode=True, sort_keys=False), encoding="utf-8"
    )
    fields = sorted({key for row in audit_rows for key in row})
    with args.observations_csv.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(audit_rows)
    print(f"review-only result: {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
