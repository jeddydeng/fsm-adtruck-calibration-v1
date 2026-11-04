#!/usr/bin/env python3
"""Joint four-lidar calibration from temporally changing 900 mm boards.

Each board candidate is independently extracted from every lidar's raw PCD
after subtracting several stationary scans from the same acquisition phase.
The rough camera pose is used only as a search window.  A graph edge is created
only when two lidars independently recover the same board within 15 cm / 8°.
The roof lidar is fixed and the three BPearl poses are jointly optimized.
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

from calibrate_camera_dynamic_boards import (
    PhaseRow,
    angle_deg,
    consensus_fit,
    matches_phase_direction,
    phase_references,
    read_rows,
)
from calibrate_lidar_heights import read_binary_pcd_xyz
from extract_camera_pair_constraints import (
    CAMERAS,
    observations_for_capture,
    transform_distance,
)
from refine_camera_extrinsics import (
    fit_board_near_prediction,
    normalize,
    pcd_for,
    voxel,
)
from refine_lidar_environment import apply, matrix_from_item
from solve_joint_extrinsics import (
    CAMERA_OPTICAL_TO_LINK,
    absolute_transforms,
    load_intrinsics,
)


LIDARS = ("lidar_top", "lidar_front_left", "lidar_front_right", "lidar_rear")
FRAMES = {
    "lidar_front_left": "velodyne_front_left_base_link",
    "lidar_front_right": "velodyne_front_right_base_link",
    "lidar_rear": "velodyne_rear_base_link",
}


@dataclass(frozen=True)
class LocalBoard:
    capture_id: str
    camera: str
    sensor: str
    center_local: np.ndarray
    normal_local: np.ndarray
    center_top_seed: np.ndarray
    normal_top_seed: np.ndarray
    points: int
    rms_m: float
    reference_count: int


@dataclass(frozen=True)
class GraphEdge:
    capture_id: str
    camera: str
    left: LocalBoard
    right: LocalBoard
    phase: str = ""


def matrix_item(matrix: np.ndarray, source: str) -> dict[str, Any]:
    return {
        "parent": "velodyne_top_base_link",
        "xyz_m": np.round(matrix[:3, 3], 9).tolist(),
        "rpy_deg": np.round(
            Rotation.from_matrix(matrix[:3, :3]).as_euler("xyz", degrees=True), 8
        ).tolist(),
        "source": source,
    }


def load_transforms(path: Path) -> dict[str, np.ndarray]:
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    result = {"lidar_top": np.eye(4)}
    for sensor, frame in FRAMES.items():
        item = data.get("transforms", {}).get(frame)
        if item is None:
            raise ValueError(f"missing candidate transform {frame}")
        result[sensor] = matrix_from_item(item)
    return result


def load_cloud(directory: Path, sensor: str, transform: np.ndarray) -> np.ndarray:
    points = apply(transform, read_binary_pcd_xyz(pcd_for(directory, sensor)))
    radius = np.linalg.norm(points[:, :2], axis=1)
    points = points[
        (radius >= 0.6) & (radius <= 14.0) & (points[:, 2] >= -2.0) & (points[:, 2] <= 1.3)
    ]
    return voxel(points, 0.025)


def to_local(
    transform: np.ndarray, center: np.ndarray, normal: np.ndarray
) -> tuple[np.ndarray, np.ndarray]:
    return (
        transform[:3, :3].T @ (center - transform[:3, 3]),
        normalize(transform[:3, :3].T @ normal),
    )


def edge_error(edge: GraphEdge, transforms: dict[str, np.ndarray]) -> tuple[float, float]:
    left_t = transforms[edge.left.sensor]
    right_t = transforms[edge.right.sensor]
    left_center = left_t[:3, :3] @ edge.left.center_local + left_t[:3, 3]
    right_center = right_t[:3, :3] @ edge.right.center_local + right_t[:3, 3]
    left_normal = left_t[:3, :3] @ edge.left.normal_local
    right_normal = right_t[:3, :3] @ edge.right.normal_local
    if left_normal @ right_normal < 0:
        right_normal = -right_normal
    left_offset = -float(left_normal @ left_center)
    right_offset = -float(right_normal @ right_center)
    return (abs(left_offset - right_offset), angle_deg(left_normal, right_normal))


def unpack(values: np.ndarray, sensors: list[str]) -> dict[str, np.ndarray]:
    result = {"lidar_top": np.eye(4)}
    for index, sensor in enumerate(sensors):
        offset = index * 6
        matrix = np.eye(4)
        matrix[:3, :3] = Rotation.from_rotvec(values[offset : offset + 3]).as_matrix()
        matrix[:3, 3] = values[offset + 3 : offset + 6]
        result[sensor] = matrix
    return result


def pack(transforms: dict[str, np.ndarray], sensors: list[str]) -> np.ndarray:
    values = []
    for sensor in sensors:
        values.extend(Rotation.from_matrix(transforms[sensor][:3, :3]).as_rotvec())
        values.extend(transforms[sensor][:3, 3])
    return np.asarray(values)


def solve_graph(
    edges: list[GraphEdge], initial: dict[str, np.ndarray]
) -> tuple[dict[str, np.ndarray], list[GraphEdge], dict[str, Any]]:
    sensors = list(FRAMES)
    seed = pack(initial, sensors)

    def optimize(items: list[GraphEdge], start: np.ndarray) -> np.ndarray:
        # The long front-board sweep contains far more captures than the
        # left/right/rear sweeps. Equalize phase+pair information so repeated
        # front poses cannot drown out an independently oriented side board.
        group_counts: dict[tuple[str, str, str], int] = {}
        for edge in items:
            pair = tuple(sorted((edge.left.sensor, edge.right.sensor)))
            key = (edge.phase or "unknown", pair[0], pair[1])
            group_counts[key] = group_counts.get(key, 0) + 1
        reference_count = float(np.median(list(group_counts.values())))

        def residual(values: np.ndarray) -> np.ndarray:
            transforms = unpack(values, sensors)
            parts: list[float] = []
            for edge in items:
                pair = tuple(sorted((edge.left.sensor, edge.right.sensor)))
                key = (edge.phase or "unknown", pair[0], pair[1])
                group_weight = math.sqrt(reference_count / group_counts[key])
                group_weight = min(3.5, max(0.45, group_weight))
                left_t = transforms[edge.left.sensor]
                right_t = transforms[edge.right.sensor]
                left_center = apply(left_t, edge.left.center_local.reshape(1, 3))[0]
                right_center = apply(right_t, edge.right.center_local.reshape(1, 3))[0]
                left_normal = left_t[:3, :3] @ edge.left.normal_local
                right_normal = right_t[:3, :3] @ edge.right.normal_local
                if left_normal @ right_normal < 0:
                    right_normal = -right_normal
                normal_sigma = 0.025
                parts.extend(group_weight * (left_normal - right_normal) / normal_sigma)
                left_offset = -float(left_normal @ left_center)
                right_offset = -float(right_normal @ right_center)
                plane_sigma = max(0.006, math.hypot(edge.left.rms_m, edge.right.rms_m))
                parts.append(group_weight * (left_offset - right_offset) / plane_sigma)
            transforms = unpack(values, sensors)
            for sensor in sensors:
                rotation_delta = Rotation.from_matrix(
                    transforms[sensor][:3, :3] @ initial[sensor][:3, :3].T
                ).as_rotvec()
                translation_delta = transforms[sensor][:3, 3] - initial[sensor][:3, 3]
                parts.extend(rotation_delta / math.radians(1.5))
                parts.extend(translation_delta[:2] / 0.05)
                # Floor calibration is independently repeatable below 0.7 mm.
                parts.append(translation_delta[2] / 0.003)
            return np.asarray(parts)

        solution = least_squares(residual, start, loss="soft_l1", f_scale=1.0, max_nfev=1800)
        if not solution.success:
            raise RuntimeError(solution.message)
        return solution.x

    final_values = optimize(edges, seed)
    accepted = list(edges)
    rejection_history = []
    for iteration in range(5):
        current = unpack(final_values, sensors)
        errors = np.asarray([edge_error(edge, current) for edge in accepted])
        position_median = float(np.median(errors[:, 0]))
        position_mad = float(np.median(np.abs(errors[:, 0] - position_median)))
        angle_median = float(np.median(errors[:, 1]))
        angle_mad = float(np.median(np.abs(errors[:, 1] - angle_median)))
        position_cut = min(0.10, max(0.015, position_median + 3.0 * position_mad))
        angle_cut = min(5.0, max(0.50, angle_median + 3.0 * angle_mad))
        filtered = [
            edge
            for edge, error in zip(accepted, errors)
            if error[0] <= position_cut and error[1] <= angle_cut
        ]
        rejection_history.append(
            {
                "iteration": iteration + 1,
                "input_edges": len(accepted),
                "output_edges": len(filtered),
                "plane_distance_cut_m": position_cut,
                "angle_cut_deg": angle_cut,
            }
        )
        if len(filtered) < 8:
            raise RuntimeError(f"only {len(filtered)} graph edges remain after robust gate")
        if len(filtered) == len(accepted):
            break
        accepted = filtered
        final_values = optimize(accepted, final_values)
    final = unpack(final_values, sensors)
    final_errors = [edge_error(edge, final) for edge in accepted]
    return (
        final,
        accepted,
        {
            "raw_edges": len(edges),
            "accepted_edges": len(accepted),
            "discarded_edges": len(edges) - len(accepted),
            "rejection_history": rejection_history,
            "plane_distance_median_m": float(np.median([item[0] for item in final_errors])),
            "plane_distance_p95_m": float(np.percentile([item[0] for item in final_errors], 95)),
            "angle_median_deg": float(np.median([item[1] for item in final_errors])),
            "angle_p95_deg": float(np.percentile([item[1] for item in final_errors], 95)),
        },
    )


def bootstrap_graph(
    edges: list[GraphEdge], initial: dict[str, np.ndarray]
) -> tuple[dict[str, np.ndarray], dict[str, Any]]:
    candidates = []
    fold_quality = []
    for fold in range(5):
        training = [edge for index, edge in enumerate(edges) if index % 5 != fold]
        candidate, _accepted, quality = solve_graph(training, initial)
        candidates.append(candidate)
        fold_quality.append(quality)
    result = {"lidar_top": np.eye(4)}
    stability: dict[str, Any] = {}
    for sensor in FRAMES:
        members: list[int] = []
        selection_tier = "none"
        # Apply the operator's >60% oscillation rule at the strictest tier
        # supported by at least three of the five bootstrap estimates.
        for tier, position_limit, angle_limit in (
            ("target_1mm_0.1deg", 0.001, 0.1),
            ("fallback_1cm_1deg", 0.01, 1.0),
            ("coarse_10cm_1deg", 0.10, 1.0),
        ):
            neighbourhoods = []
            for candidate in candidates:
                neighbourhoods.append(
                    [
                        index
                        for index, other in enumerate(candidates)
                        if transform_distance(candidate[sensor], other[sensor])[0] <= position_limit
                        and transform_distance(candidate[sensor], other[sensor])[1] <= angle_limit
                    ]
                )
            candidate_members = max(neighbourhoods, key=len)
            if len(candidate_members) / 5.0 >= 0.60:
                members = candidate_members
                selection_tier = tier
                break
        if len(members) / 5.0 < 0.60:
            raise RuntimeError(f"{sensor}: bootstrap dominant cluster {len(members)}/5")
        selected = [candidates[index][sensor] for index in members]
        matrix = np.eye(4)
        matrix[:3, 3] = np.median([item[:3, 3] for item in selected], axis=0)
        matrix[:3, :3] = (
            Rotation.from_matrix([item[:3, :3] for item in selected]).mean().as_matrix()
        )
        # Preserve the sub-millimetre floor-derived height.
        matrix[2, 3] = initial[sensor][2, 3]
        result[sensor] = matrix
        residuals = [transform_distance(matrix, item) for item in selected]
        position_p95 = float(np.percentile([item[0] for item in residuals], 95))
        angle_p95 = float(np.percentile([item[1] for item in residuals], 95))
        if position_p95 <= 0.001 and angle_p95 <= 0.1:
            achieved_tier = "target_1mm_0.1deg"
        elif position_p95 <= 0.01 and angle_p95 <= 1.0:
            achieved_tier = "fallback_1cm_1deg"
        else:
            achieved_tier = "coarse_only"
        stability[sensor] = {
            # selection_tier is the radius used to find the dominant >60%
            # cluster.  achieved_tier is intentionally derived from the
            # final cluster p95 so a boundary cluster cannot be overstated.
            "selection_tier": selection_tier,
            "achieved_tier": achieved_tier,
            "dominant_count": len(members),
            "dominant_fraction": len(members) / 5.0,
            "position_p95_m": position_p95,
            "angle_p95_deg": angle_p95,
        }
    return result, {"per_sensor": stability, "fold_quality": fold_quality}


def main() -> int:
    base = Path(__file__).resolve().parent
    parser = argparse.ArgumentParser()
    parser.add_argument("--data-root", type=Path, required=True)
    parser.add_argument("--annotations", type=Path, required=True)
    parser.add_argument("--config", type=Path, default=base / "config.yaml")
    parser.add_argument("--initial", type=Path, default=base / "initial_extrinsics.yaml")
    parser.add_argument("--intrinsics", type=Path, required=True)
    parser.add_argument("--lidar-transforms", type=Path, required=True)
    parser.add_argument("--floor-evidence", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--observations-csv", type=Path, required=True)
    args = parser.parse_args()

    config = yaml.safe_load(args.config.read_text(encoding="utf-8"))
    initial_data = yaml.safe_load(args.initial.read_text(encoding="utf-8"))
    absolute = absolute_transforms(initial_data)
    top_from_kit = np.linalg.inv(absolute["velodyne_top_base_link"])
    initial_top_optical = {
        name: top_from_kit @ absolute[f"{name}/camera_link"] @ CAMERA_OPTICAL_TO_LINK
        for name in CAMERAS
    }
    intrinsics = {name: load_intrinsics(args.intrinsics / f"{name}.yaml") for name in CAMERAS}
    transforms = load_transforms(args.lidar_transforms)
    with args.floor_evidence.open(newline="", encoding="utf-8") as stream:
        roof_floor = [
            float(row["height_or_ground_z_m"])
            for row in csv.DictReader(stream)
            if row.get("sensor") == "lidar_top"
        ]
    ground_z = float(np.median(roof_floor))
    rows = read_rows(args.annotations)
    references = phase_references(rows, args.data_root)

    reference_clouds: dict[tuple[str, str], np.ndarray] = {}
    reference_trees: dict[tuple[str, str], cKDTree] = {}
    for phase_paths in references.values():
        for path in phase_paths:
            for sensor in LIDARS:
                key = (path.name, sensor)
                cloud = load_cloud(path, sensor, transforms[sensor])
                reference_clouds[key] = cloud
                reference_trees[key] = cKDTree(cloud)

    all_boards: list[LocalBoard] = []
    edges_by_key: dict[tuple[str, str, str], tuple[float, GraphEdge]] = {}
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
        current_clouds = {
            sensor: load_cloud(directory, sensor, transforms[sensor]) for sensor in LIDARS
        }
        dynamics: dict[tuple[str, str], np.ndarray] = {}
        for sensor in LIDARS:
            for reference in references[row.phase]:
                if reference.name == row.capture_id:
                    continue
                distance = reference_trees[(reference.name, sensor)].query(
                    current_clouds[sensor], k=1, workers=-1
                )[0]
                dynamics[(sensor, reference.name)] = current_clouds[sensor][distance > 0.075]
        for camera_pose in camera_values:
            per_sensor: dict[str, LocalBoard] = {}
            for sensor in LIDARS:
                candidates = []
                for reference in references[row.phase]:
                    if reference.name == row.capture_id:
                        continue
                    dynamic = dynamics[(sensor, reference.name)]
                    digest = hashlib.sha256(
                        f"multi:{row.capture_id}:{camera_pose.camera}:{sensor}:"
                        f"{reference.name}".encode()
                    ).digest()
                    fitted = fit_board_near_prediction(
                        dynamic,
                        camera_pose.rough_center_top,
                        camera_pose.rough_normal_top,
                        int.from_bytes(digest[:4], "little"),
                        {},
                        ground_z=ground_z,
                        min_vertical_extent_m=0.12,
                        max_prediction_error_m=1.10,
                    )
                    if fitted is None:
                        continue
                    candidates.append((*fitted, reference.name))
                # Reformat the helper's (center, normal, quality, ref) input.
                candidate_values = [(item[0], item[1], item[2], item[3]) for item in candidates]
                agreed = consensus_fit(candidate_values)
                if agreed is None:
                    continue
                center, normal, quality = agreed
                center_gap = float(np.linalg.norm(center - camera_pose.rough_center_top))
                normal_gap = angle_deg(normal, camera_pose.rough_normal_top)
                if center_gap > 1.10 or normal_gap > 20.0:
                    continue
                local_center, local_normal = to_local(transforms[sensor], center, normal)
                board = LocalBoard(
                    row.capture_id,
                    camera_pose.camera,
                    sensor,
                    local_center,
                    local_normal,
                    center,
                    normal,
                    int(quality["lidar_points_median"]),
                    float(quality["lidar_rms_median_m"]),
                    int(quality["consensus_count"]),
                )
                per_sensor[sensor] = board
                all_boards.append(board)
                audit_rows.append(
                    {
                        "capture_id": row.capture_id,
                        "phase": row.phase,
                        "camera": camera_pose.camera,
                        "sensor": sensor,
                        "center_top_seed": " ".join(f"{value:.7f}" for value in center),
                        "normal_top_seed": " ".join(f"{value:.7f}" for value in normal),
                        "points": board.points,
                        "rms_m": board.rms_m,
                        "temporal_reference_consensus": board.reference_count,
                        "rough_center_gap_m": center_gap,
                        "rough_normal_gap_deg": normal_gap,
                    }
                )
            names = sorted(per_sensor, key=LIDARS.index)
            for left_index, left_name in enumerate(names):
                for right_name in names[left_index + 1 :]:
                    left = per_sensor[left_name]
                    right = per_sensor[right_name]
                    edge = GraphEdge(row.capture_id, camera_pose.camera, left, right, row.phase)
                    center_error, normal_error = edge_error(edge, transforms)
                    if center_error > 0.15 or normal_error > 8.0:
                        continue
                    score = (left.points + right.points) / max(left.rms_m + right.rms_m, 0.004)
                    key = (row.capture_id, left_name, right_name)
                    if key not in edges_by_key or score > edges_by_key[key][0]:
                        edges_by_key[key] = (score, edge)
        if capture_index % 20 == 0:
            print(
                f"processed {capture_index}/{len(rows)} captures, "
                f"boards={len(all_boards)}, edges={len(edges_by_key)}",
                flush=True,
            )

    edges = [item[1] for item in edges_by_key.values()]
    pair_counts: dict[str, int] = {}
    for edge in edges:
        key = f"{edge.left.sensor}<->{edge.right.sensor}"
        pair_counts[key] = pair_counts.get(key, 0) + 1
    result: dict[str, Any] = {
        "schema_version": 1,
        "reference_frame": "velodyne_top_base_link",
        "method": "four_lidar_temporal_dynamic_board_pose_graph",
        "raw_local_board_count": len(all_boards),
        "graph_edge_count": len(edges),
        "pair_edge_counts": pair_counts,
        "transforms": {},
        "quality": {},
    }
    try:
        fitted, stability = bootstrap_graph(edges, transforms)
        full, accepted, full_quality = solve_graph(edges, transforms)
        # Use bootstrap consensus for x/y/orientation and independent floor Z.
        for sensor, frame in FRAMES.items():
            result["transforms"][frame] = matrix_item(
                fitted[sensor], "dynamic_board_graph_bootstrap_floor_z"
            )
            correction_m, correction_deg = transform_distance(fitted[sensor], transforms[sensor])
            result["quality"][frame] = {
                "bootstrap": stability["per_sensor"][sensor],
                "correction_from_structure_candidate_m": correction_m,
                "correction_from_structure_candidate_deg": correction_deg,
            }
        result["graph_quality"] = full_quality
        result["fold_quality"] = stability["fold_quality"]
        result["accepted_edge_count_full_solve"] = len(accepted)
    except RuntimeError as error:
        result["error"] = str(error)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        yaml.safe_dump(result, allow_unicode=True, sort_keys=False), encoding="utf-8"
    )
    fields = sorted({key for row in audit_rows for key in row})
    with args.observations_csv.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(audit_rows)
    print(
        json.dumps(
            {
                "boards": len(all_boards),
                "edges": len(edges),
                "pair_counts": pair_counts,
                "error": result.get("error"),
            },
            ensure_ascii=False,
        )
    )
    print(f"review-only result: {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
