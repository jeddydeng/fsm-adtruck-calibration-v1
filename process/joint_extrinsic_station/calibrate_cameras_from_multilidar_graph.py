#!/usr/bin/env python3
"""Calibrate camera optical frames against the refined four-lidar graph.

Every image pose is obtained from the complete AprilGrid corner set.  The
corresponding physical plane is independently recovered by one or more lidars.
When several lidars see the same board, their planes are first checked against
one another and fused; this prevents one sparse BPearl return from receiving
more weight merely because the capture has more sensors online.
"""

from __future__ import annotations

import argparse
from collections import defaultdict
import csv
import json
import math
from pathlib import Path
from typing import Any

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
import yaml

from calibrate_camera_dynamic_boards import angle_deg, matches_phase_direction
from calibrate_multilidar_dynamic_boards import load_transforms, matrix_item, to_local
from extract_camera_pair_constraints import (
    CAMERAS,
    BoardPose,
    matrix_summary,
    observations_for_capture,
    transform_distance,
)
from refine_camera_extrinsics import BoardObservation, normalize, unique_board_poses
from refine_lidar_environment import apply
from solve_joint_extrinsics import (
    CAMERA_OPTICAL_TO_LINK,
    absolute_transforms,
    load_intrinsics,
)


def transform_plane(
    center_local: np.ndarray, normal_local: np.ndarray, transform: np.ndarray
) -> tuple[np.ndarray, np.ndarray]:
    center = apply(transform, center_local.reshape(1, 3))[0]
    normal = normalize(transform[:3, :3] @ normal_local)
    return center, normal


def fuse_planes(
    values: list[dict[str, Any]], camera_normal: np.ndarray
) -> tuple[np.ndarray, np.ndarray, list[dict[str, Any]]] | None:
    if not values:
        return None
    for value in values:
        if value["normal"] @ camera_normal < 0:
            value["normal"] = -value["normal"]
    neighbourhoods: list[list[int]] = []
    for left in values:
        left_offset = -float(left["normal"] @ left["center"])
        members = []
        for index, right in enumerate(values):
            right_offset = -float(right["normal"] @ right["center"])
            if (
                abs(left_offset - right_offset) <= 0.05
                and angle_deg(left["normal"], right["normal"]) <= 2.5
            ):
                members.append(index)
        neighbourhoods.append(members)
    members = max(neighbourhoods, key=len)
    selected = [values[index] for index in members]
    # With one lidar this is still a useful absolute camera constraint; the
    # audit explicitly distinguishes it from cross-lidar corroboration.
    normal = normalize(np.mean([item["normal"] for item in selected], axis=0))
    offsets = [-float(item["normal"] @ item["center"]) for item in selected]
    offset = float(np.median(offsets))
    weights = np.asarray(
        [item["points"] / max(item["rms_m"], 0.004) for item in selected], dtype=float
    )
    centers = np.asarray([item["center"] for item in selected])
    center = np.average(centers, axis=0, weights=weights)
    center -= normal * float(normal @ center + offset)
    return center, normal, selected


def plane_errors(
    items: list[BoardObservation], matrix: np.ndarray
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    distances, angles, centers = [], [], []
    for item in items:
        normal = normalize(matrix[:3, :3] @ item.camera_normal)
        camera_normal = item.camera_normal.copy()
        if normal @ item.lidar_normal < 0:
            camera_normal = -camera_normal
            normal = -normal
        camera_offset = -float(camera_normal @ item.camera_center)
        predicted_offset = camera_offset - float(normal @ matrix[:3, 3])
        lidar_offset = -float(item.lidar_normal @ item.lidar_center)
        distances.append(abs(predicted_offset - lidar_offset))
        angles.append(angle_deg(normal, item.lidar_normal))
        predicted_center = matrix[:3, :3] @ item.camera_center + matrix[:3, 3]
        centers.append(float(np.linalg.norm(predicted_center - item.lidar_center)))
    return np.asarray(distances), np.asarray(angles), np.asarray(centers)


def solve_camera(
    items: list[BoardObservation], initial: np.ndarray, minimum: int = 5
) -> tuple[np.ndarray, list[BoardObservation], dict[str, Any]]:
    if len(items) < minimum:
        raise RuntimeError(f"need >={minimum} unique board poses, got {len(items)}")
    initial_values = np.r_[Rotation.from_matrix(initial[:3, :3]).as_rotvec(), initial[:3, 3]]

    def optimize(selected: list[BoardObservation], seed: np.ndarray) -> np.ndarray:
        signs = [
            1.0
            if Rotation.from_rotvec(seed[:3]).apply(item.camera_normal) @ item.lidar_normal >= 0
            else -1.0
            for item in selected
        ]

        def residual(values: np.ndarray) -> np.ndarray:
            rotation = Rotation.from_rotvec(values[:3])
            parts: list[float] = []
            for item, sign in zip(selected, signs):
                camera_normal = item.camera_normal * sign
                normal = normalize(rotation.apply(camera_normal))
                lidar_normal = item.lidar_normal
                if normal @ lidar_normal < 0:
                    lidar_normal = -lidar_normal
                camera_offset = -float(camera_normal @ item.camera_center)
                predicted_offset = camera_offset - float(normal @ values[3:])
                lidar_offset = -float(lidar_normal @ item.lidar_center)
                plane_sigma = max(0.012, min(0.035, 2.0 * item.lidar_rms_m))
                parts.extend((normal - lidar_normal) / 0.035)
                parts.append((predicted_offset - lidar_offset) / plane_sigma)
                predicted_center = rotation.apply(item.camera_center) + values[3:]
                # The board centre reconstructed from sparse lidar rings is
                # less accurate than its plane, but its two tangent
                # components are essential to make all translations
                # observable. Robust loss and the explicit 10 cm gate keep
                # partial-board centre bias from steering the solution.
                tangent_error = predicted_center - item.lidar_center
                tangent_error -= normal * float(normal @ tangent_error)
                parts.extend(tangent_error / 0.070)
            # Measurements supplied by the operator are deliberately weak;
            # they constrain directions tangent to an under-observed plane.
            rotation_delta = (
                Rotation.from_rotvec(values[:3]) * Rotation.from_rotvec(initial_values[:3]).inv()
            )
            parts.extend(rotation_delta.as_rotvec() / math.radians(12.0))
            parts.extend((values[3:] - initial_values[3:]) / 0.12)
            return np.asarray(parts)

        lower = np.r_[np.full(3, -np.inf), initial_values[3:] - 0.25]
        upper = np.r_[np.full(3, np.inf), initial_values[3:] + 0.25]
        seed = np.clip(seed, lower + 1e-10, upper - 1e-10)
        solution = least_squares(
            residual, seed, bounds=(lower, upper), loss="soft_l1", f_scale=1.0, max_nfev=1800
        )
        if not solution.success:
            raise RuntimeError(solution.message)
        return solution.x

    current_items = list(items)
    current = initial_values.copy()
    history = []
    for iteration in range(5):
        current = optimize(current_items, current)
        matrix = np.eye(4)
        matrix[:3, :3] = Rotation.from_rotvec(current[:3]).as_matrix()
        matrix[:3, 3] = current[3:]
        distances, angles, centers = plane_errors(current_items, matrix)
        distance_median = float(np.median(distances))
        distance_mad = float(np.median(np.abs(distances - distance_median)))
        angle_median = float(np.median(angles))
        angle_mad = float(np.median(np.abs(angles - angle_median)))
        distance_cut = min(0.10, max(0.025, distance_median + 4.0 * distance_mad))
        angle_cut = min(6.0, max(2.5, angle_median + 4.0 * angle_mad))
        # The requested 10 cm disagreement ceiling is enforced on both the
        # plane and the reconstructed physical board centre.
        filtered = [
            item
            for item, distance, angle, center in zip(current_items, distances, angles, centers)
            if distance <= distance_cut and angle <= angle_cut and center <= 0.10
        ]
        history.append(
            {
                "iteration": iteration + 1,
                "input": len(current_items),
                "output": len(filtered),
                "plane_cut_m": distance_cut,
                "normal_cut_deg": angle_cut,
            }
        )
        if len(filtered) < minimum:
            raise RuntimeError(f"only {len(filtered)} observations remain after robust gate")
        if len(filtered) == len(current_items):
            break
        current_items = filtered
    current = optimize(current_items, current)
    matrix = np.eye(4)
    matrix[:3, :3] = Rotation.from_rotvec(current[:3]).as_matrix()
    matrix[:3, 3] = current[3:]
    distances, angles, centers = plane_errors(current_items, matrix)
    normal_matrix = np.asarray([normalize(item.lidar_normal) for item in current_items])
    singular_values = np.linalg.svd(normal_matrix, compute_uv=False)
    return (
        matrix,
        current_items,
        {
            "raw_count": len(items),
            "accepted_count": len(current_items),
            "discarded_count": len(items) - len(current_items),
            "rejection_history": history,
            "plane_median_m": float(np.median(distances)),
            "plane_p95_m": float(np.percentile(distances, 95)),
            "normal_median_deg": float(np.median(angles)),
            "normal_p95_deg": float(np.percentile(angles, 95)),
            "center_median_m": float(np.median(centers)),
            "center_p95_m": float(np.percentile(centers, 95)),
            "plane_normal_singular_values": singular_values.tolist(),
            "translation_observable": bool(singular_values[-1] >= 0.12),
        },
    )


def bootstrap_camera(
    items: list[BoardObservation], initial: np.ndarray
) -> tuple[np.ndarray, dict[str, Any]]:
    full, accepted, full_quality = solve_camera(items, initial)
    # Seven rear-board poses still support five folds with 5--6 training
    # observations each.  Retain an unvalidated result only below six, where
    # a held-out fold cannot meet the five-pose geometric minimum.
    if len(items) < 6:
        return full, {
            "status": "provisional_low_support",
            "achieved_tier": "unvalidated",
            "full_fit": full_quality,
            "bootstrap_count": 0,
        }
    candidates = []
    fold_audits = []
    for fold in range(5):
        training = [item for index, item in enumerate(items) if index % 5 != fold]
        candidate, _kept, quality = solve_camera(training, initial, minimum=min(5, len(training)))
        candidates.append(candidate)
        fold_audits.append(quality)
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
        raise RuntimeError(f"bootstrap dominant cluster {len(members)}/5")
    selected = [candidates[index] for index in members]
    consensus = np.eye(4)
    consensus[:3, 3] = np.median([item[:3, 3] for item in selected], axis=0)
    consensus[:3, :3] = Rotation.from_matrix([item[:3, :3] for item in selected]).mean().as_matrix()
    residuals = [transform_distance(consensus, item) for item in selected]
    position_p95 = float(np.percentile([item[0] for item in residuals], 95))
    angle_p95 = float(np.percentile([item[1] for item in residuals], 95))
    if position_p95 <= 0.001 and angle_p95 <= 0.1:
        tier = "target_1mm_0.1deg"
    elif position_p95 <= 0.01 and angle_p95 <= 1.0:
        tier = "fallback_1cm_1deg"
    else:
        tier = "coarse_only"
    return consensus, {
        "status": "bootstrap_consensus",
        "achieved_tier": tier,
        "dominant_count": len(members),
        "dominant_fraction": len(members) / 5.0,
        "position_p95_m": position_p95,
        "angle_p95_deg": angle_p95,
        "full_fit": full_quality,
        "fold_audits": fold_audits,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--data-root", type=Path, required=True)
    parser.add_argument("--annotations", type=Path, required=True)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--initial", type=Path, required=True)
    parser.add_argument("--intrinsics", type=Path, required=True)
    parser.add_argument("--lidar-observations", type=Path, required=True)
    parser.add_argument("--lidar-seed", type=Path, required=True)
    parser.add_argument("--lidar-refined", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--audit-csv", type=Path, required=True)
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
    lidar_seed = load_transforms(args.lidar_seed)
    lidar_refined = load_transforms(args.lidar_refined)

    phase_by_capture: dict[str, tuple[str, str]] = {}
    with args.annotations.open(newline="", encoding="utf-8") as stream:
        for row in csv.DictReader(stream):
            if row.get("include_nonduplicate", "").lower() == "true":
                phase_by_capture[row["capture_id"]] = (row["phase"], row["moved_board"])

    lidar_rows: dict[tuple[str, str], list[dict[str, Any]]] = defaultdict(list)
    with args.lidar_observations.open(newline="", encoding="utf-8") as stream:
        for row in csv.DictReader(stream):
            sensor = row["sensor"]
            seed_center = np.fromstring(row["center_top_seed"], sep=" ")
            seed_normal = np.fromstring(row["normal_top_seed"], sep=" ")
            local_center, local_normal = to_local(lidar_seed[sensor], seed_center, seed_normal)
            center, normal = transform_plane(local_center, local_normal, lidar_refined[sensor])
            lidar_rows[(row["capture_id"], row["camera"])].append(
                {
                    "sensor": sensor,
                    "center": center,
                    "normal": normal,
                    "points": int(row["points"]),
                    "rms_m": float(row["rms_m"]),
                }
            )

    observations: dict[str, list[BoardObservation]] = {name: [] for name in CAMERAS}
    audit: list[dict[str, Any]] = []
    for index, capture_id in enumerate(sorted(phase_by_capture), start=1):
        directory = args.data_root / capture_id
        manifest_path = directory / "manifest.json"
        if not manifest_path.is_file():
            continue
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        phase, moved_board = phase_by_capture[capture_id]
        camera_poses = observations_for_capture(
            directory, manifest, config["tag_board"], intrinsics, initial_top_optical
        )
        camera_poses = [
            item
            for item in camera_poses
            if matches_phase_direction(item.rough_center_top, moved_board)
        ]
        by_camera: dict[str, list[BoardPose]] = defaultdict(list)
        for pose in camera_poses:
            by_camera[pose.camera].append(pose)
        for camera, poses in by_camera.items():
            raw_planes = lidar_rows.get((capture_id, camera), [])
            if not raw_planes:
                continue
            # Select the camera instance most compatible with the actual lidar
            # plane in the rare case two identical-ID boards occur in one view.
            choices = []
            for pose in poses:
                fused = fuse_planes([dict(item) for item in raw_planes], pose.rough_normal_top)
                if fused is None:
                    continue
                center, normal, selected = fused
                offset_error = abs(float(normal @ pose.rough_center_top + (-normal @ center)))
                score = offset_error + 0.02 * angle_deg(pose.rough_normal_top, normal)
                choices.append((score, pose, center, normal, selected))
            if not choices:
                continue
            _score, pose, center, normal, selected = min(choices, key=lambda x: x[0])
            observations[camera].append(
                BoardObservation(
                    capture_id,
                    pose.transform_camera_from_board[:3, 3].copy(),
                    normalize(pose.transform_camera_from_board[:3, 2].copy()),
                    center,
                    normal,
                    sum(item["points"] for item in selected),
                    float(np.median([item["rms_m"] for item in selected])),
                    np.zeros(2),
                )
            )
            audit.append(
                {
                    "capture_id": capture_id,
                    "phase": phase,
                    "moved_board": moved_board,
                    "camera": camera,
                    "lidars": " ".join(sorted(item["sensor"] for item in selected)),
                    "cross_lidar_count": len(selected),
                    "camera_reprojection_rms_px": pose.reprojection_rms_px,
                    "camera_board_range_m": float(
                        np.linalg.norm(pose.transform_camera_from_board[:3, 3])
                    ),
                    "camera_center_optical": " ".join(
                        f"{value:.8f}" for value in pose.transform_camera_from_board[:3, 3]
                    ),
                    "camera_rough_center_top": " ".join(
                        f"{value:.8f}" for value in pose.rough_center_top
                    ),
                    "rough_to_lidar_center_m": float(
                        np.linalg.norm(pose.rough_center_top - center)
                    ),
                    "rough_to_lidar_plane_m": abs(float(normal @ (pose.rough_center_top - center))),
                    "rough_to_lidar_normal_deg": angle_deg(pose.rough_normal_top, normal),
                    "lidar_plane_rms_m": float(np.median([item["rms_m"] for item in selected])),
                    "lidar_center_top": " ".join(f"{value:.8f}" for value in center),
                    "lidar_normal_top": " ".join(f"{value:.8f}" for value in normal),
                }
            )
        if index % 50 == 0:
            print(f"processed {index}/{len(phase_by_capture)} captures", flush=True)

    output: dict[str, Any] = {
        "schema_version": 1,
        "reference_frame": "velodyne_top_base_link",
        "method": "full_aprilgrid_pose_to_cross_validated_multilidar_plane_graph",
        "corner_source": "fresh_redetection_from_each_saved_image",
        "transforms": {},
        "quality": {},
        "rejected": {},
    }
    for camera in CAMERAS:
        unique = unique_board_poses(observations[camera])
        frame = f"{camera}/camera_link"
        try:
            optical, quality = bootstrap_camera(unique, initial_top_optical[camera])
            link = optical @ np.linalg.inv(CAMERA_OPTICAL_TO_LINK)
            output["transforms"][frame] = {
                "parent": "velodyne_top_base_link",
                **matrix_summary(link),
                "source": "multilidar_dynamic_board_graph",
            }
            output["quality"][frame] = {
                "raw_capture_observations": len(observations[camera]),
                "unique_board_poses": len(unique),
                "cross_lidar_corroborated": sum(
                    int(row["cross_lidar_count"]) >= 2 for row in audit if row["camera"] == camera
                ),
                "difference_from_rough_position_m": transform_distance(
                    optical, initial_top_optical[camera]
                )[0],
                "difference_from_rough_angle_deg": transform_distance(
                    optical, initial_top_optical[camera]
                )[1],
                "stability": quality,
            }
            print(f"{camera}: {len(unique)} poses, {quality['achieved_tier']}", flush=True)
        except RuntimeError as error:
            output["rejected"][frame] = {
                "raw_capture_observations": len(observations[camera]),
                "unique_board_poses": len(unique),
                "error": str(error),
            }
            print(f"{camera}: rejected ({error})", flush=True)

    args.output.write_text(
        yaml.safe_dump(output, allow_unicode=True, sort_keys=False), encoding="utf-8"
    )
    fields = list(audit[0]) if audit else ["capture_id"]
    with args.audit_csv.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(audit)
    print(args.output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
