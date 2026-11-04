#!/usr/bin/env python3
"""Bundle-calibrate cameras by projecting lidar-anchored AprilGrid centres.

For every accepted capture the lidar supplies a physical board frame (centre,
gravity vertical and plane normal), while the camera supplies sub-pixel tag
centres.  Combining all moved board poses turns the extrinsic problem into one
fixed camera pose estimated from many top-lidar-frame 3-D to normalized-image
2-D correspondences.  Capture-level 10 cm outlier rejection prevents a single
partial lidar board centre from contaminating the bundle.
"""

from __future__ import annotations

import argparse
from collections import defaultdict
import csv
from dataclasses import dataclass
import json
import math
from pathlib import Path
from typing import Any

import cv2
import numpy as np
from scipy.spatial.transform import Rotation
import yaml

from calibrate_camera_dynamic_boards import angle_deg, matches_phase_direction
from calibrate_camera_rigid_board_frames import (
    RigidObservation,
    candidate_rotation,
    unique_poses,
)
from calibrate_cameras_from_multilidar_graph import fuse_planes, transform_plane
from calibrate_multilidar_dynamic_boards import load_transforms, to_local
from extract_camera_pair_constraints import (
    CAMERAS,
    BoardPose,
    matrix_summary,
    observations_for_capture,
    transform_distance,
)
from refine_camera_extrinsics import normalize
from solve_joint_extrinsics import (
    CAMERA_OPTICAL_TO_LINK,
    absolute_transforms,
    load_intrinsics,
)


@dataclass(frozen=True)
class BundleObservation:
    rigid: RigidObservation
    object_top: np.ndarray
    image_normalized: np.ndarray


def build_bundle(item: RigidObservation, initial: np.ndarray) -> BundleObservation:
    camera_rotation, _quarter, _flipped, _rough_error = candidate_rotation(item, initial)
    board_rotation = camera_rotation @ item.pose.transform_camera_from_board[:3, :3]
    object_top = item.pose.object_centres @ board_rotation.T + item.lidar_center
    return BundleObservation(item, object_top, item.pose.normalized_image_centres.copy())


def solve_bundle(
    values: list[BundleObservation], initial: np.ndarray, minimum_captures: int = 4
) -> tuple[np.ndarray, list[BundleObservation], dict[str, Any]]:
    if len(values) < minimum_captures:
        raise RuntimeError(f"need >={minimum_captures} unique poses, got {len(values)}")
    current = list(values)
    camera_from_top = np.linalg.inv(initial)

    def optimize(items: list[BundleObservation], seed: np.ndarray) -> np.ndarray:
        objects = np.concatenate([item.object_top for item in items]).astype(np.float64)
        images = np.concatenate([item.image_normalized for item in items]).astype(np.float64)
        rotation_vector = Rotation.from_matrix(seed[:3, :3]).as_rotvec().reshape(3, 1)
        translation = seed[:3, 3].reshape(3, 1).copy()
        ok, rotation_vector, translation, inliers = cv2.solvePnPRansac(
            objects,
            images,
            np.eye(3),
            None,
            rotation_vector,
            translation,
            useExtrinsicGuess=True,
            iterationsCount=1000,
            reprojectionError=0.012,
            confidence=0.999,
            flags=cv2.SOLVEPNP_ITERATIVE,
        )
        if not ok or inliers is None or len(inliers) < max(12, len(objects) // 3):
            raise RuntimeError("bundle solvePnPRansac has insufficient inliers")
        ok, rotation_vector, translation = cv2.solvePnP(
            objects[inliers.ravel()],
            images[inliers.ravel()],
            np.eye(3),
            None,
            rotation_vector,
            translation,
            True,
            flags=cv2.SOLVEPNP_ITERATIVE,
        )
        if not ok:
            raise RuntimeError("bundle solvePnP refinement failed")
        result = np.eye(4)
        result[:3, :3] = cv2.Rodrigues(rotation_vector)[0]
        result[:3, 3] = translation.ravel()
        return result

    history = []
    for iteration in range(5):
        camera_from_top = optimize(current, camera_from_top)
        top_from_camera = np.linalg.inv(camera_from_top)
        rows = []
        for item in current:
            camera_points = item.object_top @ camera_from_top[:3, :3].T + camera_from_top[:3, 3]
            projected = camera_points[:, :2] / camera_points[:, 2:3]
            reprojection = np.linalg.norm(projected - item.image_normalized, axis=1)
            predicted_center = (
                top_from_camera[:3, :3] @ item.rigid.pose.transform_camera_from_board[:3, 3]
                + top_from_camera[:3, 3]
            )
            center_error = float(np.linalg.norm(predicted_center - item.rigid.lidar_center))
            predicted_normal = normalize(
                top_from_camera[:3, :3] @ item.rigid.pose.transform_camera_from_board[:3, 2]
            )
            normal_error = angle_deg(predicted_normal, item.rigid.lidar_normal)
            rows.append(
                (float(np.sqrt(np.mean(np.square(reprojection)))), center_error, normal_error)
            )
        array = np.asarray(rows)
        reproj_median = float(np.median(array[:, 0]))
        reproj_mad = float(np.median(np.abs(array[:, 0] - reproj_median)))
        reproj_cut = min(0.018, max(0.006, reproj_median + 4.0 * reproj_mad))
        # The operator's explicit disagreement policy is enforced here.
        keep = (array[:, 0] <= reproj_cut) & (array[:, 1] <= 0.10) & (array[:, 2] <= 10.0)
        filtered = [item for item, accepted in zip(current, keep) if accepted]
        history.append(
            {
                "iteration": iteration + 1,
                "input": len(current),
                "output": len(filtered),
                "normalized_reprojection_cut": reproj_cut,
                "center_cut_m": 0.10,
                "normal_cut_deg": 10.0,
            }
        )
        if len(filtered) < minimum_captures:
            raise RuntimeError(f"only {len(filtered)} bundle captures after robust gates")
        if len(filtered) == len(current):
            break
        current = filtered
    camera_from_top = optimize(current, camera_from_top)
    top_from_camera = np.linalg.inv(camera_from_top)
    reprojection_values, center_values, normal_values = [], [], []
    for item in current:
        points = item.object_top @ camera_from_top[:3, :3].T + camera_from_top[:3, 3]
        projected = points[:, :2] / points[:, 2:3]
        reprojection_values.extend(
            np.linalg.norm(projected - item.image_normalized, axis=1).tolist()
        )
        predicted_center = (
            top_from_camera[:3, :3] @ item.rigid.pose.transform_camera_from_board[:3, 3]
            + top_from_camera[:3, 3]
        )
        center_values.append(float(np.linalg.norm(predicted_center - item.rigid.lidar_center)))
        predicted_normal = normalize(
            top_from_camera[:3, :3] @ item.rigid.pose.transform_camera_from_board[:3, 2]
        )
        normal_values.append(angle_deg(predicted_normal, item.rigid.lidar_normal))
    return (
        top_from_camera,
        current,
        {
            "raw_capture_count": len(values),
            "accepted_capture_count": len(current),
            "discarded_capture_count": len(values) - len(current),
            "tag_center_count": len(reprojection_values),
            "normalized_reprojection_rms": float(np.sqrt(np.mean(np.square(reprojection_values)))),
            "normalized_reprojection_p95": float(np.percentile(reprojection_values, 95)),
            "center_median_m": float(np.median(center_values)),
            "center_p95_m": float(np.percentile(center_values, 95)),
            "normal_median_deg": float(np.median(normal_values)),
            "normal_p95_deg": float(np.percentile(normal_values, 95)),
            "rejection_history": history,
        },
    )


def bootstrap(
    values: list[BundleObservation], initial: np.ndarray
) -> tuple[np.ndarray, dict[str, Any]]:
    full, accepted, full_quality = solve_bundle(values, initial)
    if len(values) < 8:
        return full, {
            "status": "provisional_low_support",
            "achieved_tier": "unvalidated",
            "bootstrap_count": 0,
            "full_fit": full_quality,
        }
    candidates = []
    folds = []
    for fold in range(5):
        training = [item for index, item in enumerate(values) if index % 5 != fold]
        candidate, _accepted, quality = solve_bundle(
            training, initial, minimum_captures=min(4, len(training))
        )
        candidates.append(candidate)
        folds.append(quality)
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
    result = np.eye(4)
    result[:3, 3] = np.median([item[:3, 3] for item in selected], axis=0)
    result[:3, :3] = Rotation.from_matrix([item[:3, :3] for item in selected]).mean().as_matrix()
    residuals = [transform_distance(result, item) for item in selected]
    position_p95 = float(np.percentile([item[0] for item in residuals], 95))
    angle_p95 = float(np.percentile([item[1] for item in residuals], 95))
    if position_p95 <= 0.001 and angle_p95 <= 0.1:
        tier = "target_1mm_0.1deg"
    elif position_p95 <= 0.01 and angle_p95 <= 1.0:
        tier = "fallback_1cm_1deg"
    else:
        tier = "coarse_only"
    return result, {
        "status": "bootstrap_consensus",
        "achieved_tier": tier,
        "dominant_count": len(members),
        "dominant_fraction": len(members) / 5.0,
        "position_p95_m": position_p95,
        "angle_p95_deg": angle_p95,
        "full_fit": full_quality,
        "fold_quality": folds,
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
    annotations: dict[str, tuple[str, str]] = {}
    with args.annotations.open(newline="", encoding="utf-8") as stream:
        for row in csv.DictReader(stream):
            if row.get("include_nonduplicate", "").lower() == "true":
                annotations[row["capture_id"]] = (row["phase"], row["moved_board"])
    lidar_rows: dict[tuple[str, str], list[dict[str, Any]]] = defaultdict(list)
    with args.lidar_observations.open(newline="", encoding="utf-8") as stream:
        for row in csv.DictReader(stream):
            sensor = row["sensor"]
            center_seed = np.fromstring(row["center_top_seed"], sep=" ")
            normal_seed = np.fromstring(row["normal_top_seed"], sep=" ")
            local_center, local_normal = to_local(lidar_seed[sensor], center_seed, normal_seed)
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

    rigid: dict[str, list[RigidObservation]] = {name: [] for name in CAMERAS}
    for index, (capture_id, (phase, moved_board)) in enumerate(
        sorted(annotations.items()), start=1
    ):
        directory = args.data_root / capture_id
        manifest_path = directory / "manifest.json"
        if not manifest_path.is_file():
            continue
        poses = observations_for_capture(
            directory,
            json.loads(manifest_path.read_text(encoding="utf-8")),
            config["tag_board"],
            intrinsics,
            initial_top_optical,
        )
        poses = [
            pose for pose in poses if matches_phase_direction(pose.rough_center_top, moved_board)
        ]
        by_camera: dict[str, list[BoardPose]] = defaultdict(list)
        for pose in poses:
            by_camera[pose.camera].append(pose)
        for camera, camera_poses in by_camera.items():
            choices = []
            for pose in camera_poses:
                fused = fuse_planes(
                    [dict(value) for value in lidar_rows.get((capture_id, camera), [])],
                    pose.rough_normal_top,
                )
                if fused is None:
                    continue
                center, normal, selected = fused
                score = abs(float(normal @ (pose.rough_center_top - center))) + 0.02 * angle_deg(
                    pose.rough_normal_top, normal
                )
                choices.append((score, pose, center, normal, selected))
            if not choices:
                continue
            _score, pose, center, normal, selected = min(choices, key=lambda x: x[0])
            rigid[camera].append(
                RigidObservation(
                    capture_id,
                    phase,
                    pose,
                    center,
                    normal,
                    sum(value["points"] for value in selected),
                    float(np.median([value["rms_m"] for value in selected])),
                    len(selected),
                )
            )
        if index % 50 == 0:
            print(f"processed {index}/{len(annotations)} captures", flush=True)

    output: dict[str, Any] = {
        "schema_version": 1,
        "reference_frame": "velodyne_top_base_link",
        "method": "multiview_lidar_board_frame_to_tag_center_projection_bundle",
        "excluded": {"traffic_light_front": "frozen during extrinsic session"},
        "transforms": {},
        "quality": {},
        "rejected": {},
    }
    for camera in CAMERAS:
        unique = unique_poses(rigid[camera])
        bundles = [build_bundle(item, initial_top_optical[camera]) for item in unique]
        frame = f"{camera}/camera_link"
        try:
            optical, quality = bootstrap(bundles, initial_top_optical[camera])
            link = optical @ np.linalg.inv(CAMERA_OPTICAL_TO_LINK)
            output["transforms"][frame] = {
                "parent": "velodyne_top_base_link",
                **matrix_summary(link),
                "source": "multiview_lidar_aprilgrid_projection_bundle",
            }
            output["quality"][frame] = {
                "raw_observations": len(rigid[camera]),
                "unique_board_poses": len(unique),
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
                "raw_observations": len(rigid[camera]),
                "unique_board_poses": len(unique),
                "error": str(error),
            }
            print(f"{camera}: rejected ({error})", flush=True)
    args.output.write_text(
        yaml.safe_dump(output, allow_unicode=True, sort_keys=False), encoding="utf-8"
    )
    print(args.output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
