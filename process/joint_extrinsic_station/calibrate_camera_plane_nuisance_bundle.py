#!/usr/bin/env python3
"""Camera calibration with per-capture board-centre nuisance variables.

Sparse lidar rings determine a board plane much more accurately than its
visible centre.  For the front/left/right boards standing on the floor, the
known 900 mm height fixes the vertical centre while one horizontal coordinate
is optimized per capture.  This preserves every lidar plane measurement
without forcing a partial point patch to masquerade as the physical centre.
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

from calibrate_camera_bundle_projection import BundleObservation
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


def frame_with_floor_center(
    item: RigidObservation, initial: np.ndarray, ground_z: float
) -> tuple[np.ndarray, np.ndarray, np.ndarray, float]:
    camera_rotation, _quarter, _flipped, _error = candidate_rotation(item, initial)
    board_rotation = camera_rotation @ item.pose.transform_camera_from_board[:3, :3]
    normal = normalize(board_rotation[:, 2])
    if normal @ item.lidar_normal < 0:
        # Candidate selection normally makes this unnecessary; retain a
        # proper rotation by flipping board x and z together.
        board_rotation[:, 0] *= -1.0
        board_rotation[:, 2] *= -1.0
        normal = -normal
    # The printed page can be mounted in 0/90/180/270-degree orientations.
    # Therefore neither object x nor object y is guaranteed to be vertical.
    # Recover physical up directly from gravity projected into the lidar plane.
    physical_vertical = np.array([0.0, 0.0, 1.0])
    physical_vertical -= normal * float(physical_vertical @ normal)
    physical_vertical = normalize(physical_vertical)
    horizontal = normalize(np.cross(physical_vertical, normal))
    plane_offset = -float(normal @ item.lidar_center)
    target_z = ground_z + 0.45 * physical_vertical[2]
    center = item.lidar_center.copy()
    center += physical_vertical * ((target_z - center[2]) / max(physical_vertical[2], 1e-6))
    center -= normal * float(normal @ center + plane_offset)
    # Horizontal nuisance starts at the lidar patch midpoint and can move in
    # either direction to recover the unseen part of the 900 mm board.
    return board_rotation, center, horizontal, 1.0


def solve(
    values: list[RigidObservation],
    initial: np.ndarray,
    ground_z: float,
    minimum: int = 5,
    board_shift_sigma: float = 0.035,
    translation_prior_sigma: float = 0.20,
    translation_bound: float = 0.40,
) -> tuple[np.ndarray, list[RigidObservation], dict[str, Any]]:
    # Elevated rear-board captures do not have a known vertical anchor; they
    # require a future dedicated sweep and are intentionally excluded here.
    values = [item for item in values if item.phase != "rear_sweep"]
    if len(values) < minimum:
        raise RuntimeError(f"need >={minimum} grounded board poses, got {len(values)}")
    frames = [frame_with_floor_center(item, initial, ground_z) for item in values]
    initial_values = np.r_[
        Rotation.from_matrix(initial[:3, :3]).as_rotvec(), initial[:3, 3], np.zeros(len(values))
    ]

    def optimize(selected_indices: list[int], seed: np.ndarray) -> np.ndarray:
        def residual(parameters: np.ndarray) -> np.ndarray:
            top_from_camera = np.eye(4)
            top_from_camera[:3, :3] = Rotation.from_rotvec(parameters[:3]).as_matrix()
            top_from_camera[:3, 3] = parameters[3:6]
            camera_from_top = np.linalg.inv(top_from_camera)
            parts: list[float] = []
            for index in selected_indices:
                item = values[index]
                board_rotation, base_center, horizontal, _sign = frames[index]
                board_center = base_center + horizontal * parameters[6 + index]
                object_top = item.pose.object_centres @ board_rotation.T + board_center
                object_camera = object_top @ camera_from_top[:3, :3].T + camera_from_top[:3, 3]
                projected = object_camera[:, :2] / object_camera[:, 2:3]
                difference = projected - item.pose.normalized_image_centres
                parts.extend((difference / 0.004).ravel())
                # The lidar patch midpoint is noisy along the board, not
                # meaningless. A 3.5 cm prior matches the observed roof-lidar
                # board-edge repeatability while still allowing partial-ring
                # coverage to self-correct.
                parts.append(parameters[6 + index] / board_shift_sigma)
            inactive = set(range(len(values))) - set(selected_indices)
            parts.extend(parameters[6 + index] / 0.50 for index in inactive)
            rotation_delta = (
                Rotation.from_rotvec(parameters[:3])
                * Rotation.from_rotvec(initial_values[:3]).inv()
            )
            parts.extend(rotation_delta.as_rotvec() / math.radians(12.0))
            parts.extend((parameters[3:6] - initial_values[3:6]) / translation_prior_sigma)
            return np.asarray(parts)

        lower = np.r_[
            np.full(3, -np.inf), initial[:3, 3] - translation_bound, np.full(len(values), -0.50)
        ]
        upper = np.r_[
            np.full(3, np.inf), initial[:3, 3] + translation_bound, np.full(len(values), 0.50)
        ]
        solution = least_squares(
            residual,
            np.clip(seed, lower, upper),
            bounds=(lower, upper),
            loss="soft_l1",
            f_scale=1.0,
            max_nfev=2200,
        )
        if not solution.success:
            raise RuntimeError(solution.message)
        return solution.x

    active = list(range(len(values)))
    current = initial_values.copy()
    history = []
    for iteration in range(5):
        current = optimize(active, current)
        matrix = np.eye(4)
        matrix[:3, :3] = Rotation.from_rotvec(current[:3]).as_matrix()
        matrix[:3, 3] = current[3:6]
        camera_from_top = np.linalg.inv(matrix)
        rms_values = []
        for index in active:
            item = values[index]
            rotation, center, horizontal, _sign = frames[index]
            object_top = (
                item.pose.object_centres @ rotation.T + center + horizontal * current[6 + index]
            )
            points = object_top @ camera_from_top[:3, :3].T + camera_from_top[:3, 3]
            projected = points[:, :2] / points[:, 2:3]
            rms_values.append(
                float(np.sqrt(np.mean(np.square(projected - item.pose.normalized_image_centres))))
            )
        array = np.asarray(rms_values)
        median = float(np.median(array))
        mad = float(np.median(np.abs(array - median)))
        cut = min(0.018, max(0.006, median + 4.0 * mad))
        filtered = [
            index
            for index, rms in zip(active, rms_values)
            if rms <= cut and abs(current[6 + index]) <= 0.45
        ]
        history.append(
            {
                "iteration": iteration + 1,
                "input": len(active),
                "output": len(filtered),
                "normalized_reprojection_rms_cut": cut,
            }
        )
        if len(filtered) < minimum:
            raise RuntimeError(f"only {len(filtered)} grounded bundle poses after robust gate")
        if filtered == active:
            break
        active = filtered
    current = optimize(active, current)
    matrix = np.eye(4)
    matrix[:3, :3] = Rotation.from_rotvec(current[:3]).as_matrix()
    matrix[:3, 3] = current[3:6]
    camera_from_top = np.linalg.inv(matrix)
    pixel_like, centre_shifts = [], []
    for index in active:
        item = values[index]
        rotation, center, horizontal, _sign = frames[index]
        object_top = (
            item.pose.object_centres @ rotation.T + center + horizontal * current[6 + index]
        )
        points = object_top @ camera_from_top[:3, :3].T + camera_from_top[:3, 3]
        projected = points[:, :2] / points[:, 2:3]
        pixel_like.extend(np.linalg.norm(projected - item.pose.normalized_image_centres, axis=1))
        centre_shifts.append(abs(float(current[6 + index])))
    return (
        matrix,
        [values[index] for index in active],
        {
            "raw_grounded_pose_count": len(values),
            "accepted_pose_count": len(active),
            "discarded_pose_count": len(values) - len(active),
            "normalized_reprojection_rms": float(np.sqrt(np.mean(np.square(pixel_like)))),
            "normalized_reprojection_p95": float(np.percentile(pixel_like, 95)),
            "board_horizontal_shift_median_m": float(np.median(centre_shifts)),
            "board_horizontal_shift_p95_m": float(np.percentile(centre_shifts, 95)),
            "rejection_history": history,
        },
    )


def bootstrap(
    values: list[RigidObservation],
    initial: np.ndarray,
    ground_z: float,
    board_shift_sigma: float = 0.035,
    translation_prior_sigma: float = 0.20,
    translation_bound: float = 0.40,
) -> tuple[np.ndarray, dict[str, Any]]:
    full, accepted, quality = solve(
        values,
        initial,
        ground_z,
        board_shift_sigma=board_shift_sigma,
        translation_prior_sigma=translation_prior_sigma,
        translation_bound=translation_bound,
    )
    candidates = []
    for fold in range(5):
        training = [item for index, item in enumerate(values) if index % 5 != fold]
        candidate, _accepted, _quality = solve(
            training,
            initial,
            ground_z,
            minimum=min(4, len(training)),
            board_shift_sigma=board_shift_sigma,
            translation_prior_sigma=translation_prior_sigma,
            translation_bound=translation_bound,
        )
        candidates.append(candidate)
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
    errors = [transform_distance(result, item) for item in selected]
    position = float(np.percentile([item[0] for item in errors], 95))
    angle = float(np.percentile([item[1] for item in errors], 95))
    tier = (
        "target_1mm_0.1deg"
        if position <= 0.001 and angle <= 0.1
        else "fallback_1cm_1deg"
        if position <= 0.01 and angle <= 1.0
        else "coarse_only"
    )
    return result, {
        "achieved_tier": tier,
        "dominant_count": len(members),
        "dominant_fraction": len(members) / 5.0,
        "position_p95_m": position,
        "angle_p95_deg": angle,
        "bootstrap_optical_transforms": [matrix_summary(item) for item in candidates],
        "full_fit": quality,
    }


def load_observations(
    args: argparse.Namespace,
    config: dict[str, Any],
    initial_top_optical: dict[str, np.ndarray],
    intrinsics: dict[str, Any],
) -> dict[str, list[RigidObservation]]:
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
    result: dict[str, list[RigidObservation]] = {name: [] for name in CAMERAS}
    for index, (capture_id, (phase, moved)) in enumerate(sorted(annotations.items()), 1):
        directory = args.data_root / capture_id
        manifest = directory / "manifest.json"
        if not manifest.is_file():
            continue
        poses = observations_for_capture(
            directory,
            json.loads(manifest.read_text(encoding="utf-8")),
            config["tag_board"],
            intrinsics,
            initial_top_optical,
        )
        poses = [pose for pose in poses if matches_phase_direction(pose.rough_center_top, moved)]
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
            result[camera].append(
                RigidObservation(
                    capture_id,
                    phase,
                    pose,
                    center,
                    normal,
                    sum(x["points"] for x in selected),
                    float(np.median([x["rms_m"] for x in selected])),
                    len(selected),
                )
            )
        if index % 50 == 0:
            print(f"processed {index}/{len(annotations)} captures", flush=True)
    return result


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
    parser.add_argument("--ground-z", type=float, required=True)
    parser.add_argument("--board-shift-sigma", type=float, default=0.035)
    parser.add_argument("--translation-prior-sigma", type=float, default=0.20)
    parser.add_argument("--translation-bound", type=float, default=0.40)
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
    values = load_observations(args, config, initial_top_optical, intrinsics)
    output: dict[str, Any] = {
        "schema_version": 1,
        "reference_frame": "velodyne_top_base_link",
        "method": "ground_anchored_plane_bundle_with_horizontal_board_nuisance",
        "ground_z_top_m": args.ground_z,
        "solver_parameters": {
            "board_shift_sigma_m": args.board_shift_sigma,
            "translation_prior_sigma_m": args.translation_prior_sigma,
            "translation_bound_m": args.translation_bound,
        },
        "excluded": {
            "rear_board": "elevated board has no surveyed vertical anchor",
            "traffic_frames_without_decodable_board": "skipped; content audit shows the recorded stream itself is not frozen",
        },
        "transforms": {},
        "quality": {},
        "rejected": {},
    }
    for camera in CAMERAS:
        unique = unique_poses(values[camera])
        frame = f"{camera}/camera_link"
        try:
            optical, quality = bootstrap(
                unique,
                initial_top_optical[camera],
                args.ground_z,
                board_shift_sigma=args.board_shift_sigma,
                translation_prior_sigma=args.translation_prior_sigma,
                translation_bound=args.translation_bound,
            )
            output["transforms"][frame] = {
                "parent": "velodyne_top_base_link",
                **matrix_summary(optical @ np.linalg.inv(CAMERA_OPTICAL_TO_LINK)),
                "source": "grounded_lidar_plane_aprilgrid_bundle",
            }
            output["quality"][frame] = {
                "raw_observations": len(values[camera]),
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
                "raw_observations": len(values[camera]),
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
