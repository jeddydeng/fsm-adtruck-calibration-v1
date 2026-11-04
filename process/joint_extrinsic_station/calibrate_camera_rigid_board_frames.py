#!/usr/bin/env python3
"""Recover camera extrinsics from complete rigid AprilGrid board frames.

Unlike a plane-only solve, this uses gravity to recover the physical board's
vertical axis and therefore constrains all three camera rotation axes.  A
single per-camera PnP range scale is estimated as a nuisance parameter; it is
reported (never hidden) and guarded to 0.85--1.15 so an intrinsic/metric scale
problem cannot be mistaken for camera translation.
"""

from __future__ import annotations

import argparse
from collections import Counter, defaultdict
import csv
from dataclasses import dataclass
import json
import math
from pathlib import Path
from typing import Any

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
import yaml

from calibrate_camera_dynamic_boards import angle_deg, matches_phase_direction
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
class RigidObservation:
    capture_id: str
    phase: str
    pose: BoardPose
    lidar_center: np.ndarray
    lidar_normal: np.ndarray
    lidar_points: int
    lidar_rms_m: float
    lidar_count: int


def candidate_rotation(
    item: RigidObservation, initial: np.ndarray
) -> tuple[np.ndarray, int, bool, float]:
    normal = normalize(item.lidar_normal)
    best: tuple[float, np.ndarray, int, bool] | None = None
    for flipped in (False, True):
        candidate_normal = -normal if flipped else normal
        vertical = np.array([0.0, 0.0, 1.0])
        vertical -= candidate_normal * float(vertical @ candidate_normal)
        if np.linalg.norm(vertical) < 0.2:
            continue
        vertical = normalize(vertical)
        horizontal = normalize(np.cross(vertical, candidate_normal))
        base = np.column_stack((horizontal, vertical, candidate_normal))
        for quarter in range(4):
            board_rotation = (
                base @ Rotation.from_euler("z", 90.0 * quarter, degrees=True).as_matrix()
            )
            camera_rotation = board_rotation @ item.pose.transform_camera_from_board[:3, :3].T
            error = math.degrees(
                Rotation.from_matrix(camera_rotation @ initial[:3, :3].T).magnitude()
            )
            if best is None or error < best[0]:
                best = error, camera_rotation, quarter, flipped
    if best is None:
        raise RuntimeError("cannot construct vertical board frame")
    return best[1], best[2], best[3], best[0]


def unique_poses(values: list[RigidObservation]) -> list[RigidObservation]:
    result: list[RigidObservation] = []
    for item in sorted(values, key=lambda value: value.capture_id):
        matching = None
        for index, previous in enumerate(result):
            if (
                np.linalg.norm(item.lidar_center - previous.lidar_center) <= 0.14
                and angle_deg(item.lidar_normal, previous.lidar_normal) <= 4.0
            ):
                matching = index
                break
        if matching is None:
            result.append(item)
        else:
            old = result[matching]
            new_score = (
                item.lidar_count,
                item.lidar_points / max(item.lidar_rms_m, 0.004),
                -item.pose.reprojection_rms_px,
            )
            old_score = (
                old.lidar_count,
                old.lidar_points / max(old.lidar_rms_m, 0.004),
                -old.pose.reprojection_rms_px,
            )
            if new_score > old_score:
                result[matching] = item
    return result


def rotation_consensus(
    values: list[RigidObservation], initial: np.ndarray
) -> tuple[np.ndarray, list[RigidObservation], dict[str, Any]]:
    candidates = [candidate_rotation(item, initial) for item in values]
    neighbourhoods = []
    for left, _q, _f, _d in candidates:
        neighbourhoods.append(
            [
                index
                for index, (right, _q2, _f2, _d2) in enumerate(candidates)
                # Lidar board normals are quantized by sparse scan rings.  Use a
                # broad 10-degree discovery radius, then report/validate the much
                # tighter final p95 and bootstrap spread separately.
                if math.degrees(Rotation.from_matrix(left @ right.T).magnitude()) <= 10.0
            ]
        )
    members = max(neighbourhoods, key=len)
    if len(members) < 3 or len(members) / len(values) < 0.60:
        raise RuntimeError(f"rotation dominant cluster {len(members)}/{len(values)}")
    selected = [values[index] for index in members]
    matrices = [candidates[index][0] for index in members]
    rotation = Rotation.from_matrix(matrices).mean().as_matrix()
    errors = [
        math.degrees(Rotation.from_matrix(matrix @ rotation.T).magnitude()) for matrix in matrices
    ]
    choices = Counter((candidates[index][1], candidates[index][2]) for index in members)
    return (
        rotation,
        selected,
        {
            "raw_count": len(values),
            "dominant_count": len(members),
            "dominant_fraction": len(members) / len(values),
            "rotation_median_deg": float(np.median(errors)),
            "rotation_p95_deg": float(np.percentile(errors, 95)),
            "board_axis_choice_counts": {
                f"quarter_{quarter}_normal_flipped_{str(flipped).lower()}": count
                for (quarter, flipped), count in sorted(choices.items())
            },
            "rough_rotation_difference_median_deg": float(
                np.median([candidates[index][3] for index in members])
            ),
        },
    )


def solve_translation_scale(
    values: list[RigidObservation], rotation: np.ndarray, initial: np.ndarray
) -> tuple[np.ndarray, float, list[RigidObservation], dict[str, Any]]:
    seed = np.r_[initial[:3, 3], 1.0]

    def optimize(items: list[RigidObservation], start: np.ndarray) -> np.ndarray:
        def residual(parameters: np.ndarray) -> np.ndarray:
            translation, scale = parameters[:3], parameters[3]
            parts: list[float] = []
            for item in items:
                predicted = (
                    rotation @ (item.pose.transform_camera_from_board[:3, 3] * scale) + translation
                )
                difference = predicted - item.lidar_center
                sigma = max(0.045, min(0.085, 5.0 * item.lidar_rms_m))
                parts.extend(difference / sigma)
            parts.extend((translation - initial[:3, 3]) / 0.20)
            parts.append((scale - 1.0) / 0.05)
            return np.asarray(parts)

        lower = np.r_[initial[:3, 3] - 0.40, 0.85]
        upper = np.r_[initial[:3, 3] + 0.40, 1.15]
        solution = least_squares(
            residual,
            np.clip(start, lower, upper),
            bounds=(lower, upper),
            loss="soft_l1",
            f_scale=1.0,
            max_nfev=1000,
        )
        if not solution.success:
            raise RuntimeError(solution.message)
        return solution.x

    current = list(values)
    history = []
    for iteration in range(5):
        seed = optimize(current, seed)
        errors = np.asarray(
            [
                np.linalg.norm(
                    rotation @ (item.pose.transform_camera_from_board[:3, 3] * seed[3])
                    + seed[:3]
                    - item.lidar_center
                )
                for item in current
            ]
        )
        median = float(np.median(errors))
        mad = float(np.median(np.abs(errors - median)))
        cut = min(0.10, max(0.035, median + 3.5 * mad))
        filtered = [item for item, error in zip(current, errors) if error <= cut]
        history.append(
            {
                "iteration": iteration + 1,
                "input": len(current),
                "output": len(filtered),
                "center_cut_m": cut,
            }
        )
        if len(filtered) < 3:
            raise RuntimeError(f"only {len(filtered)} rigid observations after 10 cm gate")
        if len(filtered) == len(current):
            break
        current = filtered
    seed = optimize(current, seed)
    errors = [
        float(
            np.linalg.norm(
                rotation @ (item.pose.transform_camera_from_board[:3, 3] * seed[3])
                + seed[:3]
                - item.lidar_center
            )
        )
        for item in current
    ]
    return (
        seed[:3],
        float(seed[3]),
        current,
        {
            "accepted_count": len(current),
            "discarded_count": len(values) - len(current),
            "center_median_m": float(np.median(errors)),
            "center_p95_m": float(np.percentile(errors, 95)),
            "pnp_range_scale": float(seed[3]),
            "scale_deviation_percent": float((seed[3] - 1.0) * 100.0),
            "rejection_history": history,
        },
    )


def fit(
    values: list[RigidObservation], initial: np.ndarray
) -> tuple[np.ndarray, float, list[RigidObservation], dict[str, Any]]:
    rotation, rotation_members, rotation_quality = rotation_consensus(values, initial)
    translation, scale, accepted, position_quality = solve_translation_scale(
        rotation_members, rotation, initial
    )
    matrix = np.eye(4)
    matrix[:3, :3] = rotation
    matrix[:3, 3] = translation
    return (
        matrix,
        scale,
        accepted,
        {
            "rotation": rotation_quality,
            "position_and_scale": position_quality,
        },
    )


def bootstrap(
    values: list[RigidObservation], initial: np.ndarray
) -> tuple[np.ndarray, float, dict[str, Any]]:
    full, full_scale, accepted, quality = fit(values, initial)
    if len(values) < 8:
        return (
            full,
            full_scale,
            {
                "status": "provisional_low_support",
                "achieved_tier": "unvalidated",
                "full_fit": quality,
                "bootstrap_count": 0,
            },
        )
    candidates: list[np.ndarray] = []
    scales = []
    for fold in range(5):
        training = [item for index, item in enumerate(values) if index % 5 != fold]
        candidate, scale, _accepted, _quality = fit(training, initial)
        candidates.append(candidate)
        scales.append(scale)
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
    distances = [transform_distance(result, item) for item in selected]
    position_p95 = float(np.percentile([item[0] for item in distances], 95))
    angle_p95 = float(np.percentile([item[1] for item in distances], 95))
    if position_p95 <= 0.001 and angle_p95 <= 0.1:
        tier = "target_1mm_0.1deg"
    elif position_p95 <= 0.01 and angle_p95 <= 1.0:
        tier = "fallback_1cm_1deg"
    else:
        tier = "coarse_only"
    return (
        result,
        float(np.median([scales[index] for index in members])),
        {
            "status": "bootstrap_consensus",
            "achieved_tier": tier,
            "dominant_count": len(members),
            "dominant_fraction": len(members) / 5.0,
            "position_p95_m": position_p95,
            "angle_p95_deg": angle_p95,
            "pnp_range_scale_p95": float(
                np.percentile(
                    np.abs(np.asarray(scales)[members] - np.median(np.asarray(scales)[members])), 95
                )
            ),
            "full_fit": quality,
        },
    )


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
            center_local, normal_local = to_local(lidar_seed[sensor], center_seed, normal_seed)
            center, normal = transform_plane(center_local, normal_local, lidar_refined[sensor])
            lidar_rows[(row["capture_id"], row["camera"])].append(
                {
                    "sensor": sensor,
                    "center": center,
                    "normal": normal,
                    "points": int(row["points"]),
                    "rms_m": float(row["rms_m"]),
                }
            )

    observations: dict[str, list[RigidObservation]] = {name: [] for name in CAMERAS}
    audit: list[dict[str, Any]] = []
    for index, (capture_id, (phase, moved_board)) in enumerate(
        sorted(annotations.items()), start=1
    ):
        directory = args.data_root / capture_id
        manifest_path = directory / "manifest.json"
        if not manifest_path.is_file():
            continue
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        poses = observations_for_capture(
            directory, manifest, config["tag_board"], intrinsics, initial_top_optical
        )
        poses = [
            pose for pose in poses if matches_phase_direction(pose.rough_center_top, moved_board)
        ]
        by_camera: dict[str, list[BoardPose]] = defaultdict(list)
        for pose in poses:
            by_camera[pose.camera].append(pose)
        for camera, camera_poses in by_camera.items():
            raw_planes = lidar_rows.get((capture_id, camera), [])
            choices = []
            for pose in camera_poses:
                fused = fuse_planes([dict(value) for value in raw_planes], pose.rough_normal_top)
                if fused is None:
                    continue
                center, normal, selected = fused
                plane_error = abs(float(normal @ (pose.rough_center_top - center)))
                score = plane_error + 0.02 * angle_deg(pose.rough_normal_top, normal)
                choices.append((score, pose, center, normal, selected))
            if not choices:
                continue
            _score, pose, center, normal, selected = min(choices, key=lambda item: item[0])
            item = RigidObservation(
                capture_id,
                phase,
                pose,
                center,
                normal,
                sum(value["points"] for value in selected),
                float(np.median([value["rms_m"] for value in selected])),
                len(selected),
            )
            observations[camera].append(item)
            candidate, quarter, flipped, rough_angle = candidate_rotation(
                item, initial_top_optical[camera]
            )
            audit.append(
                {
                    "capture_id": capture_id,
                    "phase": phase,
                    "camera": camera,
                    "lidars": " ".join(sorted(value["sensor"] for value in selected)),
                    "lidar_count": len(selected),
                    "board_axis_quarter": quarter,
                    "board_normal_flipped": flipped,
                    "candidate_rotation_from_rough_deg": rough_angle,
                    "camera_reprojection_rms_px": pose.reprojection_rms_px,
                }
            )
        if index % 50 == 0:
            print(f"processed {index}/{len(annotations)} captures", flush=True)

    output: dict[str, Any] = {
        "schema_version": 1,
        "reference_frame": "velodyne_top_base_link",
        "method": "rigid_aprilgrid_frame_with_gravity_and_multilidar_board_center",
        "excluded": {"traffic_light_front": "frozen during extrinsic session"},
        "transforms": {},
        "quality": {},
        "rejected": {},
    }
    for camera in CAMERAS:
        unique = unique_poses(observations[camera])
        frame = f"{camera}/camera_link"
        try:
            optical, scale, quality = bootstrap(unique, initial_top_optical[camera])
            link = optical @ np.linalg.inv(CAMERA_OPTICAL_TO_LINK)
            output["transforms"][frame] = {
                "parent": "velodyne_top_base_link",
                **matrix_summary(link),
                "source": "rigid_multilidar_board_frame_consensus",
            }
            output["quality"][frame] = {
                "raw_observations": len(observations[camera]),
                "unique_board_poses": len(unique),
                "cross_lidar_corroborated": sum(item.lidar_count >= 2 for item in unique),
                "pnp_range_scale": scale,
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
                "raw_observations": len(observations[camera]),
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
