#!/usr/bin/env python3
"""Extract robust camera-camera constraints from simultaneous AprilGrid views.

The capture manifest already stores the detected tag corners for every frame.
This tool re-solves full 6-DoF board poses from those corners, associates board
instances using the rough vehicle geometry, and forms a relative transform for
every camera pair that observed the same physical board.  No lidar detector
labels are used here, and the known frozen traffic-light stream is excluded.
"""

from __future__ import annotations

import argparse
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

from solve_joint_extrinsics import (
    CAMERA_OPTICAL_TO_LINK,
    absolute_transforms,
    grid_marker_corners,
    load_intrinsics,
    make_detector,
    split_marker_groups,
)


CAMERAS = (
    "video_front_left",
    "video_front_mid",
    "video_front_right",
    "video_rear_left",
    "video_rear_mid",
    "video_rear_right",
    "fisheye_front",
    "fisheye_left",
    "fisheye_right",
    "fisheye_rear",
    "traffic_light_front",
)


@dataclass(frozen=True)
class BoardPose:
    camera: str
    capture_id: str
    transform_camera_from_board: np.ndarray
    rough_center_top: np.ndarray
    rough_normal_top: np.ndarray
    tag_ids: tuple[int, ...]
    object_centres: np.ndarray
    normalized_image_centres: np.ndarray
    reprojection_rms_px: float


@dataclass(frozen=True)
class PairObservation:
    capture_id: str
    left: str
    right: str
    transform_left_from_right: np.ndarray
    rough_center_gap_m: float
    reprojection_rms_px: float
    object_centres: np.ndarray
    left_normalized_centres: np.ndarray
    right_normalized_centres: np.ndarray


_DETECTORS: dict[str, Any] = {}


def detect_saved_frame(
    directory: Path, manifest: dict[str, Any], name: str, board: dict[str, Any]
) -> tuple[list[np.ndarray], np.ndarray] | None:
    """Detect markers from the saved pixels, never cached live-view metadata.

    The traffic-camera capture exposed that a stream can keep saving fresh
    images while its UI detection cache is stale.  Re-detection makes the
    offline calibration evidence self-contained and gives every camera the
    same freshness rule.
    """
    sensor = manifest.get("sensors", {}).get(name, {})
    relative = sensor.get("file")
    if not relative:
        return None
    image = cv2.imread(str(directory / relative), cv2.IMREAD_COLOR)
    if image is None:
        return None
    dictionary = str(board["dictionary"])
    detector = _DETECTORS.setdefault(dictionary, make_detector(dictionary))
    corners, ids = detector(image)
    if ids is None or not len(ids):
        return None
    ids = np.asarray(ids, dtype=np.int32).reshape(-1)
    corners = [np.asarray(value, dtype=np.float32).reshape(1, 4, 2) for value in corners]
    first = int(board.get("first_id", 0))
    last = first + int(board["columns"]) * int(board["rows"])
    valid = [index for index, marker_id in enumerate(ids) if first <= int(marker_id) < last]
    if len(valid) < 3:
        return None
    return [corners[index] for index in valid], ids[valid]


def matrix_summary(matrix: np.ndarray) -> dict[str, list[float]]:
    return {
        "xyz_m": np.round(matrix[:3, 3], 9).tolist(),
        "rpy_deg": np.round(
            Rotation.from_matrix(matrix[:3, :3]).as_euler("xyz", degrees=True), 8
        ).tolist(),
    }


def eligible_ids(path: Path) -> set[str]:
    with path.open(newline="", encoding="utf-8") as stream:
        return {
            row["capture_id"]
            for row in csv.DictReader(stream)
            if row.get("include_nonduplicate", "").lower() == "true"
        }


def solve_group(
    corners: list[np.ndarray],
    ids: np.ndarray,
    group: list[int],
    board: dict[str, Any],
    camera: np.ndarray,
    distortion: np.ndarray,
    model: str,
) -> tuple[np.ndarray, tuple[int, ...], np.ndarray, np.ndarray, float] | None:
    first = int(board.get("first_id", 0))
    last = first + int(board["columns"]) * int(board["rows"])
    valid = [index for index in group if first <= int(ids[index]) < last]
    selected_ids = [int(ids[index]) for index in valid]
    if len(set(selected_ids)) < 3 or len(set(selected_ids)) != len(selected_ids):
        return None
    objects = np.concatenate(
        [grid_marker_corners(int(ids[index]), board) for index in valid]
    ).astype(np.float64)
    image = np.concatenate([np.asarray(corners[index]).reshape(4, 2) for index in valid]).astype(
        np.float64
    )
    if model == "equidistant":
        solve_image = cv2.fisheye.undistortPoints(
            image.reshape(-1, 1, 2), camera, distortion
        ).reshape(-1, 2)
        solve_camera = np.eye(3)
        solve_distortion = None
    else:
        solve_image = cv2.undistortPoints(image.reshape(-1, 1, 2), camera, distortion).reshape(
            -1, 2
        )
        solve_camera = np.eye(3)
        solve_distortion = None
    ok, rotation_vector, translation, inliers = cv2.solvePnPRansac(
        objects,
        solve_image,
        solve_camera,
        solve_distortion,
        iterationsCount=400,
        reprojectionError=0.012,
        confidence=0.999,
        flags=cv2.SOLVEPNP_ITERATIVE,
    )
    if not ok or inliers is None or len(inliers) < max(12, int(0.50 * len(objects))):
        return None
    ok, rotation_vector, translation = cv2.solvePnP(
        objects[inliers.ravel()],
        solve_image[inliers.ravel()],
        solve_camera,
        solve_distortion,
        rotation_vector,
        translation,
        True,
        flags=cv2.SOLVEPNP_ITERATIVE,
    )
    if not ok or float(translation[2, 0]) <= 0.0:
        return None
    if model == "equidistant":
        projected, _ = cv2.fisheye.projectPoints(
            objects.reshape(-1, 1, 3), rotation_vector, translation, camera, distortion
        )
    else:
        projected, _ = cv2.projectPoints(objects, rotation_vector, translation, camera, distortion)
    error = np.linalg.norm(projected.reshape(-1, 2) - image, axis=1)
    rms = float(np.sqrt(np.mean(np.square(error))))
    threshold = 4.5
    if not np.isfinite(rms) or rms > threshold:
        return None
    transform = np.eye(4)
    transform[:3, :3] = cv2.Rodrigues(rotation_vector)[0]
    transform[:3, 3] = translation.ravel()
    order = np.argsort(selected_ids)
    object_centres = np.asarray(
        [grid_marker_corners(int(ids[index]), board).mean(axis=0) for index in valid],
        dtype=np.float64,
    )[order]
    pixel_centres = np.asarray(
        [np.asarray(corners[index]).reshape(4, 2).mean(axis=0) for index in valid], dtype=np.float64
    )[order]
    if model == "equidistant":
        normalized_centres = cv2.fisheye.undistortPoints(
            pixel_centres.reshape(-1, 1, 2), camera, distortion
        ).reshape(-1, 2)
    else:
        normalized_centres = cv2.undistortPoints(
            pixel_centres.reshape(-1, 1, 2), camera, distortion
        ).reshape(-1, 2)
    return (
        transform,
        tuple(np.asarray(selected_ids)[order].tolist()),
        object_centres,
        normalized_centres,
        rms,
    )


def observations_for_capture(
    directory: Path,
    manifest: dict[str, Any],
    board: dict[str, Any],
    intrinsics: dict[str, Any],
    initial_top_optical: dict[str, np.ndarray],
) -> list[BoardPose]:
    observations: list[BoardPose] = []
    for name in CAMERAS:
        detected = detect_saved_frame(directory, manifest, name, board)
        if detected is None:
            continue
        corners, ids = detected
        camera, distortion, model = intrinsics[name]
        groups = split_marker_groups(corners, ids)
        # When one or two intermediate tags are missed, pixel-neighbour
        # connectivity can split a single distant board.  If IDs are unique,
        # they cannot come from two copies of this identical-ID board, so the
        # complete set is a safe recovery candidate.
        if len(set(ids.tolist())) == len(ids) and len(ids) >= 3:
            groups = [list(range(len(ids)))]
        for group in groups:
            solved = solve_group(corners, ids, group, board, camera, distortion, model)
            if solved is None:
                continue
            transform, tag_ids, object_centres, normalized_centres, rms = solved
            top = initial_top_optical[name]
            center_top = (top @ np.r_[transform[:3, 3], 1.0])[:3]
            normal_top = top[:3, :3] @ transform[:3, 2]
            normal_top /= np.linalg.norm(normal_top)
            observations.append(
                BoardPose(
                    name,
                    directory.name,
                    transform,
                    center_top,
                    normal_top,
                    tag_ids,
                    object_centres,
                    normalized_centres,
                    rms,
                )
            )
    return observations


def pair_observations(
    values: list[BoardPose], initial_top_optical: dict[str, np.ndarray]
) -> list[PairObservation]:
    by_camera = {name: [item for item in values if item.camera == name] for name in CAMERAS}
    result: list[PairObservation] = []
    for left_index, left_name in enumerate(CAMERAS):
        for right_name in CAMERAS[left_index + 1 :]:
            candidates = []
            for left in by_camera[left_name]:
                for right in by_camera[right_name]:
                    gap = float(np.linalg.norm(left.rough_center_top - right.rough_center_top))
                    normal_angle = math.degrees(
                        math.acos(
                            float(
                                np.clip(
                                    abs(left.rough_normal_top @ right.rough_normal_top), -1.0, 1.0
                                )
                            )
                        )
                    )
                    if gap <= 1.35 and normal_angle <= 35.0:
                        candidates.append((gap, left, right))
            # Greedy one-to-one matching prevents two copies of the same tag
            # IDs in one image from producing a combinatorial false edge.
            used_left: set[int] = set()
            used_right: set[int] = set()
            for gap, left, right in sorted(candidates, key=lambda item: item[0]):
                if id(left) in used_left or id(right) in used_right:
                    continue
                observed = left.transform_camera_from_board @ np.linalg.inv(
                    right.transform_camera_from_board
                )
                initial = (
                    np.linalg.inv(initial_top_optical[left_name]) @ initial_top_optical[right_name]
                )
                translation_gap, angle_gap = transform_distance(observed, initial)
                if translation_gap > 1.20 or angle_gap > 28.0:
                    continue
                common_ids = sorted(set(left.tag_ids) & set(right.tag_ids))
                if len(common_ids) < 6:
                    continue
                used_left.add(id(left))
                used_right.add(id(right))
                left_index_by_id = {tag_id: index for index, tag_id in enumerate(left.tag_ids)}
                right_index_by_id = {tag_id: index for index, tag_id in enumerate(right.tag_ids)}
                object_centres = np.asarray(
                    [left.object_centres[left_index_by_id[tag_id]] for tag_id in common_ids],
                    dtype=np.float64,
                )
                left_centres = np.asarray(
                    [
                        left.normalized_image_centres[left_index_by_id[tag_id]]
                        for tag_id in common_ids
                    ],
                    dtype=np.float64,
                )
                right_centres = np.asarray(
                    [
                        right.normalized_image_centres[right_index_by_id[tag_id]]
                        for tag_id in common_ids
                    ],
                    dtype=np.float64,
                )
                result.append(
                    PairObservation(
                        left.capture_id,
                        left_name,
                        right_name,
                        observed,
                        gap,
                        max(left.reprojection_rms_px, right.reprojection_rms_px),
                        object_centres,
                        left_centres,
                        right_centres,
                    )
                )
    return result


def transform_distance(left: np.ndarray, right: np.ndarray) -> tuple[float, float]:
    translation = float(np.linalg.norm(left[:3, 3] - right[:3, 3]))
    angle = math.degrees(Rotation.from_matrix(left[:3, :3] @ right[:3, :3].T).magnitude())
    return translation, angle


def robust_pair(values: list[PairObservation]) -> tuple[np.ndarray, list[int], dict[str, Any]]:
    transforms = [item.transform_left_from_right for item in values]
    neighbourhoods: list[list[int]] = []
    for candidate in transforms:
        members = []
        for index, other in enumerate(transforms):
            distance, angle = transform_distance(candidate, other)
            if distance <= 0.10 and angle <= 2.0:
                members.append(index)
        neighbourhoods.append(members)
    members = max(neighbourhoods, key=len)
    dominant_fraction = len(members) / len(values)
    if len(members) < 3 or dominant_fraction < 0.60:
        raise RuntimeError(f"dominant cluster {len(members)}/{len(values)}")
    selected = [transforms[index] for index in members]
    result = np.eye(4)
    result[:3, 3] = np.median([item[:3, 3] for item in selected], axis=0)
    result[:3, :3] = Rotation.from_matrix([item[:3, :3] for item in selected]).mean().as_matrix()
    residuals = [transform_distance(result, item) for item in selected]
    return (
        result,
        members,
        {
            "raw_count": len(values),
            "dominant_count": len(members),
            "dominant_fraction": dominant_fraction,
            "position_median_m": float(np.median([item[0] for item in residuals])),
            "position_p95_m": float(np.percentile([item[0] for item in residuals], 95)),
            "angle_median_deg": float(np.median([item[1] for item in residuals])),
            "angle_p95_deg": float(np.percentile([item[1] for item in residuals], 95)),
            "capture_count": len({values[index].capture_id for index in members}),
        },
    )


def stereo_solve(values: list[PairObservation]) -> tuple[np.ndarray, float]:
    if len(values) < 3:
        raise RuntimeError(f"stereo bundle needs >=3 board views, got {len(values)}")
    objects = [item.object_centres.astype(np.float32) for item in values]
    left_images = [item.left_normalized_centres.astype(np.float32) for item in values]
    right_images = [item.right_normalized_centres.astype(np.float32) for item in values]
    identity = np.eye(3, dtype=np.float64)
    zero = np.zeros((5, 1), dtype=np.float64)
    try:
        result = cv2.stereoCalibrate(
            objects,
            left_images,
            right_images,
            identity.copy(),
            zero.copy(),
            identity.copy(),
            zero.copy(),
            (2, 2),
            criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_COUNT, 300, 1e-12),
            flags=cv2.CALIB_FIX_INTRINSIC,
        )
    except cv2.error as error:
        raise RuntimeError(f"OpenCV stereo bundle failed: {error}") from error
    rms_normalized, _k1, _d1, _k2, _d2, right_from_left_r, right_from_left_t, _e, _f = result
    right_from_left = np.eye(4)
    right_from_left[:3, :3] = right_from_left_r
    right_from_left[:3, 3] = right_from_left_t.ravel()
    return np.linalg.inv(right_from_left), float(rms_normalized)


def bootstrap_stereo(values: list[PairObservation]) -> tuple[np.ndarray, list[int], dict[str, Any]]:
    if len(values) < 5:
        raise RuntimeError(f"stereo bootstrap needs >=5 board views, got {len(values)}")
    candidates: list[np.ndarray] = []
    rms_values: list[float] = []
    # Five leave-one-fold-out solves expose planar or viewpoint-dependent
    # oscillation while retaining enough views for each stereo bundle.
    for fold in range(5):
        selected = [item for index, item in enumerate(values) if index % 5 != fold]
        transform, rms = stereo_solve(selected)
        candidates.append(transform)
        rms_values.append(rms)
    neighbourhoods: list[list[int]] = []
    for candidate in candidates:
        members = []
        for index, other in enumerate(candidates):
            distance, angle = transform_distance(candidate, other)
            if distance <= 0.10 and angle <= 1.0:
                members.append(index)
        neighbourhoods.append(members)
    members = max(neighbourhoods, key=len)
    if len(members) / len(candidates) < 0.60:
        raise RuntimeError(f"stereo dominant bootstrap cluster {len(members)}/5")
    accepted = [candidates[index] for index in members]
    consensus = np.eye(4)
    consensus[:3, 3] = np.median([item[:3, 3] for item in accepted], axis=0)
    consensus[:3, :3] = Rotation.from_matrix([item[:3, :3] for item in accepted]).mean().as_matrix()
    errors = [transform_distance(consensus, item) for item in accepted]
    return (
        consensus,
        members,
        {
            "raw_view_count": len(values),
            "dominant_count": len(members),
            "dominant_fraction": len(members) / 5.0,
            "normalized_reprojection_rms_median": float(np.median(rms_values)),
            "position_p95_m": float(np.percentile([item[0] for item in errors], 95)),
            "angle_p95_deg": float(np.percentile([item[1] for item in errors], 95)),
            "bootstrap_transforms": [matrix_summary(item) for item in candidates],
        },
    )


def main() -> int:
    base = Path(__file__).resolve().parent
    parser = argparse.ArgumentParser()
    parser.add_argument("--data-root", type=Path, required=True)
    parser.add_argument("--annotations", type=Path, required=True)
    parser.add_argument("--config", type=Path, default=base / "config.yaml")
    parser.add_argument("--initial", type=Path, default=base / "initial_extrinsics.yaml")
    parser.add_argument("--intrinsics", type=Path, required=True)
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
    allowed = eligible_ids(args.annotations)
    board_observations: list[BoardPose] = []
    pairs: list[PairObservation] = []
    capture_counts: dict[str, int] = {}
    for index, manifest_path in enumerate(sorted(args.data_root.glob("*/manifest.json"))):
        if manifest_path.parent.name not in allowed:
            continue
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        current = observations_for_capture(
            manifest_path.parent, manifest, config["tag_board"], intrinsics, initial_top_optical
        )
        board_observations.extend(current)
        pairs.extend(pair_observations(current, initial_top_optical))
        capture_counts[manifest_path.parent.name] = len(current)
        if (index + 1) % 50 == 0:
            print(f"processed {index + 1} manifests", flush=True)

    grouped: dict[tuple[str, str], list[PairObservation]] = {}
    for item in pairs:
        grouped.setdefault((item.left, item.right), []).append(item)
    output: dict[str, Any] = {
        "schema_version": 1,
        "reference_note": "relative camera optical-frame constraints; no absolute lidar anchor yet",
        "corner_source": "fresh_redetection_from_each_saved_image",
        "exclusions": {
            "frames_without_decodable_board": "skipped for every camera",
            "traffic_stream": "included after round-22 content audit disproved full-session freeze",
        },
        "board_pose_count": len(board_observations),
        "capture_count": len(capture_counts),
        "per_camera_board_poses": {
            name: sum(item.camera == name for item in board_observations) for name in CAMERAS
        },
        "accepted_edges": {},
        "rejected_edges": {},
    }
    rows: list[dict[str, Any]] = []
    for (left, right), values in sorted(grouped.items()):
        key = f"{left}<-{right}"
        try:
            transform, members, quality = bootstrap_stereo(values)
            initial_relative = np.linalg.inv(initial_top_optical[left]) @ initial_top_optical[right]
            correction_m, correction_deg = transform_distance(transform, initial_relative)
            output["accepted_edges"][key] = {
                "transform": matrix_summary(transform),
                "quality": quality,
                "difference_from_rough_position_m": correction_m,
                "difference_from_rough_angle_deg": correction_deg,
            }
            # Stereo bootstrap members identify folds, not individual views.
            # All matched views are retained in the audit CSV; per-view
            # outlier handling occurs in the later global bundle adjustment.
            member_set = set(range(len(values)))
        except RuntimeError as error:
            output["rejected_edges"][key] = {"raw_count": len(values), "error": str(error)}
            member_set = set()
        for value_index, value in enumerate(values):
            summary = matrix_summary(value.transform_left_from_right)
            rows.append(
                {
                    "capture_id": value.capture_id,
                    "left": left,
                    "right": right,
                    "accepted_dominant_cluster": value_index in member_set,
                    "rough_center_gap_m": value.rough_center_gap_m,
                    "reprojection_rms_px": value.reprojection_rms_px,
                    "xyz_m": " ".join(str(item) for item in summary["xyz_m"]),
                    "rpy_deg": " ".join(str(item) for item in summary["rpy_deg"]),
                }
            )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        yaml.safe_dump(output, allow_unicode=True, sort_keys=False), encoding="utf-8"
    )
    args.observations_csv.parent.mkdir(parents=True, exist_ok=True)
    with args.observations_csv.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]) if rows else ["capture_id"])
        writer.writeheader()
        writer.writerows(rows)
    print(
        f"board poses={len(board_observations)}, raw pair constraints={len(pairs)}, "
        f"accepted pair edges={len(output['accepted_edges'])}"
    )
    print(f"review-only result: {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
