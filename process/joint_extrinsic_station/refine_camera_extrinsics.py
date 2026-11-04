#!/usr/bin/env python3
"""Refine camera extrinsics against locally fitted roof-lidar board planes.

The general roof-lidar detector can mistake repeated garage wall patches for a
900 mm board.  This offline solver instead starts from each camera's AprilGrid
pose and searches only a small, oriented volume around that predicted board.
Rapid duplicate captures and a known frozen stream can be excluded through the
curation CSV.  Results are review-only and never edit live Autoware files.
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

import cv2
import numpy as np
import yaml
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

from calibrate_lidar_heights import read_binary_pcd_xyz
from solve_joint_extrinsics import (
    CAMERA_OPTICAL_TO_LINK,
    absolute_transforms,
    estimate_grid_poses,
    load_intrinsics,
    make_detector,
    matrix_to_item,
)


PRIORITY_CAMERAS = (
    "video_front_left",
    "video_front_mid",
    "video_front_right",
    "video_rear_left",
    "video_rear_mid",
    "video_rear_right",
)
FISHEYE_CAMERAS = (
    "fisheye_front",
    "fisheye_left",
    "fisheye_right",
    "fisheye_rear",
)


@dataclass
class BoardObservation:
    capture_id: str
    camera_center: np.ndarray
    camera_normal: np.ndarray
    lidar_center: np.ndarray
    lidar_normal: np.ndarray
    lidar_points: int
    lidar_rms_m: float
    lidar_extent_m: np.ndarray


def normalize(vector: np.ndarray) -> np.ndarray:
    return vector / max(float(np.linalg.norm(vector)), 1e-12)


def pcd_for(capture: Path, sensor: str) -> Path:
    paths = list((capture / "lidars").glob(f"*_{sensor}.pcd"))
    if len(paths) != 1:
        raise RuntimeError(f"{capture.name}: expected one {sensor} PCD")
    return paths[0]


def matrix_from_item(item: dict[str, Any]) -> np.ndarray:
    matrix = np.eye(4)
    matrix[:3, :3] = Rotation.from_euler("xyz", item["rpy_deg"], degrees=True).as_matrix()
    matrix[:3, 3] = np.asarray(item["xyz_m"], dtype=float)
    return matrix


def voxel(points: np.ndarray, size: float) -> np.ndarray:
    cells = np.floor(points / size).astype(np.int32)
    _, index = np.unique(cells, axis=0, return_index=True)
    return points[np.sort(index)]


def fused_lidar_cloud(capture: Path, top_from_lidar: dict[str, np.ndarray]) -> np.ndarray:
    clouds = []
    for sensor, transform in top_from_lidar.items():
        points = read_binary_pcd_xyz(pcd_for(capture, sensor))
        points = points @ transform[:3, :3].T + transform[:3, 3]
        radius = np.linalg.norm(points[:, :2], axis=1)
        clouds.append(
            points[
                (radius >= 0.8) & (radius <= 14.0) & (points[:, 2] >= -2.0) & (points[:, 2] <= 1.5)
            ]
        )
    return voxel(np.concatenate(clouds), 0.025)


def included_capture_ids(path: Path | None) -> set[str] | None:
    if path is None:
        return None
    with path.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    return {
        row["capture_id"] for row in rows if row.get("include_nonduplicate", "").lower() == "true"
    }


def load_captures(
    root: Path, annotations: Path | None, limit: int, requested: set[str] | None = None
) -> list[tuple[Path, dict[str, Any]]]:
    allowed = included_capture_ids(annotations)
    captures = []
    for path in sorted(root.glob("*/manifest.json")):
        manifest = json.loads(path.read_text(encoding="utf-8"))
        capture_id = str(manifest.get("capture_id", path.parent.name))
        if (allowed is None or capture_id in allowed) and (
            requested is None or capture_id in requested
        ):
            captures.append((path.parent, manifest))
    if limit and len(captures) > limit:
        indices = np.linspace(0, len(captures) - 1, limit).round().astype(int)
        captures = [captures[index] for index in indices]
    return captures


def board_basis(normal: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    normal = normalize(normal)
    vertical = np.array([0.0, 0.0, 1.0])
    vertical -= normal * float(vertical @ normal)
    if np.linalg.norm(vertical) < 0.25:
        return np.array([1.0, 0.0, 0.0]), np.array([0.0, 1.0, 0.0])
    vertical = normalize(vertical)
    horizontal = normalize(np.cross(vertical, normal))
    return horizontal, vertical


def fit_board_near_prediction(
    points: np.ndarray,
    predicted_center: np.ndarray,
    predicted_normal: np.ndarray,
    seed: int,
    diagnostics: dict[str, Any] | None = None,
    ground_z: float = -1.742,
    min_vertical_extent_m: float = 0.25,
    max_prediction_error_m: float = 0.68,
) -> tuple[np.ndarray, np.ndarray, dict[str, Any]] | None:
    diagnostics = diagnostics if diagnostics is not None else {}
    predicted_normal = normalize(predicted_normal)
    horizontal, vertical = board_basis(predicted_normal)
    delta = points - predicted_center
    coordinates = np.column_stack((delta @ horizontal, delta @ vertical, delta @ predicted_normal))
    mask = (
        (np.abs(coordinates[:, 0]) <= 1.08)
        & (np.abs(coordinates[:, 1]) <= 1.08)
        & (np.abs(coordinates[:, 2]) <= 0.65)
    )
    candidates = points[mask]
    diagnostics["roi_points"] = len(candidates)
    if len(candidates) < 28:
        diagnostics["rejection"] = "roi_points_lt_28"
        return None
    rng = np.random.default_rng(seed)
    sample = candidates
    if len(sample) > 4500:
        sample = sample[rng.choice(len(sample), 4500, replace=False)]
    best: tuple[float, np.ndarray, float] | None = None
    max_tilt = math.cos(math.radians(18.0))
    threshold = 0.032
    orientation_candidates = 0
    shape_candidates = 0
    maximum_inliers = 0
    for _ in range(180):
        triple = sample[rng.choice(len(sample), 3, replace=False)]
        normal = np.cross(triple[1] - triple[0], triple[2] - triple[0])
        if np.linalg.norm(normal) < 1e-8:
            continue
        normal = normalize(normal)
        if normal @ predicted_normal < 0.0:
            normal = -normal
        if normal @ predicted_normal < max_tilt:
            continue
        orientation_candidates += 1
        offset = -float(normal @ triple[0])
        if abs(float(normal @ predicted_center + offset)) > 0.42:
            continue
        inlier_mask = np.abs(sample @ normal + offset) <= threshold
        count = int(np.count_nonzero(inlier_mask))
        maximum_inliers = max(maximum_inliers, count)
        if count < 22:
            continue
        candidate_horizontal, candidate_vertical = board_basis(normal)
        candidate_uv = np.column_stack(
            (
                (sample[inlier_mask] - predicted_center) @ candidate_horizontal,
                (sample[inlier_mask] - predicted_center) @ candidate_vertical,
            )
        )
        candidate_extent = np.diff(np.percentile(candidate_uv, [4.0, 96.0], axis=0), axis=0)[0]
        if np.any(candidate_extent > 1.32) or np.any(candidate_extent < 0.22):
            continue
        shape_candidates += 1
        shape_quality = float(np.prod(np.minimum(1.0, candidate_extent / 0.72)))
        size_penalty = float(
            np.exp(-1.2 * np.sum(np.square(np.maximum(candidate_extent - 0.98, 0.0))))
        )
        score = count * shape_quality * size_penalty
        if best is None or score > best[0]:
            best = score, normal, offset
    diagnostics.update(
        {
            "orientation_candidates": orientation_candidates,
            "shape_candidates": shape_candidates,
            "maximum_inliers": maximum_inliers,
        }
    )
    if best is None:
        diagnostics["rejection"] = "no_oriented_board_sized_plane"
        return None
    _, normal, offset = best
    for _ in range(3):
        residual = np.abs(candidates @ normal + offset)
        inliers = candidates[residual <= threshold]
        if len(inliers) < 22:
            diagnostics["rejection"] = "refined_inliers_lt_22"
            return None
        center = np.mean(inliers, axis=0)
        _, _, vectors_t = np.linalg.svd(inliers - center, full_matrices=False)
        fitted = vectors_t[-1]
        if fitted @ predicted_normal < 0.0:
            fitted = -fitted
        if fitted @ predicted_normal < max_tilt:
            diagnostics["rejection"] = "refined_normal_outside_18deg"
            return None
        normal = fitted
        offset = -float(normal @ center)
    residual = np.abs(candidates @ normal + offset)
    plane_points = candidates[residual <= threshold]
    horizontal, vertical = board_basis(normal)
    plane_origin = predicted_center - normal * float(normal @ predicted_center + offset)
    all_uv = np.column_stack(
        ((plane_points - plane_origin) @ horizontal, (plane_points - plane_origin) @ vertical)
    )
    # A person or tripod holding the elevated rear board can be approximately
    # coplanar and make the raw plane 1.5--2 m tall.  Search for the densest
    # board-sized 2-D window near the camera prediction before measuring edges.
    best_window: tuple[float, np.ndarray] | None = None
    for center_u in np.arange(-0.70, 0.701, 0.10):
        for center_v in np.arange(-0.70, 0.701, 0.10):
            window = (np.abs(all_uv[:, 0] - center_u) <= 0.54) & (
                np.abs(all_uv[:, 1] - center_v) <= 0.54
            )
            if np.count_nonzero(window) < 22:
                continue
            window_uv = all_uv[window]
            low_window, high_window = np.percentile(window_uv, [3.0, 97.0], axis=0)
            window_extent = high_window - low_window
            if window_extent[0] < 0.25 or window_extent[1] < min_vertical_extent_m:
                continue
            occupancy = len(np.unique(np.floor(window_uv / 0.075).astype(int), axis=0))
            size_error = float(np.sum(np.abs(window_extent - 0.90)))
            center_distance = math.hypot(center_u, center_v)
            score = (
                occupancy
                + 0.025 * np.count_nonzero(window)
                - 10.0 * size_error
                - 2.0 * center_distance
            )
            if best_window is None or score > best_window[0]:
                best_window = score, window
    if best_window is None:
        diagnostics["rejection"] = "no_dense_board_sized_plane_window"
        return None
    inliers = plane_points[best_window[1]]
    center_for_plane = np.mean(inliers, axis=0)
    _, _, vectors_t = np.linalg.svd(inliers - center_for_plane, full_matrices=False)
    normal = vectors_t[-1]
    if normal @ predicted_normal < 0.0:
        normal = -normal
    offset = -float(normal @ center_for_plane)
    horizontal, vertical = board_basis(normal)
    plane_origin = predicted_center - normal * float(normal @ predicted_center + offset)
    uv = np.column_stack(
        ((inliers - plane_origin) @ horizontal, (inliers - plane_origin) @ vertical)
    )
    low, high = np.percentile(uv, [3.0, 97.0], axis=0)
    extent = high - low
    if not (0.28 <= extent[0] <= 1.24 and min_vertical_extent_m <= extent[1] <= 1.24):
        diagnostics["rejection"] = "refined_extent_outside_board_bounds"
        diagnostics["refined_extent_m"] = extent.tolist()
        return None
    horizontal_center = float((low[0] + high[0]) / 2.0)
    visible_center = (
        plane_origin + horizontal * horizontal_center + vertical * float((low[1] + high[1]) / 2.0)
    )
    if float(np.percentile(inliers[:, 2], 5.0)) <= ground_z + 0.20:
        # The front/left/right targets stood on the floor.  Roof-lidar rings
        # often cover only their upper or lower half, so an extent midpoint is
        # biased.  The known 900 mm board height supplies its true vertical
        # centre without removing any floor points during plane detection.
        target_z = ground_z + 0.45 * vertical[2]
        vertical_center = float((target_z - plane_origin[2]) / max(vertical[2], 1e-6))
        center = plane_origin + horizontal * horizontal_center + vertical * vertical_center
    else:
        center = visible_center
    center -= normal * float(normal @ center + offset)
    rms = float(np.sqrt(np.mean(np.square(inliers @ normal + offset))))
    prediction_error = float(np.linalg.norm(center - predicted_center))
    if prediction_error > max_prediction_error_m:
        diagnostics["rejection"] = "board_center_over_limit_from_camera_prediction"
        diagnostics["candidate_prediction_error_m"] = prediction_error
        return None
    return (
        center,
        normal,
        {
            "points": len(inliers),
            "rms_m": rms,
            "extent_m": extent,
            "prediction_error_m": prediction_error,
        },
    )


def observation_errors(
    observations: list[BoardObservation], transform: np.ndarray
) -> tuple[np.ndarray, np.ndarray]:
    centers, angles = [], []
    for item in observations:
        center = transform[:3, :3] @ item.camera_center + transform[:3, 3]
        normal = transform[:3, :3] @ item.camera_normal
        sign = 1.0 if normal @ item.lidar_normal >= 0.0 else -1.0
        centers.append(float(np.linalg.norm(center - item.lidar_center)))
        angles.append(
            math.degrees(math.acos(float(np.clip((normal * sign) @ item.lidar_normal, -1.0, 1.0))))
        )
    return np.asarray(centers), np.asarray(angles)


def fit_extrinsic(
    observations: list[BoardObservation], initial: np.ndarray
) -> tuple[np.ndarray, list[BoardObservation]]:
    if len(observations) < 8:
        raise ValueError(f"need >=8 observations, got {len(observations)}")
    signs = [
        1.0 if (initial[:3, :3] @ item.camera_normal) @ item.lidar_normal >= 0 else -1.0
        for item in observations
    ]
    initial_rotvec = Rotation.from_matrix(initial[:3, :3]).as_rotvec()
    initial_translation = initial[:3, 3].copy()

    def solve(items: list[BoardObservation], item_signs: list[float]) -> np.ndarray:
        def residual(values: np.ndarray) -> np.ndarray:
            rotation = Rotation.from_rotvec(values[:3]).as_matrix()
            translation = values[3:]
            parts = []
            for item, sign in zip(items, item_signs):
                center_error = rotation @ item.camera_center + translation - item.lidar_center
                normal_error = rotation @ (item.camera_normal * sign) - item.lidar_normal
                parts.extend((center_error / 0.065, normal_error / 0.075))
            # The rough tape-measure pose is a weak guard against a planar
            # degenerate solution, not a substitute for the observations.
            parts.extend(
                (
                    (values[:3] - initial_rotvec) / math.radians(12.0),
                    (translation - initial_translation) / 0.35,
                )
            )
            return np.concatenate(parts)

        solution = least_squares(
            residual,
            np.r_[initial_rotvec, initial_translation],
            loss="soft_l1",
            f_scale=1.0,
            max_nfev=1000,
        )
        if not solution.success:
            raise ValueError(solution.message)
        matrix = np.eye(4)
        matrix[:3, :3] = Rotation.from_rotvec(solution.x[:3]).as_matrix()
        matrix[:3, 3] = solution.x[3:]
        return matrix

    fitted = solve(observations, signs)
    center_error, angle_error = observation_errors(observations, fitted)
    center_cut = max(
        0.16,
        float(
            np.median(center_error)
            + 3.5 * np.median(np.abs(center_error - np.median(center_error)))
        ),
    )
    angle_cut = max(
        12.0,
        float(
            np.median(angle_error) + 3.5 * np.median(np.abs(angle_error - np.median(angle_error)))
        ),
    )
    keep = (center_error <= center_cut) & (angle_error <= angle_cut)
    filtered = [item for item, accepted in zip(observations, keep) if accepted]
    filtered_signs = [sign for sign, accepted in zip(signs, keep) if accepted]
    if len(filtered) < 8:
        raise ValueError(f"only {len(filtered)} observations remain after outlier rejection")
    return solve(filtered, filtered_signs), filtered


def metrics(observations: list[BoardObservation], transform: np.ndarray) -> dict[str, Any]:
    if not observations:
        return {"count": 0}
    centers, angles = observation_errors(observations, transform)
    return {
        "count": len(observations),
        "capture_count": len({item.capture_id for item in observations}),
        "center_rms_m": round(float(np.sqrt(np.mean(np.square(centers)))), 6),
        "center_median_m": round(float(np.median(centers)), 6),
        "center_p95_m": round(float(np.percentile(centers, 95)), 6),
        "normal_rms_deg": round(float(np.sqrt(np.mean(np.square(angles)))), 5),
        "normal_median_deg": round(float(np.median(angles)), 5),
        "normal_p95_deg": round(float(np.percentile(angles, 95)), 5),
    }


def correction_size(refined: np.ndarray, initial: np.ndarray) -> tuple[float, float]:
    delta = refined @ np.linalg.inv(initial)
    return (
        float(np.linalg.norm(delta[:3, 3])),
        math.degrees(Rotation.from_matrix(delta[:3, :3]).magnitude()),
    )


def unique_board_poses(observations: list[BoardObservation]) -> list[BoardObservation]:
    """Keep one best observation for each unchanged physical board pose."""
    clusters: list[BoardObservation] = []
    for item in sorted(observations, key=lambda value: value.capture_id):
        matching = None
        for index, existing in enumerate(clusters):
            distance = float(np.linalg.norm(item.lidar_center - existing.lidar_center))
            cosine = abs(float(item.lidar_normal @ existing.lidar_normal))
            angle = math.degrees(math.acos(float(np.clip(cosine, -1.0, 1.0))))
            if distance <= 0.16 and angle <= 5.0:
                matching = index
                break
        if matching is None:
            clusters.append(item)
        else:
            previous = clusters[matching]
            previous_score = previous.lidar_points / max(previous.lidar_rms_m, 0.003)
            score = item.lidar_points / max(item.lidar_rms_m, 0.003)
            if score > previous_score:
                clusters[matching] = item
    return clusters


def main() -> int:
    base = Path(__file__).resolve().parent
    parser = argparse.ArgumentParser()
    parser.add_argument("--data-root", type=Path, required=True)
    parser.add_argument("--annotations", type=Path)
    parser.add_argument("--config", type=Path, default=base / "config.yaml")
    parser.add_argument("--initial", type=Path, default=base / "initial_extrinsics.yaml")
    parser.add_argument(
        "--lidar-transforms", type=Path, help="accepted environment-refined lower-lidar transforms"
    )
    parser.add_argument("--output", type=Path, default=base / "camera_lidar_refined.yaml")
    parser.add_argument(
        "--observations-csv", type=Path, default=base / "camera_lidar_observations.csv"
    )
    parser.add_argument("--max-captures", type=int, default=0)
    parser.add_argument(
        "--capture-id",
        action="append",
        help="process only this capture ID (repeatable, for diagnostics)",
    )
    args = parser.parse_args()

    config = yaml.safe_load(args.config.read_text(encoding="utf-8"))
    initial_data = yaml.safe_load(args.initial.read_text(encoding="utf-8"))
    absolute = absolute_transforms(initial_data)
    top_from_sensor_kit = np.linalg.inv(absolute["velodyne_top_base_link"])
    top_from_lidar = {"lidar_top": np.eye(4)}
    lidar_frames = {
        "lidar_front_left": "velodyne_front_left_base_link",
        "lidar_front_right": "velodyne_front_right_base_link",
        "lidar_rear": "velodyne_rear_base_link",
    }
    refined_lidars: dict[str, Any] = {}
    if args.lidar_transforms:
        refined_lidars = yaml.safe_load(args.lidar_transforms.read_text(encoding="utf-8")).get(
            "transforms", {}
        )
    for sensor, frame in lidar_frames.items():
        item = refined_lidars.get(frame, initial_data["transforms"][frame])
        if item["parent"] != "velodyne_top_base_link":
            raise SystemExit(f"{frame} must be relative to velodyne_top_base_link")
        top_from_lidar[sensor] = matrix_from_item(item)
    detector = make_detector(config["tag_board"]["dictionary"])
    requested = set(args.capture_id) if args.capture_id else None
    captures = load_captures(args.data_root, args.annotations, args.max_captures, requested)
    if not captures:
        raise SystemExit("no requested curated captures found")
    cameras = PRIORITY_CAMERAS + FISHEYE_CAMERAS
    intrinsics = {}
    initial_top_optical = {}
    for name in cameras:
        intrinsics[name] = load_intrinsics(Path(config["intrinsics_dir"]) / f"{name}.yaml")
        frame = f"{name}/camera_link"
        initial_top_link = top_from_sensor_kit @ absolute[frame]
        initial_top_optical[name] = initial_top_link @ CAMERA_OPTICAL_TO_LINK

    observations: dict[str, list[BoardObservation]] = {name: [] for name in cameras}
    extraction_rows = []
    for capture_index, (directory, manifest) in enumerate(captures):
        try:
            roof = fused_lidar_cloud(directory, top_from_lidar)
        except (OSError, ValueError, RuntimeError) as error:
            extraction_rows.append(
                {"capture_id": directory.name, "camera": "*", "status": f"roof_pcd_error:{error}"}
            )
            continue
        for name in cameras:
            sensor = manifest.get("sensors", {}).get(name, {})
            relative = sensor.get("file")
            image = cv2.imread(str(directory / relative), cv2.IMREAD_COLOR) if relative else None
            if image is None:
                extraction_rows.append(
                    {"capture_id": directory.name, "camera": name, "status": "missing_image"}
                )
                continue
            camera, distortion, model = intrinsics[name]
            corners, ids = detector(cv2.cvtColor(image, cv2.COLOR_BGR2GRAY))
            if ids is None:
                extraction_rows.append(
                    {"capture_id": directory.name, "camera": name, "status": "no_grid"}
                )
                continue
            poses = estimate_grid_poses(
                corners, ids.flatten(), config["tag_board"], camera, distortion, model
            )
            if not poses:
                extraction_rows.append(
                    {"capture_id": directory.name, "camera": name, "status": "no_complete_grid"}
                )
                continue
            accepted_here = 0
            for pose_index, (center, normal, tag_ids) in enumerate(poses):
                transform = initial_top_optical[name]
                predicted_center = transform[:3, :3] @ center + transform[:3, 3]
                predicted_normal = transform[:3, :3] @ normal
                digest = hashlib.sha256(f"{directory.name}:{name}:{pose_index}".encode()).digest()
                seed = int.from_bytes(digest[:4], "little")
                search_diagnostics: dict[str, Any] = {}
                fitted = fit_board_near_prediction(
                    roof, predicted_center, predicted_normal, seed, search_diagnostics
                )
                if fitted is None:
                    extraction_rows.append(
                        {
                            "capture_id": directory.name,
                            "camera": name,
                            "status": "grid_pose_without_local_lidar_board",
                            "tag_count": len(tag_ids),
                            "predicted_center_top": " ".join(
                                f"{value:.6f}" for value in predicted_center
                            ),
                            "predicted_normal_top": " ".join(
                                f"{value:.6f}" for value in normalize(predicted_normal)
                            ),
                            **search_diagnostics,
                        }
                    )
                    continue
                lidar_center, lidar_normal, fit = fitted
                observations[name].append(
                    BoardObservation(
                        directory.name,
                        center,
                        normal,
                        lidar_center,
                        lidar_normal,
                        int(fit["points"]),
                        float(fit["rms_m"]),
                        fit["extent_m"],
                    )
                )
                extraction_rows.append(
                    {
                        "capture_id": directory.name,
                        "camera": name,
                        "status": "accepted_local_plane",
                        "tag_count": len(tag_ids),
                        "lidar_points": fit["points"],
                        "lidar_rms_m": round(fit["rms_m"], 6),
                        "lidar_extent_x_m": round(float(fit["extent_m"][0]), 5),
                        "lidar_extent_y_m": round(float(fit["extent_m"][1]), 5),
                        "prediction_error_m": round(fit["prediction_error_m"], 5),
                        "predicted_center_top": " ".join(
                            f"{value:.6f}" for value in predicted_center
                        ),
                        "predicted_normal_top": " ".join(
                            f"{value:.6f}" for value in normalize(predicted_normal)
                        ),
                        "lidar_center_top": " ".join(f"{value:.6f}" for value in lidar_center),
                        "lidar_normal_top": " ".join(f"{value:.6f}" for value in lidar_normal),
                    }
                )
                accepted_here += 1
            if not accepted_here:
                extraction_rows.append(
                    {"capture_id": directory.name, "camera": name, "status": "no_local_lidar_board"}
                )
        if (capture_index + 1) % 25 == 0:
            print(f"processed {capture_index + 1}/{len(captures)} captures", flush=True)

    args.observations_csv.parent.mkdir(parents=True, exist_ok=True)
    fields = sorted({key for row in extraction_rows for key in row})
    with args.observations_csv.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(extraction_rows)

    result: dict[str, Any] = {
        "schema_version": 1,
        "reference_frame": "velodyne_top_base_link",
        "warning": "Review-only result; live Autoware calibration was not modified.",
        "excluded_sensors": {
            "traffic_light_front": "frozen tag pose for the complete session",
        },
        "lidar_cloud_source": list(top_from_lidar),
        "transforms": {},
        "rejected_candidates": {},
        "quality": {},
    }
    for name in cameras:
        raw_values = sorted(observations[name], key=lambda item: item.capture_id)
        values = unique_board_poses(raw_values)
        heldout = values[::5] if len(values) >= 10 else []
        heldout_ids = {id(item) for item in heldout}
        training = [item for item in values if id(item) not in heldout_ids]
        frame = f"{name}/camera_link"
        try:
            refined_optical, filtered = fit_extrinsic(training, initial_top_optical[name])
            training_metrics = metrics(filtered, refined_optical)
            heldout_metrics = metrics(heldout, refined_optical)
            translation_correction, rotation_correction = correction_size(
                refined_optical, initial_top_optical[name]
            )
            evaluation = heldout_metrics if heldout else training_metrics
            threshold_center = 0.12 if name in PRIORITY_CAMERAS else 0.16
            threshold_normal = 10.0 if name in PRIORITY_CAMERAS else 15.0
            accepted = bool(
                evaluation.get("count", 0) >= 2
                and evaluation["center_rms_m"] <= threshold_center
                and evaluation["normal_rms_deg"] <= threshold_normal
                and translation_correction <= 0.55
                and rotation_correction <= 22.0
            )
            refined_top_link = refined_optical @ np.linalg.inv(CAMERA_OPTICAL_TO_LINK)
            refined_sensor_link = absolute["velodyne_top_base_link"] @ refined_top_link
            candidate = matrix_to_item(
                refined_sensor_link,
                "sensor_kit_base_link",
                "camera_aprilgrid_to_local_roof_lidar_plane",
            )
            destination = "transforms" if accepted else "rejected_candidates"
            result[destination][frame] = candidate
            result["quality"][frame] = {
                "accepted": accepted,
                "priority": "six_usb_cameras"
                if name in PRIORITY_CAMERAS
                else "fisheye_best_effort",
                "raw_observations": len(raw_values),
                "unique_board_poses": len(values),
                "training": training_metrics,
                "heldout": heldout_metrics,
                "correction_translation_m": round(translation_correction, 6),
                "correction_rotation_deg": round(rotation_correction, 5),
                "threshold_center_rms_m": threshold_center,
                "threshold_normal_rms_deg": threshold_normal,
            }
            print(
                f"{name}: {'accepted' if accepted else 'REJECTED'}; "
                f"obs={len(values)}, heldout={evaluation}"
            )
        except ValueError as error:
            result["quality"][frame] = {
                "accepted": False,
                "raw_observations": len(raw_values),
                "unique_board_poses": len(values),
                "error": str(error),
            }
            print(f"{name}: REJECTED; {error}")
    args.output.write_text(
        yaml.safe_dump(result, allow_unicode=True, sort_keys=False), encoding="utf-8"
    )
    print(f"review-only result: {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
