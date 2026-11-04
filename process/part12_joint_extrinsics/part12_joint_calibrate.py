#!/usr/bin/env python3
"""Calibrate Part 1/2 fixed-board, moving-vehicle sensor extrinsics.

The four physical AprilGrid boards remain fixed while the vehicle moves.  All
image detections are repeated from saved pixels.  Camera/lidar timestamps are
gated before a lidar plane is associated with an image observation.  The roof
32-line lidar is the fixed reference frame; no result is deployed by this
program.

This intentionally differs from ``calibrate_*_dynamic_boards.py``: temporal
background subtraction is invalid here because the vehicle, not the boards,
moved between captures.
"""

from __future__ import annotations

import argparse
from collections import Counter, defaultdict
import csv
import hashlib
import json
import math
from pathlib import Path
import sys
from typing import Any

import numpy as np
from scipy.optimize import least_squares
from scipy.sparse import lil_matrix
from scipy.spatial.transform import Rotation
import yaml


HERE = Path(__file__).resolve().parent
JOINT = HERE.parent / "joint_extrinsic_station"
if str(JOINT) not in sys.path:
    sys.path.insert(0, str(JOINT))

from calibrate_lidar_heights import read_binary_pcd_xyz  # noqa: E402
from extract_camera_pair_constraints import (  # noqa: E402
    CAMERAS,
    observations_for_capture,
    pair_observations,
    transform_distance,
)
from refine_camera_extrinsics import (  # noqa: E402
    fit_board_near_prediction,
    pcd_for,
    voxel,
)
from refine_lidar_environment import apply, matrix_from_item  # noqa: E402
from solve_joint_extrinsics import (  # noqa: E402
    CAMERA_OPTICAL_TO_LINK,
    load_intrinsics,
)


LIDARS = ("lidar_top", "lidar_front_left", "lidar_front_right", "lidar_rear")
LIDAR_FRAMES = {
    "lidar_front_left": "velodyne_front_left_base_link",
    "lidar_front_right": "velodyne_front_right_base_link",
    "lidar_rear": "velodyne_rear_base_link",
}
CALIBRATED_CAMERAS = tuple(camera for camera in CAMERAS if camera != "traffic_light_front")
OPTIMIZED = tuple(LIDAR_FRAMES) + CALIBRATED_CAMERAS


def vec(value: np.ndarray) -> list[float]:
    return [float(x) for x in np.asarray(value).reshape(-1)]


def normalized(value: np.ndarray) -> np.ndarray:
    return value / max(float(np.linalg.norm(value)), 1e-12)


def angle_deg(left: np.ndarray, right: np.ndarray) -> float:
    return math.degrees(
        math.acos(float(np.clip(abs(normalized(left) @ normalized(right)), -1.0, 1.0)))
    )


def load_seed(path: Path) -> dict[str, np.ndarray]:
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    values: dict[str, np.ndarray] = {"lidar_top": np.eye(4)}
    for sensor, frame in LIDAR_FRAMES.items():
        values[sensor] = matrix_from_item(data["transforms"][frame])
    for camera in CAMERAS:
        link = matrix_from_item(data["transforms"][f"{camera}/camera_link"])
        values[camera] = link @ CAMERA_OPTICAL_TO_LINK
    return values


def load_all_intrinsics(base: Path) -> dict[str, tuple[np.ndarray, np.ndarray, str]]:
    result = {}
    for camera in CAMERAS:
        if camera.startswith("fisheye_"):
            path = (
                base
                / "result/fisheye_intrinsics_1600x1200_20260903_v2"
                / camera
                / "camera_info.yaml"
            )
        else:
            path = base / "result/camera_intrinsics" / f"{camera}.yaml"
        result[camera] = load_intrinsics(path)
    return result


def sensor_stamp(sensor: dict[str, Any]) -> int | None:
    value = sensor.get("source_stamp_ns") or sensor.get("received_unix_ns")
    return int(value) if value is not None else None


def load_cloud(directory: Path, sensor: str, transform: np.ndarray) -> np.ndarray:
    points = apply(transform, read_binary_pcd_xyz(pcd_for(directory, sensor)))
    radius = np.linalg.norm(points[:, :2], axis=1)
    points = points[
        (radius >= 0.50) & (radius <= 14.0) & (points[:, 2] >= -2.0) & (points[:, 2] <= 1.4)
    ]
    return voxel(points, 0.025)


def to_local(
    transform: np.ndarray, point: np.ndarray, normal: np.ndarray
) -> tuple[np.ndarray, np.ndarray]:
    return (
        transform[:3, :3].T @ (point - transform[:3, 3]),
        normalized(transform[:3, :3].T @ normal),
    )


def fit_horizontal_plane(
    points: np.ndarray,
    predicted_center: np.ndarray,
    predicted_normal: np.ndarray,
    seed: int,
    diagnostics: dict[str, Any],
) -> tuple[np.ndarray, np.ndarray, dict[str, Any]] | None:
    """Fit the one board laid flat on the garage floor.

    The general board fitter deliberately treats one tangent axis as vertical
    and therefore cannot represent a horizontal board.  Here only the plane
    equation is used; the surrounding floor is valid evidence for the same
    height/roll/pitch constraint and no false tangential board centre is
    claimed.
    """
    normal_seed = normalized(predicted_normal)
    if normal_seed[2] < 0:
        normal_seed = -normal_seed
    helper = np.array([1.0, 0.0, 0.0])
    if abs(float(helper @ normal_seed)) > 0.85:
        helper = np.array([0.0, 1.0, 0.0])
    tangent_u = normalized(helper - normal_seed * float(helper @ normal_seed))
    tangent_v = normalized(np.cross(normal_seed, tangent_u))
    delta = points - predicted_center
    coordinates = np.column_stack((delta @ tangent_u, delta @ tangent_v, delta @ normal_seed))
    candidates = points[
        (np.abs(coordinates[:, 0]) <= 0.72)
        & (np.abs(coordinates[:, 1]) <= 0.72)
        & (np.abs(coordinates[:, 2]) <= 0.20)
    ]
    diagnostics["horizontal_roi_points"] = len(candidates)
    if len(candidates) < 40:
        diagnostics["rejection"] = "horizontal_roi_points_lt_40"
        return None
    rng = np.random.default_rng(seed)
    sample = candidates
    if len(sample) > 5000:
        sample = sample[rng.choice(len(sample), 5000, replace=False)]
    best = None
    threshold = 0.022
    for _ in range(140):
        triple = sample[rng.choice(len(sample), 3, replace=False)]
        normal = np.cross(triple[1] - triple[0], triple[2] - triple[0])
        if np.linalg.norm(normal) < 1e-8:
            continue
        normal = normalized(normal)
        if normal @ normal_seed < 0:
            normal = -normal
        if normal @ normal_seed < math.cos(math.radians(14.0)):
            continue
        offset = -float(normal @ triple[0])
        if abs(float(normal @ predicted_center + offset)) > 0.16:
            continue
        mask = np.abs(sample @ normal + offset) <= threshold
        count = int(np.count_nonzero(mask))
        if count < 36:
            continue
        if best is None or count > best[0]:
            best = count, normal, offset
    if best is None:
        diagnostics["rejection"] = "no_horizontal_plane"
        return None
    _count, normal, offset = best
    for _ in range(3):
        inliers = candidates[np.abs(candidates @ normal + offset) <= threshold]
        if len(inliers) < 36:
            return None
        center = np.mean(inliers, axis=0)
        _, _, vt = np.linalg.svd(inliers - center, full_matrices=False)
        normal = vt[-1]
        if normal @ normal_seed < 0:
            normal = -normal
        offset = -float(normal @ center)
    inliers = candidates[np.abs(candidates @ normal + offset) <= threshold]
    # Keep the plane centre at the camera-predicted tangent location.  Only
    # project it normally; the optimizer consumes the plane equation, not this
    # arbitrary patch centre.
    center = predicted_center - normal * float(normal @ predicted_center + offset)
    uv = np.column_stack(((inliers - center) @ tangent_u, (inliers - center) @ tangent_v))
    extent = np.diff(np.percentile(uv, [5.0, 95.0], axis=0), axis=0)[0]
    rms = float(np.sqrt(np.mean(np.square(inliers @ normal + offset))))
    prediction_error = float(np.linalg.norm(center - predicted_center))
    if prediction_error > 0.16 or rms > 0.022 or min(extent) < 0.35:
        diagnostics["rejection"] = "horizontal_quality_gate"
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


def extract(
    base: Path,
    config_path: Path,
    seed_path: Path,
    part_lists: list[Path],
    output: Path,
    max_sync_ms: float,
) -> dict[str, Any]:
    config = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    transforms = load_seed(seed_path)
    intrinsics = load_all_intrinsics(base)
    capture_parts: list[tuple[int, str]] = []
    for part, path in enumerate(part_lists, start=1):
        capture_parts.extend(
            (part, value) for value in path.read_text(encoding="utf-8").split() if value
        )

    plane_edges: list[dict[str, Any]] = []
    camera_edges: list[dict[str, Any]] = []
    audit_rows: list[dict[str, Any]] = []
    timestamp_rejects: Counter[str] = Counter()
    detection_counts: Counter[str] = Counter()
    plane_counts: Counter[str] = Counter()
    pair_counts: Counter[str] = Counter()

    for capture_index, (part, capture_id) in enumerate(capture_parts, start=1):
        directory = base / "data/raw_extrinsic_data" / capture_id
        manifest = json.loads((directory / "manifest.json").read_text(encoding="utf-8"))
        sensor_meta = manifest.get("sensors", {})
        all_poses = observations_for_capture(
            directory,
            manifest,
            config["tag_board"],
            intrinsics,
            {camera: transforms[camera] for camera in CAMERAS},
        )
        for pose in all_poses:
            detection_counts[f"P{part}:{pose.camera}"] += 1
        # The traffic stream is systematically 222--373 ms older than the roof
        # scan in these two moving-vehicle parts.  It is solved from Part 4 by
        # the dedicated traffic-camera task, never through a lucky pair-time
        # coincidence here.
        poses = [pose for pose in all_poses if pose.camera in CALIBRATED_CAMERAS]

        # Full camera-camera board-pose constraints are independent of lidar.
        # Reject frames whose source/receive times differ by over max_sync_ms.
        for edge in pair_observations(poses, {c: transforms[c] for c in CAMERAS}):
            left_t = sensor_stamp(sensor_meta.get(edge.left, {}))
            right_t = sensor_stamp(sensor_meta.get(edge.right, {}))
            sync_ms = None if left_t is None or right_t is None else abs(left_t - right_t) / 1e6
            if sync_ms is None or sync_ms > max_sync_ms:
                timestamp_rejects[f"camera_pair:{edge.left}<->{edge.right}"] += 1
                continue
            seed_error = transform_distance(
                np.linalg.inv(transforms[edge.left]) @ transforms[edge.right],
                edge.transform_left_from_right,
            )
            if seed_error[0] > 0.10 or seed_error[1] > 3.0:
                timestamp_rejects[f"camera_pair_geometric:{edge.left}<->{edge.right}"] += 1
                continue
            item = {
                "capture_id": capture_id,
                "part": part,
                "left": edge.left,
                "right": edge.right,
                "observed_left_from_right": edge.transform_left_from_right.tolist(),
                "rough_center_gap_m": edge.rough_center_gap_m,
                "reprojection_rms_px": edge.reprojection_rms_px,
                "sync_ms": sync_ms,
            }
            camera_edges.append(item)
            pair_counts[f"P{part}:{edge.left}<->{edge.right}"] += 1

        clouds = {sensor: load_cloud(directory, sensor, transforms[sensor]) for sensor in LIDARS}
        for pose_index, pose in enumerate(poses):
            camera_t = sensor_stamp(sensor_meta.get(pose.camera, {}))
            for lidar in LIDARS:
                lidar_t = sensor_stamp(sensor_meta.get(lidar, {}))
                sync_ms = (
                    None if camera_t is None or lidar_t is None else abs(camera_t - lidar_t) / 1e6
                )
                row: dict[str, Any] = {
                    "capture_id": capture_id,
                    "part": part,
                    "camera": pose.camera,
                    "lidar": lidar,
                    "sync_ms": sync_ms,
                    "reprojection_rms_px": pose.reprojection_rms_px,
                }
                if sync_ms is None or sync_ms > max_sync_ms:
                    row["status"] = "timestamp_rejected"
                    timestamp_rejects[f"camera_lidar:{pose.camera}<->{lidar}"] += 1
                    audit_rows.append(row)
                    continue
                diagnostics: dict[str, Any] = {}
                digest = hashlib.sha256(
                    f"part12:{capture_id}:{pose.camera}:{pose_index}:{lidar}".encode()
                ).digest()
                if abs(float(pose.rough_normal_top[2])) >= 0.70:
                    fitted = fit_horizontal_plane(
                        clouds[lidar],
                        pose.rough_center_top,
                        pose.rough_normal_top,
                        int.from_bytes(digest[:4], "little"),
                        diagnostics,
                    )
                else:
                    fitted = fit_board_near_prediction(
                        clouds[lidar],
                        pose.rough_center_top,
                        pose.rough_normal_top,
                        int.from_bytes(digest[:4], "little"),
                        diagnostics,
                        ground_z=-1.72817,
                        min_vertical_extent_m=0.12,
                        max_prediction_error_m=0.60,
                    )
                row.update(diagnostics)
                if fitted is None:
                    row["status"] = "plane_not_found"
                    audit_rows.append(row)
                    continue
                center_top, normal_top, quality = fitted
                center_gap = float(np.linalg.norm(center_top - pose.rough_center_top))
                normal_gap = angle_deg(normal_top, pose.rough_normal_top)
                row.update(
                    {
                        "center_gap_m": center_gap,
                        "normal_gap_deg": normal_gap,
                        "points": int(quality["points"]),
                        "rms_m": float(quality["rms_m"]),
                        "extent_0_m": float(quality["extent_m"][0]),
                        "extent_1_m": float(quality["extent_m"][1]),
                    }
                )
                # Strong association gate: the old extrinsics are only a seed,
                # but a wall/column must never enter just because it is planar.
                if (
                    center_gap > 0.45
                    or normal_gap > 12.0
                    or quality["rms_m"] > 0.030
                    or quality["points"] < 22
                ):
                    row["status"] = "geometric_gate_rejected"
                    audit_rows.append(row)
                    continue
                center_local, normal_local = to_local(transforms[lidar], center_top, normal_top)
                plane_edges.append(
                    {
                        "capture_id": capture_id,
                        "part": part,
                        "camera": pose.camera,
                        "lidar": lidar,
                        "camera_center": vec(pose.transform_camera_from_board[:3, 3]),
                        "camera_normal": vec(pose.transform_camera_from_board[:3, 2]),
                        "lidar_center": vec(center_local),
                        "lidar_normal": vec(normal_local),
                        "points": int(quality["points"]),
                        "lidar_rms_m": float(quality["rms_m"]),
                        "reprojection_rms_px": float(pose.reprojection_rms_px),
                        "sync_ms": sync_ms,
                    }
                )
                plane_counts[f"P{part}:{pose.camera}<->{lidar}"] += 1
                row["status"] = "accepted"
                audit_rows.append(row)

        if capture_index % 10 == 0 or capture_index == len(capture_parts):
            print(
                json.dumps(
                    {
                        "stage": "extract",
                        "captures": capture_index,
                        "captures_total": len(capture_parts),
                        "camera_poses": int(sum(detection_counts.values())),
                        "plane_edges": len(plane_edges),
                        "camera_edges": len(camera_edges),
                    },
                    ensure_ascii=False,
                ),
                flush=True,
            )

    payload = {
        "schema_version": 1,
        "method": "fixed_four_boards_moving_vehicle_fresh_pixels",
        "reference_frame": "velodyne_top_base_link",
        "max_sync_ms": max_sync_ms,
        "parts": [str(path) for path in part_lists],
        "seed": str(seed_path),
        "camera_intrinsics": "normal accepted set plus fisheye 1600x1200 v2",
        "plane_edges": plane_edges,
        "camera_edges": camera_edges,
        "summary": {
            "capture_count": len(capture_parts),
            "camera_pose_counts": dict(sorted(detection_counts.items())),
            "plane_edge_counts": dict(sorted(plane_counts.items())),
            "camera_pair_counts": dict(sorted(pair_counts.items())),
            "timestamp_reject_counts": dict(sorted(timestamp_rejects.items())),
        },
    }
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")
    audit_path = output.with_suffix(".audit.csv")
    fields = sorted({key for row in audit_rows for key in row})
    with audit_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(audit_rows)
    return payload


def pack(transforms: dict[str, np.ndarray]) -> np.ndarray:
    values = []
    for sensor in OPTIMIZED:
        values.extend(Rotation.from_matrix(transforms[sensor][:3, :3]).as_rotvec())
        values.extend(transforms[sensor][:3, 3])
    return np.asarray(values, dtype=float)


def unpack(values: np.ndarray) -> dict[str, np.ndarray]:
    result = {"lidar_top": np.eye(4)}
    for index, sensor in enumerate(OPTIMIZED):
        start = 6 * index
        matrix = np.eye(4)
        matrix[:3, :3] = Rotation.from_rotvec(values[start : start + 3]).as_matrix()
        matrix[:3, 3] = values[start + 3 : start + 6]
        result[sensor] = matrix
    return result


def plane_error(edge: dict[str, Any], transforms: dict[str, np.ndarray]) -> tuple[float, float]:
    tc, tl = transforms[edge["camera"]], transforms[edge["lidar"]]
    cc, nc = np.asarray(edge["camera_center"]), np.asarray(edge["camera_normal"])
    cl, nl = np.asarray(edge["lidar_center"]), np.asarray(edge["lidar_normal"])
    nc_top, nl_top = tc[:3, :3] @ nc, tl[:3, :3] @ nl
    cc_top = tc[:3, :3] @ cc + tc[:3, 3]
    cl_top = tl[:3, :3] @ cl + tl[:3, 3]
    if nc_top @ nl_top < 0:
        nl_top = -nl_top
    distance = abs(float(-nc_top @ cc_top + nl_top @ cl_top))
    return distance, angle_deg(nc_top, nl_top)


def pair_error(edge: dict[str, Any], transforms: dict[str, np.ndarray]) -> tuple[float, float]:
    observed = np.asarray(edge["observed_left_from_right"], dtype=float)
    predicted = np.linalg.inv(transforms[edge["left"]]) @ transforms[edge["right"]]
    return transform_distance(predicted, observed)


def group_weights(items: list[dict[str, Any]], fields: tuple[str, ...]) -> list[float]:
    counts = Counter(tuple(str(item[field]) for field in fields) for item in items)
    reference = float(np.median(list(counts.values()))) if counts else 1.0
    return [
        min(
            3.0,
            max(0.40, math.sqrt(reference / counts[tuple(str(item[field]) for field in fields)])),
        )
        for item in items
    ]


def solve_once(
    plane_edges: list[dict[str, Any]],
    camera_edges: list[dict[str, Any]],
    start: dict[str, np.ndarray],
    prior: dict[str, np.ndarray],
    max_nfev: int = 120,
) -> dict[str, np.ndarray]:
    plane_weights = group_weights(plane_edges, ("part", "camera", "lidar"))
    pair_weights = group_weights(camera_edges, ("part", "left", "right"))
    prior_values = pack(prior)
    start_values = pack(start)

    def residual(values: np.ndarray) -> np.ndarray:
        transforms = unpack(values)
        parts: list[float] = []
        for edge, weight in zip(plane_edges, plane_weights):
            tc, tl = transforms[edge["camera"]], transforms[edge["lidar"]]
            cc = np.asarray(edge["camera_center"])
            nc = np.asarray(edge["camera_normal"])
            cl = np.asarray(edge["lidar_center"])
            nl = np.asarray(edge["lidar_normal"])
            nc_top, nl_top = tc[:3, :3] @ nc, tl[:3, :3] @ nl
            cc_top = tc[:3, :3] @ cc + tc[:3, 3]
            cl_top = tl[:3, :3] @ cl + tl[:3, 3]
            if nc_top @ nl_top < 0:
                nl_top = -nl_top
            sigma_d = max(0.012, min(0.035, 2.5 * float(edge["lidar_rms_m"])))
            parts.extend((weight * (nc_top - nl_top) / 0.040).tolist())
            parts.append(weight * float(-nc_top @ cc_top + nl_top @ cl_top) / sigma_d)
        for edge, weight in zip(camera_edges, pair_weights):
            observed = np.asarray(edge["observed_left_from_right"], dtype=float)
            predicted = np.linalg.inv(transforms[edge["left"]]) @ transforms[edge["right"]]
            rotation_error = Rotation.from_matrix(
                predicted[:3, :3] @ observed[:3, :3].T
            ).as_rotvec()
            parts.extend((weight * rotation_error / math.radians(0.75)).tolist())
            parts.extend((weight * (predicted[:3, 3] - observed[:3, 3]) / 0.035).tolist())
        # Priors guard disconnected or weakly observable tangent directions.
        current = unpack(values)
        for sensor in OPTIMIZED:
            rotation_delta = Rotation.from_matrix(
                current[sensor][:3, :3] @ prior[sensor][:3, :3].T
            ).as_rotvec()
            translation_delta = current[sensor][:3, 3] - prior[sensor][:3, 3]
            if sensor.startswith("lidar_"):
                parts.extend((rotation_delta / math.radians(2.0)).tolist())
                # The three BPearl heights were independently estimated from
                # many vehicle-floor returns (sub-millimetre repeatability).
                # All four calibration boards in Part 1/2 are invisible as a
                # distinct horizontal patch to the roof lidar, so allowing a
                # common Z shift would create an unobservable family of
                # equally good solutions.  Keep Z tied to that independent
                # evidence while x/y remain free to improve here.
                parts.extend((translation_delta[:2] / 0.060).tolist())
                parts.append(float(translation_delta[2] / 0.003))
            else:
                parts.extend((rotation_delta / math.radians(5.0)).tolist())
                parts.extend((translation_delta / 0.180).tolist())
        return np.asarray(parts)

    # Sparse finite differences: each observation depends on at most two
    # 6-DoF sensor blocks.  This makes the repeated robustness rounds fast.
    rows = 4 * len(plane_edges) + 6 * len(camera_edges) + 6 * len(OPTIMIZED)
    sparsity = lil_matrix((rows, 6 * len(OPTIMIZED)), dtype=int)
    row = 0
    index = {sensor: i for i, sensor in enumerate(OPTIMIZED)}
    for edge in plane_edges:
        for sensor in (edge["camera"], edge["lidar"]):
            if sensor != "lidar_top":
                col = 6 * index[sensor]
                sparsity[row : row + 4, col : col + 6] = 1
        row += 4
    for edge in camera_edges:
        for sensor in (edge["left"], edge["right"]):
            col = 6 * index[sensor]
            sparsity[row : row + 6, col : col + 6] = 1
        row += 6
    for sensor in OPTIMIZED:
        col = 6 * index[sensor]
        sparsity[row : row + 6, col : col + 6] = 1
        row += 6

    result = least_squares(
        residual,
        start_values,
        jac_sparsity=sparsity.tocsr(),
        loss="soft_l1",
        f_scale=1.0,
        max_nfev=max_nfev,
        xtol=2e-8,
        ftol=2e-8,
        gtol=2e-8,
        verbose=0,
    )
    if not result.success and result.status <= 0:
        raise RuntimeError(result.message)
    return unpack(result.x)


def subset_for_round(
    items: list[dict[str, Any]], round_number: int, fraction: float = 0.84
) -> list[dict[str, Any]]:
    selected = []
    threshold = int(fraction * 10_000)
    for index, item in enumerate(items):
        token = (
            f"{round_number}:{index}:{item['capture_id']}:"
            + str(item.get("camera", item.get("left", "")))
            + ":"
            + str(item.get("lidar", item.get("right", "")))
        )
        value = int.from_bytes(hashlib.sha256(token.encode()).digest()[:4], "little") % 10_000
        if value < threshold:
            selected.append(item)
    return selected


def summary_metrics(
    planes: list[dict[str, Any]], pairs: list[dict[str, Any]], transforms: dict[str, np.ndarray]
) -> dict[str, float]:
    pe = [plane_error(edge, transforms) for edge in planes]
    ce = [pair_error(edge, transforms) for edge in pairs]
    return {
        "plane_distance_median_m": float(np.median([x[0] for x in pe])) if pe else math.nan,
        "plane_distance_p95_m": float(np.percentile([x[0] for x in pe], 95)) if pe else math.nan,
        "plane_angle_median_deg": float(np.median([x[1] for x in pe])) if pe else math.nan,
        "plane_angle_p95_deg": float(np.percentile([x[1] for x in pe], 95)) if pe else math.nan,
        "pair_translation_median_m": float(np.median([x[0] for x in ce])) if ce else math.nan,
        "pair_translation_p95_m": float(np.percentile([x[0] for x in ce], 95)) if ce else math.nan,
        "pair_angle_median_deg": float(np.median([x[1] for x in ce])) if ce else math.nan,
        "pair_angle_p95_deg": float(np.percentile([x[1] for x in ce], 95)) if ce else math.nan,
    }


def transform_item(matrix: np.ndarray, source: str) -> dict[str, Any]:
    return {
        "parent": "velodyne_top_base_link",
        "xyz_m": [round(float(x), 9) for x in matrix[:3, 3]],
        "rpy_deg": [
            round(float(x), 8)
            for x in Rotation.from_matrix(matrix[:3, :3]).as_euler("xyz", degrees=True)
        ],
        "quaternion_xyzw": [
            round(float(x), 10) for x in Rotation.from_matrix(matrix[:3, :3]).as_quat()
        ],
        "matrix_row_major": [[round(float(x), 10) for x in row] for row in matrix],
        "source": source,
    }


def sensor_support(planes: list[dict[str, Any]], pairs: list[dict[str, Any]]) -> dict[str, Any]:
    result = {}
    for sensor in OPTIMIZED:
        p = [edge for edge in planes if sensor in (edge["camera"], edge["lidar"])]
        c = [edge for edge in pairs if sensor in (edge["left"], edge["right"])]
        result[sensor] = {
            "plane_edges": len(p),
            "camera_pair_edges": len(c),
            "parts": sorted(set(int(edge["part"]) for edge in p + c)),
            "capture_count": len(set(edge["capture_id"] for edge in p + c)),
        }
    return result


def optimize(
    payload: dict[str, Any],
    seed: dict[str, np.ndarray],
    output: Path,
    rounds_log: Path,
    max_rounds: int,
) -> dict[str, Any]:
    planes = list(payload["plane_edges"])
    pairs = list(payload["camera_edges"])
    current = dict(seed)
    # The prior is the already-audited previous calibration.  Apply the user's
    # explicit 10 cm rule before an optimizer can be pulled toward a garage
    # wall or a mismatched identical-ID board.
    active_planes = [
        edge
        for edge in planes
        if plane_error(edge, seed)[0] <= 0.10 and plane_error(edge, seed)[1] <= 6.0
    ]
    active_pairs = [
        edge
        for edge in pairs
        if pair_error(edge, seed)[0] <= 0.10 and pair_error(edge, seed)[1] <= 3.0
    ]
    pre_rejected = (len(planes) - len(active_planes)) + (len(pairs) - len(active_pairs))
    round_rows = []
    candidate_history: dict[str, list[np.ndarray]] = defaultdict(list)
    stable_rounds = 0

    rounds_log.parent.mkdir(parents=True, exist_ok=True)
    with rounds_log.open("w", encoding="utf-8") as log:
        log.write("# Part1/Part2 联合外参轮次日志\n\n")
        log.write("静态四标定板、车辆移动；主32线雷达固定为参考系。\n\n")
        log.write(f"优化前硬门限剔除：{pre_rejected} 条（>10 cm、平面>6°或共视>3°）。\n\n")

    for round_number in range(1, max_rounds + 1):
        selected_planes = subset_for_round(active_planes, round_number)
        selected_pairs = subset_for_round(active_pairs, round_number)
        previous = current
        current = solve_once(selected_planes, selected_pairs, current, seed, max_nfev=80)

        # Hard user gate after every solve.  Rejections are cumulative only in
        # the first ten rounds; later resampling estimates stability without
        # silently changing the evidence set.
        plane_errors = [plane_error(edge, current) for edge in active_planes]
        pair_errors = [pair_error(edge, current) for edge in active_pairs]
        keep_planes = [error[0] <= 0.10 and error[1] <= 6.0 for error in plane_errors]
        keep_pairs = [error[0] <= 0.10 and error[1] <= 3.0 for error in pair_errors]
        rejected_this_round = 0
        if round_number <= 10:
            next_planes = [edge for edge, keep in zip(active_planes, keep_planes) if keep]
            next_pairs = [edge for edge, keep in zip(active_pairs, keep_pairs) if keep]
            rejected_this_round = (len(active_planes) - len(next_planes)) + (
                len(active_pairs) - len(next_pairs)
            )
            active_planes, active_pairs = next_planes, next_pairs

        deltas = [transform_distance(previous[sensor], current[sensor]) for sensor in OPTIMIZED]
        max_dt = max(value[0] for value in deltas)
        max_da = max(value[1] for value in deltas)
        med_dt = float(np.median([value[0] for value in deltas]))
        med_da = float(np.median([value[1] for value in deltas]))
        metrics = summary_metrics(active_planes, active_pairs, current)
        for sensor in OPTIMIZED:
            candidate_history[sensor].append(current[sensor].copy())
        if len(round_rows) >= 9:
            recent_ok = max_dt <= 0.001 and max_da <= 0.1
            stable_rounds = stable_rounds + 1 if recent_ok else 0
        row = {
            "round": round_number,
            "selected_planes": len(selected_planes),
            "selected_camera_pairs": len(selected_pairs),
            "active_planes": len(active_planes),
            "active_camera_pairs": len(active_pairs),
            "rejected_this_round": rejected_this_round,
            "translation_change_median_m": med_dt,
            "translation_change_max_m": max_dt,
            "angle_change_median_deg": med_da,
            "angle_change_max_deg": max_da,
            **metrics,
            "step_converged": max_dt <= 0.001 and max_da <= 0.1,
        }
        round_rows.append(row)
        line = json.dumps({"stage": "round", **row}, ensure_ascii=False)
        print(line, flush=True)
        with rounds_log.open("a", encoding="utf-8") as log:
            log.write(
                f"## 第 {round_number} 轮\n\n"
                f"- 使用：平面 {len(selected_planes)}，相机共视 {len(selected_pairs)}；"
                f"活动观测 {len(active_planes) + len(active_pairs)}，本轮剔除 {rejected_this_round}\n"
                f"- 位姿变化：中位 {med_dt * 1000:.3f} mm / {med_da:.4f}°，"
                f"最大 {max_dt * 1000:.3f} mm / {max_da:.4f}°\n"
                f"- 平面残差：中位 {metrics['plane_distance_median_m'] * 1000:.3f} mm / "
                f"{metrics['plane_angle_median_deg']:.4f}°，P95 "
                f"{metrics['plane_distance_p95_m'] * 1000:.3f} mm / "
                f"{metrics['plane_angle_p95_deg']:.4f}°\n"
                f"- 共视残差：中位 {metrics['pair_translation_median_m'] * 1000:.3f} mm / "
                f"{metrics['pair_angle_median_deg']:.4f}°\n\n"
            )
        # At least 20 independent resamples are required.  Five consecutive
        # sub-mm / 0.1 degree steps stop the numerical loop; P1/P2 validation
        # below still decides whether deployment is allowed.
        if round_number >= 20 and stable_rounds >= 5:
            break

    # Consensus of the last up-to-20 deterministic resampling estimates.
    consensus: dict[str, np.ndarray] = {"lidar_top": np.eye(4)}
    stability = {}
    for sensor in OPTIMIZED:
        members = candidate_history[sensor][-min(20, len(candidate_history[sensor])) :]
        matrix = np.eye(4)
        matrix[:3, 3] = np.median([item[:3, 3] for item in members], axis=0)
        matrix[:3, :3] = Rotation.from_matrix([item[:3, :3] for item in members]).mean().as_matrix()
        consensus[sensor] = matrix
        spread = [transform_distance(matrix, item) for item in members]
        stability[sensor] = {
            "resamples": len(members),
            "translation_p95_m": float(np.percentile([x[0] for x in spread], 95)),
            "angle_p95_deg": float(np.percentile([x[1] for x in spread], 95)),
        }

    # Independent P1 / P2 fits.  A sensor must have direct evidence in both
    # parts before the difference can be called a cross-validation result.
    part_fits = {}
    cross_validation = {}
    support = sensor_support(active_planes, active_pairs)
    for part in (1, 2):
        pp = [edge for edge in active_planes if int(edge["part"]) == part]
        cp = [edge for edge in active_pairs if int(edge["part"]) == part]
        part_fits[part] = solve_once(pp, cp, consensus, seed, max_nfev=140)
    for sensor in OPTIMIZED:
        parts = support[sensor]["parts"]
        if parts == [1, 2]:
            dt, da = transform_distance(part_fits[1][sensor], part_fits[2][sensor])
            cross_validation[sensor] = {
                "status": "measured_in_both_parts",
                "translation_difference_m": dt,
                "angle_difference_deg": da,
                "fallback_1cm_1deg_pass": dt <= 0.01 and da <= 1.0,
                "target_1mm_0.1deg_pass": dt <= 0.001 and da <= 0.1,
            }
        else:
            cross_validation[sensor] = {
                "status": "not_independently_observable_in_both_parts",
                "parts_with_evidence": parts,
                "fallback_1cm_1deg_pass": False,
                "target_1mm_0.1deg_pass": False,
            }

    final_metrics = summary_metrics(active_planes, active_pairs, consensus)
    sensor_status = {}
    for sensor in OPTIMIZED:
        stab = stability[sensor]
        cv = cross_validation[sensor]
        enough = support[sensor]["capture_count"] >= 8
        fallback = (
            enough
            and stab["translation_p95_m"] <= 0.01
            and stab["angle_p95_deg"] <= 1.0
            and bool(cv["fallback_1cm_1deg_pass"])
        )
        target = (
            fallback
            and stab["translation_p95_m"] <= 0.001
            and stab["angle_p95_deg"] <= 0.1
            and bool(cv["target_1mm_0.1deg_pass"])
        )
        sensor_status[sensor] = {
            "status": "target_repeatability"
            if target
            else "fallback_1cm_1deg"
            if fallback
            else "reject_deployment",
            "support": support[sensor],
            "resample_stability": stab,
            "part1_part2_cross_validation": cv,
        }

    deploy_allowed = all(value["status"] != "reject_deployment" for value in sensor_status.values())
    transforms_out = {}
    for sensor in OPTIMIZED:
        frame = LIDAR_FRAMES.get(sensor, f"{sensor}/camera_link")
        matrix = consensus[sensor]
        if sensor in CAMERAS:
            matrix = matrix @ np.linalg.inv(CAMERA_OPTICAL_TO_LINK)
        transforms_out[frame] = transform_item(
            matrix, "part12_fixed_board_moving_vehicle_consensus"
        )
        transforms_out[frame]["calibration_status"] = sensor_status[sensor]["status"]

    result = {
        "schema_version": 1,
        "reference_frame": "velodyne_top_base_link",
        "coordinate_convention": "x_forward_y_left_z_up; camera output is camera_link",
        "method": "fixed_four_boards_moving_vehicle_joint_plane_and_camera_graph",
        "rounds_completed": len(round_rows),
        "deploy_allowed": deploy_allowed,
        "deployment_action": "none_review_only"
        if not deploy_allowed
        else "candidate_only_requires_manual_atomic_backup",
        "metrics": final_metrics,
        "sensor_status": sensor_status,
        "transforms": transforms_out,
        "rounds": round_rows,
    }
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(yaml.safe_dump(result, allow_unicode=True, sort_keys=False), encoding="utf-8")
    return result


def main() -> int:
    default_base = Path(__file__).resolve().parents[2]
    parser = argparse.ArgumentParser()
    parser.add_argument("--base", type=Path, default=default_base)
    parser.add_argument(
        "--config", type=Path, default=default_base / "process/joint_extrinsic_station/config.yaml"
    )
    parser.add_argument(
        "--seed",
        type=Path,
        default=default_base / "result/extrinsics/all_sensors_relative_to_roof_lidar.yaml",
    )
    parser.add_argument(
        "--observations",
        type=Path,
        default=default_base / "result/part12_20260903/observations.json",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=default_base / "result/part12_20260903/part12_joint_extrinsics.yaml",
    )
    parser.add_argument(
        "--rounds-log",
        type=Path,
        default=default_base / "process/part12_joint_extrinsics/ITERATION_LOG.md",
    )
    parser.add_argument("--max-sync-ms", type=float, default=100.0)
    parser.add_argument("--max-rounds", type=int, default=200)
    parser.add_argument("--extract", action="store_true")
    args = parser.parse_args()

    manifests = args.base / "data/backup_manifests/20260903"
    if args.extract or not args.observations.exists():
        payload = extract(
            args.base,
            args.config,
            args.seed,
            [manifests / "part1_capture_groups.txt", manifests / "part2_capture_groups.txt"],
            args.observations,
            args.max_sync_ms,
        )
    else:
        payload = json.loads(args.observations.read_text(encoding="utf-8"))
    seed = load_seed(args.seed)
    result = optimize(payload, seed, args.output, args.rounds_log, args.max_rounds)
    print(
        json.dumps(
            {
                "stage": "complete",
                "rounds": result["rounds_completed"],
                "deploy_allowed": result["deploy_allowed"],
                "output": str(args.output),
            },
            ensure_ascii=False,
        ),
        flush=True,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
