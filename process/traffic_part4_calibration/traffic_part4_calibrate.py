#!/usr/bin/env python3
"""Audit and calibrate the 2026-09-03 Part-4 traffic-light camera set.

The saved PNG and PCD bytes are the authority.  Live-view detection metadata is
used only for timestamp auditing.  The output is review-only; this program never
edits a vehicle or Autoware installation.
"""

from __future__ import annotations

import argparse
from collections import Counter
import csv
import hashlib
import json
import math
from pathlib import Path
import sys
from typing import Any

import cv2
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
import yaml


HERE = Path(__file__).resolve().parent
JOINT = HERE.parent / "joint_extrinsic_station"
if str(JOINT) not in sys.path:
    sys.path.insert(0, str(JOINT))

from calibrate_lidar_heights import read_binary_pcd_xyz  # noqa: E402
from extract_camera_pair_constraints import solve_group  # noqa: E402
from refine_camera_extrinsics import (  # noqa: E402
    fit_board_near_prediction,
    voxel,
)
from solve_joint_extrinsics import (  # noqa: E402
    CAMERA_OPTICAL_TO_LINK,
    absolute_transforms,
    load_intrinsics,
    make_detector,
)


CAMERA = "traffic_light_front"
TOP = "lidar_top"
LIDARS = ("lidar_top", "lidar_front_left", "lidar_front_right", "lidar_rear")
LIDAR_FRAMES = {
    "lidar_front_left": "velodyne_front_left_base_link",
    "lidar_front_right": "velodyne_front_right_base_link",
    "lidar_rear": "velodyne_rear_base_link",
}


def vec(value: np.ndarray) -> list[float]:
    return [float(x) for x in np.asarray(value).reshape(-1)]


def normalize(value: np.ndarray) -> np.ndarray:
    return value / max(float(np.linalg.norm(value)), 1e-12)


def angle_deg(left: np.ndarray, right: np.ndarray) -> float:
    cosine = abs(float(normalize(left) @ normalize(right)))
    return math.degrees(math.acos(float(np.clip(cosine, -1.0, 1.0))))


def planes_agree(left: dict[str, Any], right: dict[str, Any]) -> bool:
    nl = normalize(np.asarray(left["lidar_normal"], dtype=float))
    nr = normalize(np.asarray(right["lidar_normal"], dtype=float))
    if nl @ nr < 0:
        nr = -nr
    cl = np.asarray(left["lidar_center"], dtype=float)
    cr = np.asarray(right["lidar_center"], dtype=float)
    return (
        angle_deg(nl, nr) <= 2.5
        and abs(float(nl @ (cr - cl))) <= 0.08
        and abs(float(nr @ (cl - cr))) <= 0.08
    )


def consensus_plane(
    values: list[dict[str, Any]], predicted_center: np.ndarray
) -> dict[str, Any] | None:
    if not values:
        return None
    neighbourhoods = []
    for candidate in values:
        neighbourhoods.append([item for item in values if planes_agree(candidate, item)])
    cluster = max(neighbourhoods, key=len)
    # A single roof-lidar dense fit is admissible; a sparse equation or any
    # lower-lidar-only result must be independently seen by a second lidar.
    if len(cluster) == 1:
        only = cluster[0]
        if only["lidar"] != TOP or only.get("sparse_plane_equation_only"):
            return None
        only = dict(only)
        only["supporting_lidars"] = [TOP]
        return only
    reference = normalize(np.asarray(cluster[0]["lidar_normal"], dtype=float))
    normals = []
    offsets = []
    for item in cluster:
        normal = normalize(np.asarray(item["lidar_normal"], dtype=float))
        if normal @ reference < 0:
            normal = -normal
        normals.append(normal)
        offsets.append(float(normal @ np.asarray(item["lidar_center"], dtype=float)))
    normal = normalize(np.mean(normals, axis=0))
    offset = float(np.median(offsets))
    center = predicted_center + normal * (offset - float(normal @ predicted_center))
    sync_values = [
        item["camera_lidar_source_gap_ms"]
        for item in cluster
        if item["camera_lidar_source_gap_ms"] is not None
    ]
    return {
        "lidar": "lidar_consensus",
        "supporting_lidars": sorted(item["lidar"] for item in cluster),
        "camera_lidar_source_gap_ms": max(sync_values) if sync_values else None,
        "lidar_center": vec(center),
        "lidar_normal": vec(normal),
        "lidar_points": int(sum(item["lidar_points"] for item in cluster)),
        "lidar_rms_m": float(max(item["lidar_rms_m"] for item in cluster)),
        "lidar_extent_m": [None, None],
        "seed_center_gap_m": float(np.linalg.norm(center - predicted_center)),
        "seed_plane_gap_m": abs(float(normal @ (predicted_center - center))),
        "sparse_plane_equation_only": any(
            item.get("sparse_plane_equation_only") for item in cluster
        ),
    }


def transform_summary(matrix: np.ndarray) -> dict[str, Any]:
    return {
        "xyz_m": np.round(matrix[:3, 3], 9).tolist(),
        "rpy_deg": np.round(
            Rotation.from_matrix(matrix[:3, :3]).as_euler("xyz", degrees=True), 8
        ).tolist(),
        "matrix_row_major": np.round(matrix, 12).tolist(),
    }


def transform_delta(left: np.ndarray, right: np.ndarray) -> tuple[float, float]:
    translation = float(np.linalg.norm(left[:3, 3] - right[:3, 3]))
    rotation = Rotation.from_matrix(left[:3, :3] @ right[:3, :3].T)
    return translation, math.degrees(rotation.magnitude())


def pcd_path(capture: Path, name: str) -> Path:
    paths = list((capture / "lidars").glob(f"*_{name}.pcd"))
    if len(paths) != 1:
        raise RuntimeError(f"{capture.name}: expected one {name} PCD")
    return paths[0]


def part4_ids(path: Path) -> list[str]:
    values = [
        line.strip() for line in path.read_text(encoding="utf-8").splitlines() if line.strip()
    ]
    if len(values) != 45:
        raise ValueError(f"expected 45 Part-4 captures, got {len(values)}")
    return values


def image_difference(left: np.ndarray, right: np.ndarray) -> tuple[float, float]:
    a = cv2.resize(left, (160, 120), interpolation=cv2.INTER_AREA).astype(np.int16)
    b = cv2.resize(right, (160, 120), interpolation=cv2.INTER_AREA).astype(np.int16)
    difference = np.abs(a - b)
    return float(np.mean(difference)), float(np.mean(np.max(difference, axis=2) > 6))


def detector_result(
    image: np.ndarray,
    detector: Any,
    board: dict[str, Any],
    intrinsic: tuple[np.ndarray, np.ndarray, str],
) -> dict[str, Any]:
    corners, ids = detector(cv2.cvtColor(image, cv2.COLOR_BGR2GRAY))
    if ids is None:
        return {"status": "no_tags", "tag_count": 0}
    ids = np.asarray(ids, dtype=np.int32).reshape(-1)
    corners = [np.asarray(corner, dtype=np.float32).reshape(1, 4, 2) for corner in corners]
    first = int(board.get("first_id", 0))
    last = first + int(board["columns"]) * int(board["rows"])
    valid = [index for index, marker_id in enumerate(ids) if first <= int(marker_id) < last]
    if len(valid) < 3:
        return {"status": "too_few_tags", "tag_count": len(valid)}
    corners = [corners[index] for index in valid]
    ids = ids[valid]
    # Near a cropped image boundary OpenCV can occasionally return a small
    # false positive carrying an ID that is already present on the real grid.
    # Keep the larger quadrilateral for each ID; one physical grid cannot
    # legitimately contain duplicate IDs.
    best_by_id: dict[int, tuple[float, np.ndarray]] = {}
    for marker_id, corner in zip(ids.tolist(), corners):
        polygon = np.asarray(corner).reshape(4, 2)
        perimeter = float(np.sum(np.linalg.norm(polygon - np.roll(polygon, 1, axis=0), axis=1)))
        if marker_id not in best_by_id or perimeter > best_by_id[marker_id][0]:
            best_by_id[int(marker_id)] = (perimeter, corner)
    ids = np.asarray(sorted(best_by_id), dtype=np.int32)
    corners = [best_by_id[int(marker_id)][1] for marker_id in ids]
    # This dedicated Part-4 capture contains exactly one physical AprilGrid.
    solved = solve_group(corners, ids, list(range(len(ids))), board, *intrinsic)
    if solved is None:
        return {"status": "pnp_rejected", "tag_count": len(set(ids.tolist()))}
    transform, tag_ids, _object_centres, _normalised_centres, rms = solved
    return {
        "status": "accepted",
        "tag_count": len(tag_ids),
        "tag_ids": list(tag_ids),
        "camera_from_board": transform,
        "reprojection_rms_px": float(rms),
    }


def load_cloud(path: Path, top_from_lidar: np.ndarray) -> np.ndarray:
    points = read_binary_pcd_xyz(path)
    finite = np.isfinite(points).all(axis=1)
    points = points[finite]
    points = points @ top_from_lidar[:3, :3].T + top_from_lidar[:3, 3]
    radius = np.linalg.norm(points[:, :2], axis=1)
    points = points[
        (radius >= 0.5) & (radius <= 14.0) & (points[:, 2] >= -2.0) & (points[:, 2] <= 1.5)
    ]
    return voxel(points, 0.022)


def sparse_plane_near_prediction(
    points: np.ndarray,
    predicted_center: np.ndarray,
    predicted_normal: np.ndarray,
    seed: int,
    diagnostics: dict[str, Any],
) -> tuple[np.ndarray, np.ndarray, dict[str, Any]] | None:
    """Fallback for distant 32-line/BPearl returns with fewer than 22 hits.

    Only a plane equation is claimed.  Its returned centre is the camera
    prediction projected onto that plane, so no false tangent-axis centre is
    inferred from two or three sparse lidar rings.
    """
    normal_seed = normalize(predicted_normal)
    vertical = np.array([0.0, 0.0, 1.0])
    vertical -= normal_seed * float(vertical @ normal_seed)
    if np.linalg.norm(vertical) < 0.25:
        vertical = np.array([0.0, 1.0, 0.0])
    vertical = normalize(vertical)
    horizontal = normalize(np.cross(vertical, normal_seed))
    delta = points - predicted_center
    coordinates = np.column_stack((delta @ horizontal, delta @ vertical, delta @ normal_seed))
    candidates = points[
        (np.abs(coordinates[:, 0]) <= 0.72)
        & (np.abs(coordinates[:, 1]) <= 0.72)
        & (np.abs(coordinates[:, 2]) <= 1.15)
    ]
    diagnostics["sparse_roi_points"] = len(candidates)
    if len(candidates) < 9:
        diagnostics["sparse_rejection"] = "roi_points_lt_9"
        return None
    rng = np.random.default_rng(seed ^ 0xA5A55A5A)
    best = None
    threshold = 0.028
    for _ in range(500):
        triple = candidates[rng.choice(len(candidates), 3, replace=False)]
        normal = np.cross(triple[1] - triple[0], triple[2] - triple[0])
        if np.linalg.norm(normal) < 1e-8:
            continue
        normal = normalize(normal)
        if normal @ normal_seed < 0:
            normal = -normal
        normal_angle = angle_deg(normal, normal_seed)
        if normal_angle > 25.0:
            continue
        offset = -float(normal @ triple[0])
        prediction_offset = abs(float(normal @ predicted_center + offset))
        if prediction_offset > 1.05:
            continue
        mask = np.abs(candidates @ normal + offset) <= threshold
        count = int(np.count_nonzero(mask))
        if count < 8:
            continue
        inliers = candidates[mask]
        tangent = np.column_stack(
            ((inliers - predicted_center) @ horizontal, (inliers - predicted_center) @ vertical)
        )
        extent = np.diff(np.percentile(tangent, [3.0, 97.0], axis=0), axis=0)[0]
        if max(extent) > 1.32 or max(extent) < 0.22:
            continue
        occupancy = len(np.unique(np.floor(tangent / 0.09).astype(int), axis=0))
        score = count + 1.5 * occupancy - 3.0 * prediction_offset - 0.08 * normal_angle
        if best is None or score > best[0]:
            best = score, normal, offset, mask
    if best is None:
        diagnostics["sparse_rejection"] = "no_sparse_board_plane"
        return None
    _score, normal, offset, mask = best
    inliers = candidates[mask]
    if len(inliers) >= 12:
        center_raw = np.mean(inliers, axis=0)
        _u, _s, vt = np.linalg.svd(inliers - center_raw, full_matrices=False)
        fitted = vt[-1]
        if fitted @ normal_seed < 0:
            fitted = -fitted
        if angle_deg(fitted, normal_seed) <= 25.0:
            normal = fitted
            offset = -float(normal @ center_raw)
            inliers = candidates[np.abs(candidates @ normal + offset) <= threshold]
    center = predicted_center - normal * float(normal @ predicted_center + offset)
    tangent = np.column_stack(((inliers - center) @ horizontal, (inliers - center) @ vertical))
    extent = np.diff(np.percentile(tangent, [3.0, 97.0], axis=0), axis=0)[0]
    rms = float(np.sqrt(np.mean(np.square(inliers @ normal + offset))))
    return (
        center,
        normal,
        {
            "points": len(inliers),
            "rms_m": rms,
            "extent_m": extent,
            "prediction_error_m": float(np.linalg.norm(center - predicted_center)),
            "sparse_plane_equation_only": True,
        },
    )


def initial_top_optical(path: Path) -> np.ndarray:
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    absolute = absolute_transforms(data)
    top_from_kit = np.linalg.inv(absolute["velodyne_top_base_link"])
    return top_from_kit @ absolute[f"{CAMERA}/camera_link"] @ CAMERA_OPTICAL_TO_LINK


def lidar_anchors(path: Path) -> dict[str, np.ndarray]:
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    output = {TOP: np.eye(4)}
    for sensor, frame in LIDAR_FRAMES.items():
        item = data["transforms"][frame]
        if item["parent"] != "velodyne_top_base_link":
            raise ValueError(f"{frame} is not relative to roof lidar")
        matrix = np.eye(4)
        matrix[:3, :3] = Rotation.from_euler("xyz", item["rpy_deg"], degrees=True).as_matrix()
        matrix[:3, 3] = np.asarray(item["xyz_m"], dtype=float)
        output[sensor] = matrix
    return output


def extract(args: argparse.Namespace) -> tuple[list[dict[str, Any]], dict[str, Any]]:
    ids = part4_ids(args.part4_list)
    config = yaml.safe_load(args.config.read_text(encoding="utf-8"))
    board = config["tag_board"]
    intrinsic = load_intrinsics(args.intrinsic)
    detector = make_detector(board["dictionary"])
    seed = initial_top_optical(args.initial)
    anchors = lidar_anchors(args.lidar_anchors)
    rows: list[dict[str, Any]] = []
    hashes: Counter[str] = Counter()
    previous_image: np.ndarray | None = None
    source_stamps: list[int] = []
    sequences: list[int] = []

    for index, capture_id in enumerate(ids):
        capture = args.data_root / capture_id
        manifest = json.loads((capture / "manifest.json").read_text(encoding="utf-8"))
        camera_meta = manifest.get("sensors", {}).get(CAMERA, {})
        top_meta = manifest.get("sensors", {}).get(TOP, {})
        relative = camera_meta.get("file")
        row: dict[str, Any] = {"capture_id": capture_id, "index": index}
        if not relative:
            row["status"] = "missing_camera_file_in_manifest"
            rows.append(row)
            continue
        image_path = capture / relative
        payload = image_path.read_bytes()
        digest = hashlib.sha256(payload).hexdigest()
        hashes[digest] += 1
        image = cv2.imdecode(np.frombuffer(payload, dtype=np.uint8), cv2.IMREAD_COLOR)
        if image is None:
            row["status"] = "image_decode_failed"
            rows.append(row)
            continue
        row.update(
            {
                "image_file": relative,
                "image_sha256": digest,
                "width": int(image.shape[1]),
                "height": int(image.shape[0]),
            }
        )
        if previous_image is not None:
            mae, changed = image_difference(previous_image, image)
            row.update(
                {
                    "previous_frame_mae": mae,
                    "previous_frame_changed_fraction": changed,
                    "near_same_as_previous": mae < 0.8 and changed < 0.01,
                }
            )
        previous_image = image

        capture_ns = int(manifest["capture_unix_ns"])
        source_ns = int(camera_meta.get("source_stamp_ns", 0))
        received_ns = int(camera_meta.get("received_unix_ns", 0))
        top_ns = int(top_meta.get("source_stamp_ns", 0) or top_meta.get("received_unix_ns", 0))
        row.update(
            {
                "capture_minus_camera_source_ms": (capture_ns - source_ns) / 1e6,
                "receive_minus_camera_source_ms": (received_ns - source_ns) / 1e6,
                "camera_top_source_gap_ms": abs(source_ns - top_ns) / 1e6,
                "camera_source_stamp_ns": source_ns,
                "camera_receive_stamp_ns": received_ns,
                "top_source_stamp_ns": top_ns,
            }
        )
        if source_ns:
            source_stamps.append(source_ns)
        sequence = camera_meta.get("detection", {}).get("sample_sequence")
        if sequence is not None:
            row["sample_sequence"] = int(sequence)
            sequences.append(int(sequence))

        detected = detector_result(image, detector, board, intrinsic)
        row.update({key: value for key, value in detected.items() if key != "camera_from_board"})
        if detected["status"] != "accepted":
            rows.append(row)
            continue
        camera_from_board = detected["camera_from_board"]
        camera_center = camera_from_board[:3, 3]
        camera_normal = normalize(camera_from_board[:3, 2])
        predicted_center = seed[:3, :3] @ camera_center + seed[:3, 3]
        predicted_normal = normalize(seed[:3, :3] @ camera_normal)
        row.update(
            {
                "predicted_center_top": vec(predicted_center),
                "predicted_normal_top": vec(predicted_normal),
                "camera_center": vec(camera_center),
                "camera_normal": vec(camera_normal),
            }
        )
        lidar_values = []
        lidar_status = {}
        lidar_diagnostics = {}
        for lidar in LIDARS:
            lidar_meta = manifest.get("sensors", {}).get(lidar, {})
            lidar_ns = int(
                lidar_meta.get("source_stamp_ns", 0) or lidar_meta.get("received_unix_ns", 0)
            )
            sync_ms = abs(source_ns - lidar_ns) / 1e6 if source_ns and lidar_ns else None
            try:
                cloud = load_cloud(pcd_path(capture, lidar), anchors[lidar])
            except (OSError, ValueError, RuntimeError) as error:
                lidar_status[lidar] = f"cloud_error:{error}"
                continue
            search: dict[str, Any] = {}
            rng_seed = int.from_bytes(
                hashlib.sha256(f"traffic-part4:{capture_id}:{lidar}".encode()).digest()[:4],
                "little",
            )
            fitted = fit_board_near_prediction(
                cloud,
                predicted_center,
                predicted_normal,
                rng_seed,
                search,
                ground_z=-1.72817,
                min_vertical_extent_m=0.12,
                max_prediction_error_m=0.52,
            )
            if fitted is None:
                fitted = sparse_plane_near_prediction(
                    cloud, predicted_center, predicted_normal, rng_seed, search
                )
            lidar_diagnostics[lidar] = {
                key: value.tolist() if isinstance(value, np.ndarray) else value
                for key, value in search.items()
            }
            if fitted is None:
                lidar_status[lidar] = "board_not_found"
                continue
            lidar_center, lidar_normal, quality = fitted
            normal_gap = angle_deg(predicted_normal, lidar_normal)
            center_gap = float(np.linalg.norm(predicted_center - lidar_center))
            item = {
                "lidar": lidar,
                "camera_lidar_source_gap_ms": sync_ms,
                "lidar_center": vec(lidar_center),
                "lidar_normal": vec(normalize(lidar_normal)),
                "lidar_points": int(quality["points"]),
                "lidar_rms_m": float(quality["rms_m"]),
                "lidar_extent_m": vec(quality["extent_m"]),
                "seed_center_gap_m": center_gap,
                "seed_normal_gap_deg": normal_gap,
                "search": {
                    key: value.tolist() if isinstance(value, np.ndarray) else value
                    for key, value in search.items()
                },
                "sparse_plane_equation_only": bool(
                    quality.get("sparse_plane_equation_only", False)
                ),
            }
            plane_gap = abs(float(normalize(lidar_normal) @ (predicted_center - lidar_center)))
            item["seed_plane_gap_m"] = plane_gap
            minimum_points = 8 if quality.get("sparse_plane_equation_only") else 22
            if (
                plane_gap > 0.85
                or normal_gap > 12.0
                or float(quality["rms_m"]) > 0.025
                or int(quality["points"]) < minimum_points
            ):
                lidar_status[lidar] = "geometry_gate_rejected"
            else:
                lidar_status[lidar] = "accepted"
                lidar_values.append(item)
        row["raw_lidar_observations"] = lidar_values
        consensus = consensus_plane(lidar_values, predicted_center)
        row["lidar_observations"] = [consensus] if consensus is not None else []
        row["lidar_status"] = lidar_status
        row["lidar_diagnostics"] = lidar_diagnostics
        row["status"] = "accepted_camera_lidar_plane" if lidar_values else "no_lidar_board_accepted"
        rows.append(row)
        print(
            json.dumps(
                {
                    "stage": "extract",
                    "capture": index + 1,
                    "total": len(ids),
                    "status": row["status"],
                },
                ensure_ascii=False,
            ),
            flush=True,
        )

    freshness = [
        row["capture_minus_camera_source_ms"]
        for row in rows
        if "capture_minus_camera_source_ms" in row
    ]
    receive_age = [
        row["receive_minus_camera_source_ms"]
        for row in rows
        if "receive_minus_camera_source_ms" in row
    ]
    sync = [row["camera_top_source_gap_ms"] for row in rows if "camera_top_source_gap_ms" in row]
    differences = [row["previous_frame_mae"] for row in rows if "previous_frame_mae" in row]
    summary = {
        "capture_count": len(rows),
        "decoded_320x240_count": sum(
            row.get("width") == 320 and row.get("height") == 240 for row in rows
        ),
        "exact_duplicate_frames": int(sum(count - 1 for count in hashes.values())),
        "unique_frame_hashes": len(hashes),
        "near_same_consecutive_transitions": sum(
            bool(row.get("near_same_as_previous")) for row in rows
        ),
        "consecutive_mae_min": min(differences) if differences else None,
        "consecutive_mae_median": float(np.median(differences)) if differences else None,
        "freshness_capture_source_ms": stats(freshness),
        "transport_receive_source_ms": stats(receive_age),
        "camera_top_source_gap_ms": stats(sync),
        "source_stamp_strictly_increasing": all(
            b > a for a, b in zip(source_stamps, source_stamps[1:])
        ),
        "sample_sequence_strictly_increasing": all(b > a for a, b in zip(sequences, sequences[1:])),
        "fresh_pixel_tag_detection_count": sum(row.get("tag_count", 0) >= 3 for row in rows),
        "fresh_pixel_pnp_count": sum("reprojection_rms_px" in row for row in rows),
        "camera_lidar_plane_count": sum(len(row.get("lidar_observations", [])) for row in rows),
        "camera_lidar_capture_count": sum(bool(row.get("lidar_observations")) for row in rows),
        "plane_count_by_lidar": dict(
            Counter(item["lidar"] for row in rows for item in row.get("raw_lidar_observations", []))
        ),
        "status_counts": dict(Counter(str(row.get("status")) for row in rows)),
    }
    reprojection = [row["reprojection_rms_px"] for row in rows if "reprojection_rms_px" in row]
    summary["fresh_pixel_pnp_reprojection_px"] = stats(reprojection)
    return rows, summary


def stats(values: list[float]) -> dict[str, float | int | None]:
    if not values:
        return {"count": 0, "min": None, "median": None, "p95": None, "max": None}
    array = np.asarray(values, dtype=float)
    return {
        "count": len(values),
        "min": float(np.min(array)),
        "median": float(np.median(array)),
        "p95": float(np.percentile(array, 95)),
        "max": float(np.max(array)),
    }


def accepted(rows: list[dict[str, Any]]) -> list[dict[str, Any]]:
    result = []
    for row in rows:
        for lidar in row.get("lidar_observations", []):
            item = {
                key: value
                for key, value in row.items()
                if key
                not in (
                    "lidar_observations",
                    "lidar_status",
                    "lidar_diagnostics",
                    "raw_lidar_observations",
                )
            }
            item.update(lidar)
            result.append(item)
    return result


def residual_errors(
    rows: list[dict[str, Any]], transform: np.ndarray
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    plane, angles, centres = [], [], []
    for row in rows:
        cc = np.asarray(row["camera_center"], dtype=float)
        nc = np.asarray(row["camera_normal"], dtype=float)
        cl = np.asarray(row["lidar_center"], dtype=float)
        nl = np.asarray(row["lidar_normal"], dtype=float)
        pc = transform[:3, :3] @ cc + transform[:3, 3]
        pn = normalize(transform[:3, :3] @ nc)
        if pn @ nl < 0:
            nl = -nl
        plane.append(abs(float(nl @ (pc - cl))))
        angles.append(angle_deg(pn, nl))
        centres.append(float(np.linalg.norm(pc - cl)))
    return np.asarray(plane), np.asarray(angles), np.asarray(centres)


def solve(rows: list[dict[str, Any]], seed: np.ndarray, use_centre: bool = True) -> np.ndarray:
    if len(rows) < 6:
        raise RuntimeError(f"need at least 6 observations, got {len(rows)}")
    x0 = np.r_[Rotation.from_matrix(seed[:3, :3]).as_rotvec(), seed[:3, 3]]

    def residual(values: np.ndarray) -> np.ndarray:
        rotation = Rotation.from_rotvec(values[:3]).as_matrix()
        translation = values[3:]
        parts: list[float] = []
        for row in rows:
            cc = np.asarray(row["camera_center"], dtype=float)
            nc = np.asarray(row["camera_normal"], dtype=float)
            cl = np.asarray(row["lidar_center"], dtype=float)
            nl = np.asarray(row["lidar_normal"], dtype=float)
            pc = rotation @ cc + translation
            pn = normalize(rotation @ nc)
            if pn @ nl < 0:
                nl = -nl
            anchor_weight = 1.0 if row["lidar"] == TOP else 0.75
            weight = anchor_weight * min(1.5, math.sqrt(max(int(row["lidar_points"]), 22) / 50.0))
            parts.extend((weight * (pn - nl) / 0.035).tolist())
            parts.append(
                weight
                * float(nl @ (pc - cl))
                / max(0.012, min(0.025, 2.5 * float(row["lidar_rms_m"])))
            )
            # Sparse spinning-lidar rings do not determine the physical board
            # centre in its tangent plane.  Never optimise against that false
            # centre; plane equations across varied board poses constrain the
            # observable translation components.
        rotation_prior = Rotation.from_matrix(rotation @ seed[:3, :3].T).as_rotvec()
        parts.extend((rotation_prior / math.radians(4.0)).tolist())
        # Mechanical translation is deliberately weak except for camera height,
        # which is poorly observed when all boards are nearly vertical.
        parts.extend(((translation[:2] - seed[:2, 3]) / 0.20).tolist())
        parts.append(float((translation[2] - seed[2, 3]) / 0.060))
        return np.asarray(parts)

    result = least_squares(
        residual, x0, loss="soft_l1", f_scale=1.0, max_nfev=800, xtol=1e-11, ftol=1e-11, gtol=1e-11
    )
    if not result.success:
        raise RuntimeError(result.message)
    output = np.eye(4)
    output[:3, :3] = Rotation.from_rotvec(result.x[:3]).as_matrix()
    output[:3, 3] = result.x[3:]
    return output


def robust_fit(
    rows: list[dict[str, Any]], seed: np.ndarray
) -> tuple[np.ndarray, list[dict[str, Any]], dict[str, Any]]:
    current_rows = list(rows)
    history = []
    current = seed.copy()
    for iteration in range(4):
        current = solve(current_rows, current)
        plane, angle, centre = residual_errors(current_rows, current)
        plane_cut = max(
            0.035, float(np.median(plane) + 3.5 * np.median(np.abs(plane - np.median(plane))))
        )
        angle_cut = max(
            3.0, float(np.median(angle) + 3.5 * np.median(np.abs(angle - np.median(angle))))
        )
        centre_cut = max(
            0.12, float(np.median(centre) + 3.5 * np.median(np.abs(centre - np.median(centre))))
        )
        keep = (plane <= plane_cut) & (angle <= angle_cut) & (centre <= centre_cut)
        next_rows = [row for row, value in zip(current_rows, keep) if value]
        history.append(
            {
                "iteration": iteration + 1,
                "input": len(current_rows),
                "output": len(next_rows),
                "plane_cut_m": plane_cut,
                "angle_cut_deg": angle_cut,
                "centre_cut_m": centre_cut,
            }
        )
        if len(next_rows) < 8 or len(next_rows) == len(current_rows):
            break
        current_rows = next_rows
    return solve(current_rows, current), current_rows, {"rejection_history": history}


def metric_bundle(rows: list[dict[str, Any]], transform: np.ndarray) -> dict[str, Any]:
    plane, angle, centre = residual_errors(rows, transform)
    return {
        "count": len(rows),
        "capture_count": len({row["capture_id"] for row in rows}),
        "plane_distance_m": stats(plane.tolist()),
        "normal_angle_deg": stats(angle.tolist()),
        "board_center_distance_m": stats(centre.tolist()),
    }


def validate(rows: list[dict[str, Any]], seed: np.ndarray, full: np.ndarray) -> dict[str, Any]:
    folds = []
    ordered = sorted(rows, key=lambda row: int(row["index"]))
    for fold in range(5):
        test = [row for row in ordered if int(row["index"]) % 5 == fold]
        train = [row for row in ordered if int(row["index"]) % 5 != fold]
        candidate = solve(train, seed)
        folds.append(
            {
                "fold": fold + 1,
                "train_count": len(train),
                "test": metric_bundle(test, candidate),
                "transform": transform_summary(candidate),
                "difference_from_full": {
                    "translation_m": transform_delta(candidate, full)[0],
                    "angle_deg": transform_delta(candidate, full)[1],
                },
            }
        )
    bootstrap = []
    rng = np.random.default_rng(20260903)
    capture_ids = sorted({row["capture_id"] for row in rows})
    by_capture = {
        capture_id: [row for row in rows if row["capture_id"] == capture_id]
        for capture_id in capture_ids
    }
    for iteration in range(100):
        count = max(6, int(round(0.75 * len(capture_ids))))
        chosen = rng.choice(len(capture_ids), count, replace=True)
        sample = [row for index in chosen for row in by_capture[capture_ids[index]]]
        candidate = solve(sample, seed)
        delta = transform_delta(candidate, full)
        bootstrap.append(
            {
                "iteration": iteration + 1,
                "translation_from_full_m": delta[0],
                "angle_from_full_deg": delta[1],
            }
        )
    translation = [item["translation_from_full_m"] for item in bootstrap]
    angle = [item["angle_from_full_deg"] for item in bootstrap]
    normals = np.asarray([row["lidar_normal"] for row in rows], dtype=float)
    normal_singular = np.linalg.svd(normals, compute_uv=False).tolist()
    return {
        "five_fold_time_interleaved": folds,
        "bootstrap_100": bootstrap,
        "bootstrap_translation_m": stats(translation),
        "bootstrap_angle_deg": stats(angle),
        "normal_design_singular_values": normal_singular,
        "normal_design_condition": (normal_singular[0] / max(normal_singular[-1], 1e-12)),
    }


def write_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    serialised = []
    for row in rows:
        serialised.append(
            {
                key: json.dumps(value, ensure_ascii=False)
                if isinstance(value, (list, dict))
                else value
                for key, value in row.items()
            }
        )
    fields = sorted({key for row in serialised for key in row})
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(serialised)


def main() -> int:
    default_base = HERE.parents[1]
    parser = argparse.ArgumentParser()
    parser.add_argument("--base", type=Path, default=default_base)
    parser.add_argument("--data-root", type=Path, default=default_base / "data/raw_extrinsic_data")
    parser.add_argument(
        "--part4-list",
        type=Path,
        default=default_base / "data/backup_manifests/20260903/part4_capture_groups.txt",
    )
    parser.add_argument(
        "--config", type=Path, default=HERE.parent / "joint_extrinsic_station/config.yaml"
    )
    parser.add_argument(
        "--initial",
        type=Path,
        default=HERE.parent / "joint_extrinsic_station/initial_extrinsics_physical_v2.yaml",
    )
    parser.add_argument(
        "--intrinsic",
        type=Path,
        default=default_base / "result/camera_intrinsics/traffic_light_front.yaml",
    )
    parser.add_argument(
        "--lidar-anchors",
        type=Path,
        default=default_base / "result/extrinsics/all_sensors_relative_to_roof_lidar.yaml",
    )
    parser.add_argument(
        "--output", type=Path, default=default_base / "result/traffic_part4_20260903"
    )
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    rows, audit = extract(args)
    write_csv(args.output / "frame_audit.csv", rows)
    (args.output / "frame_audit.json").write_text(
        json.dumps({"summary": audit, "frames": rows}, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )
    observations = accepted(rows)
    seed = initial_top_optical(args.initial)
    report: dict[str, Any] = {
        "schema_version": 1,
        "status": "review_only_pending_fit",
        "deployment_action": "none",
        "reference_frame": "velodyne_top_base_link",
        "camera_frame": "traffic_light_front/camera_link",
        "coordinate_convention": "x_forward_y_left_z_up; output is camera_link",
        "data": {
            "part": 4,
            "capture_count": 45,
            "capture_range": ["20260903_160042_050", "20260903_160414_181"],
        },
        "time_and_freshness_audit": audit,
        "intrinsic_source": str(args.intrinsic),
        "lidar_anchor_source": str(args.lidar_anchors),
        "lidar_anchor_warning": "Three lower-lidar poses are frozen fallback_1cm_1deg anchors; final traffic-camera result inherits their uncertainty.",
        "mechanical_seed_top_from_camera_optical": transform_summary(seed),
    }
    if len(observations) >= 8:
        fitted, filtered, fit_info = robust_fit(observations, seed)
        link = fitted @ np.linalg.inv(CAMERA_OPTICAL_TO_LINK)
        validation = validate(filtered, seed, fitted)
        correction = transform_delta(fitted, seed)
        metrics = metric_bundle(filtered, fitted)
        reprojection = audit["fresh_pixel_pnp_reprojection_px"]
        held_plane_p95 = max(
            fold["test"]["plane_distance_m"]["p95"]
            for fold in validation["five_fold_time_interleaved"]
        )
        held_angle_p95 = max(
            fold["test"]["normal_angle_deg"]["p95"]
            for fold in validation["five_fold_time_interleaved"]
        )
        gates = {
            "all_45_images_fresh_unique": audit["unique_frame_hashes"] == 45
            and audit["near_same_consecutive_transitions"] == 0,
            "camera_lidar_time_gap_p95_le_250ms": audit["camera_top_source_gap_ms"]["p95"] <= 250.0,
            "fresh_pnp_count_ge_28": audit["fresh_pixel_pnp_count"] >= 28,
            "pnp_reprojection_p95_le_2p5px": reprojection["p95"] <= 2.5,
            "accepted_plane_count_ge_24": len(filtered) >= 24,
            "accepted_capture_count_ge_16": len({row["capture_id"] for row in filtered}) >= 16,
            "heldout_plane_p95_le_50mm": held_plane_p95 <= 0.050,
            "heldout_normal_p95_le_2deg": held_angle_p95 <= 2.0,
            "bootstrap_translation_p95_le_10mm": validation["bootstrap_translation_m"]["p95"]
            <= 0.010,
            "bootstrap_angle_p95_le_0p25deg": validation["bootstrap_angle_deg"]["p95"] <= 0.25,
            "mechanical_seed_correction_le_0p20m_5deg": correction[0] <= 0.20
            and correction[1] <= 5.0,
        }
        deploy = all(gates.values())
        report.update(
            {
                "status": "accepted_for_vehicle_integration"
                if deploy
                else "rejected_for_vehicle_integration",
                "deployment_action": "eligible_after_backup" if deploy else "none",
                "observations_before_filter": len(observations),
                "observations_after_filter": len(filtered),
                "fit": fit_info,
                "metrics": metrics,
                "validation": validation,
                "correction_from_mechanical_seed": {
                    "translation_m": correction[0],
                    "angle_deg": correction[1],
                },
                "transform_top_from_camera_optical": transform_summary(fitted),
                "transform_top_from_camera_link": transform_summary(link),
                "quality_gates": gates,
                "failed_quality_gates": [name for name, value in gates.items() if not value],
            }
        )
    else:
        report.update(
            {
                "status": "rejected_for_vehicle_integration",
                "rejection_reason": f"only {len(observations)} accepted top-plane observations",
            }
        )
    (args.output / "traffic_part4_result.yaml").write_text(
        yaml.safe_dump(report, allow_unicode=True, sort_keys=False), encoding="utf-8"
    )
    (args.output / "traffic_part4_result.json").write_text(
        json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    print(
        json.dumps(
            {
                "stage": "complete",
                "status": report["status"],
                "planes": len(observations),
                "captures": len({row["capture_id"] for row in observations}),
            },
            ensure_ascii=False,
        ),
        flush=True,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
