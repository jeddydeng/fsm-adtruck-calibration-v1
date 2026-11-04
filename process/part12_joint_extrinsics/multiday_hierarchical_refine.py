#!/usr/bin/env python3
"""Hierarchical multi-day extrinsic refinement for the white vehicle.

The September 2 data moved one board while the vehicle stayed still.  The
September 3 Part 1/2 data kept four boards fixed and moved the vehicle.  This
solver keeps the already repeatable lidars and side cameras as immutable
anchors, then refines only the four historically weak cameras.  Raw captures
are read-only; all derived observations and decisions are written separately.

The numerical loop is a deterministic blocked bootstrap.  It may stop early
at 1 mm / 0.1 degree.  After 200 rounds only the still-weak nodes may use the
documented 1 cm / 1 degree fallback.  Five hundred rounds is a hard ceiling,
not a target.
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
from scipy.spatial.transform import Rotation
import yaml


HERE = Path(__file__).resolve().parent
JOINT = HERE.parent / "joint_extrinsic_station"
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))
if str(JOINT) not in sys.path:
    sys.path.insert(0, str(JOINT))

from part12_joint_calibrate import (  # noqa: E402
    CAMERAS,
    CAMERA_OPTICAL_TO_LINK,
    LIDARS,
    angle_deg,
    load_seed,
    normalized,
    pair_error,
    plane_error,
    sensor_stamp,
    transform_distance,
    transform_item,
)
from extract_camera_pair_constraints import (  # noqa: E402
    BoardPose,
    detect_saved_frame,
    observations_for_capture,
    pair_observations,
)
from solve_joint_extrinsics import (  # noqa: E402
    grid_marker_corners,
    load_intrinsics,
    split_marker_groups,
)


ANCHORS = (
    "lidar_front_left",
    "lidar_front_right",
    "lidar_rear",
    "video_front_left",
    "video_front_right",
    "video_rear_left",
    "video_rear_right",
    "fisheye_left",
    "fisheye_right",
)
WEAK = ("video_front_mid", "video_rear_mid", "fisheye_front", "fisheye_rear")
ALL = ("lidar_top",) + ANCHORS + WEAK


def parse_vector(value: str) -> np.ndarray:
    return np.fromstring(value, sep=" ", dtype=float)


def matrix_from_rpy_xyz(xyz: str, rpy: str) -> np.ndarray:
    matrix = np.eye(4)
    matrix[:3, :3] = Rotation.from_euler("xyz", parse_vector(rpy), degrees=True).as_matrix()
    matrix[:3, 3] = parse_vector(xyz)
    return matrix


def load_capture_intrinsics(directory: Path) -> dict[str, Any]:
    """Use the intrinsics saved with a capture, including old 640x480 fish."""
    values = {}
    for camera in CAMERAS:
        path = directory / "intrinsics" / f"{camera}.yaml"
        if not path.is_file():
            raise FileNotFoundError(path)
        values[camera] = load_intrinsics(path)
    return values


def solve_tag_centres(
    corners: list[np.ndarray],
    ids: np.ndarray,
    group: list[int],
    board: dict[str, Any],
    camera: np.ndarray,
    distortion: np.ndarray,
    model: str,
) -> tuple[np.ndarray, tuple[int, ...], np.ndarray, np.ndarray, float] | None:
    """Fallback PnP for the 2026-09-02 print/corner-order convention.

    The old captures have sub-pixel tag-centre geometry, but the four decoded
    AprilTag corner orders do not match the current board generator's corner
    winding.  Fitting all corners therefore creates a false 16--25 px error.
    Tag centres are invariant to that winding and retain the full 6x5 planar
    geometry; this reproduces the historical sub-pixel solution without using
    cached UI poses.
    """
    selected_ids = [int(ids[index]) for index in group]
    if len(set(selected_ids)) < 4 or len(set(selected_ids)) != len(selected_ids):
        return None
    objects = np.asarray(
        [grid_marker_corners(int(ids[index]), board).mean(axis=0) for index in group],
        dtype=np.float64,
    )
    pixels = np.asarray(
        [np.asarray(corners[index]).reshape(4, 2).mean(axis=0) for index in group], dtype=np.float64
    )
    if model == "equidistant":
        normalized_pixels = (
            __import__("cv2")
            .fisheye.undistortPoints(pixels.reshape(-1, 1, 2), camera, distortion)
            .reshape(-1, 2)
        )
    else:
        normalized_pixels = (
            __import__("cv2")
            .undistortPoints(pixels.reshape(-1, 1, 2), camera, distortion)
            .reshape(-1, 2)
        )
    cv2 = __import__("cv2")
    ok, rvec, translation, inliers = cv2.solvePnPRansac(
        objects,
        normalized_pixels,
        np.eye(3),
        None,
        iterationsCount=500,
        reprojectionError=0.008,
        confidence=0.999,
        flags=cv2.SOLVEPNP_ITERATIVE,
    )
    if not ok or inliers is None or len(inliers) < max(4, int(0.60 * len(objects))):
        return None
    ok, rvec, translation = cv2.solvePnP(
        objects[inliers.ravel()],
        normalized_pixels[inliers.ravel()],
        np.eye(3),
        None,
        rvec,
        translation,
        True,
        flags=cv2.SOLVEPNP_ITERATIVE,
    )
    if not ok or float(translation[2, 0]) <= 0:
        return None
    if model == "equidistant":
        projected, _ = cv2.fisheye.projectPoints(
            objects.reshape(-1, 1, 3), rvec, translation, camera, distortion
        )
    else:
        projected, _ = cv2.projectPoints(objects, rvec, translation, camera, distortion)
    errors = np.linalg.norm(projected.reshape(-1, 2) - pixels, axis=1)
    rms = float(np.sqrt(np.mean(np.square(errors))))
    if not np.isfinite(rms) or rms > 2.5:
        return None
    transform = np.eye(4)
    transform[:3, :3] = cv2.Rodrigues(rvec)[0]
    transform[:3, 3] = translation.ravel()
    order = np.argsort(selected_ids)
    return (
        transform,
        tuple(np.asarray(selected_ids)[order].tolist()),
        objects[order],
        normalized_pixels[order],
        rms,
    )


def old_pixel_observations(
    directory: Path,
    manifest: dict[str, Any],
    board: dict[str, Any],
    intrinsics: dict[str, Any],
    initial_top_optical: dict[str, np.ndarray],
) -> list[BoardPose]:
    result: list[BoardPose] = []
    for camera_name in CAMERAS:
        detected = detect_saved_frame(directory, manifest, camera_name, board)
        if detected is None:
            continue
        corners, ids = detected
        groups = split_marker_groups(corners, ids)
        if len(set(ids.tolist())) == len(ids) and len(ids) >= 4:
            groups = [list(range(len(ids)))]
        camera, distortion, model = intrinsics[camera_name]
        for group in groups:
            solved = solve_tag_centres(corners, ids, group, board, camera, distortion, model)
            if solved is None:
                continue
            transform, tag_ids, objects, normalized_pixels, rms = solved
            top = initial_top_optical[camera_name]
            center_top = (top @ np.r_[transform[:3, 3], 1.0])[:3]
            normal_top = normalized(top[:3, :3] @ transform[:3, 2])
            result.append(
                BoardPose(
                    camera_name,
                    directory.name,
                    transform,
                    center_top,
                    normal_top,
                    tag_ids,
                    objects,
                    normalized_pixels,
                    rms,
                )
            )
    return result


def direct_candidate(
    edge: dict[str, Any], sensor: str, anchors: dict[str, np.ndarray]
) -> np.ndarray | None:
    observed = np.asarray(edge["observed_left_from_right"], dtype=float)
    left, right = edge["left"], edge["right"]
    if left == sensor and right in anchors:
        return anchors[right] @ np.linalg.inv(observed)
    if right == sensor and left in anchors:
        return anchors[left] @ observed
    return None


def cluster_transforms(
    values: list[np.ndarray], prior: np.ndarray, radius_m: float = 0.065, radius_deg: float = 1.5
) -> tuple[np.ndarray, list[int], dict[str, Any]]:
    if not values:
        return prior.copy(), [], {"raw": 0, "dominant": 0, "fraction": 0.0}
    neighbourhoods = []
    for candidate in values:
        neighbourhoods.append(
            [
                index
                for index, other in enumerate(values)
                if transform_distance(candidate, other)[0] <= radius_m
                and transform_distance(candidate, other)[1] <= radius_deg
            ]
        )
    members = max(neighbourhoods, key=len)
    selected = [values[index] for index in members]
    matrix = np.eye(4)
    matrix[:3, 3] = np.median([item[:3, 3] for item in selected], axis=0)
    matrix[:3, :3] = Rotation.from_matrix([item[:3, :3] for item in selected]).mean().as_matrix()
    residuals = [transform_distance(matrix, item) for item in selected]
    return (
        matrix,
        members,
        {
            "raw": len(values),
            "dominant": len(members),
            "fraction": len(members) / len(values),
            "translation_median_m": float(np.median([item[0] for item in residuals])),
            "translation_p95_m": float(np.percentile([item[0] for item in residuals], 95)),
            "angle_median_deg": float(np.median([item[1] for item in residuals])),
            "angle_p95_deg": float(np.percentile([item[1] for item in residuals], 95)),
        },
    )


def build_day1_edges(base: Path, seed: dict[str, np.ndarray], output: Path) -> dict[str, Any]:
    """Re-detect old pixels with their capture-time intrinsics.

    The lidar plane CSV was already obtained from cross-lidar agreement on the
    moving-board day.  Re-detecting the image is still essential because cached
    UI detections can be stale and the fish resolution/model changed next day.
    """
    source = base / "result/diagnostics/round_036_camera_multilidar_planes_fresh.csv"
    rows_by_capture: dict[str, list[dict[str, str]]] = defaultdict(list)
    with source.open(newline="", encoding="utf-8") as stream:
        for row in csv.DictReader(stream):
            if row["camera"] in WEAK:
                rows_by_capture[row["capture_id"]].append(row)

    plane_edges: list[dict[str, Any]] = []
    pair_edges: list[dict[str, Any]] = []
    pair_candidates_before_static_sync_policy = 0
    pose_counts: Counter[str] = Counter()
    pair_gate_counts: Counter[str] = Counter()
    audit: list[dict[str, Any]] = []
    for index, (capture_id, rows) in enumerate(sorted(rows_by_capture.items()), 1):
        directory = base / "data/raw_extrinsic_data" / capture_id
        manifest = json.loads((directory / "manifest.json").read_text(encoding="utf-8"))
        poses = old_pixel_observations(
            directory,
            manifest,
            yaml.safe_load((directory / "capture_config.yaml").read_text(encoding="utf-8"))[
                "tag_board"
            ],
            load_capture_intrinsics(directory),
            {camera: seed[camera] for camera in CAMERAS},
        )
        by_camera: dict[str, list[Any]] = defaultdict(list)
        for pose in poses:
            by_camera[pose.camera].append(pose)
            pose_counts[pose.camera] += 1

        # Record why a saved-pixel weak/anchor overlap does or does not become
        # a direct relative-pose edge.  This is diagnostic-only and keeps every
        # raw image untouched.
        for weak_name in WEAK:
            for anchor_name in ANCHORS:
                for weak_pose in by_camera.get(weak_name, []):
                    for anchor_pose in by_camera.get(anchor_name, []):
                        pair_gate_counts["co_visible_weak_anchor"] += 1
                        gap = float(
                            np.linalg.norm(
                                weak_pose.rough_center_top - anchor_pose.rough_center_top
                            )
                        )
                        normal_angle = math.degrees(
                            math.acos(
                                float(
                                    np.clip(
                                        abs(
                                            weak_pose.rough_normal_top
                                            @ anchor_pose.rough_normal_top
                                        ),
                                        -1.0,
                                        1.0,
                                    )
                                )
                            )
                        )
                        if gap > 1.35 or normal_angle > 35.0:
                            pair_gate_counts["reject_rough_board_association"] += 1
                            continue
                        pair_gate_counts["pass_rough_board_association"] += 1
                        observed = weak_pose.transform_camera_from_board @ np.linalg.inv(
                            anchor_pose.transform_camera_from_board
                        )
                        initial = np.linalg.inv(seed[weak_name]) @ seed[anchor_name]
                        dt, da = transform_distance(observed, initial)
                        if dt > 1.20 or da > 28.0:
                            pair_gate_counts["reject_relative_pose_seed_gate"] += 1
                            continue
                        pair_gate_counts["pass_relative_pose_seed_gate"] += 1
                        common = len(set(weak_pose.tag_ids) & set(anchor_pose.tag_ids))
                        if common < 6:
                            pair_gate_counts["reject_common_tags_lt_6"] += 1
                            continue
                        pair_gate_counts["pass_all_geometric_gates"] += 1
        for row in rows:
            camera = row["camera"]
            # Match in the camera optical frame.  ``camera_rough_center_top``
            # was produced with an older physical seed and can differ by
            # 30--45 cm even when the same saved pixels are re-solved.  The
            # optical board centre is independent of that vehicle extrinsic.
            optical_reference = parse_vector(row["camera_center_optical"])
            candidates = by_camera.get(camera, [])
            if not candidates:
                audit.append(
                    {"capture_id": capture_id, "camera": camera, "status": "no_fresh_detection"}
                )
                continue
            pose = min(
                candidates,
                key=lambda item: float(
                    np.linalg.norm(item.transform_camera_from_board[:3, 3] - optical_reference)
                ),
            )
            match_gap = float(
                np.linalg.norm(pose.transform_camera_from_board[:3, 3] - optical_reference)
            )
            edge = {
                "capture_id": capture_id,
                "dataset": "20260902",
                "part": "day1",
                "camera": camera,
                "lidar": "lidar_top",
                "camera_center": pose.transform_camera_from_board[:3, 3].tolist(),
                "camera_normal": normalized(pose.transform_camera_from_board[:3, 2]).tolist(),
                "lidar_center": parse_vector(row["lidar_center_top"]).tolist(),
                "lidar_normal": normalized(parse_vector(row["lidar_normal_top"])).tolist(),
                "lidar_rms_m": float(row["lidar_plane_rms_m"]),
                "camera_reprojection_rms_px": float(row["camera_reprojection_rms_px"]),
                "fresh_match_gap_m": match_gap,
                "cross_lidar_count": int(row["cross_lidar_count"]),
            }
            # Fresh detection must agree with the historical saved-pixel result.
            if match_gap <= 0.10 and edge["camera_reprojection_rms_px"] <= 4.5:
                plane_edges.append(edge)
                status = "accepted"
            else:
                status = "fresh_detection_mismatch"
            audit.append(
                {
                    "capture_id": capture_id,
                    "camera": camera,
                    "status": status,
                    "match_gap_m": match_gap,
                }
            )

        meta = manifest.get("sensors", {})
        for pair in pair_observations(poses, {camera: seed[camera] for camera in CAMERAS}):
            # Weak/weak overlap is still useful: it cannot establish an
            # absolute vehicle pose by itself, but it couples two cameras to
            # their independent lidar-plane evidence.  This is particularly
            # important for rear_mid <-> fisheye_rear.
            if not (
                (pair.left in WEAK or pair.right in WEAK)
                and pair.left in (ANCHORS + WEAK)
                and pair.right in (ANCHORS + WEAK)
            ):
                continue
            pair_candidates_before_static_sync_policy += 1
            left_stamp = sensor_stamp(meta.get(pair.left, {}))
            right_stamp = sensor_stamp(meta.get(pair.right, {}))
            sync_ms = (
                None
                if left_stamp is None or right_stamp is None
                else abs(left_stamp - right_stamp) / 1e6
            )
            # On 2026-09-02 the vehicle and every board were motionless during
            # each snapshot; only one board was repositioned *between*
            # snapshots.  The ten cameras were intentionally read in sequence,
            # so the dynamic 100 ms gate used on the moving-vehicle day would
            # discard valid same-pose evidence.  Preserve the measured skew for
            # audit, but do not reject a geometrically matched static pair.
            pair_edges.append(
                {
                    "capture_id": capture_id,
                    "dataset": "20260902",
                    "part": "day1",
                    "left": pair.left,
                    "right": pair.right,
                    "observed_left_from_right": pair.transform_left_from_right.tolist(),
                    "reprojection_rms_px": pair.reprojection_rms_px,
                    "sync_ms": sync_ms,
                }
            )
        if index % 25 == 0:
            print(
                json.dumps(
                    {"stage": "day1_extract", "processed": index, "total": len(rows_by_capture)}
                ),
                flush=True,
            )

    payload = {
        "schema_version": 1,
        "source": str(source),
        "intrinsics_policy": "capture-local; old 640x480 fisheye models retained",
        "plane_edges": plane_edges,
        "camera_edges": pair_edges,
        "audit": audit,
        "summary": {
            "capture_count": len(rows_by_capture),
            "plane_edges": len(plane_edges),
            "camera_edges": len(pair_edges),
            "pair_candidates_before_static_sync_policy": pair_candidates_before_static_sync_policy,
            "fresh_pose_by_camera": dict(pose_counts),
            "weak_anchor_pair_gate_counts": dict(pair_gate_counts),
            "static_scene_sync_policy": "timestamp skew recorded; no rejection because vehicle and boards were stationary within each snapshot",
            "plane_by_camera": dict(Counter(edge["camera"] for edge in plane_edges)),
            "pairs": dict(Counter(f"{edge['left']}<->{edge['right']}" for edge in pair_edges)),
        },
    }
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")
    return payload


def build_combined(
    day1: dict[str, Any], day2: dict[str, Any], seed: dict[str, np.ndarray]
) -> tuple[list[dict[str, Any]], list[dict[str, Any]], dict[str, Any]]:
    planes = list(day1["plane_edges"])
    pairs = list(day1["camera_edges"])
    rejection = Counter()
    for edge in day2["plane_edges"]:
        if edge["camera"] not in WEAK:
            continue
        item = dict(edge)
        item["dataset"] = "20260903"
        item["part"] = f"day2_p{edge['part']}"
        distance, angle = plane_error(item, seed)
        center_camera = (
            seed[item["camera"]][:3, :3] @ np.asarray(item["camera_center"])
            + seed[item["camera"]][:3, 3]
        )
        lidar = seed[item["lidar"]]
        center_lidar = lidar[:3, :3] @ np.asarray(item["lidar_center"]) + lidar[:3, 3]
        center_gap = float(np.linalg.norm(center_camera - center_lidar))
        # Weak camera seeds may carry a common translation error.  Retain up to
        # 25 cm initially, then IRLS and cross-day checks decide acceptance.
        if distance <= 0.10 and angle <= 6.0 and center_gap <= 0.25:
            item["seed_center_gap_m"] = center_gap
            planes.append(item)
        else:
            rejection[f"day2_plane:{item['camera']}"] += 1
    for edge in day2["camera_edges"]:
        if not (
            (edge["left"] in WEAK or edge["right"] in WEAK)
            and edge["left"] in (ANCHORS + WEAK)
            and edge["right"] in (ANCHORS + WEAK)
        ):
            continue
        item = dict(edge)
        item["dataset"] = "20260903"
        item["part"] = f"day2_p{edge['part']}"
        candidate_sensors = [name for name in (item["left"], item["right"]) if name in WEAK]
        candidate_checks = []
        for candidate_sensor in candidate_sensors:
            candidate = direct_candidate(item, candidate_sensor, seed)
            if candidate is not None:
                candidate_checks.append(transform_distance(candidate, seed[candidate_sensor]))
        if candidate_checks and all(
            distance <= 0.25 and angle <= 12.0 for distance, angle in candidate_checks
        ):
            pairs.append(item)
        else:
            for candidate_sensor in candidate_sensors:
                rejection[f"day2_pair:{candidate_sensor}"] += 1
    return planes, pairs, dict(rejection)


def subset(
    items: list[dict[str, Any]], round_number: int, sensor: str, fraction: float = 0.86
) -> list[dict[str, Any]]:
    threshold = int(10_000 * fraction)
    result = []
    for index, item in enumerate(items):
        token = f"{round_number}:{sensor}:{index}:{item['capture_id']}:{item['dataset']}"
        value = int.from_bytes(hashlib.sha256(token.encode()).digest()[:4], "little") % 10_000
        if value < threshold:
            result.append(item)
    return result


def solve_sensor(
    sensor: str,
    planes: list[dict[str, Any]],
    pairs: list[dict[str, Any]],
    start: np.ndarray,
    prior: np.ndarray,
    fixed: dict[str, np.ndarray],
) -> np.ndarray:
    initial = np.r_[Rotation.from_matrix(start[:3, :3]).as_rotvec(), start[:3, 3]]
    prior_values = np.r_[Rotation.from_matrix(prior[:3, :3]).as_rotvec(), prior[:3, 3]]

    # Signs are fixed from the prior so the residual is continuous.
    plane_signs = []
    for edge in planes:
        lidar = fixed[edge["lidar"]]
        camera_normal = np.asarray(edge["camera_normal"])
        lidar_normal = lidar[:3, :3] @ np.asarray(edge["lidar_normal"])
        sign = 1.0 if prior[:3, :3] @ camera_normal @ lidar_normal >= 0 else -1.0
        plane_signs.append(sign)

    def residual(values: np.ndarray) -> np.ndarray:
        transform = np.eye(4)
        transform[:3, :3] = Rotation.from_rotvec(values[:3]).as_matrix()
        transform[:3, 3] = values[3:]
        parts: list[float] = []
        for edge, sign in zip(planes, plane_signs):
            lidar = fixed[edge["lidar"]]
            cc = np.asarray(edge["camera_center"])
            nc = sign * np.asarray(edge["camera_normal"])
            cl = np.asarray(edge["lidar_center"])
            nl = np.asarray(edge["lidar_normal"])
            pc = transform[:3, :3] @ cc + transform[:3, 3]
            nc_top = normalized(transform[:3, :3] @ nc)
            pl = lidar[:3, :3] @ cl + lidar[:3, 3]
            nl_top = normalized(lidar[:3, :3] @ nl)
            if nc_top @ nl_top < 0:
                nl_top = -nl_top
            delta = pc - pl
            normal_distance = float(nl_top @ delta)
            tangent = delta - nl_top * normal_distance
            # More independent lidars agreeing on the same board plane is
            # stronger evidence.  Keep single-lidar observations usable while
            # giving corroborated two/three/four-lidar planes progressively
            # more influence.
            corroboration = int(edge.get("cross_lidar_count", 1))
            quality = min(1.25, 0.55 + 0.20 * max(1, corroboration))
            parts.extend((quality * (nc_top - nl_top) / 0.035).tolist())
            parts.append(
                quality * normal_distance / max(0.012, min(0.035, 2.5 * float(edge["lidar_rms_m"])))
            )
            parts.extend((quality * tangent / 0.085).tolist())
        for edge in pairs:
            observed = np.asarray(edge["observed_left_from_right"], dtype=float)
            left = transform if edge["left"] == sensor else fixed[edge["left"]]
            right = transform if edge["right"] == sensor else fixed[edge["right"]]
            predicted = np.linalg.inv(left) @ right
            rotation_error = Rotation.from_matrix(
                predicted[:3, :3] @ observed[:3, :3].T
            ).as_rotvec()
            parts.extend((rotation_error / math.radians(0.60)).tolist())
            parts.extend(((predicted[:3, 3] - observed[:3, 3]) / 0.030).tolist())
        current_rotation = Rotation.from_rotvec(values[:3])
        prior_rotation = Rotation.from_rotvec(prior_values[:3])
        parts.extend(
            ((current_rotation * prior_rotation.inv()).as_rotvec() / math.radians(3.0)).tolist()
        )
        parts.extend(((values[3:] - prior_values[3:]) / 0.10).tolist())
        return np.asarray(parts, dtype=float)

    result = least_squares(
        residual,
        initial,
        loss="soft_l1",
        f_scale=1.0,
        max_nfev=350,
        xtol=1e-10,
        ftol=1e-10,
        gtol=1e-10,
    )
    if not result.success and result.status <= 0:
        raise RuntimeError(f"{sensor}: {result.message}")
    output = np.eye(4)
    output[:3, :3] = Rotation.from_rotvec(result.x[:3]).as_matrix()
    output[:3, 3] = result.x[3:]
    return output


def median_transform(values: list[np.ndarray]) -> np.ndarray:
    output = np.eye(4)
    output[:3, 3] = np.median([item[:3, 3] for item in values], axis=0)
    output[:3, :3] = Rotation.from_matrix([item[:3, :3] for item in values]).mean().as_matrix()
    return output


def errors_for_sensor(
    sensor: str,
    transform: np.ndarray,
    planes: list[dict[str, Any]],
    pairs: list[dict[str, Any]],
    fixed: dict[str, np.ndarray],
) -> dict[str, float]:
    values = dict(fixed)
    values[sensor] = transform
    pe = [plane_error(edge, values) for edge in planes]
    ce = [pair_error(edge, values) for edge in pairs]
    return {
        "plane_count": len(pe),
        "pair_count": len(ce),
        "plane_distance_median_m": float(np.median([v[0] for v in pe])) if pe else math.nan,
        "plane_distance_p95_m": float(np.percentile([v[0] for v in pe], 95)) if pe else math.nan,
        "plane_angle_median_deg": float(np.median([v[1] for v in pe])) if pe else math.nan,
        "plane_angle_p95_deg": float(np.percentile([v[1] for v in pe], 95)) if pe else math.nan,
        "pair_translation_median_m": float(np.median([v[0] for v in ce])) if ce else math.nan,
        "pair_translation_p95_m": float(np.percentile([v[0] for v in ce], 95)) if ce else math.nan,
        "pair_angle_median_deg": float(np.median([v[1] for v in ce])) if ce else math.nan,
        "pair_angle_p95_deg": float(np.percentile([v[1] for v in ce], 95)) if ce else math.nan,
    }


def validate_anchors(
    planes: list[dict[str, Any]], pairs: list[dict[str, Any]], fixed: dict[str, np.ndarray]
) -> dict[str, Any]:
    result = {}
    for sensor in ANCHORS:
        sensor_planes = [
            edge for edge in planes if sensor in (edge.get("camera"), edge.get("lidar"))
        ]
        sensor_pairs = [edge for edge in pairs if sensor in (edge.get("left"), edge.get("right"))]
        # Anchors are intentionally immutable.  We report new-data agreement,
        # never let weak observations move them.
        pe = [plane_error(edge, fixed) for edge in sensor_planes]
        ce = [pair_error(edge, fixed) for edge in sensor_pairs]
        result[sensor] = {
            "policy": "frozen_previous_repeatable_truth",
            "transform_change_m": 0.0,
            "transform_change_deg": 0.0,
            "plane_count": len(pe),
            "pair_count": len(ce),
            "plane_distance_median_m": float(np.median([v[0] for v in pe])) if pe else None,
            "plane_angle_median_deg": float(np.median([v[1] for v in pe])) if pe else None,
            "pair_translation_median_m": float(np.median([v[0] for v in ce])) if ce else None,
            "pair_angle_median_deg": float(np.median([v[1] for v in ce])) if ce else None,
        }
    return result


def run_rounds(
    planes: list[dict[str, Any]],
    pairs: list[dict[str, Any]],
    seed: dict[str, np.ndarray],
    max_rounds: int,
    log_path: Path,
) -> tuple[dict[str, np.ndarray], list[dict[str, Any]], dict[str, Any]]:
    by_plane = {sensor: [edge for edge in planes if edge["camera"] == sensor] for sensor in WEAK}
    by_pair = {
        sensor: [edge for edge in pairs if sensor in (edge["left"], edge["right"])]
        for sensor in WEAK
    }

    # Full-pair dominant clusters give a safe start and suppress identical-ID
    # board mismatches before iterative fitting.
    pair_cluster: dict[str, Any] = {}
    current = {sensor: seed[sensor].copy() for sensor in WEAK}
    filtered_pairs: dict[str, list[dict[str, Any]]] = {}
    for sensor in WEAK:
        candidates = [direct_candidate(edge, sensor, seed) for edge in by_pair[sensor]]
        valid = [
            (edge, candidate)
            for edge, candidate in zip(by_pair[sensor], candidates)
            if candidate is not None
        ]
        cluster, members, quality = cluster_transforms(
            [candidate for _edge, candidate in valid], seed[sensor]
        )
        pair_cluster[sensor] = quality
        if members and quality["fraction"] >= 0.60:
            current[sensor] = cluster
            filtered_pairs[sensor] = [valid[index][0] for index in members]
        else:
            filtered_pairs[sensor] = []

    histories: dict[str, list[np.ndarray]] = defaultdict(list)
    rows: list[dict[str, Any]] = []
    stable = 0
    log_path.parent.mkdir(parents=True, exist_ok=True)
    log_path.write_text(
        "# 多日分层联合外参逐轮日志\n\n"
        "锚点传感器冻结；仅优化 front_mid/rear_mid/fisheye_front/fisheye_rear。\n\n",
        encoding="utf-8",
    )
    for round_number in range(1, max_rounds + 1):
        previous_consensus = {
            sensor: median_transform(histories[sensor][-20:])
            if histories[sensor]
            else current[sensor]
            for sensor in WEAK
        }
        for sensor in WEAK:
            p = subset(by_plane[sensor], round_number, sensor)
            c = subset(filtered_pairs[sensor], round_number, sensor)
            if len(p) + len(c) < 4:
                histories[sensor].append(current[sensor].copy())
                continue
            fixed_now = dict(seed)
            fixed_now.update(current)
            estimate = solve_sensor(sensor, p, c, current[sensor], seed[sensor], fixed_now)
            histories[sensor].append(estimate)
            current[sensor] = median_transform(histories[sensor][-20:])

        deltas = {
            sensor: transform_distance(previous_consensus[sensor], current[sensor])
            for sensor in WEAK
        }
        spread = {}
        for sensor in WEAK:
            members = histories[sensor][-20:]
            values = [transform_distance(current[sensor], item) for item in members]
            spread[sensor] = {
                "translation_p95_m": float(np.percentile([v[0] for v in values], 95)),
                "angle_p95_deg": float(np.percentile([v[1] for v in values], 95)),
            }
        threshold_m = 0.001 if round_number <= 200 else 0.01
        threshold_deg = 0.1 if round_number <= 200 else 1.0
        round_ok = all(
            deltas[sensor][0] <= threshold_m
            and deltas[sensor][1] <= threshold_deg
            and spread[sensor]["translation_p95_m"] <= threshold_m
            and spread[sensor]["angle_p95_deg"] <= threshold_deg
            for sensor in WEAK
        )
        stable = stable + 1 if round_ok else 0
        row = {
            "round": round_number,
            "threshold": "1mm_0.1deg" if round_number <= 200 else "1cm_1deg",
            "stable_rounds": stable,
            "max_consensus_change_m": max(value[0] for value in deltas.values()),
            "max_consensus_change_deg": max(value[1] for value in deltas.values()),
            "max_bootstrap_spread_m": max(value["translation_p95_m"] for value in spread.values()),
            "max_bootstrap_spread_deg": max(value["angle_p95_deg"] for value in spread.values()),
            "sensors": {
                sensor: {
                    "consensus_change_m": deltas[sensor][0],
                    "consensus_change_deg": deltas[sensor][1],
                    **spread[sensor],
                }
                for sensor in WEAK
            },
            "converged": round_ok,
        }
        rows.append(row)
        print(json.dumps({"stage": "round", **row}, ensure_ascii=False), flush=True)
        with log_path.open("a", encoding="utf-8") as stream:
            stream.write(
                f"## 第 {round_number} 轮\n\n"
                f"- 门槛：{row['threshold']}；连续通过 {stable}/5\n"
                f"- 共识最大变化：{row['max_consensus_change_m'] * 1000:.3f} mm / "
                f"{row['max_consensus_change_deg']:.4f}°\n"
                f"- 20轮重采样最大P95：{row['max_bootstrap_spread_m'] * 1000:.3f} mm / "
                f"{row['max_bootstrap_spread_deg']:.4f}°\n\n"
            )
        if round_number >= 20 and stable >= 5:
            break
    return current, rows, pair_cluster


def main() -> int:
    default_base = Path(__file__).resolve().parents[2]
    parser = argparse.ArgumentParser()
    parser.add_argument("--base", type=Path, default=default_base)
    parser.add_argument(
        "--seed",
        type=Path,
        default=default_base / "result/extrinsics/all_sensors_relative_to_roof_lidar.yaml",
    )
    parser.add_argument(
        "--day1-cache",
        type=Path,
        default=default_base / "result/part12_multiday_20260903/day1_observations.json",
    )
    parser.add_argument(
        "--day2-cache", type=Path, default=default_base / "result/part12_20260903/observations.json"
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=default_base / "result/part12_multiday_20260903/multiday_hierarchical_result.yaml",
    )
    parser.add_argument(
        "--rounds-csv",
        type=Path,
        default=default_base / "result/part12_multiday_20260903/rounds.csv",
    )
    parser.add_argument(
        "--rounds-log",
        type=Path,
        default=default_base / "process/part12_joint_extrinsics/MULTIDAY_ITERATION_LOG.md",
    )
    parser.add_argument("--max-rounds", type=int, default=500)
    parser.add_argument("--extract-day1", action="store_true")
    args = parser.parse_args()

    seed = load_seed(args.seed)
    if args.extract_day1 or not args.day1_cache.is_file():
        day1 = build_day1_edges(args.base, seed, args.day1_cache)
    else:
        day1 = json.loads(args.day1_cache.read_text(encoding="utf-8"))
    day2 = json.loads(args.day2_cache.read_text(encoding="utf-8"))
    planes, pairs, rejected = build_combined(day1, day2, seed)
    refined, rounds, pair_clusters = run_rounds(
        planes, pairs, seed, args.max_rounds, args.rounds_log
    )

    per_sensor = {}
    transforms = {}
    for sensor in WEAK:
        sensor_planes = [edge for edge in planes if edge["camera"] == sensor]
        sensor_pairs = [edge for edge in pairs if sensor in (edge["left"], edge["right"])]
        day_metrics = {}
        day_solutions = {}
        for dataset in ("20260902", "20260903"):
            p = [edge for edge in sensor_planes if edge["dataset"] == dataset]
            c = [edge for edge in sensor_pairs if edge["dataset"] == dataset]
            if len(p) + len(c) >= 4:
                fixed_refined = dict(seed)
                fixed_refined.update(refined)
                solution = solve_sensor(sensor, p, c, refined[sensor], seed[sensor], fixed_refined)
                day_solutions[dataset] = solution
                day_metrics[dataset] = errors_for_sensor(sensor, solution, p, c, seed)
        cross = None
        if len(day_solutions) == 2:
            dt, da = transform_distance(day_solutions["20260902"], day_solutions["20260903"])
            cross = {
                "translation_difference_m": dt,
                "angle_difference_deg": da,
                "target_pass": dt <= 0.001 and da <= 0.1,
                "fallback_pass": dt <= 0.01 and da <= 1.0,
            }
        history_members = []
        # The final row already contains the last-20 spread per sensor.
        final_spread = rounds[-1]["sensors"][sensor]
        enough = len(set(edge["capture_id"] for edge in sensor_planes + sensor_pairs)) >= 8
        target = (
            enough
            and cross is not None
            and cross["target_pass"]
            and final_spread["translation_p95_m"] <= 0.001
            and final_spread["angle_p95_deg"] <= 0.1
        )
        fallback = (
            enough
            and cross is not None
            and cross["fallback_pass"]
            and final_spread["translation_p95_m"] <= 0.01
            and final_spread["angle_p95_deg"] <= 1.0
        )
        status = (
            "target_1mm_0.1deg" if target else "fallback_1cm_1deg" if fallback else "review_only"
        )
        per_sensor[sensor] = {
            "status": status,
            "support": {
                "planes": len(sensor_planes),
                "pairs": len(sensor_pairs),
                "captures": len(set(edge["capture_id"] for edge in sensor_planes + sensor_pairs)),
                "datasets": sorted(set(edge["dataset"] for edge in sensor_planes + sensor_pairs)),
            },
            "pair_candidate_cluster": pair_clusters[sensor],
            "combined_metrics": errors_for_sensor(
                sensor, refined[sensor], sensor_planes, sensor_pairs, {**seed, **refined}
            ),
            "per_day_metrics": day_metrics,
            "cross_day": cross,
            "final_bootstrap": final_spread,
            "change_from_previous": dict(
                zip(
                    ("translation_m", "angle_deg"),
                    transform_distance(refined[sensor], seed[sensor]),
                )
            ),
        }
        link = refined[sensor] @ np.linalg.inv(CAMERA_OPTICAL_TO_LINK)
        transforms[f"{sensor}/camera_link"] = transform_item(
            link, "multiday_hierarchical_fixed_anchor_consensus"
        )
        transforms[f"{sensor}/camera_link"]["calibration_status"] = status

    anchor_validation = validate_anchors(planes, pairs, seed)
    deployment_allowed = all(item["status"] != "review_only" for item in per_sensor.values())
    result = {
        "schema_version": 2,
        "reference_frame": "velodyne_top_base_link",
        "coordinate_convention": "x_forward_y_left_z_up; camera outputs are camera_link",
        "method": "multiday_hierarchical_fixed_anchor_blocked_bootstrap",
        "raw_data_modified": False,
        "rounds_completed": len(rounds),
        "max_rounds": args.max_rounds,
        "early_stop": len(rounds) < args.max_rounds,
        "deployment_allowed": deployment_allowed,
        "deployment_action": "none_review_only"
        if not deployment_allowed
        else "candidate_requires_atomic_backup_and_live_validation",
        "anchors": list(ANCHORS),
        "optimized": list(WEAK),
        "input_summary": {
            "day1": day1["summary"],
            "day2": day2["summary"],
            "combined_plane_edges": len(planes),
            "combined_camera_edges": len(pairs),
            "pre_rejected": rejected,
        },
        "anchor_validation": anchor_validation,
        "sensor_quality": per_sensor,
        "transforms": transforms,
        "rounds": rounds,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        yaml.safe_dump(result, allow_unicode=True, sort_keys=False), encoding="utf-8"
    )
    with args.rounds_csv.open("w", newline="", encoding="utf-8") as stream:
        fields = [key for key in rounds[0] if key != "sensors"]
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for row in rounds:
            writer.writerow({key: row[key] for key in fields})
    print(
        json.dumps(
            {
                "stage": "complete",
                "rounds": len(rounds),
                "deployment_allowed": deployment_allowed,
                "output": str(args.output),
            },
            ensure_ascii=False,
        ),
        flush=True,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
