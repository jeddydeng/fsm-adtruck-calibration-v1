#!/usr/bin/env python3
"""Optimize four-fisheye BEV seams from the strict 14-frame Part-3 set.

This program deliberately separates two questions:

* the *relative planar mapping* used to render a surround BEV; and
* each camera's absolute pose in the vehicle frame.

Only the first is signed by this program.  The existing ground-plane maps are
used as a metric initialisation, then a scale-preserving SE(2) correction is
estimated from stationary-scene correspondences in adjacent camera overlaps.
The corrections are derived exclusively from the capture IDs in ``--selected``.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np
from scipy.optimize import least_squares


CAMERAS = ("fisheye_front", "fisheye_right", "fisheye_rear", "fisheye_left")
SHORT = {camera: camera.removeprefix("fisheye_") for camera in CAMERAS}
PAIRS = tuple(zip(CAMERAS, CAMERAS[1:] + CAMERAS[:1]))
BEV_SIZE = (512, 512)
ORIGIN = np.asarray([256.0, 256.0], np.float64)
METERS_PER_PIXEL = 0.015


@dataclass
class MatchSet:
    left: str
    right: str
    capture_id: str
    points_left: np.ndarray
    points_right: np.ndarray


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--data-root", type=Path, required=True)
    parser.add_argument("--selected", type=Path, required=True)
    parser.add_argument("--raw-index", type=Path, required=True)
    parser.add_argument("--baseline", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--force", action="store_true")
    return parser.parse_args()


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def load_image_index(
    data_root: Path, index_path: Path, selected: list[str]
) -> dict[str, dict[str, Path]]:
    selected_set = set(selected)
    result: dict[str, dict[str, Path]] = {capture: {} for capture in selected}
    for line in index_path.read_text().splitlines():
        relative = Path(line.strip())
        if not relative.parts or relative.parts[0] not in selected_set:
            continue
        name = relative.stem
        for camera in CAMERAS:
            if name.endswith(camera):
                result[relative.parts[0]][camera] = data_root / relative
                break
    missing = [
        (capture, camera)
        for capture in selected
        for camera in CAMERAS
        if camera not in result[capture] or not result[capture][camera].is_file()
    ]
    if missing:
        raise FileNotFoundError(f"missing strict-set images: {missing[:8]}")
    return result


def load_maps(baseline: Path) -> tuple[dict[str, np.ndarray], dict[str, np.ndarray]]:
    maps_x, maps_y = {}, {}
    for camera in CAMERAS:
        short = SHORT[camera]
        maps_x[camera] = np.load(baseline / f"{camera}_bev_map_x.npy")
        maps_y[camera] = np.load(baseline / f"{camera}_bev_map_y.npy")
        for value in (maps_x[camera], maps_y[camera]):
            if value.shape != BEV_SIZE or value.dtype != np.float32:
                raise ValueError(f"invalid baseline remap for {short}: {value.shape}/{value.dtype}")
    return maps_x, maps_y


def valid_mask(map_x: np.ndarray, map_y: np.ndarray) -> np.ndarray:
    return (map_x >= 0.0) & (map_x < 1600.0) & (map_y >= 0.0) & (map_y < 1200.0)


def ground_feature_mask(map_x: np.ndarray, map_y: np.ndarray) -> np.ndarray:
    yy, xx = np.mgrid[: BEV_SIZE[0], : BEV_SIZE[1]]
    radius_m = np.hypot(xx - ORIGIN[0], yy - ORIGIN[1]) * METERS_PER_PIXEL
    mask = valid_mask(map_x, map_y) & (map_y > 650.0) & (radius_m >= 0.70) & (radius_m <= 3.60)
    # Do not use the vehicle body, whose appearance is camera-specific.
    mask[55:345, 205:307] = False
    return mask.astype(np.uint8) * 255


def remap_image(path: Path, map_x: np.ndarray, map_y: np.ndarray) -> np.ndarray:
    image = cv2.imread(str(path), cv2.IMREAD_COLOR)
    if image is None or image.shape[:2] != (1200, 1600):
        raise ValueError(f"unreadable or wrong-size image: {path}")
    return cv2.remap(image, map_x, map_y, cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)


def ratio_matches(
    descriptors_a: np.ndarray, descriptors_b: np.ndarray, ratio: float = 0.72
) -> list[cv2.DMatch]:
    matcher = cv2.BFMatcher(cv2.NORM_HAMMING)
    forward = matcher.knnMatch(descriptors_a, descriptors_b, k=2)
    # RANSAC is applied independently per capture immediately afterwards.  A
    # one-way ratio test retains enough observations in the narrow right/rear
    # overlap; requiring a mutual best match discards repeatable ground texture
    # and makes that edge under-observed.
    return [m for m, n in forward if m.distance < ratio * n.distance]


def extract_matches(
    images: dict[str, dict[str, Path]], maps_x: dict[str, np.ndarray], maps_y: dict[str, np.ndarray]
) -> list[MatchSet]:
    detector = cv2.ORB_create(
        nfeatures=6000, scaleFactor=1.15, nlevels=10, edgeThreshold=12, fastThreshold=8
    )
    output: list[MatchSet] = []
    for capture_id, paths in images.items():
        features = {}
        for camera in CAMERAS:
            bev = remap_image(paths[camera], maps_x[camera], maps_y[camera])
            gray = cv2.cvtColor(bev, cv2.COLOR_BGR2GRAY)
            features[camera] = detector.detectAndCompute(
                gray, ground_feature_mask(maps_x[camera], maps_y[camera])
            )
        for left, right in PAIRS:
            key_left, desc_left = features[left]
            key_right, desc_right = features[right]
            if desc_left is None or desc_right is None:
                continue
            matches = ratio_matches(desc_left, desc_right)
            if len(matches) < 6:
                continue
            points_left = np.float64([key_left[m.queryIdx].pt for m in matches])
            points_right = np.float64([key_right[m.trainIdx].pt for m in matches])
            close = np.linalg.norm(points_left - points_right, axis=1) <= 50.0
            points_left, points_right = points_left[close], points_right[close]
            if len(points_left) < 6:
                continue
            _, inlier = cv2.estimateAffinePartial2D(
                points_right,
                points_left,
                method=cv2.RANSAC,
                ransacReprojThreshold=3.0,
                maxIters=3000,
                confidence=0.995,
                refineIters=20,
            )
            if inlier is None or int(inlier.sum()) < 4:
                continue
            keep = inlier.ravel().astype(bool)
            output.append(MatchSet(left, right, capture_id, points_left[keep], points_right[keep]))
    return output


def camera_matrix(parameters: np.ndarray, camera: str) -> np.ndarray:
    if camera == CAMERAS[0]:
        tx = ty = theta = 0.0
    else:
        offset = (CAMERAS.index(camera) - 1) * 3
        tx, ty, theta = parameters[offset : offset + 3]
    cosine, sine = math.cos(theta), math.sin(theta)
    # Rotate about the BEV vehicle origin so tx/ty remain physically legible.
    rotation = np.asarray([[cosine, -sine], [sine, cosine]])
    translation = ORIGIN + np.asarray([tx, ty]) - rotation @ ORIGIN
    return np.asarray(
        [[cosine, -sine, translation[0]], [sine, cosine, translation[1]], [0.0, 0.0, 1.0]],
        np.float64,
    )


def transform_points(matrix: np.ndarray, points: np.ndarray) -> np.ndarray:
    homogeneous = np.column_stack((points, np.ones(len(points))))
    return (matrix @ homogeneous.T).T[:, :2]


def pair_arrays(match_sets: list[MatchSet]) -> dict[tuple[str, str], tuple[np.ndarray, np.ndarray]]:
    result = {}
    for pair in PAIRS:
        selected = [item for item in match_sets if (item.left, item.right) == pair]
        if selected:
            result[pair] = (
                np.vstack([item.points_left for item in selected]),
                np.vstack([item.points_right for item in selected]),
            )
    missing = [pair for pair in PAIRS if pair not in result]
    if missing:
        raise RuntimeError(f"no correspondences for overlap(s): {missing}")
    return result


def residual_vector(
    parameters: np.ndarray,
    pairs: dict[tuple[str, str], tuple[np.ndarray, np.ndarray]],
    gate: dict[tuple[str, str], np.ndarray] | None = None,
) -> np.ndarray:
    residuals = []
    for pair, (points_left, points_right) in pairs.items():
        if gate is not None:
            points_left, points_right = points_left[gate[pair]], points_right[gate[pair]]
        left = transform_points(camera_matrix(parameters, pair[0]), points_left)
        right = transform_points(camera_matrix(parameters, pair[1]), points_right)
        # Weight each overlap equally instead of letting the most textured view dominate.
        residuals.append((left - right).ravel() / math.sqrt(max(len(left), 1)))
    # Gentle mechanical prior: Part3 should refine seams, not replace the rig geometry.
    translations = np.concatenate(
        [parameters[index : index + 2] for index in range(0, len(parameters), 3)]
    )
    angles = parameters[2::3]
    residuals.append(translations / 20.0)
    residuals.append(angles / math.radians(3.0))
    return np.concatenate(residuals)


def optimize(
    pairs: dict[tuple[str, str], tuple[np.ndarray, np.ndarray]],
) -> tuple[np.ndarray, dict]:
    initial = np.zeros(9, np.float64)
    first = least_squares(
        residual_vector, initial, args=(pairs, None), loss="cauchy", f_scale=0.35, max_nfev=500
    )
    distances = {}
    gates = {}
    for pair, (left, right) in pairs.items():
        corrected_left = transform_points(camera_matrix(first.x, pair[0]), left)
        corrected_right = transform_points(camera_matrix(first.x, pair[1]), right)
        value = np.linalg.norm(corrected_left - corrected_right, axis=1)
        distances[pair] = value
        gates[pair] = value <= 5.0
    if any(int(mask.sum()) < 20 for mask in gates.values()):
        raise RuntimeError("insufficient <=5 px overlap correspondences after first pass")
    second = least_squares(
        residual_vector, first.x, args=(pairs, gates), loss="soft_l1", f_scale=0.20, max_nfev=500
    )

    metrics = {}
    for pair, (left, right) in pairs.items():
        corrected_left = transform_points(camera_matrix(second.x, pair[0]), left)
        corrected_right = transform_points(camera_matrix(second.x, pair[1]), right)
        all_distance = np.linalg.norm(corrected_left - corrected_right, axis=1)
        retained = all_distance[gates[pair]]
        name = f"{SHORT[pair[0]]}<->{SHORT[pair[1]]}"
        metrics[name] = {
            "raw_correspondences": int(len(all_distance)),
            "retained_correspondences": int(len(retained)),
            "retained_fraction": float(len(retained) / len(all_distance)),
            "all_residual_median_px": float(np.median(all_distance)),
            "all_residual_p95_px": float(np.percentile(all_distance, 95)),
            "retained_residual_median_px": float(np.median(retained)),
            "retained_residual_p95_px": float(np.percentile(retained, 95)),
        }
    return second.x, metrics


def corrected_maps(
    map_x: np.ndarray, map_y: np.ndarray, correction: np.ndarray
) -> tuple[np.ndarray, np.ndarray]:
    inverse = np.linalg.inv(correction)
    yy, xx = np.mgrid[: BEV_SIZE[0], : BEV_SIZE[1]]
    target = np.stack((xx, yy, np.ones_like(xx)), axis=-1).reshape(-1, 3).T
    source = inverse @ target
    source_x = source[0].reshape(BEV_SIZE).astype(np.float32)
    source_y = source[1].reshape(BEV_SIZE).astype(np.float32)
    final_x = cv2.remap(
        map_x, source_x, source_y, cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT, borderValue=-1
    )
    final_y = cv2.remap(
        map_y, source_x, source_y, cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT, borderValue=-1
    )
    return final_x.astype(np.float32), final_y.astype(np.float32)


def blend(bevs: dict[str, np.ndarray], masks: dict[str, np.ndarray]) -> np.ndarray:
    weighted = np.zeros((*BEV_SIZE, 3), np.float64)
    weight_sum = np.zeros(BEV_SIZE, np.float64)
    for camera in CAMERAS:
        valid = masks[camera].astype(np.uint8)
        weight = cv2.distanceTransform(valid, cv2.DIST_L2, 5).astype(np.float64)
        weight = np.minimum(weight, 35.0) + valid * 0.01
        weighted += bevs[camera].astype(np.float64) * weight[..., None]
        weight_sum += weight
    result = np.zeros((*BEV_SIZE, 3), np.uint8)
    covered = weight_sum > 0
    result[covered] = np.clip(weighted[covered] / weight_sum[covered, None], 0, 255).astype(
        np.uint8
    )
    cv2.rectangle(result, (205, 55), (307, 345), (32, 32, 32), -1)
    cv2.arrowedLine(result, (256, 256), (256, 170), (255, 255, 255), 2, tipLength=0.15)
    cv2.putText(
        result,
        "+X vehicle",
        (265, 174),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.40,
        (255, 255, 255),
        1,
        cv2.LINE_AA,
    )
    return result


def overlap_overlay(
    left_image: np.ndarray,
    right_image: np.ndarray,
    left_mask: np.ndarray,
    right_mask: np.ndarray,
    label: str,
) -> np.ndarray:
    """Render a magenta/green registration overlay for human seam review."""
    left_gray = cv2.cvtColor(left_image, cv2.COLOR_BGR2GRAY)
    right_gray = cv2.cvtColor(right_image, cv2.COLOR_BGR2GRAY)
    overlap = left_mask & right_mask
    output = np.zeros((*BEV_SIZE, 3), np.uint8)
    output[..., 0][overlap] = left_gray[overlap]
    output[..., 2][overlap] = left_gray[overlap]
    output[..., 1][overlap] = right_gray[overlap]
    rows, columns = np.nonzero(overlap)
    if len(rows):
        x0, x1 = max(int(columns.min()) - 12, 0), min(int(columns.max()) + 13, 512)
        y0, y1 = max(int(rows.min()) - 12, 0), min(int(rows.max()) + 13, 512)
        crop = output[y0:y1, x0:x1]
        scale = min(512.0 / max(crop.shape[1], 1), 470.0 / max(crop.shape[0], 1))
        resized = cv2.resize(crop, None, fx=scale, fy=scale, interpolation=cv2.INTER_LINEAR)
        output = np.zeros((*BEV_SIZE, 3), np.uint8)
        y_offset = (512 - resized.shape[0]) // 2
        x_offset = (512 - resized.shape[1]) // 2
        output[y_offset : y_offset + resized.shape[0], x_offset : x_offset + resized.shape[1]] = (
            resized
        )
    cv2.putText(
        output, label, (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1, cv2.LINE_AA
    )
    return output


def write_opencv_yaml(
    path: Path,
    camera: str,
    camera_data: dict,
    board_metrics: dict,
    correction: np.ndarray,
    final_h: np.ndarray,
    passed: bool = False,
) -> None:
    short = SHORT[camera]
    k = np.asarray(camera_data["K"]).reshape(3, 3)
    d = np.asarray(camera_data["D"]).reshape(4)
    r = np.asarray(camera_data["T_vehicle_from_camera_optical"])[:3, :3]
    t = np.asarray(camera_data["T_vehicle_from_camera_optical"])[:3, 3]

    def values(array: np.ndarray) -> str:
        return ", ".join(f"{float(value):.12g}" for value in array.ravel())

    text = f"""%YAML:1.0
---
camera_name: {short}
camera_frame: {camera}/camera_optical_frame
vehicle_frame: sensor_kit_base_link
image_width: 1600
image_height: 1200
distortion_model: equidistant
bev_width: 512
bev_height: 512
meters_per_pixel: 0.015
bev_origin_u_px: 256
bev_origin_v_px: 256
bev_axes: \"u_right_is_vehicle_minus_y; v_down_is_vehicle_minus_x\"
deployment_allowed: {1 if passed else 0}
deployment_scope: \"BEV_relative_planar_mapping_only\"
bev_status: \"{"usable" if passed else "failed_self_check"}\"
absolute_vehicle_extrinsic_deployment_allowed: 0
absolute_vehicle_extrinsic_status: \"review_only_not_signed\"
absolute_vehicle_extrinsic_warning: \"rear absolute pose conflicts with cross-session calibration; all R/t below are reference-only\"
relative_calibration_source: \"strict final 14 Part3 images; ground-only overlap; scale-preserving SE2 loop optimisation\"
K: !!opencv-matrix
   rows: 3
   cols: 3
   dt: d
   data: [{values(k)}]
D: !!opencv-matrix
   rows: 4
   cols: 1
   dt: d
   data: [{values(d)}]
H_bev: !!opencv-matrix
   rows: 3
   cols: 3
   dt: d
   data: [{values(final_h)}]
H_bev_definition: \"p_bev ~ H_bev * p_rectified; NOT valid on raw equidistant pixels\"
C_bev_from_baseline: !!opencv-matrix
   rows: 3
   cols: 3
   dt: d
   data: [{values(correction)}]
C_bev_from_baseline_definition: \"p_final_bev ~ C_bev_from_baseline * p_baseline_bev\"
R_vehicle_camera: !!opencv-matrix
   rows: 3
   cols: 3
   dt: d
   data: [{values(r)}]
R_vehicle_camera_definition: \"REFERENCE ONLY: p_vehicle = R_vehicle_camera * p_camera_optical + t_vehicle_camera_m\"
t_vehicle_camera_m: !!opencv-matrix
   rows: 3
   cols: 1
   dt: d
   data: [{values(t)}]
intrinsic_calibration_rms_px: {float(camera_data["intrinsic_rms_px"]):.12g}
intrinsic_accepted_view_p95_px: {float(camera_data["intrinsic_accepted_p95_px"]):.12g}
part3_board_pose_reprojection_median_px: {float(board_metrics["reprojection_median_px"]):.12g}
part3_board_pose_reprojection_p95_px: {float(board_metrics["reprojection_p95_px"]):.12g}
bev_remap_x: \"{camera}_bev_map_x.npy\"
bev_remap_y: \"{camera}_bev_map_y.npy\"
"""
    path.write_text(text)


def main() -> None:
    args = parse_args()
    if args.output.exists() and any(args.output.iterdir()) and not args.force:
        raise FileExistsError(f"output is non-empty; use --force: {args.output}")
    args.output.mkdir(parents=True, exist_ok=True)
    selected = [line.strip() for line in args.selected.read_text().splitlines() if line.strip()]
    if len(selected) != 14:
        raise ValueError(f"strict Part3 set must contain exactly 14 captures, got {len(selected)}")
    images = load_image_index(args.data_root, args.raw_index, selected)
    baseline_x, baseline_y = load_maps(args.baseline)
    baseline_json = json.loads((args.baseline / "bev_calibration.json").read_text())

    match_sets = extract_matches(images, baseline_x, baseline_y)
    pairs = pair_arrays(match_sets)
    parameters, pair_metrics = optimize(pairs)
    for left, right in PAIRS:
        name = f"{SHORT[left]}<->{SHORT[right]}"
        pair_metrics[name]["capture_groups_with_matches"] = len(
            {item.capture_id for item in match_sets if item.left == left and item.right == right}
        )
    corrections = {camera: camera_matrix(parameters, camera) for camera in CAMERAS}

    final_x, final_y, masks = {}, {}, {}
    corrections_json = {}
    for camera in CAMERAS:
        final_x[camera], final_y[camera] = corrected_maps(
            baseline_x[camera], baseline_y[camera], corrections[camera]
        )
        masks[camera] = valid_mask(final_x[camera], final_y[camera])
        np.save(args.output / f"{camera}_bev_map_x.npy", final_x[camera])
        np.save(args.output / f"{camera}_bev_map_y.npy", final_y[camera])
        yaw = math.degrees(math.atan2(corrections[camera][1, 0], corrections[camera][0, 0]))
        centre_shift = transform_points(corrections[camera], ORIGIN[None])[0] - ORIGIN
        corrections_json[camera] = {
            "C_bev_from_baseline": corrections[camera].tolist(),
            "translation_at_vehicle_origin_px": centre_shift.tolist(),
            "translation_at_vehicle_origin_m": (centre_shift * METERS_PER_PIXEL).tolist(),
            "yaw_deg": yaw,
            "scale": 1.0,
        }

    yy, xx = np.mgrid[: BEV_SIZE[0], : BEV_SIZE[1]]
    radius_m = np.hypot(xx - ORIGIN[0], yy - ORIGIN[1]) * METERS_PER_PIXEL
    annulus = (radius_m >= 0.75) & (radius_m <= 3.50)
    coverage_count = np.sum(np.stack(list(masks.values())), axis=0)
    per_camera_coverage = {
        camera: float((masks[camera] & annulus).sum() / annulus.sum()) for camera in CAMERAS
    }
    coverage = {
        "evaluation_annulus_m": [0.75, 3.50],
        "per_camera_fraction": per_camera_coverage,
        "combined_fraction": float(((coverage_count >= 1) & annulus).sum() / annulus.sum()),
        "overlap_two_or_more_fraction": float(
            ((coverage_count >= 2) & annulus).sum() / annulus.sum()
        ),
        "hole_fraction": float(((coverage_count == 0) & annulus).sum() / annulus.sum()),
    }
    cv2.imwrite(
        str(args.output / "bev_coverage_count.png"), np.uint8(np.clip(coverage_count * 63, 0, 255))
    )

    previews = []
    first_bevs = None
    for capture_id in selected:
        bevs = {
            camera: remap_image(images[capture_id][camera], final_x[camera], final_y[camera])
            for camera in CAMERAS
        }
        composite = blend(bevs, masks)
        cv2.putText(
            composite,
            capture_id,
            (10, 24),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (255, 255, 255),
            1,
            cv2.LINE_AA,
        )
        previews.append(composite)
        if capture_id == selected[0]:
            first_bevs = bevs
            cv2.imwrite(str(args.output / "four_camera_bev_preview.png"), composite)
            for camera in CAMERAS:
                cv2.imwrite(str(args.output / f"{camera}_bev_preview.png"), bevs[camera])
    thumbnails = [cv2.resize(image, (256, 256), interpolation=cv2.INTER_AREA) for image in previews]
    rows = []
    for index in range(0, len(thumbnails), 4):
        row = thumbnails[index : index + 4]
        while len(row) < 4:
            row.append(np.zeros_like(thumbnails[0]))
        rows.append(np.hstack(row))
    cv2.imwrite(str(args.output / "strict14_bev_contact_sheet.jpg"), np.vstack(rows))
    if first_bevs is None:
        raise RuntimeError("no preview images were generated")
    overlays = [
        overlap_overlay(
            first_bevs[left],
            first_bevs[right],
            masks[left],
            masks[right],
            f"{SHORT[left]} <-> {SHORT[right]}",
        )
        for left, right in PAIRS
    ]
    cv2.imwrite(
        str(args.output / "overlap_validation_contact_sheet.jpg"),
        np.vstack((np.hstack(overlays[:2]), np.hstack(overlays[2:]))),
    )

    # Bind this result to the exact 14 capture groups and 56 source images.
    (args.output / "source_capture_groups.txt").write_text("\n".join(selected) + "\n")
    indexed_paths = [images[capture][camera] for capture in selected for camera in CAMERAS]
    (args.output / "raw_image_index.txt").write_text(
        "".join(f"{path.relative_to(args.data_root)}\n" for path in indexed_paths)
    )
    (args.output / "raw_image_sha256.txt").write_text(
        "".join(f"{sha256(path)}  {path.relative_to(args.data_root)}\n" for path in indexed_paths)
    )

    # The scale check is inherited from the same strict Part3 set and unchanged by SE(2).
    radial = baseline_json["validation"]["radial_three_stage_motion"]
    move_errors = [
        item["component_error_norm_m"]
        for edge in radial.get("moves", {}).values()
        for item in edge.values()
        if item.get("available")
    ]
    move_median = float(np.median(move_errors)) if move_errors else float("inf")
    move_maximum = float(np.max(move_errors)) if move_errors else float("inf")
    self_checks = {
        "exactly_14_selected_captures": len(selected) == 14,
        "all_four_cycle_overlaps_have_at_least_50_retained_matches": all(
            item["retained_correspondences"] >= 50 for item in pair_metrics.values()
        ),
        "all_four_cycle_overlaps_span_at_least_3_capture_groups": all(
            item["capture_groups_with_matches"] >= 3 for item in pair_metrics.values()
        ),
        "all_retained_overlap_medians_le_3px": all(
            item["retained_residual_median_px"] <= 3.0 for item in pair_metrics.values()
        ),
        "all_retained_overlap_p95_le_5px": all(
            item["retained_residual_p95_px"] <= 5.0 for item in pair_metrics.values()
        ),
        "all_overlap_retention_ge_50pct": all(
            item["retained_fraction"] >= 0.50 for item in pair_metrics.values()
        ),
        "combined_annulus_coverage_ge_95pct": coverage["combined_fraction"] >= 0.95,
        "overlap_annulus_coverage_ge_50pct": coverage["overlap_two_or_more_fraction"] >= 0.50,
        "metric_scale_preserved": all(
            abs(item["scale"] - 1.0) < 1e-12 for item in corrections_json.values()
        ),
        "part3_board_motion_median_error_le_60mm": move_median <= 0.060,
        "part3_board_motion_max_error_le_60mm": move_maximum <= 0.060,
        "maps_shape_dtype_and_finite": all(
            final_x[c].shape == BEV_SIZE
            and final_y[c].shape == BEV_SIZE
            and final_x[c].dtype == np.float32
            and final_y[c].dtype == np.float32
            and np.isfinite(final_x[c]).all()
            and np.isfinite(final_y[c]).all()
            for c in CAMERAS
        ),
        "homographies_finite_and_invertible": all(
            np.isfinite(corrections[c]).all() and abs(np.linalg.det(corrections[c])) > 1e-9
            for c in CAMERAS
        ),
        "all_generated_previews_readable": all(
            image is not None and image.shape[:2] == BEV_SIZE for image in previews
        ),
    }
    passed = all(self_checks.values())

    cameras_json = {}
    for camera in CAMERAS:
        old = baseline_json["cameras"][camera]
        board_metrics = baseline_json["validation"]["per_camera"][camera]
        old_h = np.asarray(old["H_bev_from_rectified"], np.float64)
        final_h = corrections[camera] @ old_h
        cameras_json[camera] = {
            "K": old["K"],
            "D": old["D"],
            "K_rectified": old["K_rectified"],
            "intrinsic_rms_px": old["intrinsic_rms_px"],
            "intrinsic_accepted_p95_px": old["intrinsic_accepted_p95_px"],
            "part3_board_pose_reprojection_median_px": board_metrics["reprojection_median_px"],
            "part3_board_pose_reprojection_p95_px": board_metrics["reprojection_p95_px"],
            "H_bev_from_rectified": final_h.tolist(),
            "H_bev_scope": "usable for rectified ground plane only",
            "C_bev_from_baseline": corrections[camera].tolist(),
            "bev_remap_x": f"{camera}_bev_map_x.npy",
            "bev_remap_y": f"{camera}_bev_map_y.npy",
            "T_vehicle_from_camera_optical": old["T_vehicle_from_camera_optical"],
            "absolute_extrinsic_status": "review_only_not_signed",
        }
        write_opencv_yaml(
            args.output / f"{SHORT[camera]}.yaml",
            camera,
            old,
            board_metrics,
            corrections[camera],
            final_h,
            passed,
        )

    report = {
        "schema_version": 2,
        "status": "usable" if passed else "failed_self_check",
        "deployment_allowed": 1 if passed else 0,
        "deployment_scope": "BEV_relative_planar_mapping_only",
        "absolute_vehicle_extrinsic_deployment_allowed": 0,
        "absolute_vehicle_extrinsic_status": "review_only_not_signed",
        "absolute_vehicle_extrinsic_warning": (
            "Rear fisheye absolute vehicle pose conflicts with cross-session calibration; "
            "no R/t in this package is authorised for Autoware deployment."
        ),
        "method": (
            "metric ground-plane initial maps plus strict-Part3-only, ground-feature, "
            "four-edge closed-loop, scale-preserving SE(2) refinement"
        ),
        "selected_capture_count": len(selected),
        "selected_source_image_count": len(indexed_paths),
        "selected_capture_ids": selected,
        "coordinate_convention": baseline_json["coordinate_convention"],
        "bev": baseline_json["bev"],
        "board": {
            "family": "tag16h5",
            "ids": "0..29",
            "outer_size_m": [0.9, 0.9],
            "active_size_m": [0.8, 0.8],
            "tag_size_m": 0.108,
            "horizontal_gap_m": 0.0304,
            "vertical_gap_m": 0.065,
        },
        "intrinsics_source": "fisheye_intrinsics_1600x1200_20260903_v2",
        "relative_corrections": corrections_json,
        "overlap_validation": pair_metrics,
        "coverage": coverage,
        "part3_board_motion_scale_validation": {
            "available_checks": len(move_errors),
            "median_error_m": move_median if move_errors else None,
            "max_error_m": move_maximum if move_errors else None,
            "note": "hand-moved target distances were approximate 0.45 m components",
        },
        "self_checks": self_checks,
        "all_self_checks_passed": passed,
        "cameras": cameras_json,
        "limitations": [
            "A homography is valid only after fisheye rectification and only on the ground plane.",
            "Production should use the precomputed raw-equidistant-to-BEV remap arrays.",
            "Objects above the ground plane exhibit parallax at camera seams.",
            "This relative BEV result does not validate absolute camera-to-vehicle R/t.",
        ],
    }
    (args.output / "bev_calibration.json").write_text(
        json.dumps(report, indent=2, ensure_ascii=False) + "\n"
    )

    with (args.output / "overlap_metrics.csv").open("w", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(
            [
                "pair",
                "raw_count",
                "retained_count",
                "retained_fraction",
                "all_median_px",
                "all_p95_px",
                "retained_median_px",
                "retained_p95_px",
            ]
        )
        for pair, value in pair_metrics.items():
            writer.writerow(
                [
                    pair,
                    value["raw_correspondences"],
                    value["retained_correspondences"],
                    value["retained_fraction"],
                    value["all_residual_median_px"],
                    value["all_residual_p95_px"],
                    value["retained_residual_median_px"],
                    value["retained_residual_p95_px"],
                ]
            )

    summary = [
        "# Four-fisheye relative BEV validation",
        "",
        f"- BEV status: **{'USABLE' if passed else 'FAILED SELF-CHECK'}**",
        f"- BEV deployment_allowed: **{1 if passed else 0}** (relative planar mapping only)",
        "- Absolute vehicle R/t: **review only / not deployable**",
        "- Data: final 14 Part3 captures only; 56 fresh 1600x1200 fisheye images",
        "- Map: 512x512, 0.015 m/px; metric scale held fixed",
        f"- Combined 0.75–3.50 m annulus coverage: {coverage['combined_fraction']:.2%}",
        f"- Two-or-more-camera overlap: {coverage['overlap_two_or_more_fraction']:.2%}",
        "",
        "## Seam residuals",
        "",
        "| overlap | retained/raw | retained median | retained P95 | all P95 |",
        "|---|---:|---:|---:|---:|",
    ]
    for pair, value in pair_metrics.items():
        summary.append(
            f"| {pair} | {value['retained_correspondences']}/{value['raw_correspondences']} "
            f"| {value['retained_residual_median_px']:.3f} px "
            f"| {value['retained_residual_p95_px']:.3f} px "
            f"| {value['all_residual_p95_px']:.3f} px |"
        )
    summary += [
        "",
        "## Per-camera reprojection errors",
        "",
        "| camera | intrinsic RMS | accepted intrinsic-view P95 | Part3 board-pose median | Part3 board-pose P95 |",
        "|---|---:|---:|---:|---:|",
    ]
    for camera, value in cameras_json.items():
        summary.append(
            f"| {SHORT[camera]} | {value['intrinsic_rms_px']:.3f} px "
            f"| {value['intrinsic_accepted_p95_px']:.3f} px "
            f"| {value['part3_board_pose_reprojection_median_px']:.3f} px "
            f"| {value['part3_board_pose_reprojection_p95_px']:.3f} px |"
        )
    summary += [
        "",
        "All reported usable transforms preserve scale exactly. The residual is an image-space",
        "overlap measure; it is not a millimetre claim. At 15 mm/px, 5 px equals 75 mm.",
        "Raised objects can disagree because a single ground homography cannot remove parallax.",
        "",
        "## Coordinate and matrix direction",
        "",
        "- Vehicle: x forward, y left, z up.",
        "- BEV: u right = -vehicle y; v down = -vehicle x; vehicle origin=(256,256).",
        "- `C_bev_from_baseline`: `p_final_bev ~ C * p_baseline_bev`.",
        "- `H_bev`: `p_bev ~ H * p_rectified`; never apply it to raw equidistant pixels.",
        "- Runtime path: apply the saved `*_bev_map_x.npy`/`*_bev_map_y.npy` directly with `cv2.remap`.",
        "",
        "## Absolute-pose warning",
        "",
        "The supplied R/t values are retained only for traceability. Cross-session calibration did",
        "not sign them, and the rear fisheye absolute height showed a material conflict. This package",
        "therefore authorises only surround-view ground-plane rendering, not Autoware sensor TF.",
    ]
    (args.output / "VALIDATION_SUMMARY.md").write_text("\n".join(summary) + "\n")

    manifest_files = sorted(
        path for path in args.output.iterdir() if path.is_file() and path.name != "manifest.sha256"
    )
    (args.output / "manifest.sha256").write_text(
        "".join(f"{sha256(path)}  {path.name}\n" for path in manifest_files)
    )
    print(
        json.dumps(
            {
                "output": str(args.output),
                "passed": passed,
                "pair_metrics": pair_metrics,
                "coverage": coverage,
                "corrections": corrections_json,
            },
            indent=2,
        )
    )
    if not passed:
        raise SystemExit(2)


if __name__ == "__main__":
    main()
