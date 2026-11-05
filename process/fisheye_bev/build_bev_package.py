#!/usr/bin/env python3
"""Build metric four-fisheye BEV maps and a validation package.

The output homography is explicitly H_bev_from_rectified.  A raw 180-degree
equidistant image is not related to a ground plane by one homography, so the
production path is the precomputed raw-fisheye-to-BEV cv2.remap tables.
"""

from __future__ import annotations

import argparse
import json
import math
import shutil
from collections import defaultdict
from pathlib import Path

import cv2
import numpy as np
import yaml
from scipy.optimize import linear_sum_assignment
from scipy.spatial.transform import Rotation


CAMERAS = ("fisheye_front", "fisheye_right", "fisheye_rear", "fisheye_left")
LINK_FROM_OPTICAL = np.asarray(
    [
        [0.0, 0.0, 1.0, 0.0],
        [-1.0, 0.0, 0.0, 0.0],
        [0.0, -1.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]
)


def transform_from_xyz_rpy(xyz: list[float], rpy_deg: list[float]) -> np.ndarray:
    result = np.eye(4)
    result[:3, :3] = Rotation.from_euler("xyz", rpy_deg, degrees=True).as_matrix()
    result[:3, 3] = xyz
    return result


def load_camera_link_extrinsics(path: Path, vehicle_from_roof: np.ndarray) -> dict[str, np.ndarray]:
    data = yaml.safe_load(path.read_text())
    transforms = data.get("transforms", data.get("cameras", {}))
    result = {}
    for camera in CAMERAS:
        keys = (f"{camera}/camera_link", camera)
        item = next((transforms[key] for key in keys if key in transforms), None)
        if item is None:
            raise KeyError(f"{path}: missing {camera} transform")
        if "T_vehicle_from_camera_link" in item:
            matrix = np.asarray(item["T_vehicle_from_camera_link"], np.float64).reshape(4, 4)
        elif "matrix_row_major" in item:
            parent_from_link = np.asarray(item["matrix_row_major"], np.float64).reshape(4, 4)
            parent = str(item.get("parent", ""))
            matrix = (
                vehicle_from_roof @ parent_from_link
                if "velodyne_top" in parent
                else parent_from_link
            )
        elif "xyz_m" in item and "rpy_deg" in item:
            parent_from_link = transform_from_xyz_rpy(item["xyz_m"], item["rpy_deg"])
            parent = str(item.get("parent", ""))
            matrix = (
                vehicle_from_roof @ parent_from_link
                if "velodyne_top" in parent
                else parent_from_link
            )
        else:
            raise ValueError(f"{path}: unsupported transform format for {camera}")
        result[camera] = matrix @ LINK_FROM_OPTICAL
    return result


def load_intrinsics(root: Path) -> dict[str, dict]:
    output = {}
    for camera in CAMERAS:
        info = yaml.safe_load((root / camera / "camera_info.yaml").read_text())
        report = json.loads((root / camera / "report.json").read_text())
        output[camera] = {
            "K": np.asarray(info["camera_matrix"]["data"], np.float64).reshape(3, 3),
            "D": np.asarray(info["distortion_coefficients"]["data"], np.float64).reshape(4, 1),
            "size": (int(info["image_width"]), int(info["image_height"])),
            "rms": float(report["final_fit_rms_px"]),
            "median": float(report["accepted_view_error_px"]["median"]),
            "p95": float(report["accepted_view_error_px"]["p95"]),
            "accepted_views": int(report["images"]["accepted_by_error"]),
            "quality_pass": bool(report["quality_pass"]),
        }
    return output


def read_detections(path: Path) -> dict[str, dict[str, list[dict]]]:
    records: dict[str, dict[str, list[dict]]] = defaultdict(dict)
    for line in path.read_text().splitlines():
        record = json.loads(line)
        records[record["capture_id"]][record["camera"]] = record.get("boards", [])
    return records


def align_vector(source: np.ndarray, target: np.ndarray) -> np.ndarray:
    source = source / np.linalg.norm(source)
    target = target / np.linalg.norm(target)
    cross = np.cross(source, target)
    sine = np.linalg.norm(cross)
    cosine = float(np.clip(source @ target, -1.0, 1.0))
    if sine < 1e-10:
        return np.eye(3) if cosine > 0 else Rotation.from_rotvec([math.pi, 0, 0]).as_matrix()
    return Rotation.from_rotvec(cross / sine * math.atan2(sine, cosine)).as_matrix()


def refine_from_ground(seed: np.ndarray, observations: list[dict]) -> tuple[np.ndarray, dict]:
    candidates = []
    for observation in observations:
        camera_from_board = np.asarray(observation["T_camera_optical_from_board"], np.float64)
        vehicle_from_board = seed @ camera_from_board
        normal = vehicle_from_board[:3, 2]
        centre_z = float(vehicle_from_board[2, 3])
        if abs(normal[2]) >= 0.72 and abs(centre_z) <= 0.55:
            if normal[2] < 0:
                normal = -normal
            candidates.append((camera_from_board, normal, centre_z))
    detail = {"candidate_ground_boards": len(candidates), "applied": False}
    if len(candidates) < 6:
        detail["reason"] = "fewer_than_6_ground_board_observations"
        return seed.copy(), detail
    normals = np.asarray([item[1] for item in candidates])
    median_normal = np.median(normals, axis=0)
    median_normal /= np.linalg.norm(median_normal)
    angular = np.degrees(np.arccos(np.clip(normals @ median_normal, -1, 1)))
    correction = align_vector(median_normal, np.asarray([0.0, 0.0, 1.0]))
    correction_deg = float(np.degrees(Rotation.from_matrix(correction).magnitude()))
    refined = seed.copy()
    refined[:3, :3] = correction @ seed[:3, :3]
    implied_heights = []
    for camera_from_board, _, _ in candidates:
        implied_heights.append(float(-(refined[:3, :3] @ camera_from_board[:3, 3])[2]))
    new_height = float(np.median(implied_heights))
    z_abs_error = np.abs(np.asarray(implied_heights) - new_height)
    detail.update(
        {
            "normal_dispersion_p95_deg": float(np.percentile(angular, 95)),
            "orientation_correction_deg": correction_deg,
            "seed_camera_height_m": float(seed[2, 3]),
            "refined_camera_height_m": new_height,
            "height_repeatability_p95_m": float(np.percentile(z_abs_error, 95)),
        }
    )
    if (
        correction_deg <= 5.0
        and np.percentile(angular, 95) <= 7.0
        and np.percentile(z_abs_error, 95) <= 0.12
        and 0.5 <= new_height <= 2.5
    ):
        refined[2, 3] = new_height
        detail["applied"] = True
        detail["reason"] = "quality_gated_ground_plane_refinement"
    else:
        detail["reason"] = "ground_refinement_quality_gate_failed"
        refined = seed.copy()
    return refined, detail


def board_validation(extrinsics: dict[str, np.ndarray], detections: dict) -> dict:
    transformed = defaultdict(dict)
    per_camera = {}
    for camera in CAMERAS:
        ground_z, ground_angle, all_rms = [], [], []
        for capture_id, by_camera in detections.items():
            boards = []
            for board in by_camera.get(camera, []):
                camera_from_board = np.asarray(board["T_camera_optical_from_board"], np.float64)
                vehicle_from_board = extrinsics[camera] @ camera_from_board
                normal = vehicle_from_board[:3, 2]
                angle = math.degrees(math.acos(float(np.clip(abs(normal[2]), -1, 1))))
                centre = vehicle_from_board[:3, 3]
                boards.append({"centre": centre, "normal": normal})
                all_rms.append(float(board["rms_px"]))
                if angle < 35.0 and abs(centre[2]) < 0.6:
                    ground_z.append(abs(float(centre[2])))
                    ground_angle.append(angle)
            transformed[capture_id][camera] = boards
        per_camera[camera] = {
            "fresh_board_poses": len(all_rms),
            "reprojection_median_px": float(np.median(all_rms)) if all_rms else None,
            "reprojection_p95_px": float(np.percentile(all_rms, 95)) if all_rms else None,
            "classified_ground_poses": len(ground_z),
            "ground_abs_height_median_m": float(np.median(ground_z)) if ground_z else None,
            "ground_abs_height_p95_m": float(np.percentile(ground_z, 95)) if ground_z else None,
            "ground_normal_error_median_deg": float(np.median(ground_angle))
            if ground_angle
            else None,
            "ground_normal_error_p95_deg": float(np.percentile(ground_angle, 95))
            if ground_angle
            else None,
        }

    pair_distances = defaultdict(list)
    for capture_id, by_camera in transformed.items():
        for left_index, left in enumerate(CAMERAS):
            for right in CAMERAS[left_index + 1 :]:
                a, b = by_camera.get(left, []), by_camera.get(right, [])
                if not a or not b:
                    continue
                costs = np.asarray(
                    [[np.linalg.norm(x["centre"] - y["centre"]) for y in b] for x in a]
                )
                rows, cols = linear_sum_assignment(costs)
                for row, col in zip(rows, cols):
                    normal_angle = math.degrees(
                        math.acos(float(np.clip(abs(a[row]["normal"] @ b[col]["normal"]), -1, 1)))
                    )
                    if costs[row, col] <= 0.75 and normal_angle <= 25.0:
                        pair_distances[f"{left}<->{right}"].append(float(costs[row, col]))
    pairs = {}
    for name, values in pair_distances.items():
        pairs[name] = {
            "matches": len(values),
            "centre_delta_median_m": float(np.median(values)),
            "centre_delta_p95_m": float(np.percentile(values, 95)),
        }
    return {"per_camera": per_camera, "cross_camera_same_board": pairs}


def radial_stage_validation(extrinsics: dict[str, np.ndarray], detections: dict) -> dict:
    """Check the known three-stage ±0.45 m component board motion."""
    capture_ids = list(detections)
    if len(capture_ids) != 14:
        return {"available": False, "reason": "requires_selected_14_capture_sequence"}
    slices = {
        "initial": capture_ids[:3],
        "outward_1": capture_ids[3:9],
        "outward_2": capture_ids[9:14],
    }
    centres = {}
    for stage, ids in slices.items():
        by_quadrant = defaultdict(list)
        for capture_id in ids:
            for camera in CAMERAS:
                for board in detections[capture_id].get(camera, []):
                    camera_from_board = np.asarray(board["T_camera_optical_from_board"], np.float64)
                    vehicle_from_board = extrinsics[camera] @ camera_from_board
                    normal = vehicle_from_board[:3, 2]
                    centre = vehicle_from_board[:3, 3]
                    if abs(normal[2]) < 0.72 or abs(centre[2]) > 0.30:
                        continue
                    quadrant = (
                        ("front" if centre[0] >= 0 else "rear")
                        + "_"
                        + ("left" if centre[1] >= 0 else "right")
                    )
                    by_quadrant[quadrant].append(centre[:2])
        centres[stage] = {
            quadrant: np.median(values, axis=0).tolist() for quadrant, values in by_quadrant.items()
        }
    expected_sign = {
        "front_left": (1, 1),
        "front_right": (1, -1),
        "rear_left": (-1, 1),
        "rear_right": (-1, -1),
    }
    moves = {}
    for previous, current in (("initial", "outward_1"), ("outward_1", "outward_2")):
        edge = f"{previous}_to_{current}"
        moves[edge] = {}
        for quadrant, signs in expected_sign.items():
            if quadrant not in centres[previous] or quadrant not in centres[current]:
                moves[edge][quadrant] = {"available": False}
                continue
            delta = np.asarray(centres[current][quadrant]) - np.asarray(centres[previous][quadrant])
            component_error = delta - np.asarray(signs) * 0.45
            moves[edge][quadrant] = {
                "available": True,
                "measured_delta_xy_m": delta.tolist(),
                "measured_radial_distance_m": float(np.linalg.norm(delta)),
                "expected_delta_xy_m": (np.asarray(signs) * 0.45).tolist(),
                "component_error_xy_m": component_error.tolist(),
                "component_error_norm_m": float(np.linalg.norm(component_error)),
            }
    available_errors = [
        item["component_error_norm_m"]
        for edge in moves.values()
        for item in edge.values()
        if item.get("available")
    ]
    return {
        "available": True,
        "stages": {name: len(ids) for name, ids in slices.items()},
        "median_board_centres_xy_m": centres,
        "moves": moves,
        "expected_note": "each component moved about 0.45 m radially; hand placement is approximate",
        "available_move_checks": len(available_errors),
        "component_error_median_m": (
            float(np.median(available_errors)) if available_errors else None
        ),
        "component_error_max_m": (float(max(available_errors)) if available_errors else None),
    }


def direct_bev_maps(
    K: np.ndarray,
    D: np.ndarray,
    vehicle_from_optical: np.ndarray,
    width: int,
    height: int,
    mpp: float,
    origin: tuple[float, float],
    image_size: tuple[int, int] = (1600, 1200),
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    u, v = np.meshgrid(np.arange(width, dtype=np.float64), np.arange(height, dtype=np.float64))
    ground = np.stack(
        [(origin[1] - v) * mpp, (origin[0] - u) * mpp, np.zeros_like(u), np.ones_like(u)], axis=-1
    )
    optical_from_vehicle = np.linalg.inv(vehicle_from_optical)
    camera_points = ground.reshape(-1, 4) @ optical_from_vehicle.T
    xyz = camera_points[:, :3]
    projected, _ = cv2.fisheye.projectPoints(xyz.reshape(1, -1, 3), np.zeros(3), np.zeros(3), K, D)
    projected = projected.reshape(height, width, 2)
    map_x = projected[:, :, 0].astype(np.float32)
    map_y = projected[:, :, 1].astype(np.float32)
    image_width, image_height = image_size
    valid = (
        (xyz[:, 2] > 0.02).reshape(height, width)
        & (map_x >= 1)
        & (map_x < image_width - 2)
        & (map_y >= 1)
        & (map_y < image_height - 2)
    )
    map_x[~valid] = -1
    map_y[~valid] = -1
    return map_x, map_y, valid


def project_ground_grid(
    image: np.ndarray, K: np.ndarray, D: np.ndarray, vehicle_from_optical: np.ndarray, camera: str
) -> np.ndarray:
    """Overlay a one-metre vehicle-ground grid for visual extrinsic review."""
    output = image.copy()
    optical_from_vehicle = np.linalg.inv(vehicle_from_optical)

    def line(points_vehicle: np.ndarray, colour: tuple[int, int, int]) -> None:
        homogeneous = np.c_[points_vehicle, np.ones(len(points_vehicle))]
        points_optical = (homogeneous @ optical_from_vehicle.T)[:, :3]
        valid = points_optical[:, 2] > 0.03
        projected, _ = cv2.fisheye.projectPoints(
            points_optical.reshape(1, -1, 3), np.zeros(3), np.zeros(3), K, D
        )
        projected = projected.reshape(-1, 2)
        for index in range(len(projected) - 1):
            if not (valid[index] and valid[index + 1]):
                continue
            a, b = projected[index], projected[index + 1]
            if (
                np.all(a >= 0)
                and np.all(a < [image.shape[1], image.shape[0]])
                and np.all(b >= 0)
                and np.all(b < [image.shape[1], image.shape[0]])
            ):
                cv2.line(output, tuple(a.astype(int)), tuple(b.astype(int)), colour, 2, cv2.LINE_AA)

    samples = np.linspace(-4.0, 4.0, 161)
    for x in np.arange(-4.0, 4.1, 1.0):
        line(np.c_[np.full_like(samples, x), samples, np.zeros_like(samples)], (0, 210, 255))
    for y in np.arange(-4.0, 4.1, 1.0):
        line(np.c_[samples, np.full_like(samples, y), np.zeros_like(samples)], (255, 180, 0))
    cv2.putText(
        output,
        f"{camera} | 1 m ground grid: cyan X-constant, blue Y-constant",
        (24, 48),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.85,
        (0, 255, 0),
        2,
        cv2.LINE_AA,
    )
    return output


def matrix_text(matrix: np.ndarray, indent: str = "   ") -> str:
    flat = ", ".join(f"{float(value):.12g}" for value in matrix.reshape(-1))
    return (
        f"!!opencv-matrix\n{indent}rows: {matrix.shape[0]}\n"
        f"{indent}cols: {matrix.shape[1]}\n{indent}dt: d\n"
        f"{indent}data: [{flat}]"
    )


def write_opencv_yaml(
    path: Path,
    camera: str,
    intrinsics: dict,
    vehicle_from_optical: np.ndarray,
    H: np.ndarray,
    K_rect: np.ndarray,
    bev: dict,
    source: str,
    deployment_allowed: bool,
) -> None:
    K, D = intrinsics["K"], intrinsics["D"]
    text = f"""%YAML:1.0
---
camera_name: {camera.removeprefix("fisheye_")}
camera_frame: {camera}/camera_optical_frame
vehicle_frame: sensor_kit_base_link
image_width: {intrinsics["size"][0]}
image_height: {intrinsics["size"][1]}
distortion_model: equidistant
bev_width: {bev["width"]}
bev_height: {bev["height"]}
meters_per_pixel: {bev["meters_per_pixel"]:.12g}
bev_origin_u_px: {bev["origin"][0]:.12g}
bev_origin_v_px: {bev["origin"][1]:.12g}
bev_axes: \"u_right_is_vehicle_minus_y; v_down_is_vehicle_minus_x\"
extrinsic_source: \"{source}\"
deployment_allowed: {1 if deployment_allowed else 0}
calibration_status: \"{"deployable" if deployment_allowed else "review_only_reject_deployment"}\"
K: {matrix_text(K)}
D: {matrix_text(D.reshape(4, 1))}
K_rectified: {matrix_text(K_rect)}
H_bev: {matrix_text(H)}
H_bev_definition: \"p_bev ~ H_bev * p_rectified; NOT valid on raw fisheye pixels\"
R_vehicle_camera: {matrix_text(vehicle_from_optical[:3, :3])}
R_vehicle_camera_definition: \"p_vehicle = R_vehicle_camera * p_camera_optical + t_vehicle_camera_m\"
t_vehicle_camera_m: {matrix_text(vehicle_from_optical[:3, 3].reshape(3, 1))}
calibration_rms: {intrinsics["rms"]:.12g}
bev_remap_x: \"{camera}_bev_map_x.npy\"
bev_remap_y: \"{camera}_bev_map_y.npy\"
"""
    path.write_text(text)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--data-root", type=Path, required=True)
    parser.add_argument("--detections", type=Path, required=True)
    parser.add_argument("--intrinsics-root", type=Path, required=True)
    parser.add_argument("--camera-extrinsics", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--vehicle-roof-xyz-rpy", default="1.665,0,1.742,0,0,0")
    parser.add_argument("--bev-width", type=int, default=512)
    parser.add_argument("--bev-height", type=int, default=512)
    parser.add_argument("--meters-per-pixel", type=float, default=0.015)
    parser.add_argument("--origin-u", type=float, default=256.0)
    parser.add_argument("--origin-v", type=float, default=256.0)
    parser.add_argument("--no-ground-refinement", action="store_true")
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    values = [float(value) for value in args.vehicle_roof_xyz_rpy.split(",")]
    if len(values) != 6:
        raise SystemExit("--vehicle-roof-xyz-rpy requires x,y,z,roll,pitch,yaw")
    vehicle_from_roof = transform_from_xyz_rpy(values[:3], values[3:])
    extrinsic_document = yaml.safe_load(args.camera_extrinsics.read_text())
    input_deployment_allowed = bool(extrinsic_document.get("deploy_allowed", False))
    intrinsics = load_intrinsics(args.intrinsics_root)
    detections = read_detections(args.detections)
    extrinsics = load_camera_link_extrinsics(args.camera_extrinsics, vehicle_from_roof)
    refinements = {}
    if not args.no_ground_refinement:
        for camera in CAMERAS:
            observations = [
                board for by_camera in detections.values() for board in by_camera.get(camera, [])
            ]
            extrinsics[camera], refinements[camera] = refine_from_ground(
                extrinsics[camera], observations
            )

    bev = {
        "width": args.bev_width,
        "height": args.bev_height,
        "meters_per_pixel": args.meters_per_pixel,
        "origin": (args.origin_u, args.origin_v),
    }
    H_bev_from_ground = np.asarray(
        [
            [0.0, -1.0 / args.meters_per_pixel, args.origin_u],
            [-1.0 / args.meters_per_pixel, 0.0, args.origin_v],
            [0.0, 0.0, 1.0],
        ]
    )
    sample_id = max(
        detections,
        key=lambda capture_id: sum(
            len(detections[capture_id].get(camera, [])) for camera in CAMERAS
        ),
    )
    remapped, weights = {}, {}
    configurations = {}
    validation_overlays = []
    selected_dir = args.output / "selected_raw_images"
    selected_dir.mkdir(exist_ok=True)
    source = f"{args.camera_extrinsics.name} + quality-gated Part3 ground refinement"
    for camera in CAMERAS:
        item = intrinsics[camera]
        K, D = item["K"], item["D"]
        K_rect = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(
            K, D, item["size"], np.eye(3), balance=0.55, new_size=item["size"], fov_scale=1.0
        )
        optical_from_vehicle = np.linalg.inv(extrinsics[camera])
        plane = np.column_stack(
            (optical_from_vehicle[:3, 0], optical_from_vehicle[:3, 1], optical_from_vehicle[:3, 3])
        )
        H_rect_from_ground = K_rect @ plane
        H_bev_from_rect = H_bev_from_ground @ np.linalg.inv(H_rect_from_ground)
        H_bev_from_rect /= H_bev_from_rect[2, 2]
        map_x, map_y, valid = direct_bev_maps(
            K,
            D,
            extrinsics[camera],
            args.bev_width,
            args.bev_height,
            args.meters_per_pixel,
            bev["origin"],
            item["size"],
        )
        np.save(args.output / f"{camera}_bev_map_x.npy", map_x)
        np.save(args.output / f"{camera}_bev_map_y.npy", map_y)
        cv2.imwrite(str(args.output / f"{camera}_bev_validity.png"), valid.astype(np.uint8) * 255)
        write_opencv_yaml(
            args.output / f"{camera.removeprefix('fisheye_')}.yaml",
            camera,
            item,
            extrinsics[camera],
            H_bev_from_rect,
            K_rect,
            bev,
            source,
            input_deployment_allowed,
        )
        configurations[camera] = {
            "image_size": list(item["size"]),
            "K": K.tolist(),
            "D": D.reshape(-1).tolist(),
            "K_rectified": K_rect.tolist(),
            "T_vehicle_from_camera_optical": extrinsics[camera].tolist(),
            "H_bev_from_rectified": H_bev_from_rect.tolist(),
            "intrinsic_rms_px": item["rms"],
            "intrinsic_accepted_p95_px": item["p95"],
        }
        files = sorted((args.data_root / sample_id / "cameras").glob(f"*_{camera}.png"))
        if files:
            image = cv2.imread(str(files[0]))
            shutil.copy2(files[0], selected_dir / files[0].name)
            bird = cv2.remap(image, map_x, map_y, cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
            remapped[camera] = bird
            margin = np.minimum.reduce([map_x, item["size"][0]-1-map_x, map_y, item["size"][1]-1-map_y])
            weight = np.clip(margin / 140.0, 0.0, 1.0) * valid
            weights[camera] = weight.astype(np.float32)
            cv2.imwrite(str(args.output / f"{camera}_bev_preview.png"), bird)
            rectify_x, rectify_y = cv2.fisheye.initUndistortRectifyMap(
                K, D, np.eye(3), K_rect, item["size"], cv2.CV_32FC1
            )
            rectified = cv2.remap(image, rectify_x, rectify_y, cv2.INTER_LINEAR)
            via_h = cv2.warpPerspective(
                rectified, H_bev_from_rect, (args.bev_width, args.bev_height)
            )
            # A homography has no notion of the camera's forward half-space.
            # Mask the geometrically invalid rear half to prevent ceiling/sky
            # pixels being interpreted as ground behind the camera.
            via_h[~valid] = 0
            cv2.imwrite(str(args.output / f"{camera}_rectified_preview.png"), rectified)
            cv2.imwrite(str(args.output / f"{camera}_bev_via_H_preview.png"), via_h)
            overlay = project_ground_grid(image, K, D, extrinsics[camera], camera)
            cv2.imwrite(str(args.output / f"{camera}_extrinsic_validation.png"), overlay)
            validation_overlays.append(cv2.resize(overlay, (800, 600)))

    if remapped:
        numerator = np.zeros((args.bev_height, args.bev_width, 3), np.float64)
        denominator = np.zeros((args.bev_height, args.bev_width), np.float64)
        for camera, image in remapped.items():
            weight = weights[camera]
            numerator += image.astype(np.float64) * weight[:, :, None]
            denominator += weight
        stitched = numerator / np.maximum(denominator[:, :, None], 1e-9)
        stitched[denominator < 0.02] = 0
        stitched = np.clip(stitched, 0, 255).astype(np.uint8)
        origin = (int(args.origin_u), int(args.origin_v))
        cv2.circle(stitched, origin, 6, (0, 0, 255), -1, cv2.LINE_AA)
        cv2.arrowedLine(
            stitched,
            origin,
            (origin[0], origin[1] - 80),
            (0, 255, 0),
            3,
            cv2.LINE_AA,
            tipLength=0.15,
        )
        cv2.arrowedLine(
            stitched,
            origin,
            (origin[0] - 80, origin[1]),
            (255, 0, 0),
            3,
            cv2.LINE_AA,
            tipLength=0.15,
        )
        cv2.imwrite(str(args.output / "four_camera_bev_preview.png"), stitched)
    if len(validation_overlays) == 4:
        contact = cv2.vconcat(
            (cv2.hconcat(validation_overlays[:2]), cv2.hconcat(validation_overlays[2:]))
        )
        cv2.imwrite(
            str(args.output / "extrinsic_validation_contact_sheet.jpg"),
            contact,
            [cv2.IMWRITE_JPEG_QUALITY, 91],
        )

    validation = board_validation(extrinsics, detections)
    validation["radial_three_stage_motion"] = radial_stage_validation(extrinsics, detections)
    package = {
        "schema_version": 1,
        "status": ("deployable" if input_deployment_allowed else "review_only_reject_deployment"),
        "deployment_allowed": input_deployment_allowed,
        "deployment_blocker": (
            None
            if input_deployment_allowed
            else "input Part1/2 extrinsics are marked deploy_allowed=false"
        ),
        "coordinate_convention": {
            "vehicle_frame": "sensor_kit_base_link: x forward, y left, z up",
            "camera_frame": "camera optical: x right, y down, z forward",
            "extrinsic": "p_vehicle = R_vehicle_camera * p_camera_optical + t_vehicle_camera_m",
            "bev": "u right = -vehicle y; v down = -vehicle x; vehicle origin=(256,256)",
            "H_bev": "p_bev ~ H_bev * p_rectified; never apply to raw equidistant pixels",
        },
        "bev": bev,
        "sample_capture": sample_id,
        "extrinsic_source": source,
        "ground_refinements": refinements,
        "cameras": configurations,
        "validation": validation,
    }
    (args.output / "bev_calibration.json").write_text(
        json.dumps(package, ensure_ascii=False, indent=2) + "\n"
    )
    print(
        json.dumps(
            {
                "sample_capture": sample_id,
                "ground_refinements": refinements,
                "validation": validation,
            },
            ensure_ascii=False,
            indent=2,
        )
    )


if __name__ == "__main__":
    main()
