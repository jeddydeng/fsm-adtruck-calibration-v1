#!/usr/bin/env python3
"""Estimate lidar/camera extrinsics from accepted capture-station datasets.

The roof lidar is the optimization reference.  A printed tag must be centred on
each 900 mm physical board.  Camera observations use the AprilTag black-square
size; lidar observations use the physical board plane and its outer extents.
Nothing is written to the live Autoware calibration file.
"""

from __future__ import annotations

from dataclasses import dataclass
import argparse
import json
import math
from pathlib import Path
from typing import Any

import cv2
import numpy as np
import yaml
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation


CAMERA_OPTICAL_TO_LINK = np.array(
    [
        [0.0, 0.0, 1.0, 0.0],
        [-1.0, 0.0, 0.0, 0.0],
        [0.0, -1.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]
)


@dataclass
class Observation:
    capture_id: str
    label: str
    local_center: np.ndarray
    local_normal: np.ndarray
    reference_center: np.ndarray
    reference_normal: np.ndarray


def matrix_from_item(item: dict[str, Any]) -> np.ndarray:
    result = np.eye(4)
    result[:3, :3] = Rotation.from_euler("xyz", item["rpy_deg"], degrees=True).as_matrix()
    result[:3, 3] = item["xyz_m"]
    return result


def matrix_to_item(matrix: np.ndarray, parent: str, source: str) -> dict[str, Any]:
    xyz = [round(float(value), 8) for value in matrix[:3, 3]]
    rpy = [
        round(float(value), 7)
        for value in Rotation.from_matrix(matrix[:3, :3]).as_euler("xyz", degrees=True)
    ]
    return {"parent": parent, "xyz_m": xyz, "rpy_deg": rpy, "source": source}


def absolute_transforms(initial: dict[str, Any]) -> dict[str, np.ndarray]:
    items = initial["transforms"]
    result: dict[str, np.ndarray] = {"sensor_kit_base_link": np.eye(4)}
    unresolved = dict(items)
    while unresolved:
        progressed = False
        for name, item in list(unresolved.items()):
            if item["parent"] in result:
                result[name] = result[item["parent"]] @ matrix_from_item(item)
                del unresolved[name]
                progressed = True
        if not progressed:
            raise ValueError(
                "initial transform tree has unresolved parents: " + ", ".join(unresolved)
            )
    return result


def load_intrinsics(path: Path) -> tuple[np.ndarray, np.ndarray, str]:
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    camera = np.asarray(data["camera_matrix"]["data"], dtype=np.float64).reshape(3, 3)
    distortion = np.asarray(data["distortion_coefficients"]["data"], dtype=np.float64).reshape(
        -1, 1
    )
    return camera, distortion, str(data["distortion_model"])


def make_detector(dictionary_name: str):
    dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dictionary_name))
    # OpenCV 4.7+ replaced DetectorParameters_create() with the constructor;
    # retain compatibility with the ROS 2 Humble-era OpenCV used on the old
    # workstation as well as OpenCV 4.13 on this Mac.
    if hasattr(cv2.aruco, "DetectorParameters"):
        parameters = cv2.aruco.DetectorParameters()
    else:
        parameters = cv2.aruco.DetectorParameters_create()
    parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    if hasattr(cv2.aruco, "ArucoDetector"):
        detector = cv2.aruco.ArucoDetector(dictionary, parameters)
        return lambda image: detector.detectMarkers(image)[:2]
    return lambda image: cv2.aruco.detectMarkers(image, dictionary, parameters=parameters)[:2]


def split_marker_groups(corners: list[np.ndarray], ids: np.ndarray) -> list[list[int]]:
    """Split multiple identical AprilGrids that may share one camera image."""
    centers = np.asarray([np.asarray(value).reshape(4, 2).mean(axis=0) for value in corners])
    sizes = [
        np.mean(
            np.linalg.norm(
                np.roll(np.asarray(value).reshape(4, 2), -1, axis=0)
                - np.asarray(value).reshape(4, 2),
                axis=1,
            )
        )
        for value in corners
    ]
    threshold = max(18.0, 3.2 * float(np.median(sizes)))
    neighbours = [[] for _ in corners]
    for left in range(len(corners)):
        for right in range(left + 1, len(corners)):
            if np.linalg.norm(centers[left] - centers[right]) <= threshold:
                neighbours[left].append(right)
                neighbours[right].append(left)
    groups, visited = [], set()
    for start in range(len(corners)):
        if start in visited:
            continue
        stack, group = [start], []
        visited.add(start)
        while stack:
            current = stack.pop()
            group.append(current)
            for neighbour in neighbours[current]:
                if neighbour not in visited:
                    visited.add(neighbour)
                    stack.append(neighbour)
        groups.append(group)
    return groups


def grid_marker_corners(tag_id: int, board: dict[str, Any]) -> np.ndarray:
    first = int(board.get("first_id", 0))
    relative = tag_id - first
    columns, rows = int(board["columns"]), int(board["rows"])
    if not 0 <= relative < columns * rows:
        raise ValueError("tag ID outside configured AprilGrid")
    if board.get("id_order", "column_major") == "column_major":
        column, row = divmod(relative, rows)
    else:
        row, column = divmod(relative, columns)
    size = float(board["tag_size_m"])
    pitch_x = size + float(board["horizontal_gap_m"])
    pitch_y = size + float(board["vertical_gap_m"])
    half = size / 2.0
    center_x = (column - (columns - 1) / 2.0) * pitch_x
    center_y = (row - (rows - 1) / 2.0) * pitch_y
    corners = np.asarray(
        [
            [center_x - half, center_y + half, 0.0],
            [center_x + half, center_y + half, 0.0],
            [center_x + half, center_y - half, 0.0],
            [center_x - half, center_y - half, 0.0],
        ],
        dtype=np.float64,
    )
    return np.roll(corners, int(board.get("tag_corner_rotation_quarters", 0)), axis=0)


def estimate_grid_poses(
    corners: list[np.ndarray],
    ids: np.ndarray,
    board: dict[str, Any],
    camera: np.ndarray,
    distortion: np.ndarray,
    model: str,
) -> list[tuple[np.ndarray, np.ndarray, list[int]]]:
    results = []
    minimum = int(board.get("minimum_tags_for_pose", 5))
    for group in split_marker_groups(corners, ids):
        valid = [
            index
            for index in group
            if int(board.get("first_id", 0))
            <= int(ids[index])
            < int(board.get("first_id", 0)) + int(board["columns"]) * int(board["rows"])
        ]
        unique_ids = {int(ids[index]) for index in valid}
        if len(unique_ids) < minimum:
            continue
        objects = np.concatenate([grid_marker_corners(int(ids[index]), board) for index in valid])
        image = np.concatenate(
            [np.asarray(corners[index]).reshape(4, 2) for index in valid]
        ).astype(np.float64)
        if model == "equidistant":
            image = cv2.fisheye.undistortPoints(
                image.reshape(-1, 1, 2), camera, distortion
            ).reshape(-1, 2)
            solve_camera = np.eye(3)
            solve_distortion = None
        else:
            solve_camera = camera
            solve_distortion = distortion
        ok, rotation_vector, translation = cv2.solvePnP(
            objects, image, solve_camera, solve_distortion, flags=cv2.SOLVEPNP_ITERATIVE
        )
        if not ok:
            continue
        rotation, _ = cv2.Rodrigues(rotation_vector)
        normal = rotation[:, 2]
        normal /= np.linalg.norm(normal)
        results.append((translation.reshape(3), normal, sorted(unique_ids)))
    return results


def estimate_tag_pose(
    corners: np.ndarray, size_m: float, camera: np.ndarray, distortion: np.ndarray, model: str
) -> tuple[np.ndarray, np.ndarray]:
    """Retained single-tag helper for tests and non-grid diagnostics."""
    half = size_m / 2.0
    objects = np.asarray(
        [[-half, half, 0.0], [half, half, 0.0], [half, -half, 0.0], [-half, -half, 0.0]],
        dtype=np.float64,
    )
    image = np.asarray(corners, dtype=np.float64).reshape(4, 2)
    if model == "equidistant":
        image = cv2.fisheye.undistortPoints(image.reshape(-1, 1, 2), camera, distortion).reshape(
            -1, 2
        )
        solve_camera = np.eye(3)
        solve_distortion = None
    else:
        solve_camera = camera
        solve_distortion = distortion
    ok, rotation_vector, translation = cv2.solvePnP(
        objects, image, solve_camera, solve_distortion, flags=cv2.SOLVEPNP_IPPE_SQUARE
    )
    if not ok:
        raise RuntimeError("solvePnP failed")
    rotation, _ = cv2.Rodrigues(rotation_vector)
    center = translation.reshape(3)
    normal = rotation[:, 2]
    normal /= np.linalg.norm(normal)
    return center, normal


def top_targets(manifest: dict[str, Any]) -> dict[str, dict[str, Any]]:
    detection = manifest.get("sensors", {}).get("lidar_top", {}).get("detection", {})
    return {item["direction"]: item for item in detection.get("targets", [])}


def camera_observations(
    captures: list[tuple[Path, dict[str, Any]]],
    config: dict[str, Any],
    camera_name: str,
    detector: Any,
    initial_top_optical: np.ndarray,
) -> list[Observation]:
    result: list[Observation] = []
    for directory, manifest in captures:
        sensor = manifest["sensors"].get(camera_name)
        if sensor is None or not sensor.get("saved", True) or "file" not in sensor:
            continue
        image = cv2.imread(str(directory / sensor["file"]), cv2.IMREAD_COLOR)
        if image is None:
            continue
        intrinsic_file = directory / "intrinsics" / f"{camera_name}.yaml"
        if not intrinsic_file.is_file():
            intrinsic_file = Path(config["intrinsics_dir"]) / f"{camera_name}.yaml"
        camera, distortion, model = load_intrinsics(intrinsic_file)
        corners, ids = detector(cv2.cvtColor(image, cv2.COLOR_BGR2GRAY))
        if ids is None:
            continue
        references = top_targets(manifest)
        poses = estimate_grid_poses(
            corners, ids.flatten(), config["tag_board"], camera, distortion, model
        )
        matches: dict[str, tuple[float, np.ndarray, np.ndarray]] = {}
        for local_center, local_normal, _tag_ids in poses:
            rough_center = (initial_top_optical @ np.r_[local_center, 1.0])[:3]
            distances = {
                direction: np.linalg.norm(
                    rough_center - np.asarray(target["center_top_m"], dtype=float)
                )
                for direction, target in references.items()
            }
            if not distances:
                continue
            direction = min(distances, key=distances.get)
            distance = float(distances[direction])
            if distance <= 2.0 and (direction not in matches or distance < matches[direction][0]):
                matches[direction] = (distance, local_center, local_normal)
        for direction, (_, local_center, local_normal) in matches.items():
            target = references[direction]
            result.append(
                Observation(
                    manifest["capture_id"],
                    direction,
                    local_center,
                    local_normal,
                    np.asarray(target["center_top_m"], dtype=float),
                    np.asarray(target["normal_top"], dtype=float),
                )
            )
    return result


def lidar_observations(
    captures: list[tuple[Path, dict[str, Any]]], name: str, _initial_top_sensor: np.ndarray
) -> list[Observation]:
    result: list[Observation] = []
    for _, manifest in captures:
        references = top_targets(manifest)
        if not references:
            continue
        candidates = manifest["sensors"].get(name, {}).get("detection", {}).get("targets", [])
        for candidate in candidates:
            # The capture UI used to project the roof-lidar target into the
            # lower-lidar frame and then copy it back as a "match".  Such a
            # record contains no independent lower-lidar measurement and would
            # manufacture a perfect zero residual.  Only genuine local fits may
            # participate in calibration.
            if candidate.get("matched_from_roof", False):
                continue
            if "center_sensor_m" not in candidate or "normal_sensor" not in candidate:
                continue
            initial_center = np.asarray(candidate["center_top_m"], dtype=float)
            distances = {
                direction: np.linalg.norm(
                    initial_center - np.asarray(target["center_top_m"], dtype=float)
                )
                for direction, target in references.items()
            }
            direction = min(distances, key=distances.get)
            if distances[direction] > 1.2:
                continue
            center_local = np.asarray(candidate["center_sensor_m"], dtype=float)
            normal_local = np.asarray(candidate["normal_sensor"], dtype=float)
            target = references[direction]
            result.append(
                Observation(
                    manifest["capture_id"],
                    direction,
                    center_local,
                    normal_local,
                    np.asarray(target["center_top_m"], dtype=float),
                    np.asarray(target["normal_top"], dtype=float),
                )
            )
    return result


def optimize(
    observations: list[Observation], initial: np.ndarray
) -> tuple[np.ndarray, dict[str, Any]]:
    if len(observations) < 5:
        raise ValueError(f"需要至少 5 个有效共视观测，当前只有 {len(observations)}")
    reference_centers = np.asarray([item.reference_center for item in observations])
    if np.linalg.matrix_rank(reference_centers - reference_centers.mean(axis=0), tol=0.08) < 2:
        raise ValueError("靶板中心分布退化：需要改变距离/横向位置/高度后重新采集")
    initial_rotation = initial[:3, :3]
    signs = []
    for item in observations:
        signs.append(
            1.0 if (initial_rotation @ item.local_normal) @ item.reference_normal >= 0 else -1.0
        )
    x0 = np.r_[Rotation.from_matrix(initial_rotation).as_rotvec(), initial[:3, 3]]

    def residual(values: np.ndarray) -> np.ndarray:
        rotation = Rotation.from_rotvec(values[:3]).as_matrix()
        translation = values[3:]
        parts: list[np.ndarray] = []
        for item, sign in zip(observations, signs):
            center_error = rotation @ item.local_center + translation - item.reference_center
            normal_error = rotation @ (item.local_normal * sign) - item.reference_normal
            parts.extend((center_error / 0.035, normal_error / 0.06))
        return np.concatenate(parts)

    solution = least_squares(residual, x0, loss="soft_l1", f_scale=1.0, max_nfev=800)
    matrix = np.eye(4)
    matrix[:3, :3] = Rotation.from_rotvec(solution.x[:3]).as_matrix()
    matrix[:3, 3] = solution.x[3:]
    center_errors, angle_errors = [], []
    for item, sign in zip(observations, signs):
        center_errors.append(
            np.linalg.norm(
                matrix[:3, :3] @ item.local_center + matrix[:3, 3] - item.reference_center
            )
        )
        dot = float(
            np.clip(
                (matrix[:3, :3] @ (item.local_normal * sign)) @ item.reference_normal, -1.0, 1.0
            )
        )
        angle_errors.append(math.degrees(math.acos(dot)))
    report = {
        "success": bool(solution.success),
        "message": solution.message,
        "observation_count": len(observations),
        "capture_count": len({item.capture_id for item in observations}),
        "center_rms_m": round(float(np.sqrt(np.mean(np.square(center_errors)))), 6),
        "center_max_m": round(float(max(center_errors)), 6),
        "normal_rms_deg": round(float(np.sqrt(np.mean(np.square(angle_errors)))), 5),
        "normal_max_deg": round(float(max(angle_errors)), 5),
    }
    return matrix, report


def load_captures(root: Path) -> list[tuple[Path, dict[str, Any]]]:
    result = []
    for manifest_path in sorted(root.glob("*/manifest.json")):
        try:
            manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
            if manifest.get("sensors"):
                result.append((manifest_path.parent, manifest))
        except (OSError, ValueError, KeyError):
            continue
    return result


def main() -> int:
    base = Path(__file__).resolve().parent
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, default=base / "config.yaml")
    parser.add_argument("--initial", type=Path, default=base / "initial_extrinsics.yaml")
    parser.add_argument("--data-root", type=Path)
    parser.add_argument("--output", type=Path, default=base / "refined_extrinsics.yaml")
    parser.add_argument("--validate-only", action="store_true")
    args = parser.parse_args()
    config = yaml.safe_load(args.config.read_text(encoding="utf-8"))
    repository = Path(__file__).resolve().parents[2]
    intrinsic_root = Path(config["intrinsics_dir"]).expanduser()
    config["intrinsics_dir"] = str(intrinsic_root if intrinsic_root.is_absolute() else repository / intrinsic_root)
    initial = yaml.safe_load(args.initial.read_text(encoding="utf-8"))
    root = args.data_root or Path(config["station"]["data_root"])
    captures = load_captures(root)
    if not captures:
        raise SystemExit(f"没有找到联合外参采集数据集: {root}")
    print(f"capture sets: {len(captures)}")
    if args.validate_only:
        for directory, manifest in captures:
            missing = [
                name
                for name, item in manifest["sensors"].items()
                if (
                    not item.get("saved", True)
                    or "file" not in item
                    or not (directory / item["file"]).is_file()
                )
            ]
            print(
                f"{directory.name}: sensors={len(manifest['sensors'])}, "
                f"spread={manifest.get('software_receive_spread_ms')} ms, "
                f"missing={missing}"
            )
        return 0

    tag_size = config["tag_board"].get("tag_size_m")
    gap_x = config["tag_board"].get("horizontal_gap_m")
    gap_y = config["tag_board"].get("vertical_gap_m")
    if tag_size is None or not 0.03 <= float(tag_size) <= 0.3:
        raise SystemExit("请先在 config.yaml 填写实测 AprilTag 黑色外框 tag_size_m")
    if gap_x is None or not 0.002 <= float(gap_x) <= 0.2:
        raise SystemExit("请在 config.yaml 填写横向相邻黑框净间距 horizontal_gap_m")
    if gap_y is None or not 0.002 <= float(gap_y) <= 0.2:
        raise SystemExit("请在 config.yaml 填写纵向相邻黑框净间距 vertical_gap_m")
    board = config["tag_board"]
    grid_width = int(board["columns"]) * float(tag_size) + (int(board["columns"]) - 1) * float(
        gap_x
    )
    grid_height = int(board["rows"]) * float(tag_size) + (int(board["rows"]) - 1) * float(gap_y)
    if (
        grid_width > float(board["outer_width_m"]) + 0.005
        or grid_height > float(board["outer_height_m"]) + 0.005
    ):
        raise SystemExit("Tag 尺寸/间距组成的网格大于 900 mm 板，请重新测量")

    absolute = absolute_transforms(initial)
    top_from_sensor = np.linalg.inv(absolute["velodyne_top_base_link"])
    detector = make_detector(config["tag_board"]["dictionary"])
    results: dict[str, Any] = {}
    reports: dict[str, Any] = {}

    for frame, sensor_name in (
        (item["frame_id"], item["name"]) for item in config["lidars"]["sensors"]
        if item["frame_id"] != "velodyne_top_base_link"
    ):
        initial_top = top_from_sensor @ absolute[frame]
        observations = lidar_observations(captures, sensor_name, initial_top)
        try:
            refined, report = optimize(observations, initial_top)
            results[frame] = matrix_to_item(
                refined, "velodyne_top_base_link", "joint_target_optimization"
            )
            reports[frame] = report
        except ValueError as error:
            reports[frame] = {
                "success": False,
                "error": str(error),
                "observation_count": len(observations),
            }

    camera_names = [item["name"] for item in config["usb_capture"]["cameras"]]
    camera_names += list(config["fisheye"]["sources"].values())
    if config["traffic_camera"].get("enabled", True):
        camera_names += [config["traffic_camera"]["name"]]
    for camera_name in camera_names:
        frame = f"{camera_name}/camera_link"
        if frame not in absolute or not ((Path(config["intrinsics_dir"]) / f"{camera_name}.yaml").exists() or any((directory / "intrinsics" / f"{camera_name}.yaml").exists() for directory, _ in captures)):
            reports[frame] = {"success": False, "error": "Black vehicle camera initial pose or intrinsics unavailable; calibrate before solving."}
            continue
        initial_top_link = top_from_sensor @ absolute[frame]
        initial_top_optical = initial_top_link @ CAMERA_OPTICAL_TO_LINK
        observations = camera_observations(
            captures, config, camera_name, detector, initial_top_optical
        )
        try:
            refined_optical, report = optimize(observations, initial_top_optical)
            refined_link = refined_optical @ np.linalg.inv(CAMERA_OPTICAL_TO_LINK)
            refined_sensor_link = absolute["velodyne_top_base_link"] @ refined_link
            results[frame] = matrix_to_item(
                refined_sensor_link, "sensor_kit_base_link", "joint_target_optimization"
            )
            reports[frame] = report
        except ValueError as error:
            reports[frame] = {
                "success": False,
                "error": str(error),
                "observation_count": len(observations),
            }

    output = {
        "schema_version": 1,
        "warning": "Review residuals and RViz overlay before applying to Autoware.",
        "reference_frame": "velodyne_top_base_link",
        "capture_sets": len(captures),
        "transforms": results,
        "quality": reports,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        yaml.safe_dump(output, allow_unicode=True, sort_keys=False), encoding="utf-8"
    )
    succeeded = sum(bool(value.get("success")) for value in reports.values())
    print(f"solved {succeeded}/{len(reports)} sensors -> {args.output}")
    print("No live Autoware TF file was modified.")
    return 0 if succeeded else 2


if __name__ == "__main__":
    raise SystemExit(main())
