#!/usr/bin/env python3
from __future__ import annotations

import importlib.util
from pathlib import Path
import sys

import numpy as np
import pytest
import yaml


BASE = Path(__file__).resolve().parent
try:
    import rclpy  # noqa: F401
except ImportError:
    MODULE = None
else:
    SPEC = importlib.util.spec_from_file_location(
        "joint_capture_station", BASE / "joint_capture_station.py"
    )
    MODULE = importlib.util.module_from_spec(SPEC)
    sys.modules[SPEC.name] = MODULE
    SPEC.loader.exec_module(MODULE)
SOLVER_SPEC = importlib.util.spec_from_file_location(
    "solve_joint_extrinsics", BASE / "solve_joint_extrinsics.py"
)
SOLVER = importlib.util.module_from_spec(SOLVER_SPEC)
sys.modules[SOLVER_SPEC.name] = SOLVER
SOLVER_SPEC.loader.exec_module(SOLVER)
REFINER_SPEC = importlib.util.spec_from_file_location(
    "refine_camera_extrinsics", BASE / "refine_camera_extrinsics.py"
)
REFINER = importlib.util.module_from_spec(REFINER_SPEC)
sys.modules[REFINER_SPEC.name] = REFINER
REFINER_SPEC.loader.exec_module(REFINER)


def test_configuration() -> None:
    config = (
        MODULE.load_config(BASE / "config.yaml")
        if MODULE is not None
        else yaml.safe_load((BASE / "config.yaml").read_text())
    )
    names = [item["name"] for item in config["usb_capture"]["cameras"]]
    names += list(config["fisheye"]["sources"].values())
    names += [config["traffic_camera"]["name"]]
    names += [item["name"] for item in config["lidars"]["sensors"]]
    assert len(names) == len(set(names)) == 15
    board = config["tag_board"]
    width = (
        board["columns"] * board["tag_size_m"] + (board["columns"] - 1) * board["horizontal_gap_m"]
    )
    height = board["rows"] * board["tag_size_m"] + (board["rows"] - 1) * board["vertical_gap_m"]
    assert abs(width - 0.8) < 1e-9
    assert abs(height - 0.8) < 1e-9
    lidar_map = {
        item["name"]: (item["msop_port"], item["difop_port"])
        for item in config["lidars"]["sensors"]
    }
    assert lidar_map == {
        "lidar_top": (6699, 7788),
        "lidar_front_left": (6702, 7791),
        "lidar_front_right": (6700, 7789),
        "lidar_rear": (6701, 7790),
    }
    initial = yaml.safe_load((BASE / "initial_extrinsics.yaml").read_text())
    left = initial["transforms"]["velodyne_front_left_base_link"]
    right = initial["transforms"]["velodyne_front_right_base_link"]
    rear = initial["transforms"]["velodyne_rear_base_link"]
    assert left["xyz_m"] == [0.175, 0.543, -1.2744]
    assert left["rpy_deg"] == [0.0, -90.0, -135.0]
    assert right["xyz_m"] == [0.175, -0.543, -1.2789]
    assert right["rpy_deg"] == [0.0, -90.0, 135.0]
    assert rear["xyz_m"] == [-2.191, 0.0, -1.3334]
    assert rear["rpy_deg"] == [0.0, -90.0, 0.0]


@pytest.mark.skipif(MODULE is None, reason="ROS capture requires rclpy")
def test_capture_detector_current_opencv() -> None:
    import threading

    config = yaml.safe_load((BASE / "config.yaml").read_text())
    detector = MODULE.CameraTagDetector(config, None, threading.Event(), [])
    corners, ids = detector.detect(np.full((240, 320), 255, np.uint8))
    assert ids is None and not corners


def test_camera_optical_frame_convention() -> None:
    transform = SOLVER.CAMERA_OPTICAL_TO_LINK
    # ROS optical +Z forward / +X right / +Y down becomes camera_link
    # +X forward / -Y right / -Z down.
    assert np.allclose(transform[:3, :3] @ [0.0, 0.0, 1.0], [1.0, 0.0, 0.0])
    assert np.allclose(transform[:3, :3] @ [1.0, 0.0, 0.0], [0.0, -1.0, 0.0])
    assert np.allclose(transform[:3, :3] @ [0.0, 1.0, 0.0], [0.0, 0.0, -1.0])
    assert np.allclose(transform @ np.linalg.inv(transform), np.eye(4))


def test_solver_rejects_roof_projected_side_lidar_matches() -> None:
    target = {
        "direction": "front",
        "center_top_m": [4.0, 0.0, -1.25],
        "normal_top": [-1.0, 0.0, 0.0],
    }
    manifest = {
        "capture_id": "synthetic",
        "sensors": {
            "lidar_top": {"detection": {"targets": [target]}},
            "lidar_front_left": {"detection": {"targets": [{**target, "matched_from_roof": True}]}},
        },
    }
    observations = SOLVER.lidar_observations([(BASE, manifest)], "lidar_front_left", np.eye(4))
    assert observations == []


def test_camera_guided_local_lidar_board_fit() -> None:
    rng = np.random.default_rng(7)
    center = np.array([4.2, 0.35, -1.27])
    normal = np.array([-0.985, 0.0, 0.173])
    normal /= np.linalg.norm(normal)
    horizontal, vertical = REFINER.board_basis(normal)
    u, v = np.meshgrid(np.linspace(-0.44, 0.44, 21), np.linspace(-0.44, 0.44, 19))
    board = (
        center
        + u.reshape(-1, 1) * horizontal
        + v.reshape(-1, 1) * vertical
        + rng.normal(0.0, 0.004, (u.size, 1)) * normal
    )
    floor = np.column_stack(
        (rng.uniform(3.5, 4.9, 450), rng.uniform(-0.4, 1.1, 450), np.full(450, -1.742))
    )
    fitted = REFINER.fit_board_near_prediction(
        np.vstack((board, floor)), center + [0.12, -0.08, 0.07], normal + [0.03, 0.02, 0.01], 19
    )
    assert fitted is not None
    fitted_center, fitted_normal, quality = fitted
    assert np.linalg.norm(fitted_center - center) < 0.05
    assert abs(fitted_normal @ normal) > np.cos(np.deg2rad(2.0))
    assert quality["points"] >= 300


def test_gate() -> None:
    if MODULE is None:
        pytest.skip("ROS capture requires rclpy")
    config = yaml.safe_load((BASE / "config.yaml").read_text())
    kinds = {}
    for item in config["usb_capture"]["cameras"]:
        kinds[item["name"]] = "usb_camera"
    for name in config["fisheye"]["sources"].values():
        kinds[name] = "fisheye_camera"
    kinds[config["traffic_camera"]["name"]] = "traffic_camera"
    for item in config["lidars"]["sensors"]:
        kinds[item["name"]] = "lidar"
    sensors = {name: {"online": True, "detection": {}} for name in kinds}
    sensors["lidar_top"]["detection"] = {
        "target_count": 4,
        "directions": ["front", "rear", "left", "right"],
    }
    sensors["lidar_front_left"]["detection"] = {"target_count": 2, "directions": ["front", "left"]}
    sensors["lidar_front_right"]["detection"] = {
        "target_count": 2,
        "directions": ["front", "right"],
    }
    sensors["lidar_rear"]["detection"] = {"target_count": 1, "directions": ["rear"]}
    for name in ("video_front_mid", "video_rear_mid", "video_front_left", "video_front_right"):
        sensors[name]["detection"] = {"tag_count": 1}
    ready, reasons, checks = MODULE.evaluate_gate(config, sensors)
    assert ready and not reasons and all(checks.values())
    sensors["lidar_front_left"]["detection"] = {"target_count": 1, "directions": ["left"]}
    ready, reasons, _ = MODULE.evaluate_gate(config, sensors)
    assert not ready
    assert any("lidar_front_left" in item and "front" in item for item in reasons)
    sensors["lidar_front_left"]["detection"] = {"target_count": 2, "directions": ["front", "left"]}
    sensors["video_front_mid"]["detection"] = {"tag_count": 0}
    ready, reasons, _ = MODULE.evaluate_gate(config, sensors)
    assert not ready and any("video_front_mid" in item for item in reasons)


def test_lidar_square_detection() -> None:
    if MODULE is None:
        pytest.skip("ROS capture requires rclpy")
    config = yaml.safe_load((BASE / "config.yaml").read_text())
    rng = np.random.default_rng(4)
    y, z = np.meshgrid(np.linspace(-0.45, 0.45, 20), np.linspace(-1.742, -0.842, 20))
    board = np.column_stack((np.full(y.size, 4.0), y.ravel(), z.ravel()))
    board += rng.normal(0.0, 0.004, board.shape)
    noise = rng.uniform([-6, -5, -1.5], [6, 5, 1.0], size=(300, 3))
    candidates, preview = MODULE.detect_lidar_targets(
        np.vstack((board, noise)), config["lidar_target_detection"], roof=True
    )
    assert preview is not None
    assert any(item["direction"] == "front" for item in candidates)


def test_sparse_tilted_front_and_elevated_rear_detection() -> None:
    if MODULE is None:
        pytest.skip("ROS capture requires rclpy")
    config = yaml.safe_load((BASE / "config.yaml").read_text())
    options = config["lidar_target_detection"]
    rng = np.random.default_rng(14)

    # The roof lidar may illuminate only a narrow vertical strip of the front
    # board.  It is also deliberately tilted backwards.
    horizontal, visible_vertical = np.meshgrid(
        np.linspace(-0.45, 0.45, 28), np.linspace(-0.12, 0.12, 5)
    )
    angle = np.deg2rad(22.0)
    front_center = np.array([4.0, 0.0, -0.95])
    front = (
        front_center
        + horizontal.reshape(-1, 1) * np.array([0.0, 1.0, 0.0])
        + visible_vertical.reshape(-1, 1) * np.array([np.sin(angle), 0.0, np.cos(angle)])
    )

    # The rear board is suspended approximately one metre above the ground.
    horizontal, visible_vertical = np.meshgrid(
        np.linspace(-0.45, 0.45, 24), np.linspace(-0.28, 0.28, 8)
    )
    rear_center = np.array([-8.0, 0.0, -0.28])
    rear = (
        rear_center
        + horizontal.reshape(-1, 1) * np.array([0.0, 1.0, 0.0])
        + visible_vertical.reshape(-1, 1) * np.array([0.15, 0.0, 0.989])
    )
    cloud = np.vstack((front, rear)) + rng.normal(0.0, 0.003, (len(front) + len(rear), 3))
    candidates, _ = MODULE.detect_lidar_targets(cloud, options, roof=True)
    directions = {item["direction"] for item in candidates}
    assert {"front", "rear"} <= directions

    rear_only, _ = MODULE.detect_lidar_targets(
        cloud, options, roof=False, allowed_directions={"rear"}
    )
    assert rear_only and {item["direction"] for item in rear_only} == {"rear"}


def test_side_lidar_roof_referenced_match() -> None:
    if MODULE is None:
        pytest.skip("ROS capture requires rclpy")
    config = yaml.safe_load((BASE / "config.yaml").read_text())
    options = config["lidar_target_detection"]
    y, z = np.meshgrid(np.linspace(-0.44, 0.44, 18), np.linspace(-1.69, -0.82, 18))
    board_points = np.column_stack((np.full(y.size, 4.0), y.ravel(), z.ravel()))
    reference = {
        "direction": "front",
        "center_top_m": [4.0, 0.0, -1.255],
        "normal_top": [-1.0, 0.0, 0.0],
        "corners_top_m": [
            [4.0, -0.45, -1.70],
            [4.0, 0.45, -1.70],
            [4.0, 0.45, -0.80],
            [4.0, -0.45, -0.80],
        ],
    }
    matches = MODULE.match_side_lidar_to_roof_targets(
        board_points, [reference], {"front", "left"}, options
    )
    assert len(matches) == 1
    assert matches[0]["direction"] == "front"
    assert matches[0]["matched_from_roof"] is True
    shifted = board_points.copy()
    shifted[:, 0] += 0.30
    assert not MODULE.match_side_lidar_to_roof_targets(
        shifted, [reference], {"front", "left"}, options
    )


def test_aprilgrid_pose() -> None:
    board = {
        "columns": 6,
        "rows": 5,
        "first_id": 0,
        "id_order": "row_major",
        "minimum_tags_for_pose": 5,
        "tag_size_m": 0.108,
        "horizontal_gap_m": 0.0304,
        "vertical_gap_m": 0.065,
    }
    camera = np.asarray([[600.0, 0.0, 320.0], [0.0, 600.0, 240.0], [0.0, 0.0, 1.0]])
    rvec = np.asarray([0.12, -0.08, 0.04])
    tvec = np.asarray([0.1, -0.05, 3.2])
    corners, ids = [], []
    for tag_id in range(30):
        objects = SOLVER.grid_marker_corners(tag_id, board)
        projected, _ = __import__("cv2").projectPoints(objects, rvec, tvec, camera, np.zeros(5))
        corners.append(projected.reshape(1, 4, 2).astype(np.float32))
        ids.append(tag_id)
    poses = SOLVER.estimate_grid_poses(
        corners, np.asarray(ids), board, camera, np.zeros((5, 1)), "plumb_bob"
    )
    assert len(poses) == 1
    center, _, detected_ids = poses[0]
    assert detected_ids == list(range(30))
    assert np.linalg.norm(center - tvec) < 1e-4


if __name__ == "__main__":
    test_configuration()
    test_camera_optical_frame_convention()
    test_solver_rejects_roof_projected_side_lidar_matches()
    test_camera_guided_local_lidar_board_fit()
    test_gate()
    test_lidar_square_detection()
    test_sparse_tilted_front_and_elevated_rear_detection()
    test_side_lidar_roof_referenced_match()
    test_aprilgrid_pose()
    print("station tests passed")
