#!/usr/bin/env python3
"""Record one camera and automatically retain useful ChArUco calibration frames."""

from __future__ import annotations

import argparse
import glob
import json
import math
from pathlib import Path
import time

import cv2
import numpy as np
import yaml


RED = "\033[31m"
GREEN = "\033[32m"
RESET = "\033[0m"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--device", help="V4L2 device or stable symlink")
    source.add_argument("--camera-index", type=int, help="zero-based entry in camera_devices.yaml")
    source.add_argument("--stream-url", help="HTTP/MJPEG network camera stream")
    source.add_argument(
        "--ros-compressed-topic",
        help="ROS 2 sensor_msgs/msg/CompressedImage topic",
    )
    parser.add_argument("--config-file", type=Path)
    parser.add_argument("--camera-name")
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--squares-x", type=int, default=12)
    parser.add_argument("--squares-y", type=int, default=8)
    parser.add_argument("--square-length-m", type=float, default=0.070)
    parser.add_argument("--marker-length-m", type=float, default=0.050)
    parser.add_argument("--dictionary", default="DICT_5X5_100")
    parser.add_argument("--target-samples", type=int, default=70)
    parser.add_argument("--minimum-corners", type=int, default=20)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--pixel-format", choices=("MJPG", "UYVY", "YUYV"), default="MJPG")
    parser.add_argument("--fps", type=float, default=10.0)
    parser.add_argument("--minimum-blur-score", type=float, default=60.0)
    parser.add_argument("--minimum-interval-sec", type=float, default=0.75)
    parser.add_argument("--max-duration-sec", type=float, default=1200.0)
    parser.add_argument("--stop-keys", default="qweasdzxc")
    parser.add_argument("--no-preview", action="store_true")
    return parser.parse_args()


def warn(message: str) -> None:
    print(f"{RED}WARNING: {message}{RESET}", flush=True)


def select_configured_camera(args: argparse.Namespace) -> tuple[str, str, str]:
    if args.ros_compressed_topic:
        return (
            args.ros_compressed_topic,
            args.camera_name or "traffic_light_front",
            "ros_compressed",
        )
    if args.stream_url:
        return args.stream_url, args.camera_name or "network_camera", "network_stream"
    if args.device:
        return args.device, args.camera_name or Path(args.device).name, "v4l2"
    if args.config_file is None:
        raise SystemExit("--config-file is required with --camera-index")
    document = yaml.safe_load(args.config_file.read_text(encoding="utf-8"))
    cameras = document.get("cameras", []) if isinstance(document, dict) else []
    selected = next((entry for entry in cameras if entry.get("index") == args.camera_index), None)
    if selected is None:
        raise SystemExit(f"camera menu entry {args.camera_index + 1} is not configured")

    physical = str(selected["physical_device"])
    for pattern in selected.get("identity_globs", []):
        matches = sorted(glob.glob(str(pattern)))
        if len(matches) == 1:
            return matches[0], str(selected["camera_name"]), "v4l2"
        if len(matches) > 1:
            warn(f"identity pattern {pattern} is ambiguous; using fixed physical port")
    return physical, str(selected["camera_name"]), "v4l2"


def make_charuco(
    squares_x: int,
    squares_y: int,
    square_length: float,
    marker_length: float,
    dictionary_name: str,
):
    if not hasattr(cv2, "aruco") or not hasattr(cv2.aruco, dictionary_name):
        raise SystemExit(f"OpenCV ArUco dictionary is unavailable: {dictionary_name}")
    dictionary_id = getattr(cv2.aruco, dictionary_name)
    if hasattr(cv2.aruco, "getPredefinedDictionary"):
        dictionary = cv2.aruco.getPredefinedDictionary(dictionary_id)
    else:
        dictionary = cv2.aruco.Dictionary_get(dictionary_id)
    if hasattr(cv2.aruco, "CharucoBoard_create"):
        board = cv2.aruco.CharucoBoard_create(
            squares_x, squares_y, square_length, marker_length, dictionary
        )
    else:
        board = cv2.aruco.CharucoBoard(
            (squares_x, squares_y), square_length, marker_length, dictionary
        )
    return dictionary, board


def detect_charuco(gray: np.ndarray, dictionary, board):
    marker_corners, marker_ids, _ = cv2.aruco.detectMarkers(gray, dictionary)
    if marker_ids is None or len(marker_ids) < 2:
        return None, None
    count, corners, ids = cv2.aruco.interpolateCornersCharuco(
        marker_corners, marker_ids, gray, board
    )
    if corners is None or ids is None or int(count) < 1:
        return None, None
    return corners.astype(np.float32), ids.astype(np.int32)


def descriptor(corners: np.ndarray, width: int, height: int) -> np.ndarray:
    points = corners.reshape(-1, 2)
    center = points.mean(axis=0) / np.array([width, height], dtype=np.float32)
    hull = cv2.convexHull(points.astype(np.float32))
    area = max(float(cv2.contourArea(hull)), 1.0)
    scale = math.sqrt(area / float(width * height))
    rectangle = cv2.minAreaRect(points.astype(np.float32))
    angle = math.radians(float(rectangle[2]))
    return np.array([center[0], center[1], scale, angle], dtype=np.float64)


def sufficiently_different(candidate: np.ndarray, accepted: list[np.ndarray]) -> bool:
    if not accepted:
        return True
    for previous in accepted:
        position = float(np.linalg.norm(candidate[:2] - previous[:2]))
        scale = abs(float(candidate[2] - previous[2]))
        angle = abs(
            math.atan2(
                math.sin(float(candidate[3] - previous[3])),
                math.cos(float(candidate[3] - previous[3])),
            )
        )
        if position < 0.045 and scale < 0.035 and angle < 0.10:
            return False
    return True


def open_capture(
    source: str, args: argparse.Namespace, is_network_stream: bool
) -> cv2.VideoCapture | None:
    if is_network_stream:
        parameters: list[int] = []
        if hasattr(cv2, "CAP_PROP_OPEN_TIMEOUT_MSEC"):
            parameters.extend([cv2.CAP_PROP_OPEN_TIMEOUT_MSEC, 3000])
        if hasattr(cv2, "CAP_PROP_READ_TIMEOUT_MSEC"):
            parameters.extend([cv2.CAP_PROP_READ_TIMEOUT_MSEC, 3000])
        try:
            capture = cv2.VideoCapture(source, cv2.CAP_FFMPEG, parameters)
        except (TypeError, cv2.error):
            capture = cv2.VideoCapture(source, cv2.CAP_FFMPEG)
    else:
        capture = cv2.VideoCapture(source, cv2.CAP_V4L2)
    if is_network_stream:
        if not capture.isOpened():
            capture.release()
            return None
        return capture

    capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*getattr(args, "pixel_format", "MJPG")))
    capture.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    capture.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    capture.set(cv2.CAP_PROP_FPS, args.fps)
    if not capture.isOpened():
        capture.release()
        return None
    return capture


class RosCompressedCapture:
    """Small reconnect-safe adapter for a ROS 2 CompressedImage topic."""

    def __init__(self, topic: str, fps: float) -> None:
        try:
            import rclpy
            from rclpy.context import Context
            from rclpy.executors import SingleThreadedExecutor
            from rclpy.node import Node
            from rclpy.qos import qos_profile_sensor_data
            from sensor_msgs.msg import CompressedImage
        except ImportError as error:
            raise RuntimeError(
                "ROS 2 Python packages are unavailable; source /opt/ros/jazzy/setup.bash"
            ) from error

        self._rclpy = rclpy
        self._context = Context()
        self._rclpy.init(context=self._context)
        self._node = Node("traffic_light_intrinsic_capture", context=self._context)
        self._executor = SingleThreadedExecutor(context=self._context)
        self._executor.add_node(self._node)
        self._topic = topic
        self._latest_message = None
        self._received_sequence = 0
        self._consumed_sequence = 0
        self._wait_timeout_sec = max(0.30, 1.5 / fps)
        self.last_error = ""
        self._subscription = self._node.create_subscription(
            CompressedImage,
            topic,
            self._on_message,
            qos_profile_sensor_data,
        )

    def _on_message(self, message) -> None:
        self._latest_message = message
        self._received_sequence += 1

    def read(self) -> tuple[bool, np.ndarray | None]:
        deadline = time.monotonic() + self._wait_timeout_sec
        while self._received_sequence == self._consumed_sequence:
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                self.last_error = f"no image received from {self._topic}"
                return False, None
            self._executor.spin_once(timeout_sec=min(0.05, remaining))

        message = self._latest_message
        self._consumed_sequence = self._received_sequence
        if message is None or not message.data:
            self.last_error = f"empty CompressedImage received from {self._topic}"
            return False, None
        encoded = np.frombuffer(message.data, dtype=np.uint8)
        frame = cv2.imdecode(encoded, cv2.IMREAD_COLOR)
        if frame is None or frame.size == 0:
            self.last_error = f"invalid JPEG received from {self._topic}"
            return False, None
        self.last_error = ""
        return True, frame

    def release(self) -> None:
        if self._executor is not None:
            if self._node is not None:
                self._executor.remove_node(self._node)
            self._executor.shutdown()
            self._executor = None
        if self._node is not None:
            self._node.destroy_node()
            self._node = None
        if self._context.ok():
            self._rclpy.shutdown(context=self._context)


def draw_coverage(frame: np.ndarray, occupied: set[tuple[int, int]]) -> None:
    height, width = frame.shape[:2]
    for column in range(1, 5):
        x = round(column * width / 5)
        cv2.line(frame, (x, 0), (x, height), (80, 80, 80), 1)
    for row in range(1, 4):
        y = round(row * height / 4)
        cv2.line(frame, (0, y), (width, y), (80, 80, 80), 1)
    for column, row in occupied:
        x0, x1 = round(column * width / 5), round((column + 1) * width / 5)
        y0, y1 = round(row * height / 4), round((row + 1) * height / 4)
        cv2.rectangle(frame, (x0 + 2, y0 + 2), (x1 - 2, y1 - 2), (0, 180, 0), 2)


def main() -> int:
    args = parse_args()
    if args.squares_x < 3 or args.squares_y < 3:
        raise SystemExit("ChArUco board must have at least 3x3 squares")
    if not 0.0 < args.marker_length_m < args.square_length_m:
        raise SystemExit("marker length must be positive and smaller than square length")
    if args.minimum_corners < 8 or args.target_samples < 12:
        raise SystemExit("minimum corners must be >=8 and target samples must be >=12")
    if args.width < 0 or args.height < 0 or args.fps <= 0.0:
        raise SystemExit("camera width/height cannot be negative and fps must be positive")
    if not args.stream_url and (args.width == 0 or args.height == 0):
        raise SystemExit("V4L2 camera width and height must be positive")
    if not 0.0 < args.max_duration_sec <= 1200.0:
        raise SystemExit("maximum capture duration must be in (0, 1200] seconds")

    device, camera_name, source_kind = select_configured_camera(args)
    is_network_stream = source_kind == "network_stream"
    is_ros_compressed = source_kind == "ros_compressed"
    dictionary, board = make_charuco(
        args.squares_x,
        args.squares_y,
        args.square_length_m,
        args.marker_length_m,
        args.dictionary,
    )
    images_dir = args.output_dir / "images"
    images_dir.mkdir(parents=True, exist_ok=True)
    video_path = args.output_dir / "raw_video_mjpg.avi"
    video: cv2.VideoWriter | None = None
    actual_width = 0
    actual_height = 0

    capture: cv2.VideoCapture | None = None
    ros_capture: RosCompressedCapture | None = None
    ros_source_connected = False
    accepted: list[np.ndarray] = []
    occupied: set[tuple[int, int]] = set()
    last_saved = 0.0
    last_warning = 0.0
    last_open_attempt = 0.0
    start_mono = time.monotonic()
    stop_reason = "unknown"
    print(f"Camera: {camera_name} ({device})")
    print(f"Output: {args.output_dir}")
    print(
        "Move slowly, pause at each pose, fill the center/edges/corners; "
        f"press any of [{args.stop_keys}] in the preview window to stop."
    )
    try:
        while True:
            now_mono = time.monotonic()
            elapsed = now_mono - start_mono
            if args.max_duration_sec > 0.0 and elapsed >= args.max_duration_sec:
                stop_reason = "maximum duration reached"
                break

            if is_ros_compressed:
                if ros_capture is None:
                    try:
                        ros_capture = RosCompressedCapture(device, args.fps)
                        print(f"ROS 2 subscription ready: {device}", flush=True)
                    except RuntimeError as error:
                        raise SystemExit(str(error)) from error
                ok, frame = ros_capture.read()
                if ok and frame is not None and not ros_source_connected:
                    print(f"{GREEN}Camera connected: {device}{RESET}", flush=True)
                    ros_source_connected = True
            else:
                if capture is None and now_mono - last_open_attempt >= 1.0:
                    last_open_attempt = now_mono
                    capture = open_capture(device, args, is_network_stream)
                    if capture is not None:
                        print(f"{GREEN}Camera connected: {device}{RESET}", flush=True)
                if capture is None:
                    ok, frame = False, None
                else:
                    ok, frame = capture.read()
                    if not ok or frame is None:
                        capture.release()
                        capture = None

            if not ok or frame is None:
                if now_mono - last_warning >= 5.0:
                    if is_ros_compressed and ros_capture is not None:
                        detail = ros_capture.last_error or f"waiting for {device}"
                        warn(f"{camera_name} is offline: {detail}; subscription remains active")
                        ros_source_connected = False
                    elif capture is None:
                        warn(f"{camera_name} is offline or busy; retrying {device}")
                    else:
                        warn(f"lost {camera_name}; waiting for source reconnect")
                    last_warning = now_mono
                if not args.no_preview:
                    placeholder_width = args.width if args.width > 0 else 640
                    placeholder_height = args.height if args.height > 0 else 480
                    offline = np.zeros((placeholder_height, placeholder_width, 3), dtype=np.uint8)
                    cv2.putText(
                        offline,
                        "CAMERA OFFLINE - retrying",
                        (max(8, placeholder_width // 25), placeholder_height // 2),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        max(0.45, min(0.8, placeholder_width / 800.0)),
                        (0, 0, 255),
                        2,
                    )
                    cv2.imshow("ChArUco intrinsic capture", offline)
                    key = cv2.waitKey(50) & 0xFF
                    if key != 255 and chr(key).lower() in args.stop_keys.lower():
                        stop_reason = f"stop key {chr(key).lower()}"
                        break
                else:
                    time.sleep(0.05)
                continue
            frame_height, frame_width = frame.shape[:2]
            if not is_network_stream and (frame_width != args.width or frame_height != args.height):
                if now_mono - last_warning >= 5.0:
                    warn(
                        f"unexpected frame size {frame_width}x{frame_height}; "
                        f"expected {args.width}x{args.height}"
                    )
                    last_warning = now_mono
                if capture is not None:
                    capture.release()
                    capture = None
                continue

            if video is None:
                actual_width = frame_width
                actual_height = frame_height
                video = cv2.VideoWriter(
                    str(video_path),
                    cv2.VideoWriter_fourcc(*"MJPG"),
                    args.fps,
                    (actual_width, actual_height),
                )
                if not video.isOpened():
                    raise RuntimeError(f"cannot create MJPEG video: {video_path}")
                print(
                    f"Recording native resolution {actual_width}x{actual_height} "
                    f"at {args.fps:g} fps",
                    flush=True,
                )
            elif frame_width != actual_width or frame_height != actual_height:
                warn(
                    f"{camera_name} resolution changed from "
                    f"{actual_width}x{actual_height} to {frame_width}x{frame_height}; "
                    "reconnecting without mixing resolutions"
                )
                capture.release()
                capture = None
                continue

            video.write(frame)
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            blur_score = float(cv2.Laplacian(gray, cv2.CV_64F).var())
            corners, ids = detect_charuco(gray, dictionary, board)
            corner_count = 0 if ids is None else len(ids)
            display = frame.copy()
            saved = False
            if corners is not None and ids is not None:
                cv2.aruco.drawDetectedCornersCharuco(display, corners, ids, (0, 255, 0))
                if corner_count >= args.minimum_corners and blur_score >= args.minimum_blur_score:
                    current = descriptor(corners, frame_width, frame_height)
                    if (
                        now_mono - last_saved >= args.minimum_interval_sec
                        and sufficiently_different(current, accepted)
                    ):
                        filename = images_dir / f"frame_{len(accepted):03d}_{time.time_ns()}.png"
                        if not cv2.imwrite(str(filename), frame):
                            raise RuntimeError(f"failed to write {filename}")
                        accepted.append(current)
                        last_saved = now_mono
                        column = min(4, max(0, int(current[0] * 5)))
                        row = min(3, max(0, int(current[1] * 4)))
                        occupied.add((column, row))
                        saved = True
                        print(
                            f"saved {len(accepted):03d}/{args.target_samples}: {filename.name}, "
                            f"corners={corner_count}, blur={blur_score:.1f}",
                            flush=True,
                        )

            draw_coverage(display, occupied)
            color = (0, 255, 0) if saved else (0, 220, 255)
            if frame_width <= 480:
                cv2.rectangle(display, (0, 0), (frame_width, 48), (0, 0, 0), -1)
                cv2.putText(
                    display,
                    f"{camera_name} saved={len(accepted)}/{args.target_samples}",
                    (5, 18),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.38,
                    color,
                    1,
                )
                cv2.putText(
                    display,
                    f"corners={corner_count} blur={blur_score:.0f} time={elapsed:.0f}s",
                    (5, 39),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.38,
                    color,
                    1,
                )
            else:
                status = (
                    f"{camera_name} saved={len(accepted)}/{args.target_samples} "
                    f"corners={corner_count} blur={blur_score:.0f} time={elapsed:.0f}s"
                )
                cv2.rectangle(display, (0, 0), (frame_width, 38), (0, 0, 0), -1)
                cv2.putText(
                    display,
                    status,
                    (8, 26),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.53,
                    color,
                    2,
                )
            if not args.no_preview:
                cv2.imshow("ChArUco intrinsic capture", display)
                key = cv2.waitKey(1) & 0xFF
                if key != 255 and chr(key).lower() in args.stop_keys.lower():
                    stop_reason = f"stop key {chr(key).lower()}"
                    break
    finally:
        if capture is not None:
            capture.release()
        if ros_capture is not None:
            ros_capture.release()
        if video is not None:
            video.release()
        cv2.destroyAllWindows()

    metadata = {
        "camera_name": camera_name,
        "device": device,
        "source_type": source_kind,
        "ros_topic": device if is_ros_compressed else None,
        "resolution": [actual_width or args.width, actual_height or args.height],
        "fps": args.fps,
        "video": video_path.name,
        "accepted_images": len(accepted),
        "target_samples": args.target_samples,
        "covered_grid_cells": sorted([list(cell) for cell in occupied]),
        "stop_reason": stop_reason,
        "duration_sec": time.monotonic() - start_mono,
        "charuco": {
            "squares_x": args.squares_x,
            "squares_y": args.squares_y,
            "square_length_m": args.square_length_m,
            "marker_length_m": args.marker_length_m,
            "dictionary": args.dictionary,
        },
    }
    (args.output_dir / "capture.json").write_text(json.dumps(metadata, indent=2), encoding="utf-8")
    print(f"Capture complete: {len(accepted)} usable images, reason={stop_reason}")
    if len(accepted) < 20:
        warn("fewer than 20 usable ChArUco images; collect another session before calibration")
        return 2
    if len(accepted) < args.target_samples:
        warn(
            f"captured {len(accepted)} of the {args.target_samples} target views; "
            "collect another session if the 5x4 coverage grid is incomplete"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
