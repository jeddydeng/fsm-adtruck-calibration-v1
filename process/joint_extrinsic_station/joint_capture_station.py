#!/usr/bin/env python3
"""Safe joint lidar/camera calibration capture station.

This program deliberately starts only calibration lidar drivers.  It never starts
the vehicle interface, CAN, planning, or control.  It continuously reconnects USB
and Pi camera streams, detects AprilTag 16h5 in images, detects the 900 mm physical
planar targets in point clouds, and atomically saves a software-synchronised set.
"""

from __future__ import annotations

from collections import defaultdict, deque
from dataclasses import dataclass, field
from datetime import datetime
import argparse
import glob
import json
import math
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import threading
import time
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any, Callable
from urllib.error import HTTPError, URLError
from urllib.parse import quote, urlencode, urlparse
from urllib.request import Request, urlopen

import cv2
import numpy as np
import yaml
from scipy import sparse
from scipy.sparse.csgraph import connected_components
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, PointCloud2, PointField


RED = "\033[31m"
GREEN = "\033[32m"
YELLOW = "\033[33m"
RESET = "\033[0m"


def now_iso() -> str:
    return datetime.now().astimezone().isoformat(timespec="milliseconds")


def atomic_bytes(path: Path, payload: bytes) -> None:
    temporary = path.with_name(f".{path.name}.{os.getpid()}.tmp")
    try:
        with temporary.open("xb") as stream:
            stream.write(payload)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    finally:
        temporary.unlink(missing_ok=True)


def json_bytes(value: Any) -> bytes:
    return (
        json.dumps(value, ensure_ascii=False, indent=2, sort_keys=True, allow_nan=False) + "\n"
    ).encode("utf-8")


class WarningLimiter:
    def __init__(self, period_sec: float) -> None:
        self.period_sec = period_sec
        self._last: dict[str, float] = {}
        self._lock = threading.Lock()

    def warning(self, key: str, message: str) -> None:
        now = time.monotonic()
        with self._lock:
            if now - self._last.get(key, -1e9) < self.period_sec:
                return
            self._last[key] = now
        print(f"{RED}WARNING [{key}]: {message}{RESET}", file=sys.stderr, flush=True)


@dataclass(frozen=True)
class Sample:
    name: str
    kind: str
    payload: Any
    received_mono_ns: int
    received_unix_ns: int
    source_stamp_ns: int | None = None
    sequence: int = 0
    metadata: dict[str, Any] = field(default_factory=dict)


class SharedState:
    def __init__(
        self, config: dict[str, Any], sensor_kinds: dict[str, str], mode: str = "joint"
    ) -> None:
        self.config = config
        self.sensor_kinds = sensor_kinds
        self.mode = mode
        self.samples: dict[str, Sample] = {}
        self.detections: dict[str, dict[str, Any]] = {}
        self.errors: dict[str, str] = {}
        self.previews: dict[str, bytes] = {}
        self.times: dict[str, deque[int]] = defaultdict(lambda: deque(maxlen=40))
        self.lock = threading.RLock()
        self.capture_lock = threading.Lock()
        self.last_capture: dict[str, Any] | None = None

    def update_sample(self, sample: Sample) -> None:
        with self.lock:
            self.samples[sample.name] = sample
            self.times[sample.name].append(sample.received_mono_ns)
            self.errors.pop(sample.name, None)

    def set_error(self, name: str, message: str) -> None:
        with self.lock:
            self.errors[name] = message

    def set_detection(
        self, name: str, detection: dict[str, Any], preview: bytes | None = None
    ) -> None:
        with self.lock:
            self.detections[name] = detection
            if preview is not None:
                self.previews[name] = preview

    def fps(self, name: str) -> float:
        points = self.times.get(name)
        if not points or len(points) < 2 or points[-1] <= points[0]:
            return 0.0
        return (len(points) - 1) * 1e9 / (points[-1] - points[0])

    def online_timeout(self, name: str) -> float:
        kind = self.sensor_kinds[name]
        if kind == "usb_camera":
            return float(self.config["usb_capture"]["online_timeout_sec"])
        if kind == "fisheye_camera":
            return float(self.config["fisheye"]["online_timeout_sec"])
        if kind == "traffic_camera":
            return float(self.config["traffic_camera"]["online_timeout_sec"])
        return float(self.config["lidars"]["online_timeout_sec"])

    def snapshot(self) -> tuple[dict[str, Sample], dict[str, dict[str, Any]]]:
        with self.lock:
            return dict(self.samples), json.loads(json.dumps(self.detections))

    def status(self) -> dict[str, Any]:
        now_ns = time.monotonic_ns()
        sensors: dict[str, Any] = {}
        with self.lock:
            for name, kind in self.sensor_kinds.items():
                sample = self.samples.get(name)
                age = (now_ns - sample.received_mono_ns) / 1e9 if sample is not None else None
                online = age is not None and age <= self.online_timeout(name)
                detection = self.detections.get(name, {})
                sensors[name] = {
                    "kind": kind,
                    "online": online,
                    "age_sec": round(age, 3) if age is not None else None,
                    "received_unix_ns": (sample.received_unix_ns if sample is not None else None),
                    "received_at": (
                        datetime.fromtimestamp(sample.received_unix_ns / 1e9)
                        .astimezone()
                        .isoformat(timespec="milliseconds")
                        if sample is not None
                        else None
                    ),
                    "source_stamp_ns": (sample.source_stamp_ns if sample is not None else None),
                    "sequence": sample.sequence if sample is not None else None,
                    "source_at": (
                        datetime.fromtimestamp(sample.source_stamp_ns / 1e9)
                        .astimezone()
                        .isoformat(timespec="milliseconds")
                        if sample is not None and sample.source_stamp_ns
                        else None
                    ),
                    "transport_delay_ms": (
                        round((sample.received_unix_ns - sample.source_stamp_ns) / 1e6, 3)
                        if sample is not None and sample.source_stamp_ns
                        else None
                    ),
                    "fps": round(self.fps(name), 2),
                    "error": self.errors.get(name),
                    "detection": detection,
                    "preview": name in self.previews,
                }
            if self.mode == "traffic_camera":
                ready, reasons, checks = evaluate_traffic_gate(self.config, sensors)
            else:
                ready, reasons, checks = evaluate_gate(self.config, sensors)
            return {
                "time": now_iso(),
                "mode": self.mode,
                "ready": ready,
                "reasons": reasons,
                "checks": checks,
                "sensors": sensors,
                "sensor_count": len(self.sensor_kinds),
                "last_capture": self.last_capture,
                "metric_tag_size_configured": (
                    self.config["tag_board"].get("tag_size_m") is not None
                    and self.config["tag_board"].get("horizontal_gap_m") is not None
                    and self.config["tag_board"].get("vertical_gap_m") is not None
                ),
                "capture_allowed": True,
            }


def evaluate_traffic_gate(
    config: dict[str, Any], sensors: dict[str, Any]
) -> tuple[bool, list[str], dict[str, bool]]:
    """Advisory checks for the traffic-camera-only capture page."""
    traffic_name = config["traffic_camera"]["name"]
    traffic = sensors.get(traffic_name, {})
    traffic_online = bool(traffic.get("online", False))
    tag_count = int(traffic.get("detection", {}).get("tag_count", 0))
    lidar_names = [item["name"] for item in config["lidars"]["sensors"]]
    offline_lidars = [
        name for name in lidar_names if not sensors.get(name, {}).get("online", False)
    ]
    checks = {
        "traffic_camera_online": traffic_online,
        "traffic_camera_tag_detected": tag_count > 0,
        "all_four_lidars_online": not offline_lidars,
    }
    reasons: list[str] = []
    if not traffic_online:
        reasons.append("交通灯相机离线或图像已过期")
    elif tag_count < 1:
        reasons.append("交通灯相机暂未检测到 tag16h5 标定板")
    if offline_lidars:
        reasons.append("离线激光雷达: " + ", ".join(offline_lidars))
    return all(checks.values()), reasons, checks


def evaluate_gate(
    config: dict[str, Any], sensors: dict[str, Any]
) -> tuple[bool, list[str], dict[str, bool]]:
    checks: dict[str, bool] = {}
    reasons: list[str] = []
    offline = [name for name, value in sensors.items() if not value["online"]]
    checks["all_15_sensors_online"] = not offline
    if offline:
        reasons.append("离线传感器: " + ", ".join(offline))

    for lidar in config["lidars"]["sensors"]:
        name = lidar["name"]
        detection = sensors.get(name, {}).get("detection", {})
        found_directions = set(detection.get("directions", []))
        required_directions = set(lidar.get("required_directions", []))
        minimum_targets = int(lidar.get("minimum_targets", len(required_directions)))
        target_count = int(detection.get("target_count", 0))
        missing = sorted(required_directions - found_directions)
        passed = target_count >= minimum_targets and not missing
        checks[f"{name}_targets"] = passed
        if not passed:
            details = []
            if target_count < minimum_targets:
                details.append(f"仅找到 {target_count}/{minimum_targets} 块")
            if missing:
                details.append("缺少方向 " + ", ".join(missing))
            reasons.append(f"{name} 平面靶板未通过（{'；'.join(details)}）")

    for name in config["camera_gate"]["required_single"]:
        count = int(sensors.get(name, {}).get("detection", {}).get("tag_count", 0))
        checks[f"{name}_tag"] = count >= 1
        if count < 1:
            reasons.append(f"{name} 未检测到 tag16h5")

    for group, members in config["camera_gate"]["required_any_groups"].items():
        passed = any(
            int(sensors.get(name, {}).get("detection", {}).get("tag_count", 0)) >= 1
            for name in members
        )
        checks[f"camera_group_{group}"] = passed
        if not passed:
            reasons.append(f"相机组 {group} 至少需要一颗检测到 tag16h5")
    return all(checks.values()), reasons, checks


def resolve_device(camera: dict[str, Any]) -> Path:
    candidates: list[str] = []
    for pattern in camera.get("identity_globs", []):
        candidates.extend(sorted(glob.glob(str(pattern))))
    candidates.append(str(camera["device"]))
    for candidate in candidates:
        path = Path(candidate)
        if path.exists():
            return path
    raise FileNotFoundError(str(camera["device"]))


def usb_id(path: Path) -> str | None:
    sys_device = Path("/sys/class/video4linux") / path.resolve().name / "device"
    if not sys_device.exists():
        return None
    current = sys_device.resolve()
    for parent in (current, *current.parents):
        vendor, product = parent / "idVendor", parent / "idProduct"
        if vendor.is_file() and product.is_file():
            return f"{vendor.read_text().strip()}:{product.read_text().strip()}".lower()
    return None


class UsbCameraWorker:
    def __init__(
        self,
        camera: dict[str, Any],
        settings: dict[str, Any],
        state: SharedState,
        stop: threading.Event,
        warnings: WarningLimiter,
    ) -> None:
        self.camera = camera
        self.settings = settings
        self.state = state
        self.stop = stop
        self.warnings = warnings
        self.thread = threading.Thread(target=self.run, name=f"usb-{camera['name']}", daemon=True)

    def start(self) -> None:
        self.thread.start()

    def join(self) -> None:
        self.thread.join(timeout=3.0)

    def open(self) -> tuple[cv2.VideoCapture, Path]:
        path = resolve_device(self.camera)
        expected = str(self.camera.get("expected_usb_id", "")).lower()
        actual = usb_id(path)
        if expected and actual != expected:
            raise RuntimeError(f"USB ID 应为 {expected}，实际为 {actual or 'unknown'}")
        cap = cv2.VideoCapture(str(path), cv2.CAP_V4L2)
        if not cap.isOpened():
            cap.release()
            raise RuntimeError(f"无法打开 {path}，可能被其他进程占用")
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        fmt = str(self.settings.get("pixel_format", "MJPG"))
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*fmt))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, int(self.settings["width"]))
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, int(self.settings["height"]))
        cap.set(cv2.CAP_PROP_FPS, float(self.settings["fps"]))
        return cap, path

    def run(self) -> None:
        name = self.camera["name"]
        sequence = 0
        while not self.stop.is_set():
            cap: cv2.VideoCapture | None = None
            try:
                cap, path = self.open()
                print(f"{GREEN}[{name}] USB connected: {path.resolve()}{RESET}", flush=True)
                failures = 0
                while not self.stop.is_set():
                    ok, frame = cap.read()
                    if not ok or frame is None or frame.size == 0:
                        failures += 1
                        if failures >= 3:
                            raise RuntimeError("连续三次读取失败")
                        continue
                    failures = 0
                    sequence += 1
                    mono, unix = time.monotonic_ns(), time.time_ns()
                    self.state.update_sample(
                        Sample(
                            name,
                            "usb_camera",
                            frame,
                            mono,
                            unix,
                            sequence=sequence,
                            metadata={
                                "device": str(path.resolve()),
                                "width": int(frame.shape[1]),
                                "height": int(frame.shape[0]),
                            },
                        )
                    )
            except Exception as error:
                message = str(error)
                self.state.set_error(name, message)
                self.warnings.warning(name, message + "; 将自动重连并跳过该路")
                self.stop.wait(float(self.settings.get("reconnect_delay_sec", 1.0)))
            finally:
                if cap is not None:
                    cap.release()


class FisheyeWorkers:
    """Independent MJPEG readers using the protocol from fisheye_raw_sdk.py."""

    def __init__(
        self,
        config: dict[str, Any],
        state: SharedState,
        stop: threading.Event,
        warnings: WarningLimiter,
    ) -> None:
        self.config = config
        self.state = state
        self.stop = stop
        self.warnings = warnings
        sys.path.insert(0, str(config["sdk_path"]))
        from camera_stream_client import CameraStreamClient  # type: ignore
        from fisheye_raw_sdk import FisheyeRawReceiver  # type: ignore

        self.client = CameraStreamClient(config["base_url"])
        self.read_mjpeg_frame: Callable[[Any], bytes | None] = FisheyeRawReceiver._read_mjpeg_frame
        self.previous_mode: str | None = None
        self.threads: list[threading.Thread] = []
        self.mode_thread: threading.Thread | None = None

    def start(self) -> None:
        try:
            self.previous_mode = str(self.client.status().get("mode", "bev"))
            result = self.client.set_mode("raw-all")
            if result.get("output_mode") != "raw-all":
                raise RuntimeError("树莓派未进入 raw-all 模式")
            print(f"{GREEN}[fisheye] mode {self.previous_mode} -> raw-all{RESET}", flush=True)
        except Exception as error:
            self.warnings.warning("fisheye", f"设置 raw-all 失败: {error}; 各路仍会重试")
        self.mode_thread = threading.Thread(
            target=self.mode_loop, name="fisheye-mode-watchdog", daemon=True
        )
        self.mode_thread.start()
        for source, name in self.config["sources"].items():
            thread = threading.Thread(
                target=self.receive_loop, args=(source, name), name=f"fisheye-{source}", daemon=True
            )
            self.threads.append(thread)
            thread.start()

    def mode_loop(self) -> None:
        while not self.stop.wait(5.0):
            try:
                if self.client.status().get("mode") != "raw-all":
                    result = self.client.set_mode("raw-all")
                    if result.get("output_mode") != "raw-all":
                        raise RuntimeError("raw-all mode was not accepted")
            except Exception as error:
                self.warnings.warning("fisheye", f"raw-all 模式看门狗重试失败: {error}")

    def receive_loop(self, source: str, name: str) -> None:
        query = urlencode({"source": source, "fps": self.config["fps"]})
        url = f"{self.config['base_url'].rstrip('/')}/api/stream/mjpeg?{query}"
        sequence = 0
        while not self.stop.is_set():
            try:
                response = urlopen(Request(url, headers={"Cache-Control": "no-store"}), timeout=2.0)
                with response:
                    while not self.stop.is_set():
                        jpeg = self.read_mjpeg_frame(response)
                        if jpeg is None:
                            raise RuntimeError("MJPEG 流结束")
                        image = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
                        if image is None:
                            continue
                        sequence += 1
                        mono, unix = time.monotonic_ns(), time.time_ns()
                        self.state.update_sample(
                            Sample(
                                name,
                                "fisheye_camera",
                                image,
                                mono,
                                unix,
                                sequence=sequence,
                                metadata={
                                    "source": source,
                                    "url": url,
                                    "width": int(image.shape[1]),
                                    "height": int(image.shape[0]),
                                },
                            )
                        )
            except (HTTPError, URLError, TimeoutError, OSError, ValueError, RuntimeError) as error:
                message = str(error)
                self.state.set_error(name, message)
                self.warnings.warning(name, message + "; 将自动重连并跳过该路")
                self.stop.wait(float(self.config.get("reconnect_delay_sec", 2.0)))

    def close(self) -> None:
        if self.mode_thread is not None:
            self.mode_thread.join(timeout=3.0)
        for thread in self.threads:
            thread.join(timeout=3.0)
        if self.previous_mode:
            try:
                if self.client.status().get("mode") == "raw-all":
                    self.client.set_mode(self.previous_mode)
                    print(f"[fisheye] restored mode {self.previous_mode}", flush=True)
            except Exception as error:
                self.warnings.warning("fisheye", f"恢复原模式失败: {error}")


class RosSensors(Node):
    def __init__(
        self, config: dict[str, Any], state: SharedState, warnings: WarningLimiter
    ) -> None:
        super().__init__("joint_extrinsic_capture_station")
        self.config = config
        self.state = state
        self.warnings = warnings
        self.sequences: dict[str, int] = defaultdict(int)
        for lidar in config["lidars"]["sensors"]:
            self.create_subscription(
                PointCloud2,
                lidar["topic"],
                lambda msg, name=lidar["name"]: self.on_cloud(name, msg),
                qos_profile_sensor_data,
            )
        traffic = config["traffic_camera"]
        self.create_subscription(
            CompressedImage, traffic["topic"], self.on_traffic, qos_profile_sensor_data
        )

    @staticmethod
    def stamp_ns(msg: Any) -> int | None:
        stamp = getattr(getattr(msg, "header", None), "stamp", None)
        if stamp is None:
            return None
        value = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
        return value if value else None

    def on_cloud(self, name: str, msg: PointCloud2) -> None:
        mono, unix = time.monotonic_ns(), time.time_ns()
        self.sequences[name] += 1
        self.state.update_sample(
            Sample(
                name,
                "lidar",
                msg,
                mono,
                unix,
                self.stamp_ns(msg),
                self.sequences[name],
                metadata={
                    "frame_id": msg.header.frame_id,
                    "width": int(msg.width),
                    "height": int(msg.height),
                    "point_count": int(msg.width * msg.height),
                },
            )
        )

    def on_traffic(self, msg: CompressedImage) -> None:
        name = self.config["traffic_camera"]["name"]
        try:
            frame = cv2.imdecode(np.frombuffer(bytes(msg.data), np.uint8), cv2.IMREAD_COLOR)
            if frame is None:
                raise ValueError("JPEG 解码失败")
            mono, unix = time.monotonic_ns(), time.time_ns()
            self.sequences[name] += 1
            self.state.update_sample(
                Sample(
                    name,
                    "traffic_camera",
                    frame,
                    mono,
                    unix,
                    self.stamp_ns(msg),
                    self.sequences[name],
                    metadata={
                        "topic": self.config["traffic_camera"]["topic"],
                        "format": msg.format,
                        "width": int(frame.shape[1]),
                        "height": int(frame.shape[0]),
                    },
                )
            )
        except Exception as error:
            self.state.set_error(name, str(error))
            self.warnings.warning(name, str(error))


class CameraTagDetector:
    def __init__(
        self,
        config: dict[str, Any],
        state: SharedState,
        stop: threading.Event,
        camera_names: list[str],
    ) -> None:
        self.config = config
        self.state = state
        self.stop = stop
        self.camera_names = camera_names
        dictionary_name = config["tag_board"]["dictionary"]
        dictionary_id = getattr(cv2.aruco, dictionary_name)
        dictionary = cv2.aruco.getPredefinedDictionary(dictionary_id)
        parameters = (
            cv2.aruco.DetectorParameters()
            if hasattr(cv2.aruco, "DetectorParameters")
            else cv2.aruco.DetectorParameters_create()
        )
        parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        if hasattr(cv2.aruco, "ArucoDetector"):
            detector = cv2.aruco.ArucoDetector(dictionary, parameters)
            self.detect = lambda gray: detector.detectMarkers(gray)[:2]
        else:
            self.detect = lambda gray: cv2.aruco.detectMarkers(
                gray, dictionary, parameters=parameters
            )[:2]
        self.thread = threading.Thread(target=self.run, name="camera-tag-detector", daemon=True)
        self.last_sequence: dict[str, int] = {}

    def start(self) -> None:
        self.thread.start()

    def run(self) -> None:
        period = 1.0 / float(self.config["station"]["camera_detection_hz"])
        while not self.stop.wait(period):
            with self.state.lock:
                samples = {name: self.state.samples.get(name) for name in self.camera_names}
            for name, sample in samples.items():
                if sample is None or self.last_sequence.get(name) == sample.sequence:
                    continue
                self.last_sequence[name] = sample.sequence
                frame = sample.payload
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                corners, ids = self.detect(gray)
                id_list = [int(value) for value in ids.flatten()] if ids is not None else []
                corner_list = [
                    np.asarray(value).reshape(-1, 2).round(3).tolist() for value in corners
                ]
                detection = {
                    "type": "tag16h5_image",
                    "tag_count": len(id_list),
                    "tag_ids": id_list,
                    "corners_px": corner_list,
                    "sample_sequence": sample.sequence,
                    "detected_at": now_iso(),
                }
                preview = frame.copy()
                if ids is not None:
                    cv2.aruco.drawDetectedMarkers(preview, corners, ids)
                label = f"{name}  tags={id_list}" if id_list else f"{name}  no tag16h5"
                color = (0, 220, 0) if id_list else (0, 0, 230)
                cv2.rectangle(preview, (0, 0), (preview.shape[1], 34), (0, 0, 0), -1)
                cv2.putText(
                    preview, label, (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2, cv2.LINE_AA
                )
                ok, encoded = cv2.imencode(".jpg", preview, [cv2.IMWRITE_JPEG_QUALITY, 72])
                self.state.set_detection(name, detection, bytes(encoded) if ok else None)

    def join(self) -> None:
        self.thread.join(timeout=3.0)


def rpy_matrix_deg(rpy: list[float]) -> np.ndarray:
    return Rotation.from_euler("xyz", rpy, degrees=True).as_matrix()


def transform_matrix(xyz: list[float], rpy: list[float]) -> np.ndarray:
    result = np.eye(4)
    result[:3, :3] = rpy_matrix_deg(rpy)
    result[:3, 3] = np.asarray(xyz, dtype=float)
    return result


def load_top_transforms(path: Path) -> dict[str, np.ndarray]:
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    transforms = data["transforms"]
    result = {"velodyne_top_base_link": np.eye(4)}
    for frame in (
        "velodyne_front_left_base_link",
        "velodyne_front_right_base_link",
        "velodyne_rear_base_link",
    ):
        item = transforms[frame]
        if item["parent"] != "velodyne_top_base_link":
            raise ValueError(f"{frame} must be relative to roof lidar")
        result[frame] = transform_matrix(item["xyz_m"], item["rpy_deg"])
    return result


POINT_TYPES = {
    PointField.INT8: ("i", 1, "i1"),
    PointField.UINT8: ("u", 1, "u1"),
    PointField.INT16: ("i", 2, "i2"),
    PointField.UINT16: ("u", 2, "u2"),
    PointField.INT32: ("i", 4, "i4"),
    PointField.UINT32: ("u", 4, "u4"),
    PointField.FLOAT32: ("f", 4, "f4"),
    PointField.FLOAT64: ("f", 8, "f8"),
}


def cloud_field_array(msg: PointCloud2, name: str) -> np.ndarray:
    field = next((value for value in msg.fields if value.name == name), None)
    if field is None or field.datatype not in POINT_TYPES:
        raise ValueError(f"PointCloud2 缺少数值字段 {name}")
    _, size, dtype_code = POINT_TYPES[field.datatype]
    if field.count != 1:
        raise ValueError(f"PointCloud2 字段 {name} count 不是 1")
    byte_order = ">" if msg.is_bigendian and size > 1 else "<"
    dtype = np.dtype(byte_order + dtype_code)
    return np.ndarray(
        (msg.height, msg.width),
        dtype=dtype,
        buffer=msg.data,
        offset=field.offset,
        strides=(msg.row_step, msg.point_step),
    )


def cloud_xyz(msg: PointCloud2, max_points: int) -> np.ndarray:
    arrays = [cloud_field_array(msg, name).reshape(-1) for name in ("x", "y", "z")]
    count = arrays[0].size
    stride = max(1, math.ceil(count / max_points))
    result = np.column_stack([value[::stride] for value in arrays]).astype(np.float64, copy=False)
    return result[np.isfinite(result).all(axis=1)]


def pointcloud_to_pcd(msg: PointCloud2) -> bytes:
    fields = [
        field
        for field in sorted(msg.fields, key=lambda item: item.offset)
        if field.datatype in POINT_TYPES and field.count > 0
    ]
    if not fields:
        raise ValueError("PointCloud2 没有可写入 PCD 的数值字段")
    names: list[str] = []
    formats: list[Any] = []
    offsets: list[int] = []
    sizes: list[str] = []
    types: list[str] = []
    counts: list[str] = []
    packed_offset = 0
    for index, field in enumerate(fields):
        pcd_type, size, code = POINT_TYPES[field.datatype]
        name = field.name or f"field_{index}"
        if name in names:
            name = f"{name}_{index}"
        names.append(name)
        scalar = np.dtype("<" + code)
        formats.append(scalar if field.count == 1 else (scalar, (field.count,)))
        offsets.append(packed_offset)
        packed_offset += size * field.count
        sizes.append(str(size))
        types.append(pcd_type.upper())
        counts.append(str(field.count))
    packed = np.empty(
        msg.width * msg.height,
        dtype=np.dtype(
            {"names": names, "formats": formats, "offsets": offsets, "itemsize": packed_offset}
        ),
    )
    for output_name, field in zip(names, fields):
        _, size, code = POINT_TYPES[field.datatype]
        order = ">" if msg.is_bigendian and size > 1 else "<"
        scalar = np.dtype(order + code)
        shape = (
            (msg.height, msg.width) if field.count == 1 else (msg.height, msg.width, field.count)
        )
        strides = (
            (msg.row_step, msg.point_step)
            if field.count == 1
            else (msg.row_step, msg.point_step, size)
        )
        values = np.ndarray(
            shape, dtype=scalar, buffer=msg.data, offset=field.offset, strides=strides
        )
        packed[output_name] = values.reshape(packed[output_name].shape)
    stamp = RosSensors.stamp_ns(msg) or 0
    viewpoint = "0 0 0 1 0 0 0"
    header = (
        "# .PCD v0.7 - Point Cloud Data file format\nVERSION 0.7\n"
        f"FIELDS {' '.join(names)}\nSIZE {' '.join(sizes)}\n"
        f"TYPE {' '.join(types)}\nCOUNT {' '.join(counts)}\n"
        f"WIDTH {msg.width}\nHEIGHT {msg.height}\nVIEWPOINT {viewpoint}\n"
        f"POINTS {msg.width * msg.height}\nDATA binary\n"
    ).encode("ascii")
    _ = stamp
    return header + packed.tobytes(order="C")


def union_find_labels(points: np.ndarray, radius: float) -> np.ndarray:
    if len(points) == 0:
        return np.empty(0, dtype=np.int32)
    pairs = cKDTree(points).query_pairs(radius, output_type="ndarray")
    if len(pairs) == 0:
        return np.arange(len(points), dtype=np.int32)
    rows = np.concatenate((pairs[:, 0], pairs[:, 1], np.arange(len(points))))
    cols = np.concatenate((pairs[:, 1], pairs[:, 0], np.arange(len(points))))
    graph = sparse.coo_matrix(
        (np.ones(len(rows), dtype=np.uint8), (rows, cols)), shape=(len(points), len(points))
    )
    _, labels = connected_components(graph, directed=False)
    return labels


def classify_direction(center: np.ndarray) -> str:
    x, y = float(center[0]), float(center[1])
    if abs(x) >= abs(y):
        return "front" if x >= 0 else "rear"
    return "left" if y >= 0 else "right"


def detect_lidar_targets(
    points_top: np.ndarray,
    options: dict[str, Any],
    roof: bool,
    sensor_origin_top: np.ndarray | None = None,
    allowed_directions: set[str] | None = None,
) -> tuple[list[dict[str, Any]], bytes | None]:
    distance = np.linalg.norm(points_top[:, :2], axis=1)
    ground_z = float(options.get("ground_z_top_m", -1.742))
    mask = (
        (distance >= float(options["range_min_m"]))
        & (distance <= float(options["range_max_m"]))
        & (points_top[:, 2] >= ground_z + float(options.get("ground_clearance_m", 0.10)))
        & (points_top[:, 2] <= float(options["z_max_m"]))
    )
    points = points_top[mask]
    if len(points) == 0:
        return [], None
    voxels = np.floor(points / float(options["voxel_m"])).astype(np.int32)
    _, indices = np.unique(voxels, axis=0, return_index=True)
    points = points[np.sort(indices)]
    max_points = int(options["max_points"])
    if len(points) > max_points:
        points = points[np.linspace(0, len(points) - 1, max_points).astype(int)]
    labels = union_find_labels(points, float(options["cluster_radius_m"]))
    counts = np.bincount(labels)
    candidates: list[dict[str, Any]] = []
    min_points = int(options["min_cluster_points"])
    sensor_origin = (
        np.zeros(3) if sensor_origin_top is None else np.asarray(sensor_origin_top, dtype=float)
    )
    for label, count in enumerate(counts):
        if count < min_points:
            continue
        cluster = points[labels == label]
        center = np.median(cluster, axis=0)
        direction = classify_direction(center)
        if allowed_directions and direction not in allowed_directions:
            continue
        height_bands = options.get("direction_center_z_m", {})
        z_limits = height_bands.get(
            direction,
            [
                options.get("target_center_z_min_m", -1e9),
                options.get("target_center_z_max_m", 1e9),
            ],
        )
        if not (float(z_limits[0]) <= center[2] <= float(z_limits[1])):
            continue
        centered = cluster - center
        covariance = centered.T @ centered / max(1, len(cluster) - 1)
        values, vectors = np.linalg.eigh(covariance)
        normal = vectors[:, 0]
        if abs(float(normal[2])) > float(options["vertical_normal_abs_z_max"]):
            continue
        view = center - sensor_origin
        view[2] = 0.0
        if np.linalg.norm(view) < 1e-6:
            continue
        view /= np.linalg.norm(view)
        horizontal_normal = normal.copy()
        horizontal_normal[2] = 0.0
        horizontal_normal /= max(np.linalg.norm(horizontal_normal), 1e-9)
        if abs(float(view @ horizontal_normal)) < float(options.get("view_normal_cos_min", 0.0)):
            continue
        residuals = np.abs(centered @ normal)
        rms = float(np.sqrt(np.mean(residuals**2)))
        if rms > float(options["plane_rms_max_m"]):
            continue
        vertical = np.array([0.0, 0.0, 1.0])
        vertical = vertical - normal * float(vertical @ normal)
        vertical /= max(np.linalg.norm(vertical), 1e-9)
        horizontal = np.cross(vertical, normal)
        horizontal /= max(np.linalg.norm(horizontal), 1e-9)
        uv = np.column_stack((centered @ horizontal, centered @ vertical))
        low, high = np.percentile(uv, [2.0, 98.0], axis=0)
        extents = high - low
        horizontal_min = float(
            options.get("target_horizontal_extent_min_m", options.get("target_extent_min_m", 0.5))
        )
        horizontal_max = float(
            options.get("target_horizontal_extent_max_m", options.get("target_extent_max_m", 1.25))
        )
        vertical_min = float(
            options.get("target_vertical_extent_min_m", options.get("target_extent_min_m", 0.5))
        )
        vertical_max = float(
            options.get("target_vertical_extent_max_m", options.get("target_extent_max_m", 1.25))
        )
        if not (
            horizontal_min <= extents[0] <= horizontal_max
            and vertical_min <= extents[1] <= vertical_max
        ):
            continue
        if normal @ center > 0:
            normal = -normal
        corners = [
            center + horizontal * u + vertical * v
            for u, v in ((low[0], low[1]), (high[0], low[1]), (high[0], high[1]), (low[0], high[1]))
        ]
        nominal = float(options["target_nominal_m"])
        width_quality = max(0.0, 1.0 - abs(float(extents[0]) - nominal) / nominal)
        visible_height_quality = min(1.0, float(extents[1]) / nominal)
        plane_quality = max(0.0, 1.0 - rms / float(options["plane_rms_max_m"]))
        confidence = max(
            0.0,
            min(1.0, 0.55 * width_quality + 0.20 * visible_height_quality + 0.25 * plane_quality),
        )
        candidates.append(
            {
                "center_top_m": np.round(center, 5).tolist(),
                "normal_top": np.round(normal, 6).tolist(),
                "corners_top_m": np.round(np.asarray(corners), 5).tolist(),
                "extent_m": np.round(extents, 4).tolist(),
                "plane_rms_m": round(rms, 5),
                "points": int(count),
                "confidence": round(confidence, 3),
                "direction": direction,
            }
        )
    candidates.sort(key=lambda value: (-value["confidence"], -value["points"]))
    if roof:
        best: dict[str, dict[str, Any]] = {}
        for candidate in candidates:
            best.setdefault(candidate["direction"], candidate)
        candidates = list(best.values())
    else:
        best = {}
        for candidate in candidates:
            best.setdefault(candidate["direction"], candidate)
        candidates = list(best.values())[:4]

    canvas = np.full((420, 420, 3), 248, np.uint8)
    scale, origin = 18.0, np.array([210.0, 210.0])
    sampled = points[:: max(1, len(points) // 3500)]
    xy = np.column_stack(
        (origin[0] - sampled[:, 1] * scale, origin[1] - sampled[:, 0] * scale)
    ).astype(int)
    valid = (xy[:, 0] >= 0) & (xy[:, 0] < 420) & (xy[:, 1] >= 0) & (xy[:, 1] < 420)
    canvas[xy[valid, 1], xy[valid, 0]] = (120, 120, 120)
    cv2.arrowedLine(canvas, (210, 210), (210, 150), (0, 0, 0), 2)
    cv2.putText(canvas, "front", (218, 158), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 0), 1)
    for candidate in candidates:
        x, y, _ = candidate["center_top_m"]
        pixel = (int(origin[0] - y * scale), int(origin[1] - x * scale))
        cv2.circle(canvas, pixel, 8, (0, 190, 0), -1)
        cv2.putText(
            canvas,
            candidate["direction"],
            (pixel[0] + 7, pixel[1]),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.42,
            (0, 120, 0),
            1,
        )
    ok, encoded = cv2.imencode(".jpg", canvas, [cv2.IMWRITE_JPEG_QUALITY, 76])
    return candidates, bytes(encoded) if ok else None


def match_side_lidar_to_roof_targets(
    points_top: np.ndarray,
    roof_targets: list[dict[str, Any]],
    allowed_directions: set[str],
    options: dict[str, Any],
) -> list[dict[str, Any]]:
    """Confirm side-lidar returns inside roof-detected physical board windows."""
    plane_tolerance = float(options.get("side_match_plane_tolerance_m", 0.07))
    border = float(options.get("side_match_border_margin_m", 0.08))
    minimum_points = int(options.get("side_match_min_points", 12))
    minimum_span = float(options.get("side_match_min_span_m", 0.20))
    ground_limit = float(options.get("ground_z_top_m", -1.742)) + float(
        options.get("side_match_ground_clearance_m", 0.015)
    )
    matches: list[dict[str, Any]] = []
    for reference in roof_targets:
        direction = str(reference.get("direction", ""))
        if direction not in allowed_directions:
            continue
        center = np.asarray(reference.get("center_top_m", []), dtype=float)
        normal = np.asarray(reference.get("normal_top", []), dtype=float)
        corners = np.asarray(reference.get("corners_top_m", []), dtype=float)
        if center.shape != (3,) or normal.shape != (3,) or corners.shape != (4, 3):
            continue
        normal /= max(np.linalg.norm(normal), 1e-9)
        axis_u = corners[1] - corners[0]
        axis_v = corners[3] - corners[0]
        axis_u /= max(np.linalg.norm(axis_u), 1e-9)
        axis_v /= max(np.linalg.norm(axis_v), 1e-9)
        corner_u = (corners - center) @ axis_u
        corner_v = (corners - center) @ axis_v
        delta = points_top - center
        point_u = delta @ axis_u
        point_v = delta @ axis_v
        mask = (
            (points_top[:, 2] >= ground_limit)
            & (np.abs(delta @ normal) <= plane_tolerance)
            & (point_u >= float(corner_u.min()) - border)
            & (point_u <= float(corner_u.max()) + border)
            & (point_v >= float(corner_v.min()) - border)
            & (point_v <= float(corner_v.max()) + border)
        )
        support = points_top[mask]
        if len(support) < minimum_points:
            continue
        support_u = point_u[mask]
        support_v = point_v[mask]
        low_u, high_u = np.percentile(support_u, [2.0, 98.0])
        low_v, high_v = np.percentile(support_v, [2.0, 98.0])
        extents = np.asarray([high_u - low_u, high_v - low_v])
        if float(np.min(extents)) < minimum_span:
            continue
        match = dict(reference)
        match.update(
            {
                "extent_m": np.round(extents, 4).tolist(),
                "points": int(len(support)),
                "confidence": round(min(1.0, len(support) / 60.0), 3),
                "matched_from_roof": True,
            }
        )
        matches.append(match)
    return matches


def annotate_lidar_preview(preview: bytes | None, targets: list[dict[str, Any]]) -> bytes | None:
    if preview is None or not targets:
        return preview
    image = cv2.imdecode(np.frombuffer(preview, np.uint8), cv2.IMREAD_COLOR)
    if image is None:
        return preview
    scale, origin = 18.0, np.array([210.0, 210.0])
    for target in targets:
        x, y, _ = target["center_top_m"]
        pixel = (int(origin[0] - y * scale), int(origin[1] - x * scale))
        cv2.circle(image, pixel, 9, (0, 220, 0), -1)
        cv2.putText(
            image,
            str(target["direction"]),
            (pixel[0] + 8, pixel[1]),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.42,
            (0, 105, 0),
            1,
        )
    ok, encoded = cv2.imencode(".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, 76])
    return bytes(encoded) if ok else preview


class LidarTargetDetector:
    def __init__(
        self,
        config: dict[str, Any],
        state: SharedState,
        stop: threading.Event,
        transforms: dict[str, np.ndarray],
        warnings: WarningLimiter,
    ) -> None:
        self.config = config
        self.state = state
        self.stop = stop
        self.transforms = transforms
        self.warnings = warnings
        self.frames = {item["name"]: item["frame_id"] for item in config["lidars"]["sensors"]}
        self.expected_directions = {
            item["name"]: set(item.get("expected_directions", []))
            for item in config["lidars"]["sensors"]
        }
        self.last_received: dict[str, int] = {}
        history_size = int(config["lidar_target_detection"].get("stability_history_frames", 3))
        self.history: dict[str, deque[list[dict[str, Any]]]] = {
            name: deque(maxlen=history_size) for name in self.frames
        }
        self.thread = threading.Thread(target=self.run, name="lidar-target-detector", daemon=True)

    def start(self) -> None:
        self.thread.start()

    def run(self) -> None:
        period = 1.0 / float(self.config["station"]["lidar_detection_hz"])
        options = self.config["lidar_target_detection"]
        while not self.stop.wait(period):
            with self.state.lock:
                samples = {name: self.state.samples.get(name) for name in self.frames}
            for name, sample in samples.items():
                if sample is None or self.last_received.get(name) == sample.received_mono_ns:
                    continue
                self.last_received[name] = sample.received_mono_ns
                try:
                    xyz = cloud_xyz(
                        sample.payload,
                        int(options.get("max_input_points", int(options["max_points"]) * 2)),
                    )
                    transform = self.transforms[self.frames[name]]
                    top = xyz @ transform[:3, :3].T + transform[:3, 3]
                    targets, preview = detect_lidar_targets(
                        top,
                        options,
                        roof=(name == "lidar_top"),
                        sensor_origin_top=transform[:3, 3],
                        allowed_directions=(self.expected_directions[name] or None),
                    )
                    if name in ("lidar_front_left", "lidar_front_right"):
                        with self.state.lock:
                            roof_targets = list(
                                self.state.detections.get("lidar_top", {}).get("targets", [])
                            )
                        if roof_targets:
                            targets = match_side_lidar_to_roof_targets(
                                top, roof_targets, self.expected_directions[name], options
                            )
                            preview = annotate_lidar_preview(preview, targets)
                    self.history[name].append(targets)
                    tolerance = float(options.get("stability_center_tolerance_m", 0.45))
                    minimum_hits = int(options.get("stability_min_hits", 2))
                    stable_targets = []
                    for target in targets:
                        center = np.asarray(target["center_top_m"], dtype=float)
                        hits = sum(
                            any(
                                previous["direction"] == target["direction"]
                                and np.linalg.norm(
                                    np.asarray(previous["center_top_m"], dtype=float) - center
                                )
                                <= tolerance
                                for previous in frame_targets
                            )
                            for frame_targets in self.history[name]
                        )
                        if hits >= minimum_hits:
                            stable = dict(target)
                            stable["stability_hits"] = hits
                            stable_targets.append(stable)
                    detection = {
                        "type": "physical_planar_target_900mm",
                        "target_count": len(stable_targets),
                        "directions": sorted({item["direction"] for item in stable_targets}),
                        "targets": stable_targets,
                        "raw_target_count": len(targets),
                        "stability_rule": (
                            f"center_within_{tolerance:.2f}m_in_at_least_"
                            f"{minimum_hits}_of_last_{self.history[name].maxlen}"
                        ),
                        "sample_received_unix_ns": sample.received_unix_ns,
                        "detected_at": now_iso(),
                    }
                    self.state.set_detection(name, detection, preview)
                except Exception as error:
                    self.state.set_error(name, f"靶板检测失败: {error}")
                    self.warnings.warning(name, f"靶板检测失败: {error}")

    def join(self) -> None:
        self.thread.join(timeout=3.0)


class LidarLaunchManager:
    def __init__(
        self,
        config: dict[str, Any],
        node: Node,
        stop: threading.Event,
        log_dir: Path,
        warnings: WarningLimiter,
    ) -> None:
        self.config = config
        self.node = node
        self.stop = stop
        self.log_dir = log_dir
        self.warnings = warnings
        self.process: subprocess.Popen[bytes] | None = None
        self.log_stream: Any = None

    def start_if_needed(self) -> None:
        if not self.config["lidars"].get("auto_start", True):
            return
        topics = [item["topic"] for item in self.config["lidars"]["sensors"]]
        publishers = [0] * len(topics)
        # DDS discovery can take a few seconds.  Rechecking avoids launching a
        # second driver against UDP ports that an existing process already owns.
        for _ in range(8):
            publishers = [len(self.node.get_publishers_info_by_topic(topic)) for topic in topics]
            if any(value > 0 for value in publishers):
                break
            if self.stop.wait(0.5):
                return
        if all(value > 0 for value in publishers):
            print("[lidar] existing four-topic driver detected; it will be reused", flush=True)
            return
        if any(value > 0 for value in publishers):
            self.warnings.warning(
                "lidar-launch",
                "仅部分雷达话题已有发布者，为避免 UDP 端口冲突不重复启动；请先停止残留雷达进程",
            )
            return
        self.log_dir.mkdir(parents=True, exist_ok=True)
        path = self.log_dir / f"lidars_{datetime.now():%Y%m%d_%H%M%S}.log"
        self.log_stream = path.open("ab", buffering=0)
        command = [
            "ros2",
            "launch",
            self.config["lidars"]["launch_package"],
            self.config["lidars"]["launch_file"],
        ]
        self.process = subprocess.Popen(
            command, stdout=self.log_stream, stderr=subprocess.STDOUT, start_new_session=True
        )
        print(f"{GREEN}[lidar] calibration-only driver started, log={path}{RESET}", flush=True)

    def close(self) -> None:
        if self.process is not None and self.process.poll() is None:
            try:
                os.killpg(self.process.pid, signal.SIGTERM)
                self.process.wait(timeout=8.0)
            except subprocess.TimeoutExpired:
                os.killpg(self.process.pid, signal.SIGKILL)
                self.process.wait(timeout=3.0)
        if self.log_stream is not None:
            self.log_stream.close()


def safe_detection(value: dict[str, Any] | None) -> dict[str, Any]:
    return json.loads(json.dumps(value or {}))


class CaptureWriter:
    def __init__(
        self, config: dict[str, Any], state: SharedState, config_path: Path, initial_path: Path
    ) -> None:
        self.config = config
        self.state = state
        self.config_path = config_path
        self.initial_path = initial_path

    def capture(self) -> dict[str, Any]:
        if not self.state.capture_lock.acquire(blocking=False):
            raise RuntimeError("已有保存任务正在执行")
        try:
            status = self.state.status()
            samples, detections = self.state.snapshot()
            samples = {
                name: sample
                for name, sample in samples.items()
                if status["sensors"].get(name, {}).get("online", False)
            }
            missing = sorted(set(self.state.sensor_kinds) - set(samples))
            if missing:
                print(
                    f"{RED}WARNING [capture]: offline sensors skipped: {', '.join(missing)}{RESET}",
                    file=sys.stderr,
                    flush=True,
                )
            capture_unix_ns = time.time_ns()
            capture_mono_ns = time.monotonic_ns()
            stamp = datetime.fromtimestamp(capture_unix_ns / 1e9).strftime("%Y%m%d_%H%M%S_%f")[:-3]
            root = Path(self.config["station"]["data_root"])
            root.mkdir(parents=True, exist_ok=True)
            final = root / stamp
            if final.exists():
                final = root / f"{stamp}_{capture_unix_ns % 1_000_000:06d}"
            stage = root / f".{final.name}.{os.getpid()}.tmp"
            stage.mkdir(mode=0o750)
            try:
                (stage / "cameras").mkdir()
                (stage / "lidars").mkdir()
                (stage / "intrinsics").mkdir()
                manifest: dict[str, Any] = {
                    "schema_version": 1,
                    "capture_mode": self.state.mode,
                    "capture_id": final.name,
                    "capture_time": datetime.fromtimestamp(capture_unix_ns / 1e9)
                    .astimezone()
                    .isoformat(timespec="milliseconds"),
                    "capture_unix_ns": capture_unix_ns,
                    "sync_type": "software_latest-sample_at_trigger",
                    "sensors": {},
                    "capture_policy": "manual_capture_advisory_only",
                    "requirements": {
                        "checks": status["checks"],
                        "met": status["ready"],
                        "warnings": status["reasons"],
                    },
                    "complete_sensor_set": not missing,
                    "missing_sensors": missing,
                    "tag_board": self.config["tag_board"],
                }
                receive_times: list[int] = []
                failed: dict[str, str] = {}
                for name in sorted(samples):
                    sample = samples[name]
                    file_stamp = datetime.fromtimestamp(sample.received_unix_ns / 1e9).strftime(
                        "%Y%m%d_%H%M%S_%f"
                    )[:-3]
                    try:
                        if sample.kind == "lidar":
                            relative = Path("lidars") / f"{file_stamp}_{name}.pcd"
                            atomic_bytes(stage / relative, pointcloud_to_pcd(sample.payload))
                        else:
                            relative = Path("cameras") / f"{file_stamp}_{name}.png"
                            ok, encoded = cv2.imencode(
                                ".png", sample.payload, [cv2.IMWRITE_PNG_COMPRESSION, 3]
                            )
                            if not ok:
                                raise RuntimeError("PNG 编码失败")
                            atomic_bytes(stage / relative, bytes(encoded))
                        receive_times.append(sample.received_mono_ns)
                        manifest["sensors"][name] = {
                            "kind": sample.kind,
                            "saved": True,
                            "file": str(relative),
                            "received_unix_ns": sample.received_unix_ns,
                            "source_stamp_ns": sample.source_stamp_ns,
                            "trigger_offset_ms": round(
                                (sample.received_mono_ns - capture_mono_ns) / 1e6, 3
                            ),
                            "metadata": sample.metadata,
                            "detection": safe_detection(detections.get(name)),
                        }
                    except Exception as error:
                        failed[name] = str(error)
                        manifest["sensors"][name] = {
                            "kind": sample.kind,
                            "saved": False,
                            "error": str(error),
                            "metadata": sample.metadata,
                        }
                        print(
                            f"{RED}WARNING [capture/{name}]: {error}; skipped{RESET}",
                            file=sys.stderr,
                            flush=True,
                        )
                manifest["failed_sensors"] = failed
                manifest["saved_sensor_count"] = len(receive_times)
                manifest["complete_sensor_set"] = not missing and not failed
                manifest["software_receive_spread_ms"] = (
                    round((max(receive_times) - min(receive_times)) / 1e6, 3)
                    if len(receive_times) >= 2
                    else 0.0
                )
                shutil.copy2(self.config_path, stage / "capture_config.yaml")
                shutil.copy2(self.initial_path, stage / "initial_extrinsics.yaml")
                intrinsics_dir = Path(self.config["intrinsics_dir"])
                for source in intrinsics_dir.glob("*.yaml"):
                    shutil.copy2(source, stage / "intrinsics" / source.name)
                atomic_bytes(stage / "manifest.json", json_bytes(manifest))
                os.replace(stage, final)
            except BaseException:
                shutil.rmtree(stage, ignore_errors=True)
                raise
            result = {
                "capture_id": final.name,
                "path": str(final),
                "time": now_iso(),
                "saved_sensor_count": manifest["saved_sensor_count"],
                "missing_sensors": missing,
                "failed_sensors": failed,
                "requirements_met": status["ready"],
                "software_receive_spread_ms": manifest["software_receive_spread_ms"],
            }
            with self.state.lock:
                self.state.last_capture = result
            print(f"{GREEN}CAPTURE COMPLETE: {final}{RESET}", flush=True)
            return result
        finally:
            self.state.capture_lock.release()


HTML = r"""<!doctype html>
<html lang="zh-CN"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>车辆 联合外参采集站</title>
<style>
:root{color-scheme:dark;--ok:#25f58b;--bad:#ff354d;--muted:#9eabba;--card:#151e29;--panel:#0d141d}
*{box-sizing:border-box}html,body{height:100%;overflow:hidden}
body{margin:0;display:grid;grid-template-rows:auto minmax(0,1fr);font:14px system-ui,-apple-system,"Segoe UI",sans-serif;background:#080d13;color:#edf4f8}
header{z-index:3;background:#101823f2;padding:10px 14px 8px;border-bottom:1px solid #344250;box-shadow:0 5px 18px #0008}
.topbar{display:grid;grid-template-columns:minmax(620px,1fr) minmax(390px,520px);gap:18px;align-items:center}
h1{font-size:21px;line-height:1.15;margin:0 0 6px}.summary{display:flex;gap:16px;align-items:center;white-space:nowrap}
#ready{font-size:16px;font-weight:800}.ok{color:var(--ok)}.bad{color:var(--bad)}.muted{color:var(--muted)}
.actions{display:grid;grid-template-columns:minmax(270px,330px) minmax(0,1fr);gap:12px;align-items:center;justify-content:end}
button{height:62px;padding:0 26px;border:2px solid #7fffc1;border-radius:13px;background:linear-gradient(135deg,#20e881,#55ffad);box-shadow:0 0 20px #25f58b66;color:#04130b;font-size:21px;font-weight:900;letter-spacing:.04em;cursor:pointer;transition:.15s transform,.15s filter}
button:hover{transform:translateY(-1px);filter:brightness(1.08)}button:active{transform:translateY(1px)}button:disabled{cursor:wait;filter:grayscale(.65);transform:none}
#result{max-height:48px;overflow:hidden;color:#d8e4ec;font-size:12px;line-height:1.35;word-break:break-all}
#reasons{margin-top:6px;height:20px;line-height:20px;white-space:nowrap;overflow:hidden;text-overflow:ellipsis;color:#ff98a4;font-size:13px}
main{min-height:0;padding:7px 10px 9px;overflow:auto}#content{height:100%;display:grid;grid-template-rows:repeat(3,minmax(0,1fr));gap:6px}
.group{min-height:0;display:grid;grid-template-rows:23px minmax(0,1fr)}.group h2{display:flex;align-items:center;justify-content:space-between;font-size:15px;line-height:20px;margin:0 3px;color:#dce8f0}
.group-count{font-size:12px;color:var(--muted);font-weight:600}.grid{min-height:0;display:grid;gap:7px}.group.lidars .grid{grid-template-columns:repeat(4,minmax(0,1fr))}.group.usb .grid{grid-template-columns:repeat(6,minmax(0,1fr))}.group.network .grid{grid-template-columns:repeat(5,minmax(0,1fr))}
.card{position:relative;min-width:0;min-height:0;overflow:hidden;display:grid;grid-template-rows:25px 19px 23px minmax(0,1fr);padding:6px;background:linear-gradient(160deg,#182330,#101720);border:3px solid var(--bad);border-radius:10px;box-shadow:0 0 12px #ff354d99,inset 0 0 10px #ff354d22;transition:border-color .18s,box-shadow .18s}
.card.pass{border-color:var(--ok);box-shadow:0 0 14px #25f58baa,inset 0 0 11px #25f58b2b}.card.offline{background:linear-gradient(160deg,#25161b,#130d10)}
.title{min-width:0;display:flex;justify-content:space-between;align-items:center;gap:5px;font-size:13px;font-weight:800}.sensor-name{min-width:0;overflow:hidden;text-overflow:ellipsis;white-space:nowrap}
.verdict{flex:none;display:inline-flex;align-items:center;gap:5px;padding:3px 7px;border-radius:999px;background:#5e101b;color:#ffcad0;font-size:12px;font-weight:900}.pass .verdict{background:#075d36;color:#baffd9}.lamp{width:10px;height:10px;border-radius:50%;background:var(--bad);box-shadow:0 0 10px var(--bad)}.pass .lamp{background:var(--ok);box-shadow:0 0 12px var(--ok)}
.meta{min-width:0;overflow:hidden;text-overflow:ellipsis;white-space:nowrap;color:var(--muted);font-size:12px}.online-text{color:#79e8b0}.offline-text{color:#ff8592}.det{min-width:0;overflow:hidden;text-overflow:ellipsis;white-space:nowrap;font-size:12px;color:#dce7ee}.error{position:absolute;left:7px;right:7px;bottom:7px;z-index:2;overflow:hidden;text-overflow:ellipsis;white-space:nowrap;padding:3px 5px;border-radius:4px;background:#4e111bdd;color:#ffc1c8;font-size:11px}
.preview{display:block;width:100%;height:100%;min-height:0;object-fit:contain;background:#05090d;border-radius:5px}.placeholder{display:grid;place-items:center;height:100%;min-height:0;border-radius:5px;background:#090e14;color:#748292;font-size:12px}
@media(max-width:1500px){html,body{overflow:auto}body{display:block}.topbar{grid-template-columns:1fr}.actions{grid-template-columns:minmax(260px,330px) 1fr}main{overflow:visible}#content{height:auto;display:block}.group{margin-bottom:10px}.grid,.group.lidars .grid,.group.usb .grid,.group.network .grid{grid-template-columns:repeat(auto-fit,minmax(230px,1fr))}.card{height:255px}}
@media(max-height:850px){.card{grid-template-rows:24px 18px 22px minmax(0,1fr)}button{height:54px}header{padding-top:7px;padding-bottom:6px}}
</style></head><body><header><div class="topbar"><div><h1>车辆 · 激光雷达 + 摄像头联合外参采集站</h1>
<div class="summary"><span id="ready">正在连接…</span><span id="clock" class="muted"></span><span id="scale" class="muted"></span></div></div>
<div class="actions"><button id="capture">采集并保存当前数据</button><span id="result" aria-live="polite"></span></div></div>
<div id="reasons" title=""></div></header><main><div id="content"></div></main>
<script>
const groups=[
{title:'四颗激光雷达',cls:'lidars',names:['lidar_top','lidar_front_left','lidar_front_right','lidar_rear']},
{title:'六颗普通 USB 摄像头',cls:'usb',names:['video_front_mid','video_front_left','video_front_right','video_rear_mid','video_rear_left','video_rear_right']},
{title:'网络摄像头（四鱼眼 + 交通灯）',cls:'network',names:['fisheye_front','fisheye_rear','fisheye_left','fisheye_right','traffic_light_front']}];let latest=null;
const lidarRequired={lidar_top:['front','rear','left','right'],lidar_front_left:['front','left'],lidar_front_right:['front','right'],lidar_rear:['rear']};
function passed(name,s){if(!s.online)return false;const d=s.detection||{};if(s.kind==='lidar'){const found=d.directions||[];return (lidarRequired[name]||[]).every(x=>found.includes(x))}return (d.tag_count||0)>0}
function detText(name,s){const d=s.detection||{};if(s.kind==='lidar'){const required=lidarRequired[name]||[];return `平面靶板 ${d.target_count||0} 个 · 已见 ${(d.directions||[]).join(', ')||'-'} · 应见 ${required.join(', ')||'-'}`}return `tag16h5 ${d.tag_count||0} 个 · ID ${(d.tag_ids||[]).join(', ')||'-'}`}
function render(x){latest=x;document.getElementById('clock').textContent=x.time;const ready=document.getElementById('ready');ready.textContent=x.ready?'● 建议条件全部通过':'● 部分检测未通过（仍可采集）';ready.className=x.ready?'ok':'bad';
document.getElementById('capture').disabled=false;const reasonText=x.reasons.length?'提示：'+x.reasons.join('；'):'全部建议条件已满足';const reasons=document.getElementById('reasons');reasons.textContent=reasonText;reasons.title=reasonText;document.getElementById('scale').textContent=x.metric_tag_size_configured?'标定板：900 mm / 有效区800 mm':'标定板尺度配置缺失';
let html='';for(const group of groups){let count=0;for(const name of group.names)if(passed(name,x.sensors[name]))count++;html+=`<section class="group ${group.cls}"><h2><span>${group.title}</span><span class="group-count">通过 ${count}/${group.names.length}</span></h2><div class="grid">`;
for(const name of group.names){const s=x.sensors[name];const pass=passed(name,s);const age=s.age_sec===null?'-':s.age_sec.toFixed(2);const detail=detText(name,s);html+=`<article class="card ${pass?'pass':'fail'} ${s.online?'online':'offline'}"><div class="title"><span class="sensor-name" title="${name}">${name}</span><span class="verdict"><i class="lamp"></i>${pass?'已通过':'未通过'}</span></div>`+
`<div class="meta"><span class="${s.online?'online-text':'offline-text'}">${s.online?'数据在线':'数据离线'}</span> · ${s.fps.toFixed(1)} Hz · ${age}s</div><div class="det" title="${detail}">${detail}</div>`+
`${s.preview?`<img class="preview" alt="${name}实时预览" src="/api/preview/${name}?t=${Date.now()}">`:`<div class="placeholder">等待预览数据</div>`}${s.error?`<div class="error" title="${s.error}">${s.error}</div>`:''}</article>`}html+='</div></section>'}document.getElementById('content').innerHTML=html;
if(x.last_capture)document.getElementById('result').textContent=`最近保存：${x.last_capture.saved_sensor_count}/15 路 · ${x.last_capture.path}`}
async function poll(){try{const response=await fetch('/api/status',{cache:'no-store'});render(await response.json())}catch(e){const ready=document.getElementById('ready');ready.textContent='● 页面与采集后端失去连接';ready.className='bad'}setTimeout(poll,1000)}
document.getElementById('capture').onclick=async()=>{const button=document.getElementById('capture');button.disabled=true;button.textContent='正在保存，请稍候…';try{const response=await fetch('/api/capture',{method:'POST'});const data=await response.json();if(!response.ok)throw Error(data.error);document.getElementById('result').textContent=`保存完成：${data.saved_sensor_count}/15 路 · ${data.path}`}catch(e){document.getElementById('result').textContent=`保存失败：${e.message}`}finally{button.textContent='采集并保存当前数据';button.disabled=false}};poll();
</script></body></html>"""


TRAFFIC_HTML = r"""<!doctype html>
<html lang="zh-CN"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>交通灯相机专项标定采集</title>
<style>
:root{color-scheme:dark;--ok:#26ed8a;--warn:#ffc34a;--bad:#ff4157;--muted:#9eabba;--panel:#121c27}
*{box-sizing:border-box}body{margin:0;min-height:100vh;font:15px system-ui,-apple-system,"Segoe UI",sans-serif;background:#070c12;color:#eef5f8}
header{position:sticky;top:0;z-index:5;display:grid;grid-template-columns:1fr auto;gap:22px;align-items:center;padding:15px 22px;background:#101823f5;border-bottom:1px solid #344250;box-shadow:0 5px 20px #0009}
h1{margin:0 0 7px;font-size:25px}.subtitle{color:var(--muted)}#overall{margin-left:12px;font-size:17px;font-weight:850}.ok{color:var(--ok)}.bad{color:var(--bad)}
button{height:66px;padding:0 34px;border:2px solid #9affce;border-radius:14px;background:linear-gradient(135deg,#20dd7f,#65ffb7);box-shadow:0 0 22px #25ef8b66;color:#03140a;font-size:22px;font-weight:900;cursor:pointer}button:disabled{filter:grayscale(.7);cursor:wait}
main{width:min(1380px,calc(100% - 30px));margin:15px auto 24px}.camera{display:grid;grid-template-columns:minmax(0,2fr) minmax(320px,.8fr);gap:14px;padding:12px;border:4px solid var(--bad);border-radius:15px;background:var(--panel);box-shadow:0 0 20px #ff415766}.camera.fresh{border-color:var(--ok);box-shadow:0 0 22px #26ed8a77}
.preview-wrap{min-height:480px;display:grid;place-items:center;border-radius:10px;background:#03070b;overflow:hidden}.preview{width:100%;height:100%;max-height:700px;object-fit:contain;image-rendering:auto}.placeholder{color:#8392a2;font-size:20px}
.metrics{display:flex;flex-direction:column;gap:10px}.metric{padding:13px;border:1px solid #324354;border-radius:10px;background:#0b121a}.label{display:block;margin-bottom:5px;color:var(--muted);font-size:13px}.value{font-size:19px;font-weight:760;word-break:break-all}.age{font-size:36px}.fresh-text{color:var(--ok)}.warn-text{color:var(--warn)}.stale-text{color:var(--bad)}
#notice{min-height:24px;margin:11px 3px;color:#ff9ca7;font-weight:700}#result{margin:9px 3px;color:#d5e0e8;word-break:break-all}
.lidars{display:grid;grid-template-columns:repeat(4,minmax(0,1fr));gap:10px;margin-top:13px}.lidar{padding:11px;border:2px solid var(--bad);border-radius:10px;background:#111a24}.lidar.on{border-color:var(--ok)}.lidar-name{font-weight:800}.lidar-meta{margin-top:5px;color:var(--muted);font-size:13px}
@media(max-width:900px){header{grid-template-columns:1fr}button{width:100%}.camera{grid-template-columns:1fr}.preview-wrap{min-height:260px}.lidars{grid-template-columns:repeat(2,minmax(0,1fr))}}
</style></head><body>
<header><div><h1>交通信号灯相机 · 外参专项采集</h1><div class="subtitle">只启动并保存交通灯相机 + 四颗激光雷达 <span id="overall">正在连接…</span></div></div><button id="capture">保存当前标定数据</button></header>
<main><section id="camera" class="camera"><div id="previewWrap" class="preview-wrap"><div class="placeholder">等待交通灯相机图像</div></div><div class="metrics">
<div class="metric"><span class="label">图像状态</span><span id="online" class="value stale-text">未收到</span></div>
<div class="metric"><span class="label">当前新鲜度（当前时刻 − 工控机接收时刻）</span><span id="age" class="value age stale-text">—</span></div>
<div class="metric"><span class="label">工控机接收时间</span><span id="received" class="value">—</span></div>
<div class="metric"><span class="label">ROS 消息时间</span><span id="source" class="value">—</span></div>
<div class="metric"><span class="label">频率 / 帧序号</span><span id="rate" class="value">—</span></div>
<div class="metric"><span class="label">Tag16h5 检测</span><span id="tags" class="value">—</span></div>
</div></section><div id="notice"></div><div id="result"></div><section id="lidars" class="lidars"></section></main>
<script>
const trafficName='traffic_light_front';const lidarNames=['lidar_top','lidar_front_left','lidar_front_right','lidar_rear'];
function text(id,value){document.getElementById(id).textContent=value}
function render(x){const s=x.sensors[trafficName];const age=s.age_sec;const fresh=s.online;document.getElementById('camera').classList.toggle('fresh',fresh);text('overall',x.ready?'● 数据满足建议条件':'● 数据尚未满足建议条件');document.getElementById('overall').className=x.ready?'ok':'bad';
text('online',fresh?'数据在线':'数据离线或已过期');document.getElementById('online').className='value '+(fresh?'fresh-text':'stale-text');const ageMs=age===null?null:Math.max(0,age*1000);text('age',ageMs===null?'—':`${ageMs.toFixed(0)} ms`);document.getElementById('age').className='value age '+(ageMs!==null&&ageMs<500?'fresh-text':ageMs!==null&&ageMs<2000?'warn-text':'stale-text');
text('received',s.received_at||'—');text('source',s.source_at||'消息未提供有效时间戳');text('rate',`${s.fps.toFixed(1)} Hz / ${s.sequence===null?'—':s.sequence}`);const d=s.detection||{};text('tags',`${d.tag_count||0} 个 · ID ${(d.tag_ids||[]).join(', ')||'—'}`);
const wrap=document.getElementById('previewWrap');if(s.preview){wrap.innerHTML=`<img class="preview" alt="交通灯相机实时画面" src="/api/preview/${trafficName}?t=${Date.now()}">`}else{wrap.innerHTML='<div class="placeholder">等待交通灯相机图像</div>'}
text('notice',x.reasons.length?'提示：'+x.reasons.join('；'):'交通灯相机、Tag 和四颗雷达均已通过');document.getElementById('lidars').innerHTML=lidarNames.map(name=>{const l=x.sensors[name];const age=l.age_sec===null?'—':`${(l.age_sec*1000).toFixed(0)} ms`;return `<article class="lidar ${l.online?'on':''}"><div class="lidar-name">${name} · ${l.online?'在线':'离线'}</div><div class="lidar-meta">${l.fps.toFixed(1)} Hz · 新鲜度 ${age}</div></article>`}).join('');if(x.last_capture)text('result',`最近保存：${x.last_capture.saved_sensor_count}/${x.sensor_count} 路 · ${x.last_capture.path}`)}
async function poll(){try{const response=await fetch('/api/status',{cache:'no-store'});if(!response.ok)throw Error(response.status);render(await response.json())}catch(error){text('overall','● 页面与采集后端失去连接');document.getElementById('overall').className='bad'}setTimeout(poll,400)}
document.getElementById('capture').onclick=async()=>{const button=document.getElementById('capture');button.disabled=true;button.textContent='正在保存…';try{const response=await fetch('/api/capture',{method:'POST'});const data=await response.json();if(!response.ok)throw Error(data.error);text('result',`保存完成：${data.saved_sensor_count}/5 路 · ${data.path}`)}catch(error){text('result',`保存失败：${error.message}`)}finally{button.textContent='保存当前标定数据';button.disabled=false}};poll();
</script></body></html>"""


def make_handler(
    state: SharedState, writer: CaptureWriter, html: str = HTML
) -> type[BaseHTTPRequestHandler]:
    class Handler(BaseHTTPRequestHandler):
        server_version = "WhiteCalibrationStation/1.0"

        def log_message(self, fmt: str, *args: Any) -> None:
            if self.path != "/api/status":
                super().log_message(fmt, *args)

        def send_payload(self, status: int, payload: bytes, content_type: str) -> None:
            self.send_response(status)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(payload)))
            self.send_header("Cache-Control", "no-store")
            self.send_header("X-Content-Type-Options", "nosniff")
            self.end_headers()
            self.wfile.write(payload)

        def do_GET(self) -> None:
            path = urlparse(self.path).path
            if path == "/":
                self.send_payload(HTTPStatus.OK, html.encode("utf-8"), "text/html; charset=utf-8")
            elif path == "/api/status":
                self.send_payload(
                    HTTPStatus.OK, json_bytes(state.status()), "application/json; charset=utf-8"
                )
            elif path.startswith("/api/preview/"):
                name = path.removeprefix("/api/preview/")
                with state.lock:
                    payload = state.previews.get(name)
                if payload is None:
                    self.send_payload(HTTPStatus.NOT_FOUND, b"not ready\n", "text/plain")
                else:
                    self.send_payload(HTTPStatus.OK, payload, "image/jpeg")
            else:
                self.send_payload(HTTPStatus.NOT_FOUND, b"not found\n", "text/plain")

        def do_POST(self) -> None:
            if urlparse(self.path).path != "/api/capture":
                self.send_payload(HTTPStatus.NOT_FOUND, b"not found\n", "text/plain")
                return
            length = int(self.headers.get("Content-Length", "0"))
            if length > 4096:
                self.send_payload(
                    HTTPStatus.REQUEST_ENTITY_TOO_LARGE,
                    json_bytes({"error": "request too large"}),
                    "application/json",
                )
                return
            if length:
                self.rfile.read(length)
            try:
                result = writer.capture()
                self.send_payload(
                    HTTPStatus.OK, json_bytes(result), "application/json; charset=utf-8"
                )
            except Exception as error:
                self.send_payload(
                    HTTPStatus.CONFLICT,
                    json_bytes({"error": str(error)}),
                    "application/json; charset=utf-8",
                )

    return Handler


def load_config(path: Path) -> dict[str, Any]:
    result = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(result, dict):
        raise ValueError("config top level must be a mapping")
    root = Path(__file__).resolve().parents[2]
    for section, key in (
        (result["station"], "data_root"),
        (result["fisheye"], "sdk_path"),
        (result, "intrinsics_dir"),
    ):
        value = Path(section[key]).expanduser()
        section[key] = str(value if value.is_absolute() else root / value)
    if os.environ.get("CALIBRATION_DATA_ROOT"):
        result["station"]["data_root"] = str(
            Path(os.environ["CALIBRATION_DATA_ROOT"]) / "raw_extrinsic_data"
        )
    result["fisheye"]["base_url"] = os.environ.get(
        "PI_STREAM_BASE_URL", result["fisheye"]["base_url"]
    )
    result["station"]["bind_host"] = os.environ.get(
        "CALIBRATION_BIND_HOST", result["station"]["bind_host"]
    )
    result["station"]["port"] = int(os.environ.get("CALIBRATION_PORT", result["station"]["port"]))
    if os.environ.get("CALIBRATION_AUTO_LIDAR") == "0":
        result["lidars"]["auto_start"] = False
    names = [item["name"] for item in result["usb_capture"]["cameras"]]
    names += list(result["fisheye"]["sources"].values())
    names += [result["traffic_camera"]["name"]]
    names += [item["name"] for item in result["lidars"]["sensors"]]
    if len(names) != 15 or len(set(names)) != 15:
        raise ValueError("configuration must contain exactly 15 unique sensors")
    lidar_items = result["lidars"]["sensors"]
    for key in ("msop_port", "difop_port"):
        ports = [int(item[key]) for item in lidar_items]
        if len(ports) != len(set(ports)):
            raise ValueError(f"lidar {key} values must be unique")
    return result


def main() -> int:
    parser = argparse.ArgumentParser()
    base = Path(__file__).resolve().parent
    parser.add_argument("--config", type=Path, default=base / "config.yaml")
    parser.add_argument("--initial-extrinsics", type=Path, default=base / "initial_extrinsics.yaml")
    parser.add_argument(
        "--check",
        action="store_true",
        help="validate imports/configuration without opening hardware",
    )
    parser.add_argument(
        "--traffic-only",
        action="store_true",
        help="start only four lidars and the ROS traffic-light camera",
    )
    args = parser.parse_args()
    config = load_config(args.config.resolve())
    transforms = load_top_transforms(args.initial_extrinsics.resolve())
    if args.check:
        print("configuration/import check passed: 4 lidars + 11 cameras")
        return 0

    stop = threading.Event()
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda _s, _f: stop.set())
    warnings = WarningLimiter(float(config["station"]["log_warning_period_sec"]))
    kinds: dict[str, str] = {}
    if not args.traffic_only:
        for camera in config["usb_capture"]["cameras"]:
            kinds[camera["name"]] = "usb_camera"
        for name in config["fisheye"]["sources"].values():
            kinds[name] = "fisheye_camera"
    kinds[config["traffic_camera"]["name"]] = "traffic_camera"
    for lidar in config["lidars"]["sensors"]:
        kinds[lidar["name"]] = "lidar"
    mode = "traffic_camera" if args.traffic_only else "joint"
    if args.traffic_only:
        # At 320x240 and 5 Hz, fresh detection is inexpensive and prevents the
        # preview from appearing frozen between the old 0.5 Hz detector passes.
        config["station"]["camera_detection_hz"] = 5.0
    state = SharedState(config, kinds, mode=mode)

    rclpy.init(args=None)
    node = RosSensors(config, state, warnings)
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    ros_thread = threading.Thread(target=executor.spin, name="ros-executor", daemon=True)
    ros_thread.start()

    log_dir = base / "run_logs"
    lidar_manager = LidarLaunchManager(config, node, stop, log_dir, warnings)
    time.sleep(0.8)
    lidar_manager.start_if_needed()
    usb_workers = (
        []
        if args.traffic_only
        else [
            UsbCameraWorker(camera, config["usb_capture"], state, stop, warnings)
            for camera in config["usb_capture"]["cameras"]
        ]
    )
    for worker in usb_workers:
        worker.start()
    fisheye = (
        None if args.traffic_only else FisheyeWorkers(config["fisheye"], state, stop, warnings)
    )
    if fisheye is not None:
        fisheye.start()
    camera_detector = CameraTagDetector(
        config, state, stop, [name for name, kind in kinds.items() if kind.endswith("camera")]
    )
    lidar_detector = LidarTargetDetector(config, state, stop, transforms, warnings)
    camera_detector.start()
    lidar_detector.start()
    writer = CaptureWriter(config, state, args.config.resolve(), args.initial_extrinsics.resolve())
    server = ThreadingHTTPServer(
        (config["station"]["bind_host"], int(config["station"]["port"])),
        make_handler(state, writer, TRAFFIC_HTML if args.traffic_only else HTML),
    )
    server.timeout = 0.5
    print(
        f"{GREEN}Calibration page: http://127.0.0.1:{config['station']['port']}{RESET}", flush=True
    )
    try:
        while not stop.is_set():
            server.handle_request()
    finally:
        stop.set()
        server.server_close()
        for worker in usb_workers:
            worker.join()
        if fisheye is not None:
            fisheye.close()
        camera_detector.join()
        lidar_detector.join()
        lidar_manager.close()
        executor.shutdown(timeout_sec=3.0)
        node.destroy_node()
        rclpy.shutdown()
        ros_thread.join(timeout=3.0)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
