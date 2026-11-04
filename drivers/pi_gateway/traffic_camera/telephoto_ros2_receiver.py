#!/usr/bin/env python3
"""Lease-controlled UDP JPEG receiver and local-only ROS 2 camera bridge."""

import argparse
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import os
from pathlib import Path
import secrets
import socket
import struct
import threading
import time
import zlib
from dataclasses import dataclass, field
from typing import Dict, Optional

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, CompressedImage


MAGIC = b"TJPG"
VERSION = 2
HEADER = struct.Struct("!4sBBHIQIIHHHHHH")
HEADER_BYTES = HEADER.size
MAX_FRAME_BYTES = 2 * 1024 * 1024
HEARTBEAT_MAGIC = b"TLHB"
STATUS_MAGIC = b"TLSA"
CONTROL_VERSION = 1
HEARTBEAT = struct.Struct("!4sBBHQQI")
STATUS = struct.Struct("!4sBBHQQQIIIIII")


@dataclass
class PendingFrame:
    frame_size: int
    checksum: int
    fragment_count: int
    width: int
    height: int
    capture_ns: int
    first_rx: float = field(default_factory=time.monotonic)
    fragments: Dict[int, bytes] = field(default_factory=dict)


class FrameAssembler:
    def __init__(self) -> None:
        self.pending: Dict[int, PendingFrame] = {}
        self.invalid_datagrams = 0
        self.crc_errors = 0
        self.expired_frames = 0

    def add(self, datagram: bytes) -> Optional[tuple[int, int, int, int, bytes]]:
        if len(datagram) < HEADER_BYTES:
            self.invalid_datagrams += 1
            return None
        fields = HEADER.unpack_from(datagram)
        (magic, version, _flags, header_bytes, frame_id, capture_ns, frame_size,
         checksum, width, height, fragment_index, fragment_count, payload_size,
         _reserved) = fields
        payload = datagram[HEADER_BYTES:]
        if (magic != MAGIC or version != VERSION or header_bytes != HEADER_BYTES or
                frame_size <= 0 or frame_size > MAX_FRAME_BYTES or
                fragment_count <= 0 or fragment_index >= fragment_count or
                payload_size != len(payload) or capture_ns <= 0):
            self.invalid_datagrams += 1
            return None

        current = self.pending.get(frame_id)
        if current is None:
            current = PendingFrame(frame_size, checksum, fragment_count, width,
                                   height, capture_ns)
            self.pending[frame_id] = current
        elif (current.frame_size != frame_size or current.checksum != checksum or
              current.fragment_count != fragment_count or
              current.capture_ns != capture_ns):
            self.invalid_datagrams += 1
            self.pending.pop(frame_id, None)
            return None
        current.fragments[fragment_index] = payload

        now = time.monotonic()
        for old_id in [key for key, value in self.pending.items()
                       if now - value.first_rx > 1.0]:
            self.pending.pop(old_id, None)
            self.expired_frames += 1

        if len(current.fragments) != current.fragment_count:
            return None
        try:
            jpeg = b"".join(current.fragments[index]
                            for index in range(current.fragment_count))
        except KeyError:
            return None
        self.pending.pop(frame_id, None)
        if (len(jpeg) != current.frame_size or
                (zlib.crc32(jpeg) & 0xFFFFFFFF) != current.checksum):
            self.crc_errors += 1
            return None
        if not (jpeg.startswith(b"\xff\xd8") and jpeg.endswith(b"\xff\xd9")):
            self.invalid_datagrams += 1
            return None
        return frame_id, current.width, current.height, current.capture_ns, jpeg


def default_calibration(width: int, height: int) -> dict:
    return {
        "calibrated": False,
        "width": width,
        "height": height,
        "distortion_model": "plumb_bob",
        "D": [0.0] * 5,
        "K": [0.0] * 9,
        "R": [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
        "P": [0.0] * 12,
    }


def load_calibration(path: str, width: int, height: int) -> dict:
    calibration = default_calibration(width, height)
    if path:
        supplied = json.loads(Path(path).read_text(encoding="utf-8"))
        if not isinstance(supplied, dict):
            raise ValueError("camera calibration must be a JSON object")
        calibration.update(supplied)
    if calibration.get("width") != width or calibration.get("height") != height:
        raise ValueError("calibration width/height do not match the fixed stream size")
    for key, size in (("D", 5), ("K", 9), ("R", 9), ("P", 12)):
        values = calibration.get(key)
        if not isinstance(values, list) or len(values) != size:
            raise ValueError(f"camera calibration {key} must contain {size} values")
        calibration[key] = [float(value) for value in values]
    return calibration


class LocalStatusServer(threading.Thread):
    def __init__(self, owner: "TrafficCameraReceiver", host: str, port: int) -> None:
        super().__init__(name="traffic-camera-http", daemon=True)
        self.owner = owner
        self.server = ThreadingHTTPServer((host, port), self._handler())
        self.server.daemon_threads = True

    def _handler(self):
        owner = self.owner

        class Handler(BaseHTTPRequestHandler):
            def do_GET(self) -> None:  # noqa: N802
                path = self.path.split("?", 1)[0]
                if path == "/status":
                    body = json.dumps(owner.status(), ensure_ascii=False,
                                      separators=(",", ":")).encode("utf-8")
                    self._send(200, body, "application/json; charset=utf-8")
                elif path == "/image.jpg":
                    jpeg = owner.latest_jpeg()
                    if jpeg is None:
                        self._send(503, b"frame not ready\n", "text/plain")
                    else:
                        self._send(200, jpeg, "image/jpeg")
                else:
                    self._send(404, b"not found\n", "text/plain")

            def _send(self, code: int, body: bytes, content_type: str) -> None:
                self.send_response(code)
                self.send_header("Content-Type", content_type)
                self.send_header("Content-Length", str(len(body)))
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                try:
                    self.wfile.write(body)
                except (BrokenPipeError, ConnectionResetError):
                    pass

            def log_message(self, _format: str, *_args) -> None:
                return

        return Handler

    def run(self) -> None:
        self.server.serve_forever(poll_interval=0.25)

    def stop(self) -> None:
        self.server.shutdown()
        self.server.server_close()


class TrafficCameraReceiver(Node):
    def __init__(self, arguments: argparse.Namespace) -> None:
        super().__init__("w102_traffic_light_front_receiver")
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.image_publisher = self.create_publisher(
            CompressedImage, arguments.topic, qos)
        self.info_publisher = self.create_publisher(
            CameraInfo, arguments.camera_info_topic, qos)
        self.arguments = arguments
        self.calibration = load_calibration(arguments.calibration,
                                            arguments.width, arguments.height)
        self.assembler = FrameAssembler()
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 * 1024 * 1024)
        self.socket.bind((arguments.bind_ip, arguments.port))
        self.socket.settimeout(0.25)
        self.control_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.control_socket.settimeout(0.25)
        self.lock = threading.Lock()
        self.latest: Optional[tuple[int, int, int, int, bytes, float]] = None
        self.last_published_id: Optional[int] = None
        self.running = True
        self.session_id = secrets.randbits(64)
        self.frames_received = 0
        self.frames_published = 0
        self.bytes_received = 0
        self.last_frame_time = 0.0
        self.last_status_time = 0.0
        self.pi_status: dict = {}
        self.frame_times: deque[float] = deque(maxlen=50)
        self.stats_time = time.monotonic()
        self.stats_frames = 0
        self.thread = threading.Thread(target=self._receive_loop,
                                       name="traffic-camera-udp", daemon=True)
        self.control_thread = threading.Thread(target=self._control_loop,
                                               name="traffic-camera-lease", daemon=True)
        self.thread.start()
        self.control_thread.start()
        self.http = LocalStatusServer(self, arguments.http_bind,
                                      arguments.http_port)
        self.http.start()
        self.timer = self.create_timer(0.02, self._publish_latest)
        self.stats_timer = self.create_timer(10.0, self._report_stats)
        self.get_logger().info(
            f"listening {arguments.bind_ip}:{arguments.port}; lease query "
            f"{arguments.source_ip}:{arguments.control_port}; publishing "
            f"{arguments.topic} and {arguments.camera_info_topic}; "
            f"calibrated={bool(self.calibration.get('calibrated'))}"
        )

    def _receive_loop(self) -> None:
        while self.running:
            try:
                datagram, source = self.socket.recvfrom(65535)
            except socket.timeout:
                continue
            except OSError:
                break
            if self.arguments.source_ip and source[0] != self.arguments.source_ip:
                continue
            complete = self.assembler.add(datagram)
            if complete is None:
                continue
            frame_id, width, height, capture_ns, jpeg = complete
            if width != self.arguments.width or height != self.arguments.height:
                self.assembler.invalid_datagrams += 1
                continue
            received_at = time.monotonic()
            with self.lock:
                self.latest = (frame_id, width, height, capture_ns, jpeg,
                               received_at)
                self.frame_times.append(received_at)
            self.frames_received += 1
            self.stats_frames += 1
            self.bytes_received += len(jpeg)
            self.last_frame_time = received_at

    def _control_loop(self) -> None:
        destination = (self.arguments.source_ip, self.arguments.control_port)
        next_heartbeat = 0.0
        while self.running:
            now = time.monotonic()
            if now >= next_heartbeat:
                packet = HEARTBEAT.pack(
                    HEARTBEAT_MAGIC, CONTROL_VERSION, 0, HEARTBEAT.size,
                    self.session_id, time.time_ns(), self.arguments.lease_ms)
                try:
                    self.control_socket.sendto(packet, destination)
                except OSError:
                    if not self.running:
                        break
                next_heartbeat = now + self.arguments.heartbeat_interval
            try:
                packet, source = self.control_socket.recvfrom(1024)
            except socket.timeout:
                continue
            except OSError:
                break
            if source[0] != self.arguments.source_ip or len(packet) != STATUS.size:
                continue
            fields = STATUS.unpack(packet)
            (magic, version, flags, packet_bytes, session, sender_ns,
             last_capture_ns, frames_sent, send_errors, lease_ms, width,
             height, fps_milli) = fields
            if (magic != STATUS_MAGIC or version != CONTROL_VERSION or
                    packet_bytes != STATUS.size or session != self.session_id):
                continue
            with self.lock:
                self.last_status_time = time.monotonic()
                self.pi_status = {
                    "camera_open": bool(flags & 0x01),
                    "capture_fresh": bool(flags & 0x02),
                    "lease_active": bool(flags & 0x04),
                    "sender_monotonic_ns": sender_ns,
                    "last_capture_monotonic_ns": last_capture_ns,
                    "frames_sent": frames_sent,
                    "send_errors": send_errors,
                    "lease_ms": lease_ms,
                    "width": width,
                    "height": height,
                    "configured_fps": fps_milli / 1000.0,
                }

    def _publish_latest(self) -> None:
        with self.lock:
            latest = self.latest
        if latest is None or latest[0] == self.last_published_id:
            return
        frame_id, width, height, capture_ns, jpeg, _received_at = latest
        stamp = self.get_clock().now().to_msg()
        stamp.sec = int(capture_ns // 1_000_000_000)
        stamp.nanosec = int(capture_ns % 1_000_000_000)

        image = CompressedImage()
        image.header.stamp = stamp
        image.header.frame_id = self.arguments.frame_id
        image.format = "bgr8; jpeg compressed bgr8"
        image.data = jpeg

        info = CameraInfo()
        info.header.stamp = stamp
        info.header.frame_id = self.arguments.frame_id
        info.width = width
        info.height = height
        info.distortion_model = str(self.calibration["distortion_model"])
        info.d = self.calibration["D"]
        info.k = self.calibration["K"]
        info.r = self.calibration["R"]
        info.p = self.calibration["P"]
        info.binning_x = int(self.calibration.get("binning_x", 0))
        info.binning_y = int(self.calibration.get("binning_y", 0))
        roi = self.calibration.get("roi", {})
        info.roi.x_offset = int(roi.get("x_offset", 0))
        info.roi.y_offset = int(roi.get("y_offset", 0))
        info.roi.height = int(roi.get("height", 0))
        info.roi.width = int(roi.get("width", 0))
        info.roi.do_rectify = bool(roi.get("do_rectify", False))

        self.image_publisher.publish(image)
        self.info_publisher.publish(info)
        self.last_published_id = frame_id
        self.frames_published += 1

    def latest_jpeg(self) -> Optional[bytes]:
        with self.lock:
            return None if self.latest is None else self.latest[4]

    def status(self) -> dict:
        now = time.monotonic()
        with self.lock:
            frame_age = None if not self.last_frame_time else now - self.last_frame_time
            status_age = None if not self.last_status_time else now - self.last_status_time
            pi_status = dict(self.pi_status)
            times = list(self.frame_times)
            frame_id = None if self.latest is None else self.latest[0]
        fps = 0.0
        if len(times) >= 2 and times[-1] > times[0]:
            fps = (len(times) - 1) / (times[-1] - times[0])
        if frame_age is None or frame_age > 2.0:
            fps = 0.0
        if frame_age is not None and frame_age <= 5.0:
            query = "streaming"
        elif status_age is not None and status_age <= 2.5:
            query = "pi_responded_no_image"
        else:
            query = "pi_unreachable"
        return {
            "online": frame_age is not None and frame_age <= 2.0,
            "frame_age_s": frame_age,
            "rx_fps": fps,
            "frame_id": frame_id,
            "frames_received": self.frames_received,
            "frames_published": self.frames_published,
            "status_ack_age_s": status_age,
            "query_state": query,
            "querying_after_5s": frame_age is None or frame_age > 5.0,
            "pi": pi_status,
            "calibrated": bool(self.calibration.get("calibrated")),
            "width": self.arguments.width,
            "height": self.arguments.height,
            "image_topic": self.arguments.topic,
            "camera_info_topic": self.arguments.camera_info_topic,
            "frame_id_name": self.arguments.frame_id,
            "qos": "sensor_data/keep_last=5/best_effort/volatile",
            "crc_errors": self.assembler.crc_errors,
            "invalid_datagrams": self.assembler.invalid_datagrams,
        }

    def _report_stats(self) -> None:
        now = time.monotonic()
        elapsed = now - self.stats_time
        fps = self.stats_frames / elapsed if elapsed > 0 else 0.0
        status = self.status()
        self.get_logger().info(
            f"rx_fps={fps:.2f} age_s={status['frame_age_s']} "
            f"query={status['query_state']} received={self.frames_received} "
            f"published={self.frames_published} expired={self.assembler.expired_frames} "
            f"crc_errors={self.assembler.crc_errors} "
            f"invalid={self.assembler.invalid_datagrams}"
        )
        self.stats_time = now
        self.stats_frames = 0

    def destroy_node(self) -> bool:
        self.running = False
        self.socket.close()
        self.control_socket.close()
        self.thread.join(timeout=1.0)
        self.control_thread.join(timeout=1.0)
        self.http.stop()
        self.http.join(timeout=1.0)
        return super().destroy_node()


def parse_arguments() -> argparse.Namespace:
    project_root = Path(os.environ.get(
        "FSM_PI_MASTER_ROOT", Path(__file__).resolve().parents[1]))
    parser = argparse.ArgumentParser(
        description="Receive Pi traffic camera JPEG and publish local ROS2")
    parser.add_argument("--bind-ip", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=5620)
    parser.add_argument("--source-ip", default="192.168.1.186")
    parser.add_argument("--control-port", type=int, default=5621)
    parser.add_argument("--heartbeat-interval", type=float, default=1.0)
    parser.add_argument("--lease-ms", type=int, default=3000)
    parser.add_argument("--width", type=int, default=320)
    parser.add_argument("--height", type=int, default=240)
    parser.add_argument("--topic", default=
                        "/sensing/camera/traffic_light_front/image_raw/compressed")
    parser.add_argument("--camera-info-topic", default=
                        "/sensing/camera/traffic_light_front/camera_info")
    parser.add_argument("--frame-id", default=
                        "traffic_light_front/camera_optical_link")
    parser.add_argument("--calibration", default=
                        str(project_root / "traffic_camera/calibration.json"))
    parser.add_argument("--http-bind", default="127.0.0.1")
    parser.add_argument("--http-port", type=int, default=8090)
    return parser.parse_args()


def main() -> None:
    arguments = parse_arguments()
    rclpy.init()
    node = TrafficCameraReceiver(arguments)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
