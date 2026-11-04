#!/usr/bin/env python3
"""Lightweight W102 BEV receiver and Raspberry Pi telemetry dashboard."""

from __future__ import annotations

import argparse
from collections import deque
from dataclasses import asdict
import json
import os
from pathlib import Path
import secrets
import signal
import socket
import sys
import threading
import time
from typing import Any, Optional

import cv2
import numpy as np

from w102_protocol import (
    BEV_JPEG, CAMERA_JPEG, CONTROL_JSON, HEALTH_JSON,
    CONTROL_SET_OUTPUT_MODE, OUTPUT_MODE_MASKS,
    FLAG_ILLUMINATION, FLAG_PARTIAL_CAMERAS, FLAG_REPEATED, FLAG_REPLAY,
    FLAG_SYNTHETIC_FILL, CompleteFrame, Reassembler, encode_control,
    encode_hello,
    source_is_gateway,
)
from telemetry import cpu_metrics, image_rate_metrics, memory_percent
from web_dashboard import WebDashboardServer


class SlidingRate:
    def __init__(self, window_s: float = 2.0) -> None:
        self.window_s = window_s
        self._times: deque[float] = deque()

    def add(self, now: float) -> None:
        self._times.append(now)
        self._trim(now)

    def value(self, now: Optional[float] = None) -> float:
        current = time.monotonic() if now is None else now
        self._trim(current)
        if len(self._times) < 2:
            return 0.0
        span = self._times[-1] - self._times[0]
        return (len(self._times) - 1) / span if span > 0 else 0.0

    def _trim(self, now: float) -> None:
        while self._times and now - self._times[0] > self.window_s:
            self._times.popleft()


class ReceiverState:
    def __init__(self) -> None:
        self.lock = threading.RLock()
        self.bev_image_payload: Optional[bytes] = None
        self.bev_frame_id = 0
        self.raw_payloads: dict[int, bytes] = {}
        self.raw_frame_ids: dict[int, int] = {}
        self.raw_received_at: dict[int, float] = {}
        self.raw_flags: dict[int, int] = {}
        self.raw_rates = {index: SlidingRate() for index in range(4)}
        self.raw_generation = 0
        self._mosaic_cache_key: Optional[tuple[int, ...]] = None
        self._mosaic_cache: Optional[bytes] = None
        self.health: dict[str, Any] = {}
        self.control: dict[str, Any] = {}
        self.last_packet_at = 0.0
        self.last_bev_at = 0.0
        self.last_health_at = 0.0
        self.last_source = ""
        self.last_flags = 0
        self.last_frame_id = 0
        self.decode_errors = 0
        self.bev_frames = 0
        self.raw_frames = 0
        self.replay_frames = 0
        self.bev_rx_rate = SlidingRate()
        self.generation_rate = SlidingRate()
        self._last_generation: Optional[int] = None
        self.boot_id: Optional[int] = None

    def consume(self, frame: CompleteFrame) -> None:
        with self.lock:
            self.last_packet_at = frame.received_at
            self.last_source = f"{frame.source[0]}:{frame.source[1]}"
            self.boot_id = frame.header.boot_id
            if frame.header.message_type == BEV_JPEG:
                self.bev_image_payload = frame.payload
                self.bev_frame_id = frame.header.frame_id
                self.last_bev_at = frame.received_at
                self.last_flags = frame.header.flags
                self.last_frame_id = frame.header.frame_id
                self.bev_frames += 1
                if frame.header.flags & FLAG_REPLAY:
                    self.replay_frames += 1
                self.bev_rx_rate.add(frame.received_at)
            elif frame.header.message_type == CAMERA_JPEG:
                if frame.header.stream_id not in self.raw_rates:
                    return
                self.raw_payloads[frame.header.stream_id] = frame.payload
                self.raw_frame_ids[frame.header.stream_id] = frame.header.frame_id
                self.raw_received_at[frame.header.stream_id] = frame.received_at
                self.raw_flags[frame.header.stream_id] = frame.header.flags
                self.raw_rates[frame.header.stream_id].add(frame.received_at)
                self.raw_generation += 1
                self._mosaic_cache_key = None
                self._mosaic_cache = None
                self.raw_frames += 1
            elif frame.header.message_type in (HEALTH_JSON, CONTROL_JSON):
                try:
                    value = json.loads(frame.payload.decode("utf-8"))
                except (UnicodeDecodeError, json.JSONDecodeError):
                    self.decode_errors += 1
                    return
                if frame.header.message_type == HEALTH_JSON:
                    self.health = value
                    self.last_health_at = frame.received_at
                    generation = value.get("bev_generation")
                    if isinstance(generation, int) and generation != self._last_generation:
                        self.generation_rate.add(frame.received_at)
                        self._last_generation = generation
                else:
                    self.control = value

    def snapshot(self, protocol_stats: Any,
                 include_image: bool = True) -> dict[str, Any]:
        now = time.monotonic()
        with self.lock:
            payload, active_frame_id = self._latest_jpeg_locked("active")
            image = None
            if include_image and payload is not None:
                image = cv2.imdecode(np.frombuffer(payload, dtype=np.uint8),
                                     cv2.IMREAD_COLOR)
                if image is None:
                    self.decode_errors += 1
            mode = self._output_mode_locked()
            active_age = self._source_age_locked(mode, now)
            raw_fps = {str(index): round(rate.value(now), 2)
                       for index, rate in self.raw_rates.items()}
            if mode == "bev":
                receive_fps = self.bev_rx_rate.value(now)
            elif mode == "raw-all":
                available = [self.raw_rates[index].value(now) for index in range(4)
                             if index in self.raw_payloads]
                receive_fps = min(available) if available else 0.0
            else:
                stream_id = self._source_stream_id(mode)
                receive_fps = (self.raw_rates[stream_id].value(now)
                               if stream_id is not None else 0.0)
            return {
                "image": image,
                "health": dict(self.health),
                "control": dict(self.control),
                "last_packet_age_s": None if self.last_packet_at == 0 else now - self.last_packet_at,
                "last_bev_age_s": None if self.last_bev_at == 0 else now - self.last_bev_at,
                "last_image_age_s": active_age,
                "last_health_age_s": None if self.last_health_at == 0 else now - self.last_health_at,
                "source": self.last_source,
                "flags": self.last_flags,
                "frame_id": active_frame_id,
                "bev_frames": self.bev_frames,
                "raw_frames": self.raw_frames,
                "replay_frames": self.replay_frames,
                "decode_errors": self.decode_errors,
                "rx_fps": receive_fps,
                "raw_fps": raw_fps,
                "output_mode": mode,
                "generation_fps": self.generation_rate.value(now),
                "boot_id": self.boot_id,
                "protocol": asdict(protocol_stats),
            }

    def save_latest(self, path: str) -> bool:
        with self.lock:
            payload, _frame_id = self._latest_jpeg_locked("active")
        if not payload:
            return False
        temporary = path + ".tmp"
        with open(temporary, "wb") as output:
            output.write(payload)
            output.flush()
            os.fsync(output.fileno())
        os.replace(temporary, path)
        return True

    @staticmethod
    def _source_stream_id(source: str) -> Optional[int]:
        return {"front": 0, "rear": 1, "left": 2, "right": 3}.get(source)

    def _output_mode_locked(self) -> str:
        mode = self.health.get("output_mode")
        if not isinstance(mode, str):
            mode = self.control.get("output_mode")
        return mode if mode in OUTPUT_MODE_MASKS else "bev"

    def _source_age_locked(self, source: str, now: float) -> Optional[float]:
        if source == "bev":
            return None if self.last_bev_at == 0 else now - self.last_bev_at
        if source == "raw-all":
            values = [self.raw_received_at.get(index) for index in range(4)]
            if any(value is None for value in values):
                return None
            return max(now - float(value) for value in values if value is not None)
        stream_id = self._source_stream_id(source)
        received = self.raw_received_at.get(stream_id) if stream_id is not None else None
        return None if received is None else now - received

    def _raw_mosaic_locked(self) -> tuple[Optional[bytes], int]:
        if any(index not in self.raw_payloads for index in range(4)):
            return None, self.raw_generation
        key = tuple(self.raw_frame_ids.get(index, 0) for index in range(4))
        if key == self._mosaic_cache_key and self._mosaic_cache is not None:
            return self._mosaic_cache, self.raw_generation
        tiles = []
        for index, role in enumerate(("FRONT", "REAR", "LEFT", "RIGHT")):
            image = cv2.imdecode(np.frombuffer(self.raw_payloads[index], dtype=np.uint8),
                                 cv2.IMREAD_COLOR)
            if image is None:
                return None, self.raw_generation
            image = cv2.resize(image, (320, 240), interpolation=cv2.INTER_AREA)
            cv2.putText(image, role, (10, 26), cv2.FONT_HERSHEY_SIMPLEX,
                        0.7, (40, 230, 255), 2, cv2.LINE_AA)
            tiles.append(image)
        mosaic = cv2.vconcat((cv2.hconcat((tiles[0], tiles[1])),
                              cv2.hconcat((tiles[2], tiles[3]))))
        ok, encoded = cv2.imencode(".jpg", mosaic,
                                   [cv2.IMWRITE_JPEG_QUALITY, 82])
        if not ok:
            return None, self.raw_generation
        self._mosaic_cache_key = key
        self._mosaic_cache = encoded.tobytes()
        return self._mosaic_cache, self.raw_generation

    def _latest_jpeg_locked(self, source: str) -> tuple[Optional[bytes], int]:
        selected = self._output_mode_locked() if source == "active" else source
        if selected == "bev":
            result = (self.bev_image_payload, self.bev_frame_id)
        elif selected == "raw-all":
            result = self._raw_mosaic_locked()
        else:
            stream_id = self._source_stream_id(selected)
            if stream_id is None:
                return None, 0
            result = (self.raw_payloads.get(stream_id),
                      self.raw_frame_ids.get(stream_id, 0))
        age = self._source_age_locked(selected, time.monotonic())
        if age is None or age > 1.0:
            return None, result[1]
        return result

    def latest_jpeg(self, source: str = "active") -> tuple[Optional[bytes], int]:
        with self.lock:
            return self._latest_jpeg_locked(source)

    def stream_status(self) -> dict[str, Any]:
        now = time.monotonic()
        with self.lock:
            mode = self._output_mode_locked()
            sources = {}
            for source in ("bev", "front", "rear", "left", "right"):
                payload, frame_id = self._latest_jpeg_locked(source)
                stream_id = self._source_stream_id(source)
                fps = (self.bev_rx_rate.value(now) if source == "bev" else
                       self.raw_rates[stream_id].value(now)
                       if stream_id is not None else 0.0)
                sources[source] = {
                    "available": payload is not None,
                    "frame_id": frame_id,
                    "age_s": self._source_age_locked(source, now),
                    "fps": round(fps, 2),
                    "flags": (self.last_flags if source == "bev" else
                              self.raw_flags.get(stream_id, 0)
                              if stream_id is not None else 0),
                }
            return {
                "mode": mode,
                "output_stream_mask": self.health.get(
                    "output_stream_mask", self.control.get("output_stream_mask")),
                "modes": list(OUTPUT_MODE_MASKS),
                "sources": sources,
            }


class UdpReceiver(threading.Thread):
    def __init__(self, state: ReceiverState, bind_host: str, data_port: int,
                 gateway_ip: str, control_port: int, hello_interval: float) -> None:
        super().__init__(name="w102-udp-receiver", daemon=True)
        self.state = state
        self.bind_host = bind_host
        self.data_port = data_port
        self.gateway_ip = gateway_ip
        self.control_port = control_port
        self.hello_interval = hello_interval
        self.reassembler = Reassembler()
        self.stop_event = threading.Event()
        self.ready_event = threading.Event()
        self.error: Optional[BaseException] = None

    def stop(self) -> None:
        self.stop_event.set()

    def run(self) -> None:
        sock: Optional[socket.socket] = None
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 8 * 1024 * 1024)
            sock.bind((self.bind_host, self.data_port))
            sock.settimeout(0.2)
            self.ready_event.set()
            next_hello = 0.0
            while not self.stop_event.is_set():
                now = time.monotonic()
                if now >= next_hello:
                    hello = encode_hello(self.data_port)
                    try:
                        sock.sendto(hello, (self.gateway_ip, self.control_port))
                    except OSError:
                        pass
                    next_hello = now + self.hello_interval
                try:
                    datagram, source = sock.recvfrom(65535)
                except socket.timeout:
                    self.reassembler.expire()
                    continue
                if not source_is_gateway(source[0], self.gateway_ip):
                    continue
                frame = self.reassembler.push(datagram, source)
                if frame is not None:
                    self.state.consume(frame)
        except BaseException as error:  # exposed to the main thread
            self.error = error
            self.ready_event.set()
        finally:
            if sock is not None:
                sock.close()


class GatewayControlClient:
    def __init__(self, gateway_ip: str, control_port: int,
                 timeout_s: float = 2.0) -> None:
        self.gateway_ip = gateway_ip
        self.control_port = control_port
        self.timeout_s = timeout_s
        self.lock = threading.Lock()

    def set_mode(self, mode: str) -> dict[str, Any]:
        normalized = str(mode).strip().lower()
        if normalized not in OUTPUT_MODE_MASKS:
            raise ValueError(
                "mode must be one of: " + ", ".join(OUTPUT_MODE_MASKS))
        return self._request(CONTROL_SET_OUTPUT_MODE,
                             OUTPUT_MODE_MASKS[normalized])

    def _request(self, command: int, stream_mask: int) -> dict[str, Any]:
        with self.lock:
            nonce = secrets.randbits(64) or 1
            reassembler = Reassembler(timeout_s=self.timeout_s)
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as control:
                control.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF,
                                   4 * 1024 * 1024)
                control.bind(("0.0.0.0", 0))
                reply_port = int(control.getsockname()[1])
                packet = encode_control(command, reply_port, stream_mask, nonce)
                control.sendto(packet, (self.gateway_ip, self.control_port))
                deadline = time.monotonic() + self.timeout_s
                while time.monotonic() < deadline:
                    control.settimeout(max(0.01, deadline - time.monotonic()))
                    try:
                        datagram, source = control.recvfrom(65535)
                    except socket.timeout:
                        break
                    if not source_is_gateway(source[0], self.gateway_ip):
                        continue
                    frame = reassembler.push(datagram, source)
                    if frame is None or frame.header.message_type != CONTROL_JSON:
                        continue
                    try:
                        result = json.loads(frame.payload.decode("utf-8"))
                    except (UnicodeDecodeError, json.JSONDecodeError):
                        continue
                    if result.get("nonce") != nonce:
                        continue
                    if not result.get("accepted"):
                        raise RuntimeError(str(result.get("error", "gateway rejected command")))
                    return result
        raise RuntimeError("Raspberry Pi did not acknowledge the mode command")


def _text(canvas: np.ndarray, text: str, x: int, y: int,
          color: tuple[int, int, int] = (235, 235, 235), scale: float = 0.53,
          thickness: int = 1) -> None:
    cv2.putText(canvas, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, scale,
                color, thickness, cv2.LINE_AA)


def _number(value: Any, suffix: str = "", digits: int = 1) -> str:
    return f"{float(value):.{digits}f}{suffix}" if isinstance(value, (int, float)) and not isinstance(value, bool) else "N/A"


def render_dashboard(snapshot: dict[str, Any], bind_label: str,
                     gateway_ip: str) -> np.ndarray:
    canvas = np.full((800, 1280, 3), (22, 24, 28), np.uint8)
    image = snapshot["image"]
    if image is None:
        cv2.rectangle(canvas, (20, 54), (929, 779), (8, 8, 8), -1)
        _text(canvas, "WAITING FOR BEV UDP", 275, 370, (70, 190, 255), 1.0, 2)
        _text(canvas, f"listen {bind_label}  gateway {gateway_ip}", 245, 410,
              (180, 180, 180), 0.55)
        _text(canvas, "Check Pi peer-ip if no packets arrive.", 295, 445,
              (150, 150, 150), 0.5)
    else:
        source_h, source_w = image.shape[:2]
        scale = min(910 / source_w, 726 / source_h)
        resized = cv2.resize(image, (int(source_w * scale), int(source_h * scale)),
                             interpolation=cv2.INTER_AREA)
        x = 20 + (910 - resized.shape[1]) // 2
        y = 54 + (726 - resized.shape[0]) // 2
        canvas[y:y + resized.shape[0], x:x + resized.shape[1]] = resized

    cv2.rectangle(canvas, (0, 0), (1279, 42), (37, 42, 49), -1)
    connected = snapshot["last_packet_age_s"] is not None and snapshot["last_packet_age_s"] < 1.0
    health_ok = snapshot["last_health_age_s"] is not None and snapshot["last_health_age_s"] < 1.0
    _text(canvas, "W102 BEV MONITOR", 18, 28, (245, 245, 245), 0.68, 2)
    _text(canvas, "ONLINE" if connected else "NO DATA", 1148, 28,
          (80, 220, 110) if connected else (80, 90, 240), 0.58, 2)

    health = snapshot["health"]
    x0, line = 950, 76
    _text(canvas, "TRANSPORT", x0, line, (70, 190, 255), 0.58, 2); line += 30
    send_fps, processing_fps = image_rate_metrics(
        health, snapshot["generation_fps"])
    _text(canvas, f"Rx FPS       {snapshot['rx_fps']:.1f}", x0, line); line += 24
    _text(canvas, f"Pi send FPS  {_number(send_fps)}", x0, line); line += 24
    _text(canvas, f"Process FPS  {_number(processing_fps)}", x0, line); line += 24
    _text(canvas, f"Frame ID     {snapshot['frame_id']}", x0, line); line += 24
    _text(canvas, f"Data age     {_number(snapshot['last_packet_age_s'], ' s')}", x0, line); line += 24
    _text(canvas, f"Source       {snapshot['source'] or '--'}", x0, line, scale=0.43); line += 35
    protocol = snapshot.get("protocol", {})
    _text(canvas, f"Pi restarts  {protocol.get('sender_restarts', 0)}", x0, line); line += 24
    _text(canvas, f"Frame gaps   {protocol.get('frame_gaps', 0)}", x0, line); line += 35

    _text(canvas, "RASPBERRY PI", x0, line, (70, 190, 255), 0.58, 2); line += 30
    system_cpu, process_cpu = cpu_metrics(health)
    _text(canvas, f"CPU system   {_number(system_cpu, '%')}", x0, line); line += 24
    _text(canvas, f"Gateway CPU  {_number(process_cpu, '%')}", x0, line); line += 24
    memory = memory_percent(health)
    _text(canvas, f"Memory       {_number(memory, '%')}", x0, line); line += 24
    used_mb, total_mb = health.get("memory_used_mb"), health.get("memory_total_mb")
    if isinstance(used_mb, (int, float)) and isinstance(total_mb, (int, float)):
        _text(canvas, f"             {used_mb:.0f}/{total_mb:.0f} MiB", x0, line, scale=0.44); line += 22
    temperature = health.get("temperature_c")
    temp_color = (80, 220, 110) if isinstance(temperature, (int, float)) and temperature < 75 else (70, 165, 255)
    _text(canvas, f"Temperature  {_number(temperature, ' C')}", x0, line, temp_color); line += 24
    _text(canvas, f"BEV work     {_number(health.get('bev_process_ms'), ' ms')}", x0, line); line += 24
    _text(canvas, f"BEV age      {_number(health.get('bev_age_ms'), ' ms')}", x0, line); line += 24
    _text(canvas, f"Health       {'fresh' if health_ok else 'stale'}", x0, line,
          (80, 220, 110) if health_ok else (80, 90, 240)); line += 35

    _text(canvas, "CAMERAS", x0, line, (70, 190, 255), 0.58, 2); line += 30
    cameras = health.get("cameras") if isinstance(health.get("cameras"), list) else []
    default_roles = ("FRONT", "REAR", "LEFT", "RIGHT")
    for index in range(4):
        camera = cameras[index] if index < len(cameras) and isinstance(cameras[index], dict) else {}
        online = bool(camera.get("online"))
        color = (80, 220, 110) if online else (80, 90, 240)
        age = camera.get("age_ms")
        role = str(camera.get("role", default_roles[index])).upper()[:6]
        _text(canvas, f"{index} {role:<6} {'OK' if online else 'OFF'} age {_number(age, ' ms', 0)}",
              x0, line, color, 0.48, 1)
        line += 23
    active_mask = health.get("bev_active_camera_mask")
    fresh_mask = health.get("bev_fresh_camera_mask")
    _text(canvas, f"mask active/fresh {active_mask if active_mask is not None else '--'}/{fresh_mask if fresh_mask is not None else '--'}",
          x0, line, (180, 180, 180), 0.43); line += 34

    flags = snapshot["flags"]
    flag_names = []
    if flags & FLAG_PARTIAL_CAMERAS: flag_names.append("PARTIAL")
    if flags & FLAG_REPEATED: flag_names.append("REPEAT")
    if flags & FLAG_SYNTHETIC_FILL: flag_names.append("FILL")
    if flags & FLAG_ILLUMINATION: flag_names.append("LIGHT")
    _text(canvas, "FLAGS " + (" ".join(flag_names) if flag_names else "normal"),
          x0, line, (140, 190, 250), 0.44)
    return canvas


def summary(snapshot: dict[str, Any], receiver: UdpReceiver) -> dict[str, Any]:
    health = snapshot["health"]
    vehicle_status = health.get("vehicle_status")
    vehicle_status = vehicle_status if isinstance(vehicle_status, dict) else {}
    can_status = vehicle_status.get("can")
    can_status = can_status if isinstance(can_status, dict) else {}
    vehicle = vehicle_status.get("vehicle")
    vehicle = vehicle if isinstance(vehicle, dict) else {}
    battery = vehicle.get("battery")
    battery = battery if isinstance(battery, dict) else {}
    system_cpu, process_cpu = cpu_metrics(health)
    send_fps, processing_fps = image_rate_metrics(
        health, snapshot["generation_fps"])
    return {
        "connected": snapshot["last_packet_age_s"] is not None and snapshot["last_packet_age_s"] < 1.0,
        "source": snapshot["source"],
        "boot_id": snapshot["boot_id"],
        "bev_frames": snapshot["bev_frames"],
        "raw_frames": snapshot["raw_frames"],
        "output_mode": snapshot["output_mode"],
        "output_stream_mask": health.get("output_stream_mask"),
        "replay_frames": receiver.reassembler.stats.replay_frames_seen,
        "rx_fps": round(snapshot["rx_fps"], 2),
        "raw_fps": snapshot["raw_fps"],
        "generation_fps": round(snapshot["generation_fps"], 2),
        "send_fps": send_fps,
        "image_processing_fps": processing_fps,
        "cpu_percent": system_cpu,
        "gateway_cpu_percent": process_cpu,
        "memory_percent": memory_percent(health),
        "temperature_c": health.get("temperature_c"),
        "camera_count": health.get("camera_count"),
        "fresh_camera_mask": health.get("bev_fresh_camera_mask"),
        "bev_age_ms": health.get("bev_age_ms"),
        "sender_restarts": receiver.reassembler.stats.sender_restarts,
        "frame_gaps": receiver.reassembler.stats.frame_gaps,
        "invalid_datagrams": receiver.reassembler.stats.invalid_datagrams,
        "decode_errors": snapshot["decode_errors"],
        "can_online": bool(can_status.get("online", False)),
        "soc_percent": battery.get("soc_percent"),
    }


def web_status(snapshot: dict[str, Any], receiver: UdpReceiver) -> dict[str, Any]:
    result = summary(snapshot, receiver)
    health = snapshot["health"]
    defaults = ("FRONT", "REAR", "LEFT", "RIGHT")
    received_cameras = health.get("cameras")
    received_cameras = received_cameras if isinstance(received_cameras, list) else []
    cameras = []
    for index, default_role in enumerate(defaults):
        item = (received_cameras[index] if index < len(received_cameras) and
                isinstance(received_cameras[index], dict) else {})
        cameras.append({
            "index": index,
            "role": str(item.get("role", default_role)).upper(),
            "online": bool(item.get("online", False)),
            "age_ms": item.get("age_ms"),
            "sequence": item.get("sequence"),
            "reopens": item.get("reopens"),
        })
    flags = snapshot["flags"]
    flag_names = []
    if flags & FLAG_PARTIAL_CAMERAS: flag_names.append("PARTIAL")
    if flags & FLAG_REPEATED: flag_names.append("REPEAT")
    if flags & FLAG_SYNTHETIC_FILL: flag_names.append("FILL")
    if flags & FLAG_ILLUMINATION: flag_names.append("LIGHT")
    result.update({
        "frame_id": snapshot["frame_id"],
        "data_age_s": snapshot["last_packet_age_s"],
        "bev_receive_age_s": snapshot["last_bev_age_s"],
        "image_receive_age_s": snapshot["last_image_age_s"],
        "health_age_s": snapshot["last_health_age_s"],
        "bev_process_ms": health.get("bev_process_ms"),
        "memory_used_mb": health.get("memory_used_mb"),
        "memory_total_mb": health.get("memory_total_mb"),
        "active_camera_mask": health.get("bev_active_camera_mask"),
        "cameras": cameras,
        "flags": flags,
        "flag_names": flag_names,
        "vehicle_status": (health.get("vehicle_status")
                           if isinstance(health.get("vehicle_status"), dict)
                           else {"can": {"online": False}, "vehicle": {}}),
        "stream": {
            "mode": snapshot["output_mode"],
            "output_stream_mask": health.get("output_stream_mask"),
            "raw_stream_mask": health.get("raw_stream_mask"),
            "raw_fps": snapshot["raw_fps"],
        },
    })
    return result


def parse_args() -> argparse.Namespace:
    project_root = Path(os.environ.get(
        "FSM_PI_MASTER_ROOT", Path(__file__).resolve().parents[1]))
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--gateway-ip", default="192.168.1.186")
    parser.add_argument("--bind", default="0.0.0.0")
    parser.add_argument("--data-port", type=int, default=5600)
    parser.add_argument("--control-port", type=int, default=5601)
    parser.add_argument("--hello-interval", type=float, default=2.0)
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--duration", type=float, default=0,
                        help="Exit after N seconds; 0 means until interrupted")
    parser.add_argument("--report-interval", type=float, default=1.0)
    parser.add_argument("--require-frames", type=int, default=0)
    parser.add_argument("--min-fps", type=float, default=0.0)
    parser.add_argument("--require-replay", action="store_true",
                        help="Fail acceptance if no cached HELLO BEV is received")
    parser.add_argument("--web", action="store_true",
                        help="Serve the local browser dashboard")
    parser.add_argument("--web-bind", default="127.0.0.1")
    parser.add_argument("--web-port", type=int, default=8088)
    parser.add_argument("--vehicle-io-url", default="http://192.168.1.186:8089")
    parser.add_argument("--voice-cache",
                        default=str(project_root / "runtime/voice-cache"))
    parser.add_argument("--capture-program",
                        default=str(project_root / "capture/capture_cameras.py"))
    parser.add_argument("--capture-output",
                        default=str(project_root / "runtime/capture-image"))
    parser.add_argument("--traffic-camera-url",
                        default="http://127.0.0.1:8090")
    parser.add_argument("--dump-latest", default="")
    parser.add_argument("--window-name", default="W102 BEV Monitor")
    return parser.parse_args()


def routed_local_ip(gateway_ip: str, control_port: int) -> str:
    probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        probe.connect((gateway_ip, control_port))
        return str(probe.getsockname()[0])
    except OSError:
        return "N/A"
    finally:
        probe.close()


def main() -> int:
    args = parse_args()
    if (not 1 <= args.data_port <= 65535 or
            not 1 <= args.control_port <= 65535 or
            not 1 <= args.web_port <= 65535):
        print("invalid UDP port", file=sys.stderr)
        return 2
    state = ReceiverState()
    receiver = UdpReceiver(state, args.bind, args.data_port, args.gateway_ip,
                           args.control_port, max(0.25, args.hello_interval))
    stop = threading.Event()
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda _sig, _frame: stop.set())
    receiver.start()
    receiver.ready_event.wait(2.0)
    if receiver.error:
        print(f"receiver failed: {receiver.error}", file=sys.stderr)
        return 2
    web_server: Optional[WebDashboardServer] = None
    if args.web:
        io_client = None
        try:
            from vehicle_io_client import VehicleIoClient
            io_client = VehicleIoClient(args.vehicle_io_url, args.voice_cache)
        except Exception as error:
            print(f"vehicle I/O control unavailable: {error}", file=sys.stderr)
        capture_client = None
        try:
            from camera_capture_client import CameraCaptureClient
            capture_client = CameraCaptureClient(
                args.capture_program, args.capture_output,
                args.gateway_ip, args.control_port)
        except Exception as error:
            print(f"camera capture unavailable: {error}", file=sys.stderr)
        stream_control = GatewayControlClient(args.gateway_ip,
                                              args.control_port)
        traffic_camera = None
        try:
            from traffic_camera_client import TrafficCameraClient
            traffic_camera = TrafficCameraClient(args.traffic_camera_url)
        except Exception as error:
            print(f"traffic camera preview unavailable: {error}", file=sys.stderr)
        web_server = WebDashboardServer(
            args.web_bind, args.web_port,
            lambda: web_status(
                state.snapshot(receiver.reassembler.stats, include_image=False),
                receiver),
            state.latest_jpeg, io_client, capture_client,
            state.stream_status, stream_control, traffic_camera)
        web_server.start()
        web_server.ready_event.wait(2.0)
        if web_server.error:
            receiver.stop()
            receiver.join(timeout=2.0)
            print(f"web dashboard failed: {web_server.error}", file=sys.stderr)
            return 2
        print(f"WEB http://{args.web_bind}:{web_server.bound_port}/", flush=True)
    started = time.monotonic()
    next_report = started
    local_ip = routed_local_ip(args.gateway_ip, args.control_port)
    bind_label = f"{args.bind}:{args.data_port} (LAN {local_ip})"
    try:
        while not stop.is_set():
            now = time.monotonic()
            if args.duration > 0 and now - started >= args.duration:
                break
            snap = state.snapshot(receiver.reassembler.stats,
                                  include_image=not args.headless)
            if now >= next_report:
                print(json.dumps(summary(snap, receiver), ensure_ascii=False), flush=True)
                next_report = now + max(0.2, args.report_interval)
            if not args.headless:
                dashboard = render_dashboard(snap, bind_label, args.gateway_ip)
                cv2.imshow(args.window_name, dashboard)
                key = cv2.waitKey(30) & 0xFF
                if key in (27, ord("q")):
                    break
            else:
                time.sleep(0.02)
            if receiver.error:
                print(f"receiver failed: {receiver.error}", file=sys.stderr)
                return 2
    finally:
        if web_server is not None:
            web_server.stop()
            web_server.join(timeout=2.0)
        receiver.stop()
        receiver.join(timeout=2.0)
        if not args.headless:
            cv2.destroyAllWindows()
    snap = state.snapshot(receiver.reassembler.stats, include_image=False)
    result = summary(snap, receiver)
    print("FINAL " + json.dumps(result, ensure_ascii=False), flush=True)
    if args.dump_latest and not state.save_latest(args.dump_latest):
        print("no BEV frame available to dump", file=sys.stderr)
        return 3
    if snap["bev_frames"] < args.require_frames:
        print(f"expected {args.require_frames} BEV frames, got {snap['bev_frames']}", file=sys.stderr)
        return 4
    if args.min_fps > 0 and snap["rx_fps"] < args.min_fps:
        print(f"BEV FPS {snap['rx_fps']:.2f} below {args.min_fps:.2f}", file=sys.stderr)
        return 5
    if args.require_replay and receiver.reassembler.stats.replay_frames_seen == 0:
        print("no HELLO replay BEV frame received", file=sys.stderr)
        return 6
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
