#!/usr/bin/env python3
"""W102 read-only CAN, legacy RS485 display and priority voice runtime."""

from __future__ import annotations

import argparse
import base64
import copy
import hashlib
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import os
from pathlib import Path
import queue
import re
import select
import signal
import socket
import struct
import subprocess
import tempfile
import termios
import threading
import time
from typing import Any, Optional


FEEDBACK_IDS = (0x352, 0x354, 0x356, 0x358, 0x360, 0x362, 0x364)
EVENT_ORDER = ("reverse", "left", "right", "automatic", "remote")
EVENT_LABELS = {
    "reverse": "倒车",
    "left": "左转",
    "right": "右转",
    "automatic": "自动驾驶",
    "remote": "遥控驾驶",
}
ORIGINAL_AUDIO_MAP = {
    "reverse": "back.mp3",
    "left": "left.mp3",
    "right": "right.mp3",
    "automatic": "auto.mp3",
    # The legacy ROS package had no dedicated remote-driving recording.
    "remote": "move.mp3",
}
DEFAULT_CONFIG: dict[str, Any] = {
    "schema_version": 2,
    "can": {"interfaces": ["can10", "can11"], "stale_ms": 500},
    "serial": {
        "enabled": True,
        "device": "/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A9IM4UBG-if00-port0",
        "baud": 9600,
        "interval_s": 0.5,
    },
    "audio": {
        "enabled": True,
        "source": "original",
        "muted": False,
        "output": "hw:CARD=Audio,DEV=0",
        "voice": "voice_f3",
        "volume_percent": 80,
        "max_volume_percent": 80,
        "rate_percent": 0,
        "playback_rate": 1.0,
        "events": {
            "reverse": {"enabled": True, "text": "车辆正在倒车，请注意安全", "interval_s": 3.0},
            "left": {"enabled": True, "text": "车辆正在左转，请注意安全", "interval_s": 4.0},
            "right": {"enabled": True, "text": "车辆正在右转，请注意安全", "interval_s": 4.0},
            "automatic": {"enabled": True, "text": "车辆处于自动驾驶状态", "interval_s": 12.0},
            "remote": {"enabled": True, "text": "车辆处于遥控驾驶状态", "interval_s": 12.0},
        },
    },
}


def monotonic_ms() -> int:
    return int(time.monotonic() * 1000)


def _signal(packed: int, start: int, length: int, signed: bool = False) -> int:
    value = (packed >> start) & ((1 << length) - 1)
    if signed and value & (1 << (length - 1)):
        value -= 1 << length
    return value


def decode_can(state: dict[str, Any], can_id: int, data: bytes,
               interface: str, now_ms: Optional[int] = None) -> bool:
    if can_id not in FEEDBACK_IDS or len(data) < 8:
        return False
    now_ms = monotonic_ms() if now_ms is None else now_ms
    packed = int.from_bytes(data[:8], "little")
    state["interface"] = interface
    state["last_frame_ms"] = now_ms
    state["received_frames"] = int(state.get("received_frames", 0)) + 1
    state.setdefault("message_times_ms", {})[f"0x{can_id:03X}"] = now_ms
    if can_id == 0x352:
        state["control_mode"] = _signal(packed, 0, 2)
        state["stop_state"] = _signal(packed, 2, 3)
        state["ignition_on"] = bool(_signal(packed, 5, 1))
        state["vcu_fault"] = _signal(packed, 8, 3)
        state["vcu_checksum_ok"] = bytes([data[0] ^ data[1] ^ data[2] ^ data[3] ^ data[4] ^ data[5] ^ data[6]])[0] == data[7]
    elif can_id == 0x354:
        state["gear"] = _signal(packed, 0, 2)
        state["speed_mps"] = _signal(packed, 2, 12, True) * 0.01
        state["acceleration_mps2"] = _signal(packed, 14, 12, True) * 0.01
        state["motor_torque"] = _signal(packed, 26, 12)
        state["motor_rpm"] = _signal(packed, 38, 16)
    elif can_id == 0x356:
        state["parking_brake_state"] = _signal(packed, 0, 3)
        state["parking_brake_fault"] = _signal(packed, 16, 8)
    elif can_id == 0x358:
        state["front_wheel_angle"] = _signal(packed, 0, 16, True) * 0.1
        state["steering_fault"] = _signal(packed, 24, 8)
    elif can_id == 0x360:
        state["brake_deceleration_mps2"] = _signal(packed, 0, 12, True) * 0.01
        state["brake_pressure"] = _signal(packed, 16, 8)
        state["brake_fault"] = _signal(packed, 32, 8)
    elif can_id == 0x362:
        soc_raw = _signal(packed, 1, 12)
        state["soc_percent"] = soc_raw * 0.1 if soc_raw <= 1000 else None
        state["battery_current_a"] = _signal(packed, 16, 16, True) * 0.1
        state["battery_voltage_v"] = _signal(packed, 32, 16) * 0.1
        state["charging"] = bool(_signal(packed, 0, 1))
        state["battery_fault"] = _signal(packed, 48, 8)
    elif can_id == 0x364:
        state["headlight"] = _signal(packed, 0, 2)
        state["turn_signal"] = _signal(packed, 2, 2)
        state["reverse_light"] = _signal(packed, 4, 2)
        state["brake_light"] = _signal(packed, 6, 2)
        state["horn"] = _signal(packed, 8, 2)
        state["external_voice"] = _signal(packed, 10, 2)
        state["hazard_light"] = _signal(packed, 12, 2)
    return True


def can_is_fresh(state: dict[str, Any], stale_ms: int,
                 now_ms: Optional[int] = None) -> bool:
    now_ms = monotonic_ms() if now_ms is None else now_ms
    last = state.get("last_frame_ms")
    return isinstance(last, int) and 0 <= now_ms - last <= stale_ms


def selected_event(state: dict[str, Any], config: dict[str, Any],
                   fresh: bool) -> Optional[str]:
    if (not fresh or not config["audio"]["enabled"]
            or config["audio"].get("muted", False)):
        return None
    active = {
        "reverse": state.get("gear") == 2,
        "left": state.get("turn_signal") == 1,
        "right": state.get("turn_signal") == 2,
        "automatic": state.get("control_mode") == 1,
        "remote": state.get("control_mode") == 2,
    }
    events = config["audio"]["events"]
    return next((name for name in EVENT_ORDER
                 if active[name] and events[name]["enabled"]), None)


def effective_volume_percent(audio: dict[str, Any]) -> int:
    """Return the software volume after applying mute and the safety ceiling."""
    if bool(audio.get("muted", False)):
        return 0
    requested = max(0, min(100, int(audio.get("volume_percent", 80))))
    ceiling = max(0, min(100, int(audio.get("max_volume_percent", 80))))
    return min(requested, ceiling)


def modbus_crc(data: bytes) -> int:
    crc = 0xFFFF
    for value in data:
        crc ^= value
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc & 0xFFFF


def legacy_rs485_frame(state: dict[str, Any], fresh: bool) -> bytes:
    frame = bytearray([
        0x01, 0x10, 0x00, 0x00, 0x00, 0x0A, 0x14,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x06, 0x00, 0x07, 0x00, 0x08,
        0x00, 0x09, 0x00, 0x0A, 0x00, 0x00,
    ])
    if fresh:
        frame[8] = 0x03 if state.get("control_mode") == 1 else 0x02
        soc = state.get("soc_percent")
        if isinstance(soc, (int, float)):
            raw_soc = max(0, min(1000, int(round(float(soc) * 10))))
            frame[11], frame[12] = (raw_soc >> 8) & 0xFF, raw_soc & 0xFF
        turn = state.get("turn_signal")
        speed = float(state.get("speed_mps", 0.0) or 0.0)
        if turn == 1:
            frame[10], frame[14] = 0x03, 0x04
        elif turn == 2:
            frame[10], frame[14] = 0x04, 0x03
        elif state.get("gear") == 2 or speed < -0.005:
            frame[10], frame[14] = 0x05, 0x05
        elif abs(speed) <= 0.005:
            frame[10], frame[14] = 0x01, 0x01
        else:
            frame[10], frame[14] = 0x02, 0x02
        frame[15], frame[16] = 0x00, 0x01
    crc = modbus_crc(frame[:-2])
    frame[-2], frame[-1] = crc & 0xFF, (crc >> 8) & 0xFF
    return bytes(frame)


def validate_config(value: Any) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise ValueError("config must be an object")
    result = copy.deepcopy(DEFAULT_CONFIG)
    can = value.get("can", result["can"])
    serial_cfg = value.get("serial", result["serial"])
    audio = value.get("audio", result["audio"])
    interfaces = can.get("interfaces")
    if not isinstance(interfaces, list) or not interfaces or len(interfaces) > 4:
        raise ValueError("CAN interfaces must contain 1-4 names")
    if any(not isinstance(item, str) or not re.fullmatch(r"can[0-9]+", item)
           for item in interfaces):
        raise ValueError("invalid CAN interface")
    result["can"] = {
        "interfaces": interfaces,
        "stale_ms": max(100, min(5000, int(can.get("stale_ms", 500)))),
    }
    device = str(serial_cfg.get("device", "/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A9IM4UBG-if00-port0"))
    if not re.fullmatch(r"/dev/[A-Za-z0-9_./-]+", device):
        raise ValueError("invalid serial device")
    baud = int(serial_cfg.get("baud", 9600))
    if baud not in (9600, 19200, 38400, 57600, 115200):
        raise ValueError("unsupported serial baud")
    result["serial"] = {
        "enabled": bool(serial_cfg.get("enabled", True)),
        "device": device,
        "baud": baud,
        "interval_s": max(0.1, min(10.0, float(serial_cfg.get("interval_s", 0.5)))),
    }
    output = str(audio.get("output", "hw:CARD=Audio,DEV=0"))
    if len(output) > 100 or not re.fullmatch(r"[A-Za-z0-9_:,=.+-]+", output):
        raise ValueError("invalid ALSA output")
    voice = str(audio.get("voice", "voice_f3"))
    if len(voice) > 80 or not re.fullmatch(r"[A-Za-z0-9_+.-]+", voice):
        raise ValueError("invalid voice")
    events_in = audio.get("events")
    if not isinstance(events_in, dict):
        raise ValueError("audio events must be an object")
    events: dict[str, Any] = {}
    for name in EVENT_ORDER:
        item = events_in.get(name, result["audio"]["events"][name])
        if not isinstance(item, dict):
            raise ValueError(f"invalid event {name}")
        text = str(item.get("text", "")).strip()
        if not text or len(text) > 120:
            raise ValueError(f"event {name} text must be 1-120 characters")
        events[name] = {
            "enabled": bool(item.get("enabled", True)),
            "text": text,
            "interval_s": max(1.0, min(300.0, float(item.get("interval_s", 5.0)))),
        }
    source = str(audio.get("source", "original"))
    if source not in ("original", "neural"):
        raise ValueError("audio source must be original or neural")
    result["audio"] = {
        "enabled": bool(audio.get("enabled", True)),
        "source": source,
        "muted": bool(audio.get("muted", False)),
        "output": output,
        "voice": voice,
        "volume_percent": max(0, min(100, int(audio.get("volume_percent", 80)))),
        "max_volume_percent": max(
            0, min(100, int(audio.get("max_volume_percent", 80)))),
        "rate_percent": max(-50, min(100, int(audio.get("rate_percent", 0)))),
        "playback_rate": max(
            0.5, min(2.0, float(audio.get("playback_rate", 1.0)))),
        "events": events,
    }
    return result


def atomic_json(path: Path, value: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, temporary = tempfile.mkstemp(prefix=path.name + ".", dir=path.parent)
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as stream:
            json.dump(value, stream, ensure_ascii=False, indent=2)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    finally:
        try:
            os.unlink(temporary)
        except FileNotFoundError:
            pass


class CanReader(threading.Thread):
    def __init__(self, runtime: "Runtime") -> None:
        super().__init__(name="w102-can-reader", daemon=True)
        self.runtime = runtime
        self.stop_event = threading.Event()
        self.sockets: dict[str, socket.socket] = {}

    def stop(self) -> None:
        self.stop_event.set()

    def _open(self, interface: str) -> None:
        pf_can = getattr(socket, "PF_CAN", 29)
        can_raw = getattr(socket, "CAN_RAW", 1)
        sock = socket.socket(pf_can, socket.SOCK_RAW, can_raw)
        filters = b"".join(struct.pack("=II", can_id, 0xC00007FF)
                           for can_id in FEEDBACK_IDS)
        sock.setsockopt(getattr(socket, "SOL_CAN_RAW", 101),
                        getattr(socket, "CAN_RAW_FILTER", 1), filters)
        sock.setblocking(False)
        sock.bind((interface,))
        self.sockets[interface] = sock

    def run(self) -> None:
        next_open = 0.0
        while not self.stop_event.is_set():
            with self.runtime.lock:
                interfaces = list(self.runtime.config["can"]["interfaces"])
            now = time.monotonic()
            if now >= next_open:
                for interface in interfaces:
                    if interface not in self.sockets:
                        try:
                            self._open(interface)
                        except OSError as error:
                            self.runtime.set_can_error(f"{interface}: {error}")
                next_open = now + 2.0
            readable, _, exceptional = select.select(list(self.sockets.values()), [],
                                                       list(self.sockets.values()), 0.2)
            for sock in exceptional:
                self._drop(sock)
            for sock in readable:
                try:
                    raw = sock.recv(16)
                    can_id, length, data = struct.unpack("=IB3x8s", raw)
                    interface = next(name for name, item in self.sockets.items()
                                     if item is sock)
                    with self.runtime.lock:
                        decode_can(self.runtime.vehicle, can_id & 0x1FFFFFFF,
                                   data[:length], interface)
                        self.runtime.can_error = None
                except (OSError, StopIteration, struct.error):
                    self._drop(sock)
        for sock in self.sockets.values():
            sock.close()

    def _drop(self, sock: socket.socket) -> None:
        for name, item in list(self.sockets.items()):
            if item is sock:
                item.close()
                del self.sockets[name]


class Rs485Sender(threading.Thread):
    SPEEDS = {
        9600: termios.B9600, 19200: termios.B19200, 38400: termios.B38400,
        57600: termios.B57600, 115200: termios.B115200,
    }

    def __init__(self, runtime: "Runtime") -> None:
        super().__init__(name="w102-rs485", daemon=True)
        self.runtime = runtime
        self.stop_event = threading.Event()
        self.fd: Optional[int] = None
        self.open_key: Optional[tuple[str, int]] = None

    def stop(self) -> None:
        self.stop_event.set()

    def _close(self) -> None:
        if self.fd is not None:
            os.close(self.fd)
        self.fd, self.open_key = None, None

    def _open(self, device: str, baud: int) -> None:
        fd = os.open(device, os.O_WRONLY | os.O_NOCTTY | os.O_NONBLOCK)
        attrs = termios.tcgetattr(fd)
        attrs[0] = attrs[1] = attrs[3] = 0
        attrs[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
        attrs[4] = attrs[5] = self.SPEEDS[baud]
        termios.tcsetattr(fd, termios.TCSANOW, attrs)
        self.fd, self.open_key = fd, (device, baud)

    def run(self) -> None:
        next_send = time.monotonic()
        while not self.stop_event.wait(0.02):
            with self.runtime.lock:
                cfg = copy.deepcopy(self.runtime.config["serial"])
                vehicle = dict(self.runtime.vehicle)
                stale = self.runtime.config["can"]["stale_ms"]
            if not cfg["enabled"]:
                self._close()
                continue
            key = (cfg["device"], cfg["baud"])
            if self.open_key != key:
                self._close()
            if self.fd is None:
                try:
                    self._open(*key)
                    self.runtime.set_serial_error(None)
                except OSError as error:
                    self.runtime.set_serial_error(str(error))
                    self.stop_event.wait(1.0)
                    continue
            now = time.monotonic()
            if now < next_send:
                continue
            frame = legacy_rs485_frame(vehicle, can_is_fresh(vehicle, stale))
            try:
                assert self.fd is not None
                written = os.write(self.fd, frame)
                if written != len(frame):
                    raise OSError(f"short serial write {written}/{len(frame)}")
                self.runtime.record_serial_frame(frame)
            except OSError as error:
                self.runtime.set_serial_error(str(error))
                self._close()
            next_send = now + cfg["interval_s"]
        self._close()


class AudioManager(threading.Thread):
    def __init__(self, runtime: "Runtime", ffmpeg: str) -> None:
        super().__init__(name="w102-audio", daemon=True)
        self.runtime = runtime
        self.ffmpeg = ffmpeg
        self.stop_event = threading.Event()
        self.tests: queue.Queue[dict[str, Any]] = queue.Queue(maxsize=4)
        self.config_changed = threading.Event()
        self.process: Optional[subprocess.Popen[bytes]] = None
        self.process_event: Optional[str] = None
        self.current_event: Optional[str] = None
        self.next_play = 0.0
        self.temporary_asset: Optional[Path] = None

    def stop(self) -> None:
        self.stop_event.set()
        self._terminate()

    def request_test(self, request: dict[str, Any]) -> None:
        try:
            self.tests.put_nowait(request)
        except queue.Full as error:
            raise ValueError("audio test queue is busy") from error

    def _terminate(self) -> None:
        if self.process is not None and self.process.poll() is None:
            self.process.terminate()
            try:
                self.process.wait(timeout=1.0)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait(timeout=1.0)
        self.process = None
        if self.temporary_asset is not None:
            self.temporary_asset.unlink(missing_ok=True)
            self.temporary_asset = None

    def _collect(self) -> None:
        if self.process is None or self.process.poll() is None:
            return
        stderr = self.process.communicate()[1].decode("utf-8", "replace").strip()
        code = self.process.returncode
        self.runtime.record_audio_result(self.process_event, code, stderr)
        self.process = None
        if self.temporary_asset is not None:
            self.temporary_asset.unlink(missing_ok=True)
            self.temporary_asset = None

    def _play(self, path: Path, event: str, output: str, volume: int,
              max_volume: int, playback_rate: float,
              temporary: bool = False) -> None:
        if not path.is_file():
            self.runtime.record_audio_result(event, 2, f"missing asset {path.name}")
            return
        self._terminate()
        command = [
            self.ffmpeg, "-nostdin", "-hide_banner", "-loglevel", "error",
            "-i", str(path), "-vn", "-ac", "2", "-ar", "48000",
            "-filter:a", (
                f"atempo={max(0.5, min(2.0, playback_rate)):.3f},"
                f"volume={min(volume, max_volume) / 100.0:.2f}"
            ),
            "-f", "alsa", output,
        ]
        try:
            self.process = subprocess.Popen(command, stdin=subprocess.DEVNULL,
                                            stdout=subprocess.DEVNULL,
                                            stderr=subprocess.PIPE)
            self.process_event = event
            self.temporary_asset = path if temporary else None
            self.runtime.record_audio_started(event)
        except OSError as error:
            self.runtime.record_audio_result(event, 2, str(error))

    def _run_test(self, request: dict[str, Any]) -> None:
        if request["muted"]:
            raise ValueError("audio is muted")
        if request.get("original_event") is not None:
            event = str(request["original_event"])
            filename = ORIGINAL_AUDIO_MAP.get(event)
            if filename is None:
                raise ValueError("unknown original audio event")
            self._play(self.runtime.original_asset_dir / filename,
                       f"preview_{event}", request["output"],
                       request["volume_percent"],
                       request["max_volume_percent"],
                       request["playback_rate"])
            return
        audio = base64.b64decode(request["audio_base64"], validate=True)
        if not 44 <= len(audio) <= 2_000_000:
            raise ValueError("test audio must be 44 bytes to 2 MB")
        fd, filename = tempfile.mkstemp(prefix="preview-", suffix=".wav",
                                        dir=self.runtime.asset_dir)
        with os.fdopen(fd, "wb") as stream:
            stream.write(audio)
        self._play(Path(filename), "preview", request["output"],
                   request["volume_percent"], request["max_volume_percent"],
                   request["playback_rate"], temporary=True)

    def run(self) -> None:
        while not self.stop_event.wait(0.05):
            self._collect()
            if self.config_changed.is_set():
                self.config_changed.clear()
                self._terminate()
                self.current_event = None
                self.next_play = 0.0
            try:
                request = self.tests.get_nowait()
                self._run_test(request)
                continue
            except queue.Empty:
                pass
            except Exception as error:  # preview input is validated again here
                self.runtime.record_audio_result("preview", 2, str(error))
            with self.runtime.lock:
                config = copy.deepcopy(self.runtime.config)
                vehicle = dict(self.runtime.vehicle)
            fresh = can_is_fresh(vehicle, config["can"]["stale_ms"])
            event = selected_event(vehicle, config, fresh)
            if event != self.current_event:
                if self.process is not None and self.process.poll() is None:
                    self._terminate()
                self.current_event = event
                self.next_play = time.monotonic()
                self.runtime.set_selected_event(event)
            if event is None or self.process is not None:
                continue
            if time.monotonic() >= self.next_play:
                audio_cfg = config["audio"]
                if audio_cfg["source"] == "original":
                    path = (self.runtime.original_asset_dir /
                            ORIGINAL_AUDIO_MAP[event])
                else:
                    path = self.runtime.asset_dir / f"{event}.wav"
                self._play(path, event, audio_cfg["output"],
                           audio_cfg["volume_percent"],
                           audio_cfg["max_volume_percent"],
                           audio_cfg["playback_rate"])
                self.next_play = (time.monotonic()
                                  + audio_cfg["events"][event]["interval_s"])
        self._terminate()


class Runtime:
    def __init__(self, config_path: Path, asset_dir: Path,
                 original_asset_dir: Path, ffmpeg: str) -> None:
        self.config_path = config_path
        self.asset_dir = asset_dir
        self.original_asset_dir = original_asset_dir
        self.asset_dir.mkdir(parents=True, exist_ok=True)
        self.lock = threading.RLock()
        self.vehicle: dict[str, Any] = {"received_frames": 0}
        self.can_error: Optional[str] = None
        self.serial_error: Optional[str] = None
        self.serial_frames = 0
        self.serial_last_hex: Optional[str] = None
        self.serial_last_ms: Optional[int] = None
        self.selected_voice_event: Optional[str] = None
        self.audio_last_event: Optional[str] = None
        self.audio_last_started_ms: Optional[int] = None
        self.audio_last_exit_code: Optional[int] = None
        self.audio_error: Optional[str] = None
        self.config_revision = 0
        self.started_ms = monotonic_ms()
        try:
            self.config = validate_config(json.loads(config_path.read_text("utf-8")))
        except (FileNotFoundError, ValueError, json.JSONDecodeError):
            self.config = copy.deepcopy(DEFAULT_CONFIG)
            atomic_json(config_path, self.config)
        self.can_reader = CanReader(self)
        self.rs485 = Rs485Sender(self)
        self.audio = AudioManager(self, ffmpeg)

    def start(self) -> None:
        self.can_reader.start()
        self.rs485.start()
        self.audio.start()

    def stop(self) -> None:
        self.can_reader.stop(); self.rs485.stop(); self.audio.stop()
        self.can_reader.join(timeout=2); self.rs485.join(timeout=2); self.audio.join(timeout=2)

    def set_can_error(self, error: Optional[str]) -> None:
        with self.lock: self.can_error = error

    def set_serial_error(self, error: Optional[str]) -> None:
        with self.lock: self.serial_error = error

    def record_serial_frame(self, frame: bytes) -> None:
        with self.lock:
            self.serial_frames += 1
            self.serial_last_hex = frame.hex().upper()
            self.serial_last_ms = monotonic_ms()
            self.serial_error = None

    def set_selected_event(self, event: Optional[str]) -> None:
        with self.lock: self.selected_voice_event = event

    def record_audio_started(self, event: Optional[str]) -> None:
        with self.lock:
            self.audio_last_event = event
            self.audio_last_started_ms = monotonic_ms()
            self.audio_error = None

    def record_audio_result(self, event: Optional[str], code: int, error: str) -> None:
        with self.lock:
            self.audio_last_event = event
            self.audio_last_exit_code = code
            self.audio_error = error or None if code else None

    def apply(self, config_value: Any, assets: Any) -> dict[str, Any]:
        config = validate_config(config_value)
        if not isinstance(assets, dict):
            raise ValueError("assets must be an object")
        decoded: dict[str, bytes] = {}
        digests: dict[str, str] = {}
        if config["audio"]["source"] == "original":
            missing = [filename for filename in ORIGINAL_AUDIO_MAP.values()
                       if not (self.original_asset_dir / filename).is_file()]
            if missing:
                raise ValueError("missing original audio: " + ", ".join(missing))
        for event in EVENT_ORDER:
            if (config["audio"]["source"] == "neural"
                    and config["audio"]["events"][event]["enabled"]):
                encoded = assets.get(event)
                if not isinstance(encoded, str):
                    raise ValueError(f"missing audio asset for {event}")
                try:
                    payload = base64.b64decode(encoded, validate=True)
                except ValueError as error:
                    raise ValueError(f"invalid base64 asset for {event}") from error
                if not 44 <= len(payload) <= 2_000_000:
                    raise ValueError(f"audio asset {event} must be 44 bytes to 2 MB")
                decoded[event] = payload
                digests[event] = hashlib.sha256(payload).hexdigest()
        for event, payload in decoded.items():
            temporary = self.asset_dir / f".{event}.{os.getpid()}.tmp"
            temporary.write_bytes(payload)
            os.replace(temporary, self.asset_dir / f"{event}.wav")
        config["audio"]["asset_sha256"] = digests
        atomic_json(self.config_path, config)
        with self.lock:
            self.config = config
            self.config_revision += 1
        self.audio.config_changed.set()
        return self.config_snapshot()

    def config_snapshot(self) -> dict[str, Any]:
        with self.lock: return copy.deepcopy(self.config)

    def set_muted(self, muted: bool) -> dict[str, Any]:
        with self.lock:
            config = copy.deepcopy(self.config)
            config["audio"]["muted"] = bool(muted)
            atomic_json(self.config_path, config)
            self.config = config
            self.config_revision += 1
        self.audio.config_changed.set()
        return self.config_snapshot()

    def status(self) -> dict[str, Any]:
        now = monotonic_ms()
        with self.lock:
            vehicle = copy.deepcopy(self.vehicle)
            config = copy.deepcopy(self.config)
            last_can = vehicle.get("last_frame_ms")
            last_serial = self.serial_last_ms
            last_audio = self.audio_last_started_ms
            result = {
                "service": {"online": True, "uptime_s": round((now - self.started_ms) / 1000, 1),
                            "config_revision": self.config_revision},
                "can": {"online": can_is_fresh(vehicle, config["can"]["stale_ms"], now),
                        "read_only": True, "interface": vehicle.get("interface"),
                        "received_frames": vehicle.get("received_frames", 0),
                        "age_ms": now - last_can if isinstance(last_can, int) else None,
                        "error": self.can_error},
                "vehicle": vehicle,
                "serial": {"enabled": config["serial"]["enabled"],
                           "device": config["serial"]["device"], "baud": config["serial"]["baud"],
                           "frames_sent": self.serial_frames,
                           "age_ms": now - last_serial if isinstance(last_serial, int) else None,
                           "last_frame_hex": self.serial_last_hex, "error": self.serial_error},
                "audio": {"enabled": config["audio"]["enabled"],
                          "source": config["audio"]["source"],
                          "muted": config["audio"]["muted"],
                          "output": config["audio"]["output"], "voice": config["audio"]["voice"],
                          "volume_percent": config["audio"]["volume_percent"],
                          "max_volume_percent": config["audio"]["max_volume_percent"],
                          "effective_volume_percent": effective_volume_percent(config["audio"]),
                          "playback_rate": config["audio"]["playback_rate"],
                          "selected_event": self.selected_voice_event,
                          "last_event": self.audio_last_event,
                          "last_play_age_ms": now - last_audio if isinstance(last_audio, int) else None,
                          "last_exit_code": self.audio_last_exit_code, "error": self.audio_error},
            }
        for key in ("last_frame_ms", "message_times_ms"):
            vehicle.pop(key, None)
        return result


class ApiServer(threading.Thread):
    def __init__(self, runtime: Runtime, bind: str, port: int) -> None:
        super().__init__(name="w102-config-api", daemon=True)
        self.runtime = runtime
        self.server = ThreadingHTTPServer((bind, port), self._handler())

    def _handler(self):
        runtime = self.runtime

        class Handler(BaseHTTPRequestHandler):
            server_version = "W102VehicleIO/1"

            def _send(self, status: int, value: Any) -> None:
                body = json.dumps(value, ensure_ascii=False, separators=(",", ":")).encode()
                self.send_response(status)
                self.send_header("Content-Type", "application/json; charset=utf-8")
                self.send_header("Content-Length", str(len(body)))
                self.send_header("Cache-Control", "no-store")
                self.send_header("X-Content-Type-Options", "nosniff")
                self.end_headers()
                self.wfile.write(body)

            def _read(self) -> Any:
                length = int(self.headers.get("Content-Length", "0"))
                if length <= 0 or length > 12_000_000:
                    raise ValueError("invalid request size")
                return json.loads(self.rfile.read(length).decode("utf-8"))

            def _trusted(self) -> bool:
                return self.client_address[0] in ("127.0.0.1", "::1", "192.168.1.102")

            def do_GET(self) -> None:  # noqa: N802
                path = self.path.split("?", 1)[0]
                if path == "/api/status": self._send(200, runtime.status())
                elif path == "/api/config": self._send(200, runtime.config_snapshot())
                else: self._send(404, {"error": "not found"})

            def do_POST(self) -> None:  # noqa: N802
                if not self._trusted():
                    self._send(403, {"error": "writes are restricted to 192.168.1.102"}); return
                try:
                    body = self._read()
                    if self.path == "/api/config":
                        self._send(200, {"ok": True, "config": runtime.apply(body.get("config"), body.get("assets"))})
                    elif self.path == "/api/mute":
                        if not isinstance(body.get("muted"), bool):
                            raise ValueError("muted must be a boolean")
                        self._send(200, {"ok": True,
                                         "config": runtime.set_muted(body["muted"])})
                    elif self.path in ("/api/test", "/api/original-test"):
                        output = str(body.get("output", ""))
                        if not re.fullmatch(r"[A-Za-z0-9_:,=.+-]{1,100}", output):
                            raise ValueError("invalid ALSA output")
                        request = {
                            "audio_base64": str(body.get("audio_base64", "")),
                            "output": output,
                            "volume_percent": max(
                                0, min(100, int(body.get("volume_percent", 80)))),
                            "max_volume_percent": max(
                                0, min(100, int(body.get("max_volume_percent", 80)))),
                            "playback_rate": max(
                                0.5, min(2.0, float(body.get("playback_rate", 1.0)))),
                            "muted": bool(body.get("muted", False)),
                            "original_event": (
                                str(body.get("event", ""))
                                if self.path == "/api/original-test" else None
                            ),
                        }
                        runtime.audio.request_test(request)
                        self._send(202, {"ok": True})
                    else:
                        self._send(404, {"error": "not found"})
                except (ValueError, TypeError, json.JSONDecodeError) as error:
                    self._send(400, {"error": str(error)})

            def log_message(self, _format: str, *_args: Any) -> None:
                return

        return Handler

    def run(self) -> None:
        self.server.serve_forever(poll_interval=0.2)

    def stop(self) -> None:
        self.server.shutdown(); self.server.server_close()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default="/home/ubuntu/Pi-5-image/vehicle-io/state/config.json")
    parser.add_argument("--asset-dir", default="/home/ubuntu/Pi-5-image/vehicle-io/cache")
    parser.add_argument("--original-asset-dir",
                        default="/home/ubuntu/Pi-5-image/vehicle-io/original-audio")
    parser.add_argument("--api-bind", default="0.0.0.0")
    parser.add_argument("--api-port", type=int, default=8089)
    parser.add_argument("--ffmpeg", default="/usr/bin/ffmpeg")
    args = parser.parse_args()
    runtime = Runtime(Path(args.config), Path(args.asset_dir),
                      Path(args.original_asset_dir), args.ffmpeg)
    api = ApiServer(runtime, args.api_bind, args.api_port)
    stop = threading.Event()
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda _sig, _frame: stop.set())
    runtime.start(); api.start()
    try:
        while not stop.wait(1.0):
            pass
    finally:
        api.stop(); api.join(timeout=2); runtime.stop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
