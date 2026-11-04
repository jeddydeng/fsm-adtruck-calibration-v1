#!/usr/bin/env python3
"""Monitor the six direct-capture cameras without publishing them to ROS."""

from __future__ import annotations

from dataclasses import dataclass
import glob
import os
from pathlib import Path
import time
from typing import Any

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
import yaml


@dataclass(frozen=True)
class CameraConfig:
    index: int
    role: str
    ros_namespace: str
    physical_device: Path
    identity_globs: tuple[str, ...]
    expected_usb_id: str
    frame_id: str
    camera_name: str

    def select_device(self) -> tuple[Path | None, str | None]:
        """Prefer a unique hardware identity, then the confirmed physical port."""
        matches = {Path(match) for pattern in self.identity_globs for match in glob.glob(pattern)}
        if len(matches) > 1:
            names = ", ".join(str(path) for path in sorted(matches))
            return None, f"multiple identity matches ({names})"
        if matches:
            return next(iter(matches)), None
        if self.identity_globs:
            return None, "no configured camera identity is connected"
        return self.physical_device, None


@dataclass
class CameraState:
    online: bool = False
    resolved_device: str | None = None
    last_error_time: float = 0.0


def _require_string(entry: dict[str, Any], key: str) -> str:
    value = entry.get(key)
    if not isinstance(value, str) or not value:
        raise ValueError(f"camera entry requires non-empty string '{key}'")
    return value


def load_camera_configs(config_file: Path) -> tuple[CameraConfig, ...]:
    """Load and strictly validate the physical camera-to-role assignment."""
    with config_file.open(encoding="utf-8") as stream:
        document = yaml.safe_load(stream)
    entries = document.get("cameras") if isinstance(document, dict) else None
    if not isinstance(entries, list) or len(entries) != 6:
        raise ValueError("camera_devices.yaml must define exactly six cameras")

    configs: list[CameraConfig] = []
    for entry in entries:
        if not isinstance(entry, dict):
            raise ValueError("each camera entry must be a mapping")
        index = entry.get("index")
        if not isinstance(index, int) or not 0 <= index <= 5:
            raise ValueError("camera index must be an integer from 0 through 5")
        patterns = entry.get("identity_globs", [])
        if not isinstance(patterns, list) or not all(
            isinstance(pattern, str) and pattern for pattern in patterns
        ):
            raise ValueError("identity_globs must be a list of non-empty strings")
        if "publish_to_ros" in entry:
            raise ValueError(
                "the six observation cameras must not define publish_to_ros; "
                "the Raspberry Pi traffic-light camera uses a separate pipeline"
            )
        configs.append(
            CameraConfig(
                index=index,
                role=_require_string(entry, "role"),
                ros_namespace=_require_string(entry, "ros_namespace"),
                physical_device=Path(_require_string(entry, "physical_device")),
                identity_globs=tuple(patterns),
                expected_usb_id=_require_string(entry, "expected_usb_id").lower(),
                frame_id=_require_string(entry, "frame_id"),
                camera_name=_require_string(entry, "camera_name"),
            )
        )

    configs.sort(key=lambda config: config.index)
    if [config.index for config in configs] != list(range(6)):
        raise ValueError("camera indexes must be unique and cover 0 through 5")
    if len({config.role for config in configs}) != 6:
        raise ValueError("camera roles must be unique")
    if len({config.ros_namespace for config in configs}) != 6:
        raise ValueError("camera namespaces must be unique")
    if len({config.camera_name for config in configs}) != 6:
        raise ValueError("camera names must be unique")
    return tuple(configs)


def read_usb_id(video_device: str) -> str | None:
    """Read VID:PID from sysfs without launching a subprocess."""
    sys_device = Path("/sys/class/video4linux") / Path(video_device).name / "device"
    try:
        resolved = sys_device.resolve(strict=True)
    except (FileNotFoundError, OSError):
        return None
    for candidate in (resolved, *resolved.parents):
        vendor_file = candidate / "idVendor"
        product_file = candidate / "idProduct"
        if not vendor_file.is_file() or not product_file.is_file():
            continue
        try:
            vendor = vendor_file.read_text(encoding="ascii").strip().lower()
            product = product_file.read_text(encoding="ascii").strip().lower()
        except OSError:
            return None
        return f"{vendor}:{product}"
    return None


class CameraSupervisor(Node):
    """Report hot-plug state while leaving all camera devices unopened."""

    def __init__(self) -> None:
        super().__init__("camera_supervisor")
        config_dir = Path(get_package_share_directory("camera_usb_pkg")) / "config"
        self._configs = load_camera_configs(config_dir / "camera_devices.yaml")
        self._states = {config.index: CameraState() for config in self._configs}
        self._timer = self.create_timer(1.0, self._service)
        mapping = ", ".join(f"{config.camera_name}={config.role}" for config in self._configs)
        self.get_logger().info("Direct-capture camera registry loaded: " + mapping)
        self.get_logger().info(
            "All six vehicle cameras are excluded from ROS and Autoware; "
            "monitoring USB presence only"
        )

    def _error(self, state: CameraState, message: str) -> None:
        now = time.monotonic()
        if now - state.last_error_time >= 5.0:
            self.get_logger().error(message)
            state.last_error_time = now

    def _service_camera(self, config: CameraConfig) -> None:
        state = self._states[config.index]
        stable_device, selection_error = config.select_device()
        if selection_error is not None or stable_device is None:
            state.online = False
            state.resolved_device = None
            self._error(
                state,
                f"{config.camera_name} ({config.role}) offline: "
                f"{selection_error}; waiting for USB reconnect",
            )
            return

        if not stable_device.exists():
            state.online = False
            state.resolved_device = None
            self._error(
                state,
                f"{config.camera_name} ({config.role}) offline: "
                f"{stable_device}; waiting for USB reconnect",
            )
            return

        resolved = os.path.realpath(stable_device)
        actual_usb_id = read_usb_id(resolved)
        if actual_usb_id != config.expected_usb_id:
            state.online = False
            state.resolved_device = None
            self._error(
                state,
                f"{config.camera_name} ({config.role}) identity mismatch at "
                f"{stable_device}: expected {config.expected_usb_id}, "
                f"found {actual_usb_id or 'unknown'}",
            )
            return

        if not state.online or state.resolved_device != resolved:
            self.get_logger().info(
                f"{config.camera_name} online: {stable_device} -> {resolved}; "
                "available for direct capture"
            )
        state.online = True
        state.resolved_device = resolved

    def _service(self) -> None:
        for config in self._configs:
            try:
                self._service_camera(config)
            except Exception as error:  # keep other cameras observable
                state = self._states[config.index]
                state.online = False
                self._error(
                    state,
                    f"{config.camera_name} monitor error: {type(error).__name__}: {error}",
                )


def main() -> int:
    rclpy.init()
    node = CameraSupervisor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
