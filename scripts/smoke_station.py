#!/usr/bin/env python3
"""Exercise the capture API using synthetic ROS images/clouds; no hardware is opened."""

from __future__ import annotations

import json
import os
from pathlib import Path
import tempfile
import time
from urllib.request import Request, urlopen

import cv2
import numpy as np
import rclpy
import yaml
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, PointCloud2, PointField

from station_manager import ROOT, StationManager


def api(url: str, capture: bool = False) -> dict:
    request = Request(url, data=b"" if capture else None, method="POST" if capture else "GET")
    with urlopen(request, timeout=10) as response:
        return json.load(response)


def main() -> int:
    # Refuse accidental use on the production domain, even if the wrapper is bypassed.
    if os.environ.get("ROS_DOMAIN_ID") != "193":
        raise RuntimeError("Run via scripts/smoke_station.sh on isolated ROS_DOMAIN_ID=193")
    os.environ.update(
        {
            "CALIBRATION_AUTO_LIDAR": "0",
            "CALIBRATION_AUTO_GATEWAY": "0",
            "CALIBRATION_AUTO_TRAFFIC": "0",
            "CALIBRATION_PORT": "18765",
            "CALIBRATION_BIND_HOST": "127.0.0.1",
        }
    )
    rclpy.init()
    node = rclpy.create_node("calibration_synthetic_fixture")
    profile = ROOT / os.environ.get("CALIBRATION_STATION_CONFIG", "process/joint_extrinsic_station/config.yaml")
    config = yaml.safe_load(profile.read_text())
    topics = [item["topic"] for item in config["lidars"]["sensors"]]
    publishers = [
        node.create_publisher(PointCloud2, topic, qos_profile_sensor_data) for topic in topics
    ]
    image_publisher = node.create_publisher(
        CompressedImage,
        "/sensing/camera/traffic_light_front/image_raw/compressed",
        qos_profile_sensor_data,
    )
    synthetic = np.zeros((240, 320, 3), np.uint8)
    cv2.putText(
        synthetic, "SYNTHETIC TEST", (15, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 0), 2
    )
    ok, encoded = cv2.imencode(".jpg", synthetic)
    assert ok
    manager = StationManager(ROOT)
    with tempfile.TemporaryDirectory(prefix="fsm-calibration-smoke-") as directory:
        os.environ["CALIBRATION_DATA_ROOT"] = directory
        try:
            with manager.lock():
                if manager.entries:
                    raise RuntimeError(
                        "Stop this checkout's existing station before the smoke test"
                    )
                manager.start(traffic_only=True)
            deadline = time.monotonic() + 15
            received = False
            while time.monotonic() < deadline:
                stamp = node.get_clock().now().to_msg()
                for publisher in publishers:
                    cloud = PointCloud2()
                    cloud.header.stamp = stamp
                    cloud.header.frame_id = "synthetic_lidar"
                    cloud.height, cloud.width = 1, 32
                    cloud.fields = [
                        PointField(
                            name=name, offset=index * 4, datatype=PointField.FLOAT32, count=1
                        )
                        for index, name in enumerate(("x", "y", "z", "intensity"))
                    ]
                    cloud.point_step, cloud.row_step = 16, 512
                    cloud.is_dense = True
                    points = np.zeros((32, 4), dtype="<f4")
                    points[:, 0] = np.linspace(2, 3, 32)
                    points[:, 1] = 0.5
                    points[:, 3] = 25
                    cloud.data = points.tobytes()
                    publisher.publish(cloud)
                image = CompressedImage()
                image.header.stamp = stamp
                image.header.frame_id = "traffic_light_front/camera_optical_link"
                image.format = "jpeg"
                image.data = encoded.tobytes()
                image_publisher.publish(image)
                rclpy.spin_once(node, timeout_sec=0.1)
                status = api(manager.url + "/api/status")
                if all(item["online"] for item in status["sensors"].values()):
                    received = True
                    break
            assert received, "Synthetic streams were not received"
            result = api(manager.url + "/api/capture", capture=True)
            assert result["saved_sensor_count"] == 5, result
            capture = Path(result["path"])
            assert len(list((capture / "lidars").glob("*.pcd"))) == 4
            assert len(list((capture / "cameras").glob("*.png"))) == 1
            manifest = json.loads((capture / "manifest.json").read_text())
            assert manifest["complete_sensor_set"] and not manifest["failed_sensors"]
            for path in (capture / "lidars").glob("*.pcd"):
                assert b"POINTS 32\n" in path.read_bytes()
            # Stop publishing, let stale data expire, and verify partial-save behavior.
            time.sleep(2.5)
            stale = api(manager.url + "/api/status")
            assert not any(item["online"] for item in stale["sensors"].values())
            partial = api(manager.url + "/api/capture", capture=True)
            assert partial["saved_sensor_count"] == 0
            assert len(partial["missing_sensors"]) == 5
            print(
                "PASS: 4 PCD + 1 PNG saved; timestamps/manifest; stale detection; missing-input skip"
            )
        finally:
            with manager.lock():
                manager.stop()
            node.destroy_node()
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
