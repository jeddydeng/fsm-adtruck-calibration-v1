#!/usr/bin/env python3
"""Validate the final offline calibration bundle without changing candidates."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
from typing import Any

import numpy as np
from scipy.spatial.transform import Rotation
import yaml


CAMERAS = (
    "video_front_left",
    "video_front_mid",
    "video_front_right",
    "video_rear_left",
    "video_rear_mid",
    "video_rear_right",
    "fisheye_front",
    "fisheye_left",
    "fisheye_right",
    "fisheye_rear",
    "traffic_light_front",
)
LIDARS = (
    "velodyne_front_left_base_link",
    "velodyne_front_right_base_link",
    "velodyne_rear_base_link",
)


def load(path: Path) -> dict[str, Any]:
    return yaml.safe_load(path.read_text(encoding="utf-8"))


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("result", type=Path)
    parser.add_argument("--output-json", type=Path, required=True)
    parser.add_argument("--output-md", type=Path, required=True)
    args = parser.parse_args()
    checks: list[dict[str, Any]] = []

    def check(name: str, passed: bool, detail: str) -> None:
        checks.append({"name": name, "passed": bool(passed), "detail": detail})

    intrinsic_dir = args.result / "camera_intrinsics"
    for name in CAMERAS:
        path = intrinsic_dir / f"{name}.yaml"
        report_path = intrinsic_dir / f"{name}.report.json"
        check(
            f"intrinsic_files:{name}",
            path.is_file() and report_path.is_file(),
            "YAML and source report present",
        )
        if not path.is_file() or not report_path.is_file():
            continue
        item = load(path)
        matrix = np.asarray(item["camera_matrix"]["data"], dtype=float).reshape(3, 3)
        distortion = np.asarray(item["distortion_coefficients"]["data"], dtype=float)
        report = json.loads(report_path.read_text(encoding="utf-8"))
        rms = float(report["rms_reprojection_error_px"])
        maximum = float(report["maximum_view_error_px"])
        valid = (
            int(item["image_width"]) > 0
            and int(item["image_height"]) > 0
            and np.all(np.isfinite(matrix))
            and np.all(np.isfinite(distortion))
            and matrix[0, 0] > 0
            and matrix[1, 1] > 0
            and rms < 0.8
            and maximum < 1.0
        )
        check(
            f"intrinsic_numeric:{name}",
            valid,
            f"model={item['distortion_model']}, rms={rms:.4f}px, max={maximum:.4f}px",
        )

    lidar = load(args.result / "extrinsics/lidar_relative_to_roof_lidar.yaml")
    camera = load(args.result / "extrinsics/camera_relative_to_roof_lidar.yaml")
    combined = load(args.result / "extrinsics/all_sensors_relative_to_roof_lidar.yaml")
    expected_camera_frames = {f"{name}/camera_link" for name in CAMERAS}
    check(
        "lidar_frame_set",
        set(lidar["transforms"]) == set(LIDARS),
        f"found {len(lidar['transforms'])}/3 lower lidar frames",
    )
    check(
        "camera_frame_set",
        set(camera["transforms"]) == expected_camera_frames,
        f"found {len(camera['transforms'])}/11 camera frames",
    )
    check(
        "combined_frame_set",
        set(combined["transforms"]) == set(LIDARS) | expected_camera_frames,
        f"found {len(combined['transforms'])}/14 non-reference sensor frames",
    )
    for frame, item in combined["transforms"].items():
        xyz = np.asarray(item["xyz_m"], dtype=float)
        rotation = Rotation.from_euler("xyz", item["rpy_deg"], degrees=True).as_matrix()
        quaternion = np.asarray(item.get("quaternion_xyzw", []), dtype=float)
        stored_matrix = np.asarray(item.get("matrix_row_major", []), dtype=float)
        orthogonality = float(np.linalg.norm(rotation.T @ rotation - np.eye(3)))
        determinant = float(np.linalg.det(rotation))
        representation_error = float("inf")
        if quaternion.shape == (4,) and stored_matrix.shape == (4, 4):
            representation_error = max(
                float(np.linalg.norm(Rotation.from_quat(quaternion).as_matrix() - rotation)),
                float(np.linalg.norm(stored_matrix[:3, :3] - rotation)),
                float(np.linalg.norm(stored_matrix[:3, 3] - xyz)),
                abs(float(np.linalg.norm(quaternion)) - 1.0),
            )
        valid = (
            xyz.shape == (3,)
            and np.all(np.isfinite(xyz))
            and np.linalg.norm(xyz) < 20.0
            and orthogonality < 1e-10
            and abs(determinant - 1.0) < 1e-10
            and representation_error < 1e-7
            and item.get("parent") == "velodyne_top_base_link"
        )
        check(
            f"extrinsic_numeric:{frame}",
            valid,
            f"status={item.get('calibration_status')}, det={determinant:.12f}, "
            f"orthogonality={orthogonality:.3e}, representations={representation_error:.3e}",
        )

    autoware = load(args.result / "extrinsics/autoware_sensor_kit_candidate.yaml")
    expected_blocked = {
        "video_front_mid/camera_link",
        "fisheye_front/camera_link",
        "video_rear_mid/camera_link",
        "fisheye_rear/camera_link",
        "traffic_light_front/camera_link",
    }
    blocked = set(autoware.get("deployment_blocked_sensors", []))
    check(
        "deployment_block_list", blocked == expected_blocked, "blocked=" + ",".join(sorted(blocked))
    )
    check(
        "autoware_frame_count",
        len(autoware["transforms"]) == 15,
        f"found {len(autoware['transforms'])}/15 including roof lidar",
    )

    manifest = args.result / "manifest.sha256"
    hash_failures = []
    if manifest.is_file():
        for line in manifest.read_text(encoding="utf-8").splitlines():
            digest, relative = line.split("  ", 1)
            path = args.result / relative
            actual = hashlib.sha256(path.read_bytes()).hexdigest() if path.is_file() else "missing"
            if actual != digest:
                hash_failures.append(relative)
    else:
        hash_failures.append("manifest.sha256 missing")
    check(
        "manifest_hashes",
        not hash_failures,
        "all tracked files match" if not hash_failures else ",".join(hash_failures),
    )

    passed = sum(item["passed"] for item in checks)
    report = {
        "schema_version": 1,
        "status": "pass" if passed == len(checks) else "fail",
        "passed_checks": passed,
        "total_checks": len(checks),
        "checks": checks,
    }
    args.output_json.write_text(
        json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    lines = [
        "# 最终标定文件自动校验",
        "",
        f"结果：**{report['status'].upper()}**（{passed}/{len(checks)}）",
        "",
    ]
    for item in checks:
        lines.append(f"- {'PASS' if item['passed'] else 'FAIL'} `{item['name']}`：{item['detail']}")
    args.output_md.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(
        json.dumps(
            {"status": report["status"], "passed": passed, "total": len(checks)}, ensure_ascii=False
        )
    )
    return 0 if report["status"] == "pass" else 1


if __name__ == "__main__":
    raise SystemExit(main())
