#!/usr/bin/env python3
"""Assemble traceable calibration candidates and explicit quality gates."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import shutil
from typing import Any

import numpy as np
from scipy.spatial.transform import Rotation
import yaml

from extract_camera_pair_constraints import matrix_summary
from refine_lidar_environment import matrix_from_item


CAMERA_FRAMES = (
    "video_front_left/camera_link",
    "video_front_mid/camera_link",
    "video_front_right/camera_link",
    "video_rear_left/camera_link",
    "video_rear_mid/camera_link",
    "video_rear_right/camera_link",
    "fisheye_front/camera_link",
    "fisheye_left/camera_link",
    "fisheye_right/camera_link",
    "fisheye_rear/camera_link",
    "traffic_light_front/camera_link",
)


def load(path: Path) -> dict[str, Any]:
    return yaml.safe_load(path.read_text(encoding="utf-8"))


def classify(position: float | None, angle: float | None, force_recapture: bool = False) -> str:
    if force_recapture or position is None or angle is None:
        return "requires_recapture"
    if position <= 0.001 and angle <= 0.1:
        return "target_repeatability_only"
    if position <= 0.01 and angle <= 1.0:
        return "fallback_1cm_1deg"
    return "coarse_candidate"


def extended_summary(matrix: np.ndarray) -> dict[str, Any]:
    summary: dict[str, Any] = matrix_summary(matrix)
    summary["quaternion_xyzw"] = np.round(
        Rotation.from_matrix(matrix[:3, :3]).as_quat(), 10
    ).tolist()
    summary["matrix_row_major"] = np.round(matrix, 10).tolist()
    return summary


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--lidar", type=Path, required=True)
    parser.add_argument("--camera-graph", type=Path, required=True)
    parser.add_argument("--rear-mid", type=Path, required=True)
    parser.add_argument("--rear-fisheye", type=Path, required=True)
    parser.add_argument("--traffic", type=Path, required=True)
    parser.add_argument("--intrinsics", type=Path, required=True)
    parser.add_argument("--lidar-driver", type=Path, required=True)
    parser.add_argument("--legacy-lidar-driver", type=Path)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()

    lidar = load(args.lidar)
    graph = load(args.camera_graph)
    rear_mid = load(args.rear_mid)
    rear_fisheye = load(args.rear_fisheye)
    traffic = load(args.traffic)
    args.output_dir.mkdir(parents=True, exist_ok=True)
    extrinsic_dir = args.output_dir / "extrinsics"
    lidar_intrinsic_dir = args.output_dir / "lidar_intrinsics"
    extrinsic_dir.mkdir(exist_ok=True)
    lidar_intrinsic_dir.mkdir(exist_ok=True)

    lidar_output: dict[str, Any] = {
        "schema_version": 1,
        "reference_frame": "velodyne_top_base_link",
        "coordinate_convention": "x_forward_y_left_z_up_fixed_xyz_rpy",
        "warning": "Repeatability is not surveyed absolute accuracy.",
        "transforms": {},
    }
    for frame, item in lidar["transforms"].items():
        quality = lidar["quality"][frame]
        position = float(quality["position_p95_m"])
        angle = float(quality["angle_p95_deg"])
        lidar_output["transforms"][frame] = {
            **item,
            **extended_summary(matrix_from_item(item)),
            "calibration_status": classify(position, angle),
            "repeatability_position_p95_m": position,
            "repeatability_angle_p95_deg": angle,
            "dominant_fraction": float(quality["dominant_fraction"]),
            "evidence": "round_029_lidar_variant_consensus",
        }
    (extrinsic_dir / "lidar_relative_to_roof_lidar.yaml").write_text(
        yaml.safe_dump(lidar_output, allow_unicode=True, sort_keys=False), encoding="utf-8"
    )

    camera_transforms = dict(graph["transforms"])
    camera_quality: dict[str, dict[str, Any]] = {}
    for frame, quality in graph["quality"].items():
        camera_quality[frame] = {
            "position": float(quality["position_p95_m"]),
            "angle": float(quality["angle_p95_deg"]),
            "evidence": "round_044_camera_graph_variant_consensus",
            "note": "three graph variants; estimator sensitivity",
        }
    camera_transforms["video_rear_mid/camera_link"] = rear_mid["transforms"][
        "video_rear_mid/camera_link"
    ]
    rear_mid_stability = rear_mid["quality"]["video_rear_mid/camera_link"]["stability"]
    camera_quality["video_rear_mid/camera_link"] = {
        "position": float(rear_mid_stability["position_p95_m"]),
        "angle": float(rear_mid_stability["angle_p95_deg"]),
        "evidence": "round_037_seven_rear_board_poses",
        "note": "translation not observable; mechanical prior plus low support",
        "force_recapture": True,
    }
    camera_transforms["fisheye_rear/camera_link"] = rear_fisheye["transforms"][
        "fisheye_rear/camera_link"
    ]
    camera_quality["fisheye_rear/camera_link"] = {
        "position": None,
        "angle": None,
        "evidence": "round_036_six_of_seven_rear_board_poses",
        "note": "no valid bootstrap; translation not observable",
        "force_recapture": True,
    }
    camera_transforms["traffic_light_front/camera_link"] = traffic["transforms"][
        "traffic_light_front/camera_link"
    ]
    traffic_stability = traffic["quality"]["traffic_light_front/camera_link"]["stability"]
    camera_quality["traffic_light_front/camera_link"] = {
        "position": float(traffic_stability["position_p95_m"]),
        "angle": float(traffic_stability["angle_p95_deg"]),
        "evidence": "round_030_fresh_pixel_direct_lidar_bundle",
        "note": "5 Hz cross-camera association failed the physical gate",
        "force_recapture": True,
    }
    # The front-mid/front-fisheye pair is internally precise, but the pair has
    # a shared 2--3 cm translation mode relative to the roof lidar.
    for frame in ("video_front_mid/camera_link", "fisheye_front/camera_link"):
        camera_quality[frame]["force_recapture"] = True
        camera_quality[frame]["note"] += "; common translation mode relative to roof lidar"

    camera_output: dict[str, Any] = {
        "schema_version": 1,
        "reference_frame": "velodyne_top_base_link",
        "coordinate_convention": "camera_link_x_forward_y_left_z_up_fixed_xyz_rpy",
        "status_note": {
            "target_repeatability_only": "numerical repeatability target met; not surveyed truth",
            "fallback_1cm_1deg": "user fallback repeatability threshold met",
            "coarse_candidate": "usable only as a seed",
            "requires_recapture": "candidate retained but blocked from precision deployment",
        },
        "transforms": {},
    }
    for frame in CAMERA_FRAMES:
        item = camera_transforms[frame]
        quality = camera_quality[frame]
        position, angle = quality["position"], quality["angle"]
        camera_output["transforms"][frame] = {
            **item,
            **extended_summary(matrix_from_item(item)),
            "calibration_status": classify(
                position, angle, bool(quality.get("force_recapture", False))
            ),
            "repeatability_position_p95_m": position,
            "repeatability_angle_p95_deg": angle,
            "evidence": quality["evidence"],
            "quality_note": quality["note"],
        }
    (extrinsic_dir / "camera_relative_to_roof_lidar.yaml").write_text(
        yaml.safe_dump(camera_output, allow_unicode=True, sort_keys=False), encoding="utf-8"
    )

    combined = {
        "schema_version": 1,
        "reference_frame": "velodyne_top_base_link",
        "top_lidar_identity": {
            "parent": "velodyne_top_base_link",
            "xyz_m": [0.0, 0.0, 0.0],
            "rpy_deg": [0.0, 0.0, 0.0],
        },
        "transforms": {**lidar_output["transforms"], **camera_output["transforms"]},
    }
    (extrinsic_dir / "all_sensors_relative_to_roof_lidar.yaml").write_text(
        yaml.safe_dump(combined, allow_unicode=True, sort_keys=False), encoding="utf-8"
    )

    kit_from_top = np.eye(4)
    kit_from_top[:3, 3] = [1.665, 0.0, 1.742]
    autoware = {
        "schema_version": 1,
        "parent_frame": "sensor_kit_base_link",
        "warning": "Candidate only. Sensors marked requires_recapture must not be treated as precision calibrated.",
        "transforms": {},
        "deployment_blocked_sensors": [],
    }
    for frame, item in combined["transforms"].items():
        matrix = kit_from_top @ matrix_from_item(item)
        status = item["calibration_status"]
        autoware["transforms"][frame] = {
            "parent": "sensor_kit_base_link",
            **extended_summary(matrix),
            "calibration_status": status,
        }
        if status == "requires_recapture":
            autoware["deployment_blocked_sensors"].append(frame)
    autoware["transforms"]["velodyne_top_base_link"] = {
        "parent": "sensor_kit_base_link",
        **extended_summary(kit_from_top),
        "calibration_status": "existing_vehicle_config",
    }
    (extrinsic_dir / "autoware_sensor_kit_candidate.yaml").write_text(
        yaml.safe_dump(autoware, allow_unicode=True, sort_keys=False), encoding="utf-8"
    )

    intrinsic_audit: dict[str, Any] = {"schema_version": 1, "cameras": {}}
    for frame in CAMERA_FRAMES:
        name = frame.split("/")[0]
        source = args.intrinsics / f"{name}.yaml"
        report = args.intrinsics / f"{name}.report.json"
        data = load(source)
        report_data = json.loads(report.read_text(encoding="utf-8"))
        intrinsic_audit["cameras"][name] = {
            "file": f"../camera_intrinsics/{name}.yaml",
            "model": data["distortion_model"],
            "resolution": [int(data["image_width"]), int(data["image_height"])],
            "rms_reprojection_error_px": float(report_data["rms_reprojection_error_px"]),
            "maximum_view_error_px": float(report_data["maximum_view_error_px"]),
            "status": "accepted_existing_intrinsic_calibration",
        }
    (args.output_dir / "camera_intrinsics_audit.yaml").write_text(
        yaml.safe_dump(intrinsic_audit, allow_unicode=True, sort_keys=False), encoding="utf-8"
    )

    shutil.copy2(args.lidar_driver, lidar_intrinsic_dir / "active_driver_models_ports.yaml")
    if args.legacy_lidar_driver and args.legacy_lidar_driver.is_file():
        shutil.copy2(args.legacy_lidar_driver, lidar_intrinsic_dir / "legacy_config_600_5in1.yaml")
    (lidar_intrinsic_dir / "README.md").write_text(
        "# LiDAR intrinsic/configuration files\n\n"
        "RoboSense per-laser factory corrections are supplied by each unit's DIFOP stream "
        "(`wait_for_difop: true`); they cannot be estimated from the saved XYZ PCD frames. "
        "The files here preserve model, port, distance and topic configuration. "
        "Extrinsics are in `../extrinsics/lidar_relative_to_roof_lidar.yaml`.\n",
        encoding="utf-8",
    )

    report_lines = [
        "# 车辆传感器标定结果（Mac 离线处理）",
        "",
        "## 结论",
        "",
        "所有数值均以车顶主激光雷达为参考，x 向前、y 向左、z 向上。"
        "本结果包区分数值重复性与绝对真值；没有量具/全站仪证据时不宣称毫米级绝对精度。",
        "",
        "竖装 BPearl 接近欧拉角奇异位形；文件同时提供 RPY、四元数和 4×4 矩阵，程序应优先读取四元数/矩阵。",
        "",
        "### 激光雷达",
        "",
    ]
    for frame, item in lidar_output["transforms"].items():
        report_lines.append(
            f"- `{frame}`：{item['calibration_status']}，p95 重复性 "
            f"{item['repeatability_position_p95_m'] * 1000:.2f} mm / "
            f"{item['repeatability_angle_p95_deg']:.3f}°。"
        )
    report_lines += ["", "### 摄像头外参", ""]
    for frame, item in camera_output["transforms"].items():
        position = item["repeatability_position_p95_m"]
        angle = item["repeatability_angle_p95_deg"]
        metric = (
            "无有效 bootstrap" if position is None else f"{position * 1000:.2f} mm / {angle:.3f}°"
        )
        report_lines.append(f"- `{frame}`：{item['calibration_status']}，{metric}。")
    report_lines += [
        "",
        "## 必须补采",
        "",
        "- 交通灯相机：图像文件是新的，但采集时角点缓存陈旧；离线重检测后，5 Hz 与 USB 相机的同板尺度/同步检查仍失败。",
        "- 后中相机、后鱼眼：只有 7 个悬空后板位姿，平面法的最小奇异值过低，平移不可完全观测。",
        "- 前中与前鱼眼：二者相对关系稳定，但相对主雷达存在约 2–3 cm 共同平移弱模态。",
        "",
        "## 文件",
        "",
        "- `camera_intrinsics/`：11 颗相机 ROS 2 CameraInfo YAML 与原始报告。",
        "- `extrinsics/all_sensors_relative_to_roof_lidar.yaml`：统一参考坐标结果。",
        "- `extrinsics/autoware_sensor_kit_candidate.yaml`：转换到 sensor_kit_base_link 的候选值，含部署阻断清单。",
        "- `lidar_intrinsics/`：雷达型号/端口及 DIFOP 工厂校正说明。",
    ]
    (args.output_dir / "CALIBRATION_REPORT.md").write_text(
        "\n".join(report_lines) + "\n", encoding="utf-8"
    )

    hashes = []
    for path in sorted(args.output_dir.rglob("*")):
        if path.is_file() and path.name != "manifest.sha256":
            digest = hashlib.sha256(path.read_bytes()).hexdigest()
            hashes.append(f"{digest}  {path.relative_to(args.output_dir)}")
    (args.output_dir / "manifest.sha256").write_text("\n".join(hashes) + "\n", encoding="utf-8")
    print(f"assembled {len(hashes)} result files in {args.output_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
