#!/usr/bin/env python3
"""Validate the four 1600x1200 equidistant camera calibration bundles."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path

import cv2
import numpy as np
import yaml


CAMERAS = ("fisheye_front", "fisheye_rear", "fisheye_left", "fisheye_right")


def check(condition: bool, label: str, checks: list[dict[str, object]]) -> None:
    checks.append({"check": label, "pass": bool(condition)})


def matrix_from_yaml(block: dict[str, object]) -> np.ndarray:
    return np.asarray(block["data"], dtype=float).reshape(int(block["rows"]), int(block["cols"]))


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("result_dir", type=Path)
    args = parser.parse_args()
    checks: list[dict[str, object]] = []
    summary: dict[str, object] = {}
    for name in CAMERAS:
        folder = args.result_dir / name
        report = json.loads((folder / "report.json").read_text(encoding="utf-8"))
        camera_info = yaml.safe_load((folder / "camera_info.yaml").read_text(encoding="utf-8"))
        camera = np.asarray(report["camera_matrix"], dtype=float)
        distortion = np.asarray(report["distortion"], dtype=float)
        accepted = int(report["images"]["accepted_by_error"])
        error = report["accepted_view_error_px"]
        stability = report["stability"]
        coverage = report["coverage"]
        prefix = name + ": "
        check(report["model"] == "equidistant", prefix + "equidistant model", checks)
        check(report["resolution"] == [1600, 1200], prefix + "1600x1200 resolution", checks)
        check(bool(report["quality_pass"]), prefix + "calibrator quality gate", checks)
        check(accepted >= 100, prefix + "at least 100 accepted views", checks)
        check(float(error["median"]) <= 0.65, prefix + "median <= 0.65 px", checks)
        check(float(error["p95"]) <= 0.90, prefix + "p95 <= 0.90 px", checks)
        check(float(error["maximum"]) <= 0.90, prefix + "max accepted <= 0.90 px", checks)
        check(int(stability["successful_folds"]) == 5, prefix + "5/5 held-out folds", checks)
        check(float(stability["focal_range_percent"]) <= 2.0, prefix + "focal spread <= 2%", checks)
        check(
            float(stability["principal_point_range_px"]) <= 15.0,
            prefix + "principal spread <= 15 px",
            checks,
        )
        check(
            float(stability["maximum_held_out_p95_px"]) <= 1.2,
            prefix + "held-out p95 <= 1.2 px",
            checks,
        )
        check(
            float(coverage["x_min_fraction"]) < 0.18
            and float(coverage["x_max_fraction"]) > 0.82
            and float(coverage["y_min_fraction"]) < 0.18
            and float(coverage["y_max_fraction"]) > 0.82,
            prefix + "full-frame target coverage",
            checks,
        )
        check(
            camera.shape == (3, 3) and np.all(np.isfinite(camera)), prefix + "finite 3x3 K", checks
        )
        check(
            distortion.shape == (4,) and np.all(np.isfinite(distortion)),
            prefix + "finite 4-coefficient D",
            checks,
        )
        check(
            0.98 <= camera[0, 0] / camera[1, 1] <= 1.02, prefix + "square-pixel focal ratio", checks
        )
        check(
            camera_info["distortion_model"] == "equidistant",
            prefix + "ROS2 distortion model",
            checks,
        )
        check(
            np.allclose(matrix_from_yaml(camera_info["camera_matrix"]), camera),
            prefix + "ROS2 K matches report",
            checks,
        )
        check(
            np.allclose(
                matrix_from_yaml(camera_info["distortion_coefficients"]).reshape(-1), distortion
            ),
            prefix + "ROS2 D matches report",
            checks,
        )
        for preview in ("reprojection_preview.jpg", "undistortion_preview.jpg"):
            image = cv2.imread(str(folder / preview))
            check(image is not None and image.size > 0, prefix + preview + " readable", checks)
        summary[name] = {
            "K": camera.tolist(),
            "D": distortion.tolist(),
            "accepted_views": accepted,
            "median_px": float(error["median"]),
            "p95_px": float(error["p95"]),
            "held_out_p95_max_px": float(stability["maximum_held_out_p95_px"]),
            "focal_spread_percent": float(stability["focal_range_percent"]),
            "principal_point_spread_px": float(stability["principal_point_range_px"]),
        }
    passed = sum(bool(item["pass"]) for item in checks)
    payload = {
        "quality_pass": passed == len(checks),
        "passed": passed,
        "total": len(checks),
        "cameras": summary,
        "checks": checks,
    }
    (args.result_dir / "VALIDATION_REPORT.json").write_text(
        json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    lines = [
        "# 1600×1200 鱼眼相机内参验证",
        "",
        f"结论：{'通过' if payload['quality_pass'] else '未通过'}（{passed}/{len(checks)} 项）",
        "",
        "模型：OpenCV/ROS2 `equidistant`（4 参数 fisheye），未采用 pinhole 候选。",
        "",
        "| 相机 | 接受帧 | 中位误差(px) | P95(px) | 留出P95最大(px) | 焦距跨度 | 主点跨度(px) |",
        "|---|---:|---:|---:|---:|---:|---:|",
    ]
    for name in CAMERAS:
        item = summary[name]
        lines.append(
            f"| {name} | {item['accepted_views']} | {item['median_px']:.3f} | "
            f"{item['p95_px']:.3f} | {item['held_out_p95_max_px']:.3f} | "
            f"{item['focal_spread_percent']:.3f}% | "
            f"{item['principal_point_spread_px']:.3f} |"
        )
    lines += [
        "",
        "说明：约 180°鱼眼转为单张普通透视图时，边缘拉伸和黑区是投影特性；"
        "BEV 应直接使用 equidistant 模型和外参生成映射。",
        "同一画面中的多块同 ID ChArUco 板已先做空间分组，每帧只使用一块物理板。",
        "",
    ]
    (args.result_dir / "VALIDATION_REPORT.md").write_text("\n".join(lines), encoding="utf-8")
    hashes = []
    for path in sorted(args.result_dir.rglob("*")):
        if not path.is_file() or path.name == "manifest.sha256":
            continue
        digest = hashlib.sha256(path.read_bytes()).hexdigest()
        hashes.append(f"{digest}  {path.relative_to(args.result_dir)}")
    (args.result_dir / "manifest.sha256").write_text("\n".join(hashes) + "\n", encoding="utf-8")
    print(
        json.dumps(
            {"quality_pass": payload["quality_pass"], "passed": passed, "total": len(checks)}
        )
    )
    return 0 if payload["quality_pass"] else 3


if __name__ == "__main__":
    raise SystemExit(main())
