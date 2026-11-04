#!/usr/bin/env python3
"""Fuse camera-lidar absolute poses with simultaneous camera-pair edges."""

from __future__ import annotations

import argparse
import math
from pathlib import Path
from typing import Any

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
import yaml

from extract_camera_pair_constraints import matrix_summary, transform_distance
from refine_lidar_environment import matrix_from_item
from solve_joint_extrinsics import CAMERA_OPTICAL_TO_LINK


def pack(transforms: dict[str, np.ndarray], names: list[str]) -> np.ndarray:
    result = []
    for name in names:
        result.extend(Rotation.from_matrix(transforms[name][:3, :3]).as_rotvec())
        result.extend(transforms[name][:3, 3])
    return np.asarray(result)


def unpack(values: np.ndarray, names: list[str]) -> dict[str, np.ndarray]:
    result = {}
    for index, name in enumerate(names):
        offset = 6 * index
        matrix = np.eye(4)
        matrix[:3, :3] = Rotation.from_rotvec(values[offset : offset + 3]).as_matrix()
        matrix[:3, 3] = values[offset + 3 : offset + 6]
        result[name] = matrix
    return result


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--absolute", type=Path, required=True)
    parser.add_argument("--pairs", type=Path, required=True)
    parser.add_argument("--low-support-pairs", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    absolute_data = yaml.safe_load(args.absolute.read_text(encoding="utf-8"))
    pair_data = yaml.safe_load(args.pairs.read_text(encoding="utf-8"))
    low_data = yaml.safe_load(args.low_support_pairs.read_text(encoding="utf-8"))
    absolute: dict[str, np.ndarray] = {}
    absolute_sigma: dict[str, tuple[float, float]] = {}
    for frame, item in absolute_data.get("transforms", {}).items():
        name = frame.removesuffix("/camera_link")
        absolute[name] = matrix_from_item(item) @ CAMERA_OPTICAL_TO_LINK
        quality_item = absolute_data["quality"][frame]
        stability = quality_item.get("stability", quality_item)
        absolute_sigma[name] = (
            max(0.006, float(stability["position_p95_m"])),
            max(0.10, float(stability["angle_p95_deg"])),
        )
    names = sorted(absolute)
    edges: list[dict[str, Any]] = []
    rejected = {}
    for key, item in pair_data.get("accepted_edges", {}).items():
        left, right = key.split("<-")
        if left not in absolute or right not in absolute:
            rejected[key] = "one or both nodes lack an absolute candidate"
            continue
        rough_position = float(item["difference_from_rough_position_m"])
        rough_angle = float(item["difference_from_rough_angle_deg"])
        if rough_position > 0.50 or rough_angle > 15.0:
            rejected[key] = "relative edge exceeds 0.5 m / 15 deg rough-geometry gate"
            continue
        observed = matrix_from_item(item["transform"])
        before = np.linalg.inv(absolute[left]) @ absolute[right]
        closure_position, closure_angle = transform_distance(before, observed)
        if closure_position > 0.10 or closure_angle > 1.0:
            rejected[key] = (
                "pre-fit independent closure exceeds 0.10 m / 1 deg gate: "
                f"{closure_position:.6f} m, {closure_angle:.6f} deg"
            )
            continue
        edges.append(
            {
                "key": key,
                "left": left,
                "right": right,
                "transform": observed,
                "position_sigma": max(0.006, float(item["quality"]["position_p95_m"])),
                "angle_sigma": max(0.10, float(item["quality"]["angle_p95_deg"])),
                "support": "five_fold",
            }
        )
    for key, item in low_data.get("edges", {}).items():
        left, right = key.split("<-")
        if left not in absolute or right not in absolute:
            continue
        if any(edge["key"] == key for edge in edges):
            # A freshly re-detected five-fold edge supersedes an older
            # low-support version of the same physical camera pair.
            continue
        edges.append(
            {
                "key": key,
                "left": left,
                "right": right,
                "transform": matrix_from_item(item["transform"]),
                "position_sigma": max(0.010, float(item["position_p95_m"])),
                "angle_sigma": max(0.15, float(item["angle_p95_deg"])),
                "support": "four_view_leave_one_out_provisional",
            }
        )

    seed = pack(absolute, names)

    def residual(values: np.ndarray) -> np.ndarray:
        transforms = unpack(values, names)
        parts: list[float] = []
        for name in names:
            candidate = transforms[name]
            position_sigma, angle_sigma = absolute_sigma[name]
            parts.extend((candidate[:3, 3] - absolute[name][:3, 3]) / position_sigma)
            rotation_error = Rotation.from_matrix(
                candidate[:3, :3] @ absolute[name][:3, :3].T
            ).as_rotvec()
            parts.extend(rotation_error / math.radians(angle_sigma))
        for edge in edges:
            predicted = np.linalg.inv(transforms[edge["left"]]) @ transforms[edge["right"]]
            observed = edge["transform"]
            parts.extend((predicted[:3, 3] - observed[:3, 3]) / edge["position_sigma"])
            rotation_error = Rotation.from_matrix(
                predicted[:3, :3] @ observed[:3, :3].T
            ).as_rotvec()
            parts.extend(rotation_error / math.radians(edge["angle_sigma"]))
        return np.asarray(parts)

    fitted = least_squares(residual, seed, loss="soft_l1", f_scale=1.0, max_nfev=2000)
    if not fitted.success:
        raise SystemExit(fitted.message)
    final = unpack(fitted.x, names)
    output: dict[str, Any] = {
        "schema_version": 1,
        "reference_frame": "velodyne_top_base_link",
        "method": "robust_camera_absolute_and_relative_pose_graph",
        "transforms": {},
        "quality": {},
        "pair_closure": {},
        "rejected_pair_edges": rejected,
    }
    for name in names:
        frame = f"{name}/camera_link"
        link = final[name] @ np.linalg.inv(CAMERA_OPTICAL_TO_LINK)
        output["transforms"][frame] = {
            "parent": "velodyne_top_base_link",
            **matrix_summary(link),
            "source": "camera_pose_graph_consensus",
        }
        change = transform_distance(final[name], absolute[name])
        output["quality"][frame] = {
            "absolute_input_position_p95_m": absolute_sigma[name][0],
            "absolute_input_angle_p95_deg": absolute_sigma[name][1],
            "graph_adjustment_m": change[0],
            "graph_adjustment_deg": change[1],
        }
    for edge in edges:
        predicted = np.linalg.inv(final[edge["left"]]) @ final[edge["right"]]
        before = np.linalg.inv(absolute[edge["left"]]) @ absolute[edge["right"]]
        output["pair_closure"][edge["key"]] = {
            "support": edge["support"],
            "before_position_m": transform_distance(before, edge["transform"])[0],
            "before_angle_deg": transform_distance(before, edge["transform"])[1],
            "after_position_m": transform_distance(predicted, edge["transform"])[0],
            "after_angle_deg": transform_distance(predicted, edge["transform"])[1],
            "edge_position_p95_m": edge["position_sigma"],
            "edge_angle_p95_deg": edge["angle_sigma"],
        }
    args.output.write_text(
        yaml.safe_dump(output, allow_unicode=True, sort_keys=False), encoding="utf-8"
    )
    print(f"nodes={len(names)} edges={len(edges)}")
    print(args.output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
