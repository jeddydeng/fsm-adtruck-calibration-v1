#!/usr/bin/env python3
"""Resolve the user's rough pose descriptions into matrices and quaternions."""

from __future__ import annotations

import argparse
import os
from pathlib import Path
from typing import Any
import warnings

import numpy as np
from scipy.spatial.transform import Rotation
import yaml

from solve_joint_extrinsics import CAMERA_OPTICAL_TO_LINK, absolute_transforms


def pose(matrix: np.ndarray) -> dict[str, Any]:
    with warnings.catch_warnings():
        # A 90-degree pitch has multiple equivalent RPY representations.  The
        # quaternion/matrix remain unique and are the authoritative outputs.
        warnings.simplefilter("ignore", UserWarning)
        rpy = Rotation.from_matrix(matrix[:3, :3]).as_euler("xyz", degrees=True)
    return {
        "xyz_m": [round(float(value), 8) for value in matrix[:3, 3]],
        "rpy_deg": [round(float(value), 7) for value in rpy],
        "quaternion_xyzw": [
            round(float(value), 9) for value in Rotation.from_matrix(matrix[:3, :3]).as_quat()
        ],
        "matrix_row_major": [[round(float(value), 9) for value in row] for row in matrix],
    }


def main() -> int:
    base = Path(__file__).resolve().parent
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", type=Path, default=base / "initial_extrinsics.yaml")
    parser.add_argument("--output", type=Path, default=base / "calculated_initial_transforms.yaml")
    args = parser.parse_args()
    source = yaml.safe_load(args.input.read_text(encoding="utf-8"))
    absolute = absolute_transforms(source)
    sensor_from_top = absolute["velodyne_top_base_link"]
    top_from_sensor = np.linalg.inv(sensor_from_top)
    result: dict[str, Any] = {
        "schema_version": 1,
        "status": "rough_initial_values_only_not_final_calibration",
        "coordinate_convention": source["coordinate_convention"],
        "reference_frames": {
            "absolute": "sensor_kit_base_link",
            "relative": "velodyne_top_base_link",
        },
        "transforms": {},
    }
    for name, matrix in absolute.items():
        if name == "sensor_kit_base_link":
            continue
        item = {
            "relative_to_sensor_kit": pose(matrix),
            "relative_to_roof_lidar": pose(top_from_sensor @ matrix),
            "source": source["transforms"][name]["source"],
        }
        if name.endswith("/camera_link"):
            optical = matrix @ CAMERA_OPTICAL_TO_LINK
            item["optical_frame_relative_to_sensor_kit"] = pose(optical)
            item["optical_frame_relative_to_roof_lidar"] = pose(top_from_sensor @ optical)
        result["transforms"][name] = item
    payload = yaml.safe_dump(result, allow_unicode=True, sort_keys=False)
    temporary = args.output.with_name(f".{args.output.name}.{os.getpid()}.tmp")
    temporary.write_text(payload, encoding="utf-8")
    os.replace(temporary, args.output)
    print(f"calculated {len(result['transforms'])} rough transforms -> {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
