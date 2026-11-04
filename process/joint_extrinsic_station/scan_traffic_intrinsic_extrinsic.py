#!/usr/bin/env python3
"""Measure traffic-camera extrinsic sensitivity to intrinsic candidates."""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import Any

import numpy as np
import yaml

from calibrate_camera_plane_nuisance_bundle import bootstrap, load_observations
from calibrate_camera_rigid_board_frames import unique_poses
from extract_camera_pair_constraints import CAMERAS, matrix_summary, transform_distance
from solve_joint_extrinsics import (
    CAMERA_OPTICAL_TO_LINK,
    absolute_transforms,
    load_intrinsics,
)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--data-root", type=Path, required=True)
    parser.add_argument("--annotations", type=Path, required=True)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--initial", type=Path, required=True)
    parser.add_argument("--intrinsics", type=Path, required=True)
    parser.add_argument("--traffic-intrinsic", action="append", type=Path, required=True)
    parser.add_argument("--lidar-observations", type=Path, required=True)
    parser.add_argument("--lidar-seed", type=Path, required=True)
    parser.add_argument("--lidar-refined", type=Path, required=True)
    parser.add_argument("--ground-z", type=float, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()

    config = yaml.safe_load(args.config.read_text(encoding="utf-8"))
    initial_data = yaml.safe_load(args.initial.read_text(encoding="utf-8"))
    absolute = absolute_transforms(initial_data)
    top_from_kit = np.linalg.inv(absolute["velodyne_top_base_link"])
    initial_top_optical = {
        name: top_from_kit @ absolute[f"{name}/camera_link"] @ CAMERA_OPTICAL_TO_LINK
        for name in CAMERAS
    }
    base_intrinsics = {name: load_intrinsics(args.intrinsics / f"{name}.yaml") for name in CAMERAS}
    output: dict[str, Any] = {
        "schema_version": 1,
        "reference_frame": "velodyne_top_base_link",
        "method": "traffic_intrinsic_candidate_extrinsic_sensitivity",
        "warning": "A low extrinsic fold spread cannot repair a wrong intrinsic model.",
        "candidates": {},
    }
    for path in args.traffic_intrinsic:
        intrinsics = dict(base_intrinsics)
        intrinsics["traffic_light_front"] = load_intrinsics(path)
        values = load_observations(args, config, initial_top_optical, intrinsics)
        unique = unique_poses(values["traffic_light_front"])
        item: dict[str, Any] = {"unique_board_poses": len(unique)}
        try:
            optical, quality = bootstrap(
                unique, initial_top_optical["traffic_light_front"], args.ground_z
            )
            link = optical @ np.linalg.inv(CAMERA_OPTICAL_TO_LINK)
            item.update(
                {
                    "transform": {"parent": "velodyne_top_base_link", **matrix_summary(link)},
                    "difference_from_mechanical_seed_m": transform_distance(
                        optical, initial_top_optical["traffic_light_front"]
                    )[0],
                    "difference_from_mechanical_seed_deg": transform_distance(
                        optical, initial_top_optical["traffic_light_front"]
                    )[1],
                    "stability": quality,
                }
            )
        except RuntimeError as error:
            item["error"] = str(error)
        output["candidates"][path.name] = item
        print(
            path.name,
            item.get("stability", {}).get("achieved_tier", "rejected"),
            item.get("error", ""),
            flush=True,
        )
    args.output.write_text(
        yaml.safe_dump(output, allow_unicode=True, sort_keys=False), encoding="utf-8"
    )
    print(args.output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
