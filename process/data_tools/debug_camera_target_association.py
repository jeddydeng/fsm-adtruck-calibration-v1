#!/usr/bin/env python3
"""Explain camera AprilGrid-to-roof-target association for one capture."""

from __future__ import annotations

import argparse
import importlib.util
import json
from pathlib import Path
import sys

import cv2
import numpy as np
import yaml


def load_module(path: Path):
    spec = importlib.util.spec_from_file_location("joint_solver", path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("capture", type=Path)
    parser.add_argument("station", type=Path)
    args = parser.parse_args()
    solver = load_module(args.station / "solve_joint_extrinsics.py")
    config = yaml.safe_load((args.station / "config.yaml").read_text())
    initial = yaml.safe_load((args.station / "initial_extrinsics.yaml").read_text())
    absolute = solver.absolute_transforms(initial)
    top_from_sensor = np.linalg.inv(absolute["velodyne_top_base_link"])
    manifest = json.loads((args.capture / "manifest.json").read_text())
    references = solver.top_targets(manifest)
    detector = solver.make_detector(config["tag_board"]["dictionary"])
    camera_names = [item["name"] for item in config["usb_capture"]["cameras"]]
    camera_names += list(config["fisheye"]["sources"].values())
    camera_names += [config["traffic_camera"]["name"]]
    for name in camera_names:
        sensor = manifest["sensors"][name]
        image = cv2.imread(str(args.capture / sensor["file"]))
        intrinsic = args.capture / "intrinsics" / f"{name}.yaml"
        camera, distortion, model = solver.load_intrinsics(intrinsic)
        corners, ids = detector(cv2.cvtColor(image, cv2.COLOR_BGR2GRAY))
        tag_count = 0 if ids is None else len(ids)
        poses = (
            []
            if ids is None
            else solver.estimate_grid_poses(
                corners, ids.flatten(), config["tag_board"], camera, distortion, model
            )
        )
        initial_top_link = top_from_sensor @ absolute[f"{name}/camera_link"]
        initial_top_optical = initial_top_link @ solver.CAMERA_LINK_TO_OPTICAL
        rows = []
        for local_center, _normal, tag_ids in poses:
            rough = (initial_top_optical @ np.r_[local_center, 1.0])[:3]
            distances = {
                direction: float(
                    np.linalg.norm(rough - np.asarray(target["center_top_m"], dtype=float))
                )
                for direction, target in references.items()
            }
            nearest = min(distances, key=distances.get) if distances else None
            rows.append(
                {
                    "tag_ids": tag_ids,
                    "local_center_m": np.round(local_center, 3).tolist(),
                    "rough_top_center_m": np.round(rough, 3).tolist(),
                    "nearest": nearest,
                    "nearest_distance_m": round(distances[nearest], 3) if nearest else None,
                }
            )
        print(json.dumps({"camera": name, "tags": tag_count, "poses": rows}, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
