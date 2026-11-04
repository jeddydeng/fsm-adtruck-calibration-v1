#!/usr/bin/env python3
"""Recover and explicitly label 3--4 view camera pair constraints.

The main pair extractor requires five views for its five-fold bootstrap.  This
tool handles the useful four-view USB side-camera overlap with leave-one-view-
out fits, while keeping every result marked low-support rather than silently
promoting it to a fully validated edge.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

import numpy as np
from scipy.spatial.transform import Rotation
import yaml

from extract_camera_pair_constraints import (
    CAMERAS,
    eligible_ids,
    matrix_summary,
    observations_for_capture,
    pair_observations,
    stereo_solve,
    transform_distance,
)
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
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    config = yaml.safe_load(args.config.read_text(encoding="utf-8"))
    initial = yaml.safe_load(args.initial.read_text(encoding="utf-8"))
    absolute = absolute_transforms(initial)
    top_from_kit = np.linalg.inv(absolute["velodyne_top_base_link"])
    top_optical = {
        name: top_from_kit @ absolute[f"{name}/camera_link"] @ CAMERA_OPTICAL_TO_LINK
        for name in CAMERAS
    }
    intrinsics = {name: load_intrinsics(args.intrinsics / f"{name}.yaml") for name in CAMERAS}
    pairs = []
    for capture_id in sorted(eligible_ids(args.annotations)):
        directory = args.data_root / capture_id
        manifest = json.loads((directory / "manifest.json").read_text(encoding="utf-8"))
        poses = observations_for_capture(
            directory, manifest, config["tag_board"], intrinsics, top_optical
        )
        pairs.extend(pair_observations(poses, top_optical))
    grouped = {}
    for item in pairs:
        grouped.setdefault((item.left, item.right), []).append(item)
    output: dict[str, Any] = {"schema_version": 1, "edges": {}, "rejected": {}}
    for (left, right), values in sorted(grouped.items()):
        if not 3 <= len(values) <= 4:
            continue
        key = f"{left}<-{right}"
        try:
            full, full_rms = stereo_solve(values)
            candidates = []
            if len(values) == 4:
                for omitted in range(4):
                    candidate, _rms = stereo_solve(
                        [item for index, item in enumerate(values) if index != omitted]
                    )
                    candidates.append(candidate)
            else:
                candidates.append(full)
            consensus = np.eye(4)
            consensus[:3, 3] = np.median([item[:3, 3] for item in candidates], axis=0)
            consensus[:3, :3] = (
                Rotation.from_matrix([item[:3, :3] for item in candidates]).mean().as_matrix()
            )
            errors = [transform_distance(consensus, item) for item in candidates]
            position = float(np.percentile([item[0] for item in errors], 95))
            angle = float(np.percentile([item[1] for item in errors], 95))
            output["edges"][key] = {
                "status": "provisional_low_support",
                "view_count": len(values),
                "leave_one_out_count": len(candidates),
                "transform": matrix_summary(consensus),
                "normalized_reprojection_rms": full_rms,
                "position_p95_m": position,
                "angle_p95_deg": angle,
                "achieved_tier": (
                    "fallback_1cm_1deg" if position <= 0.01 and angle <= 1.0 else "coarse_only"
                ),
                "candidate_transforms": [matrix_summary(item) for item in candidates],
            }
        except RuntimeError as error:
            output["rejected"][key] = {"view_count": len(values), "error": str(error)}
    args.output.write_text(
        yaml.safe_dump(output, allow_unicode=True, sort_keys=False), encoding="utf-8"
    )
    print(args.output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
