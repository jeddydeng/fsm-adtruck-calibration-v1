#!/usr/bin/env python3
"""Cross-check front-right camera pose by two independent observation paths.

Path A is the camera-to-lidar grounded-board solver.  Path B starts at the
independently calibrated rear-right camera and traverses a simultaneous
front-right/rear-right board observation edge.  Agreement is assessed before
the two paths are fused, so pose-graph closure cannot masquerade as accuracy.
"""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import Any

import numpy as np
from scipy.spatial.transform import Rotation
import yaml

from extract_camera_pair_constraints import matrix_summary, transform_distance
from refine_lidar_environment import matrix_from_item
from solve_joint_extrinsics import CAMERA_OPTICAL_TO_LINK


FRAME = "video_front_right/camera_link"
REAR_FRAME = "video_rear_right/camera_link"
EDGE = "video_front_right<-video_rear_right"


def consensus(matrices: list[np.ndarray]) -> np.ndarray:
    result = np.eye(4)
    result[:3, 3] = np.median([matrix[:3, 3] for matrix in matrices], axis=0)
    result[:3, :3] = (
        Rotation.from_matrix([matrix[:3, :3] for matrix in matrices]).mean().as_matrix()
    )
    return result


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--direct", action="append", type=Path, required=True)
    parser.add_argument("--rear-absolute", type=Path, required=True)
    parser.add_argument("--pairs", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()

    direct_inputs: list[tuple[str, np.ndarray]] = []
    for path in args.direct:
        data = yaml.safe_load(path.read_text(encoding="utf-8"))
        direct_inputs.append((path.name, matrix_from_item(data["transforms"][FRAME])))
    if len(direct_inputs) < 2:
        raise SystemExit("at least two direct solver variants are required")
    direct = consensus([matrix for _name, matrix in direct_inputs])
    direct_errors = [transform_distance(direct, matrix) for _name, matrix in direct_inputs]

    rear_data = yaml.safe_load(args.rear_absolute.read_text(encoding="utf-8"))
    rear_link = matrix_from_item(rear_data["transforms"][REAR_FRAME])
    top_from_rear_optical = rear_link @ CAMERA_OPTICAL_TO_LINK
    pair_data = yaml.safe_load(args.pairs.read_text(encoding="utf-8"))
    pair_item = pair_data["accepted_edges"][EDGE]
    front_from_rear_optical = matrix_from_item(pair_item["transform"])
    top_from_front_optical = top_from_rear_optical @ np.linalg.inv(front_from_rear_optical)
    derived = top_from_front_optical @ np.linalg.inv(CAMERA_OPTICAL_TO_LINK)

    cross_position, cross_angle = transform_distance(direct, derived)
    direct_position = float(np.percentile([value[0] for value in direct_errors], 95))
    direct_angle = float(np.percentile([value[1] for value in direct_errors], 95))
    edge_position = float(pair_item["quality"]["position_p95_m"])
    edge_angle = float(pair_item["quality"]["angle_p95_deg"])

    # Fuse in SE(3) after the independent comparison.  Equal weighting is
    # intentional: both paths have similar ~8 mm translation uncertainty.
    fused = np.eye(4)
    fused[:3, 3] = (direct[:3, 3] + derived[:3, 3]) * 0.5
    fused[:3, :3] = Rotation.from_matrix([direct[:3, :3], derived[:3, :3]]).mean().as_matrix()
    evidence_position = max(cross_position, direct_position, edge_position)
    evidence_angle = max(cross_angle, direct_angle, edge_angle)
    tier = (
        "target_1mm_0.1deg"
        if evidence_position <= 0.001 and evidence_angle <= 0.1
        else "fallback_1cm_1deg"
        if evidence_position <= 0.01 and evidence_angle <= 1.0
        else "coarse_only"
    )

    output: dict[str, Any] = {
        "schema_version": 1,
        "reference_frame": "velodyne_top_base_link",
        "method": "independent_direct_and_stereo_path_cross_validation",
        "transform": {
            "frame": FRAME,
            "parent": "velodyne_top_base_link",
            **matrix_summary(fused),
            "source": "equal_weight_direct_lidar_and_camera_pair_fusion",
        },
        "independent_paths": {
            "direct_camera_lidar_consensus": {
                **matrix_summary(direct),
                "input_variants": [name for name, _matrix in direct_inputs],
                "variant_position_p95_m": direct_position,
                "variant_angle_p95_deg": direct_angle,
            },
            "rear_right_plus_simultaneous_camera_pair": {
                **matrix_summary(derived),
                "rear_absolute_source": args.rear_absolute.name,
                "pair_edge_source": args.pairs.name,
                "pair_edge_position_p95_m": edge_position,
                "pair_edge_angle_p95_deg": edge_angle,
                "pair_view_count": int(pair_item["quality"]["raw_view_count"]),
            },
        },
        "independent_cross_check": {
            "position_difference_m": cross_position,
            "angle_difference_deg": cross_angle,
            "evidence_position_p95_m": evidence_position,
            "evidence_angle_p95_deg": evidence_angle,
            "achieved_tier": tier,
            "note": "Tier uses pre-fusion path disagreement, not post-fit closure.",
        },
    }
    args.output.write_text(
        yaml.safe_dump(output, allow_unicode=True, sort_keys=False), encoding="utf-8"
    )
    print(
        f"front-right independent difference: {cross_position * 1000:.3f} mm, "
        f"{cross_angle:.4f} deg; tier={tier}"
    )
    print(args.output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
