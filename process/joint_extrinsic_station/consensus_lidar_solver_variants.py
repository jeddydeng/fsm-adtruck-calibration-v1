#!/usr/bin/env python3
"""Select the >60% stable lidar pose cluster across solver variants."""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import Any

import numpy as np
from scipy.spatial.transform import Rotation
import yaml

from extract_camera_pair_constraints import matrix_summary, transform_distance
from refine_lidar_environment import matrix_from_item


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--variant", action="append", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if len(args.variant) < 3:
        raise SystemExit("at least three variants are required")
    inputs = [
        (path.name, yaml.safe_load(path.read_text(encoding="utf-8"))) for path in args.variant
    ]
    frames = sorted(set.intersection(*[set(data.get("transforms", {})) for _name, data in inputs]))
    output: dict[str, Any] = {
        "schema_version": 1,
        "reference_frame": "velodyne_top_base_link",
        "method": "dominant_solver_variant_cluster",
        "policy": {
            "minimum_dominant_fraction": 0.60,
            "cluster_position_radius_m": 0.10,
            "cluster_angle_radius_deg": 1.0,
            "warning": "Variant sensitivity is repeatability evidence, not surveyed absolute accuracy.",
        },
        "input_variants": [name for name, _data in inputs],
        "transforms": {},
        "quality": {},
        "rejected": {},
    }
    for frame in frames:
        matrices = [matrix_from_item(data["transforms"][frame]) for _name, data in inputs]
        neighbourhoods = []
        for candidate in matrices:
            neighbourhoods.append(
                [
                    index
                    for index, other in enumerate(matrices)
                    if transform_distance(candidate, other)[0] <= 0.10
                    and transform_distance(candidate, other)[1] <= 1.0
                ]
            )
        members = max(neighbourhoods, key=len)
        fraction = len(members) / len(matrices)
        if fraction < 0.60:
            output["rejected"][frame] = {
                "dominant_count": len(members),
                "dominant_fraction": fraction,
            }
            continue
        selected = [matrices[index] for index in members]
        consensus = np.eye(4)
        consensus[:3, 3] = np.median([matrix[:3, 3] for matrix in selected], axis=0)
        consensus[:3, :3] = (
            Rotation.from_matrix([matrix[:3, :3] for matrix in selected]).mean().as_matrix()
        )
        residuals = [transform_distance(consensus, matrix) for matrix in selected]
        position = float(np.percentile([item[0] for item in residuals], 95))
        angle = float(np.percentile([item[1] for item in residuals], 95))
        tier = (
            "target_1mm_0.1deg"
            if position <= 0.001 and angle <= 0.1
            else "fallback_1cm_1deg"
            if position <= 0.01 and angle <= 1.0
            else "coarse_only"
        )
        output["transforms"][frame] = {
            "parent": "velodyne_top_base_link",
            **matrix_summary(consensus),
            "source": "dominant_solver_variant_consensus",
        }
        output["quality"][frame] = {
            "dominant_count": len(members),
            "dominant_fraction": fraction,
            "member_variants": [inputs[index][0] for index in members],
            "excluded_variants": [
                inputs[index][0] for index in range(len(inputs)) if index not in members
            ],
            "position_p95_m": position,
            "angle_p95_deg": angle,
            "achieved_tier": tier,
            "member_deltas": [
                {
                    "variant": inputs[index][0],
                    "position_m": transform_distance(consensus, matrices[index])[0],
                    "angle_deg": transform_distance(consensus, matrices[index])[1],
                }
                for index in members
            ],
        }
    args.output.write_text(
        yaml.safe_dump(output, allow_unicode=True, sort_keys=False), encoding="utf-8"
    )
    for frame, quality in output["quality"].items():
        print(
            frame,
            quality["achieved_tier"],
            f"{quality['position_p95_m'] * 1000:.3f} mm",
            f"{quality['angle_p95_deg']:.4f} deg",
            f"cluster={quality['dominant_count']}/{len(inputs)}",
        )
    print(args.output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
