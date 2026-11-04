#!/usr/bin/env python3
"""Measure camera-pose sensitivity across accepted nuisance-prior variants."""

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
    parser.add_argument("--variant", type=Path, action="append", required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    inputs = [
        (path.name, yaml.safe_load(path.read_text(encoding="utf-8"))) for path in args.variant
    ]
    frames = sorted(
        set.intersection(*[set(value.get("transforms", {})) for _name, value in inputs])
    )
    output: dict[str, Any] = {
        "schema_version": 1,
        "method": "solver_hyperparameter_consensus",
        "warning": "This measures estimator sensitivity, not independent absolute accuracy.",
        "input_count": len(inputs),
        "input_variants": [name for name, _value in inputs],
        "transforms": {},
        "quality": {},
    }
    for frame in frames:
        matrices = [matrix_from_item(value["transforms"][frame]) for _name, value in inputs]
        consensus = np.eye(4)
        consensus[:3, 3] = np.median([item[:3, 3] for item in matrices], axis=0)
        consensus[:3, :3] = (
            Rotation.from_matrix([item[:3, :3] for item in matrices]).mean().as_matrix()
        )
        errors = [transform_distance(consensus, item) for item in matrices]
        position = float(np.percentile([item[0] for item in errors], 95))
        angle = float(np.percentile([item[1] for item in errors], 95))
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
            "source": "nuisance_variant_consensus",
        }
        output["quality"][frame] = {
            "dominant_count": len(matrices),
            "dominant_fraction": 1.0,
            "position_p95_m": position,
            "angle_p95_deg": angle,
            "achieved_tier": tier,
            "variant_deltas": [
                {
                    "variant": name,
                    "position_m": transform_distance(consensus, matrix)[0],
                    "angle_deg": transform_distance(consensus, matrix)[1],
                }
                for (name, _value), matrix in zip(inputs, matrices)
            ],
        }
    args.output.write_text(
        yaml.safe_dump(output, allow_unicode=True, sort_keys=False), encoding="utf-8"
    )
    print(args.output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
