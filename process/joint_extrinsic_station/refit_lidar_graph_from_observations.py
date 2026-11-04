#!/usr/bin/env python3
"""Re-optimize a lidar graph from an audited dynamic-board observation CSV."""

from __future__ import annotations

import argparse
import csv
from pathlib import Path
from typing import Any

import numpy as np
import yaml

from calibrate_multilidar_dynamic_boards import (
    FRAMES,
    GraphEdge,
    LIDARS,
    LocalBoard,
    bootstrap_graph,
    edge_error,
    load_transforms,
    matrix_item,
    solve_graph,
    to_local,
)
from extract_camera_pair_constraints import transform_distance


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--observations-csv", type=Path, required=True)
    parser.add_argument("--initial-transforms", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    transforms = load_transforms(args.initial_transforms)
    grouped: dict[tuple[str, str], dict[str, LocalBoard]] = {}
    phases: dict[tuple[str, str], str] = {}
    with args.observations_csv.open(newline="", encoding="utf-8") as stream:
        for row in csv.DictReader(stream):
            sensor = row["sensor"]
            center = np.fromstring(row["center_top_seed"], sep=" ")
            normal = np.fromstring(row["normal_top_seed"], sep=" ")
            local_center, local_normal = to_local(transforms[sensor], center, normal)
            board = LocalBoard(
                row["capture_id"],
                row["camera"],
                sensor,
                local_center,
                local_normal,
                center,
                normal,
                int(row["points"]),
                float(row["rms_m"]),
                int(row["temporal_reference_consensus"]),
            )
            grouped.setdefault((row["capture_id"], row["camera"]), {})[sensor] = board
            phases[(row["capture_id"], row["camera"])] = row["phase"]
    deduplicated: dict[tuple[str, str, str], tuple[float, GraphEdge]] = {}
    for (_capture, _camera), boards in grouped.items():
        names = sorted(boards, key=LIDARS.index)
        for left_index, left_name in enumerate(names):
            for right_name in names[left_index + 1 :]:
                edge = GraphEdge(
                    boards[left_name].capture_id,
                    boards[left_name].camera,
                    boards[left_name],
                    boards[right_name],
                    phases[(_capture, _camera)],
                )
                error = edge_error(edge, transforms)
                if error[0] > 0.15 or error[1] > 8.0:
                    continue
                score = (edge.left.points + edge.right.points) / max(
                    edge.left.rms_m + edge.right.rms_m, 0.004
                )
                key = (edge.capture_id, left_name, right_name)
                if key not in deduplicated or score > deduplicated[key][0]:
                    deduplicated[key] = (score, edge)
    edges = [value[1] for value in deduplicated.values()]
    fitted, stability = bootstrap_graph(edges, transforms)
    _full, accepted, graph_quality = solve_graph(edges, transforms)
    result: dict[str, Any] = {
        "schema_version": 1,
        "reference_frame": "velodyne_top_base_link",
        "source_observations": str(args.observations_csv),
        "transforms": {},
        "quality": {},
        "graph_quality": graph_quality,
        "accepted_edge_count": len(accepted),
    }
    for sensor, frame in FRAMES.items():
        result["transforms"][frame] = matrix_item(
            fitted[sensor], "robust_refit_dynamic_board_graph_bootstrap_floor_z"
        )
        correction = transform_distance(fitted[sensor], transforms[sensor])
        result["quality"][frame] = {
            "bootstrap": stability["per_sensor"][sensor],
            "correction_from_initial_m": correction[0],
            "correction_from_initial_deg": correction[1],
        }
    args.output.write_text(
        yaml.safe_dump(result, allow_unicode=True, sort_keys=False), encoding="utf-8"
    )
    print(f"edges={len(edges)}, accepted={len(accepted)}")
    print(args.output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
