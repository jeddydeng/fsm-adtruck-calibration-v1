#!/usr/bin/env python3
"""Audit a multi-lidar board graph by phase and leave-one-phase-out fits.

This is deliberately separate from the estimator: it reconstructs all graph
edges from the immutable observation audit and asks whether acquisition order,
one moved board, or one lidar pair dominates the reported transform.
"""

from __future__ import annotations

import argparse
from collections import defaultdict
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
    edge_error,
    load_transforms,
    solve_graph,
    to_local,
)
from extract_camera_pair_constraints import transform_distance


def summarize(values: list[tuple[float, float]]) -> dict[str, Any]:
    if not values:
        return {"count": 0}
    array = np.asarray(values)
    return {
        "count": len(values),
        "plane_median_m": float(np.median(array[:, 0])),
        "plane_p95_m": float(np.percentile(array[:, 0], 95)),
        "angle_median_deg": float(np.median(array[:, 1])),
        "angle_p95_deg": float(np.percentile(array[:, 1], 95)),
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--observations-csv", type=Path, required=True)
    parser.add_argument("--initial-transforms", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()

    initial = load_transforms(args.initial_transforms)
    grouped: dict[tuple[str, str], dict[str, LocalBoard]] = {}
    phases: dict[tuple[str, str], str] = {}
    with args.observations_csv.open(newline="", encoding="utf-8") as stream:
        for row in csv.DictReader(stream):
            sensor = row["sensor"]
            center = np.fromstring(row["center_top_seed"], sep=" ")
            normal = np.fromstring(row["normal_top_seed"], sep=" ")
            center_local, normal_local = to_local(initial[sensor], center, normal)
            key = (row["capture_id"], row["camera"])
            grouped.setdefault(key, {})[sensor] = LocalBoard(
                row["capture_id"],
                row["camera"],
                sensor,
                center_local,
                normal_local,
                center,
                normal,
                int(row["points"]),
                float(row["rms_m"]),
                int(row["temporal_reference_consensus"]),
            )
            phases[key] = row["phase"]

    best: dict[tuple[str, str, str], tuple[float, GraphEdge, str]] = {}
    for key, boards in grouped.items():
        names = sorted(boards, key=LIDARS.index)
        for left_index, left_name in enumerate(names):
            for right_name in names[left_index + 1 :]:
                edge = GraphEdge(key[0], key[1], boards[left_name], boards[right_name], phases[key])
                position, angle = edge_error(edge, initial)
                if position > 0.15 or angle > 8.0:
                    continue
                score = (edge.left.points + edge.right.points) / max(
                    edge.left.rms_m + edge.right.rms_m, 0.004
                )
                edge_key = (edge.capture_id, left_name, right_name)
                if edge_key not in best or score > best[edge_key][0]:
                    best[edge_key] = (score, edge, phases[key])
    items = [(edge, phase) for _score, edge, phase in best.values()]
    edges = [edge for edge, _phase in items]
    full, accepted, quality = solve_graph(edges, initial)
    accepted_ids = {id(edge) for edge in accepted}

    by_phase: dict[str, list[tuple[float, float]]] = defaultdict(list)
    by_pair: dict[str, list[tuple[float, float]]] = defaultdict(list)
    by_phase_pair: dict[str, list[tuple[float, float]]] = defaultdict(list)
    accepted_phase_counts: dict[str, int] = defaultdict(int)
    for edge, phase in items:
        error = edge_error(edge, full)
        pair = f"{edge.left.sensor}__{edge.right.sensor}"
        by_phase[phase].append(error)
        by_pair[pair].append(error)
        by_phase_pair[f"{phase}:{pair}"].append(error)
        if id(edge) in accepted_ids:
            accepted_phase_counts[phase] += 1

    leave_one_phase_out: dict[str, Any] = {}
    all_phases = sorted({phase for _edge, phase in items})
    for omitted in all_phases:
        training = [edge for edge, phase in items if phase != omitted]
        if len(training) < 20:
            continue
        candidate, _accepted, candidate_quality = solve_graph(training, initial)
        per_sensor = {}
        for sensor, frame in FRAMES.items():
            position, angle = transform_distance(full[sensor], candidate[sensor])
            per_sensor[frame] = {
                "translation_delta_m": position,
                "rotation_delta_deg": angle,
            }
        holdout = [(edge_error(edge, candidate), edge) for edge, phase in items if phase == omitted]
        leave_one_phase_out[omitted] = {
            "training_edges": len(training),
            "holdout_edges": len(holdout),
            "holdout_residuals": summarize([value for value, _edge in holdout]),
            "transform_delta_from_full": per_sensor,
            "training_quality": candidate_quality,
        }

    output = {
        "schema_version": 1,
        "reference_frame": "velodyne_top_base_link",
        "raw_edge_count": len(edges),
        "accepted_edge_count": len(accepted),
        "full_fit_quality": quality,
        "residuals_by_phase_all_edges": {
            key: {**summarize(value), "accepted_count": accepted_phase_counts[key]}
            for key, value in sorted(by_phase.items())
        },
        "residuals_by_lidar_pair_all_edges": {
            key: summarize(value) for key, value in sorted(by_pair.items())
        },
        "residuals_by_phase_and_pair_all_edges": {
            key: summarize(value) for key, value in sorted(by_phase_pair.items())
        },
        "leave_one_phase_out": leave_one_phase_out,
        "interpretation": {
            "translation_gate_m": 0.01,
            "rotation_gate_deg": 1.0,
            "note": "Leave-one-phase-out deltas above these gates indicate phase-dependent bias; all residual tables include rejected edges so failures remain visible.",
        },
    }
    args.output.write_text(
        yaml.safe_dump(output, allow_unicode=True, sort_keys=False), encoding="utf-8"
    )
    print(f"edges={len(edges)} accepted={len(accepted)} phases={len(all_phases)}")
    print(args.output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
