#!/usr/bin/env python3
"""Create a reversible curated index for the 2026-09-02 white-car dataset."""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
from pathlib import Path
from typing import Any

import numpy as np


PHASES = (
    ("front_sweep_1", "front", "20260902_200659_745", "20260902_205205_066"),
    ("left_sweep", "left", "20260902_205553_287", "20260902_210925_071"),
    ("front_to_right", "front", "20260902_211035_366", "20260902_211209_118"),
    ("right_sweep", "right", "20260902_211230_428", "20260902_212716_303"),
    ("front_center", "front", "20260902_212804_986", "20260902_213000_092"),
    ("rear_sweep", "rear", "20260902_213009_531", "20260902_213841_026"),
)

RELEVANT_CAMERAS = {
    "front": ("video_front_mid", "fisheye_front"),
    "left": ("video_front_left", "video_rear_left", "fisheye_left"),
    "right": ("video_front_right", "video_rear_right", "fisheye_right"),
    "rear": ("video_rear_mid", "video_rear_left", "video_rear_right", "fisheye_rear"),
}


def angle_deg(left: np.ndarray, right: np.ndarray) -> float:
    left = left / max(float(np.linalg.norm(left)), 1e-12)
    right = right / max(float(np.linalg.norm(right)), 1e-12)
    return math.degrees(math.acos(float(np.clip(abs(left @ right), -1.0, 1.0))))


def phase_for(capture_id: str) -> tuple[str, str]:
    for phase, board, start, end in PHASES:
        if start <= capture_id <= end:
            return phase, board
    raise ValueError(f"capture outside declared chronology: {capture_id}")


def top_target(manifest: dict[str, Any], direction: str) -> dict[str, Any] | None:
    values = (
        manifest.get("sensors", {}).get("lidar_top", {}).get("detection", {}).get("targets", [])
    )
    candidates = [value for value in values if value.get("direction") == direction]
    return (
        max(
            candidates,
            key=lambda item: (float(item.get("confidence", 0)), int(item.get("points", 0))),
        )
        if candidates
        else None
    )


def plausible(direction: str, center: np.ndarray) -> bool:
    x, y, z = center
    if direction == "front":
        return x > 1.0 and abs(y) < 5.0 and -1.65 < z < -0.35
    if direction == "left":
        return y > 1.0 and abs(x) < 7.0 and -1.65 < z < -0.35
    if direction == "right":
        return y < -1.0 and abs(x) < 7.0 and -1.65 < z < -0.35
    return x < -1.0 and abs(y) < 5.0 and -1.25 < z < 0.75


def quality(manifest: dict[str, Any], direction: str, target: dict[str, Any] | None) -> float:
    sensors = manifest.get("sensors", {})
    score = 0.0
    if target is not None:
        score += 25.0 * float(target.get("confidence", 0))
        score += min(25.0, float(target.get("points", 0)) / 4.0)
    for camera in RELEVANT_CAMERAS[direction]:
        tags = int(sensors.get(camera, {}).get("detection", {}).get("tag_count", 0))
        score += min(tags, 20)
    spread = manifest.get("software_receive_spread_ms")
    if isinstance(spread, (int, float)):
        score -= min(10.0, float(spread) / 100.0)
    return round(score, 3)


def make_symlink(link: Path, target: Path) -> None:
    link.parent.mkdir(parents=True, exist_ok=True)
    if link.is_symlink():
        if link.resolve() == target.resolve():
            return
        link.unlink()
    elif link.exists():
        raise FileExistsError(f"refusing to replace non-symlink: {link}")
    link.symlink_to(os.path.relpath(target, link.parent), target_is_directory=True)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("raw_root", type=Path)
    parser.add_argument("output_root", type=Path)
    parser.add_argument("--rapid-click-sec", type=float, default=4.0)
    parser.add_argument("--pose-radius-m", type=float, default=0.22)
    parser.add_argument("--normal-radius-deg", type=float, default=5.0)
    args = parser.parse_args()

    captures = []
    for path in sorted(args.raw_root.glob("*/manifest.json")):
        manifest = json.loads(path.read_text(encoding="utf-8"))
        captures.append((path.parent, manifest))
    if not captures:
        raise SystemExit("no captures found")
    if (
        captures[0][1].get("capture_id") != PHASES[0][2]
        or captures[-1][1].get("capture_id") != PHASES[-1][3]
    ):
        raise SystemExit("dataset endpoints do not match the declared acquisition")

    rows: list[dict[str, Any]] = []
    previous_ns = None
    clusters: dict[str, list[dict[str, Any]]] = {phase[0]: [] for phase in PHASES}
    for directory, manifest in captures:
        capture_id = str(manifest.get("capture_id", directory.name))
        phase, moved_board = phase_for(capture_id)
        capture_ns = int(manifest.get("capture_unix_ns", 0))
        gap = None if previous_ns is None else (capture_ns - previous_ns) / 1e9
        previous_ns = capture_ns
        rapid = gap is not None and gap < args.rapid_click_sec and rows[-1]["phase"] == phase
        target = top_target(manifest, moved_board)
        center = None if target is None else np.asarray(target["center_top_m"], float)
        normal = None if target is None else np.asarray(target["normal_top"], float)
        target_plausible = center is not None and plausible(moved_board, center)
        score = quality(manifest, moved_board, target)
        cluster_id = None
        if target_plausible and not rapid:
            best = None
            best_distance = float("inf")
            for index, cluster in enumerate(clusters[phase]):
                distance = float(np.linalg.norm(center - cluster["center"]))
                normal_angle = angle_deg(normal, cluster["normal"])
                if (
                    distance <= args.pose_radius_m
                    and normal_angle <= args.normal_radius_deg
                    and distance < best_distance
                ):
                    best, best_distance = index, distance
            if best is None:
                clusters[phase].append(
                    {
                        "center": center.copy(),
                        "normal": normal.copy(),
                        "members": [capture_id],
                        "best": capture_id,
                        "best_score": score,
                    }
                )
                cluster_id = len(clusters[phase]) - 1
            else:
                cluster = clusters[phase][best]
                cluster["members"].append(capture_id)
                count = len(cluster["members"])
                cluster["center"] += (center - cluster["center"]) / count
                cluster["normal"] += (normal - cluster["normal"]) / count
                if score > cluster["best_score"]:
                    cluster["best"], cluster["best_score"] = capture_id, score
                cluster_id = best
        sensors = manifest.get("sensors", {})
        row = {
            "capture_id": capture_id,
            "phase": phase,
            "moved_board": moved_board,
            "gap_s": None if gap is None else round(gap, 3),
            "rapid_repeat_click": rapid,
            "include_nonduplicate": not rapid,
            "roof_target_available": target is not None,
            "roof_target_plausible": bool(target_plausible),
            "pose_cluster_id": cluster_id,
            "quality_score": score,
            "traffic_light_front_status": "excluded_frozen_entire_session",
            "camera_tag_counts": {
                name: int(item.get("detection", {}).get("tag_count", 0))
                for name, item in sensors.items()
                if "camera" in str(item.get("kind", ""))
            },
        }
        rows.append(row)

    # These clusters originate from the online roof-lidar detector.  In this
    # garage that detector sometimes labelled a repeated wall patch as a
    # 900 mm target.  Keep the representatives for visual/debug review only;
    # downstream calibration must re-fit the raw PCD and must not use them as
    # geometric truth.
    selected = {
        cluster["best"] for phase_clusters in clusters.values() for cluster in phase_clusters
    }
    for row in rows:
        row["advisory_roof_pose_candidate"] = row["capture_id"] in selected

    nonduplicate_dir = args.output_root / "all_nonduplicate"
    advisory_dir = args.output_root / "advisory_roof_pose_candidates"
    for row, (directory, _manifest) in zip(rows, captures):
        if row["include_nonduplicate"]:
            make_symlink(nonduplicate_dir / directory.name, directory)
        if row["advisory_roof_pose_candidate"]:
            make_symlink(advisory_dir / directory.name, directory)

    args.output_root.mkdir(parents=True, exist_ok=True)
    csv_rows = []
    for row in rows:
        flat = {key: value for key, value in row.items() if key != "camera_tag_counts"}
        flat.update({f"{name}_tags": value for name, value in row["camera_tag_counts"].items()})
        csv_rows.append(flat)
    with (args.output_root / "capture_annotations.csv").open(
        "w", newline="", encoding="utf-8"
    ) as stream:
        writer = csv.DictWriter(stream, fieldnames=list(csv_rows[0]))
        writer.writeheader()
        writer.writerows(csv_rows)

    summary = {
        "schema_version": 1,
        "source": str(args.raw_root),
        "raw_capture_count": len(rows),
        "nonduplicate_capture_count": sum(row["include_nonduplicate"] for row in rows),
        "rapid_repeat_click_count": sum(row["rapid_repeat_click"] for row in rows),
        "advisory_roof_pose_candidate_count": len(selected),
        "traffic_light_front": {
            "included": False,
            "reason": "Tag corners remained unchanged for the entire session; user confirmed frozen stream.",
        },
        "phase_definitions": [
            {
                "phase": phase,
                "moved_board": board,
                "start": start,
                "end": end,
                "raw_count": sum(row["phase"] == phase for row in rows),
                "nonduplicate_count": sum(
                    row["phase"] == phase and row["include_nonduplicate"] for row in rows
                ),
                "advisory_roof_pose_candidate_count": sum(
                    row["phase"] == phase and row["advisory_roof_pose_candidate"] for row in rows
                ),
            }
            for phase, board, start, end in PHASES
        ],
        "policy": {
            "raw_data_deleted": False,
            "rapid_repeat_definition": f"same phase and trigger gap < {args.rapid_click_sec:.1f}s",
            "pose_cluster_radius_m": args.pose_radius_m,
            "pose_cluster_normal_radius_deg": args.normal_radius_deg,
            "notes": [
                "Raw captures remain immutable; curated directories contain relative symlinks.",
                "Roof-lidar pose candidates are advisory only because garage walls caused false positives.",
                "No advisory roof pose is used as calibration truth until it is independently re-fitted from raw PCD.",
                "Board phase labels follow the operator-reported acquisition order.",
            ],
        },
    }
    (args.output_root / "curation_summary.json").write_text(
        json.dumps(summary, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
