#!/usr/bin/env python3
"""Calibrate one camera from simultaneous views with an anchored camera."""

from __future__ import annotations

import argparse
import csv
import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
import yaml

from calibrate_camera_dynamic_boards import angle_deg
from extract_camera_pair_constraints import (
    CAMERAS,
    BoardPose,
    matrix_summary,
    observations_for_capture,
    transform_distance,
)
from refine_lidar_environment import matrix_from_item
from solve_joint_extrinsics import (
    CAMERA_OPTICAL_TO_LINK,
    absolute_transforms,
    load_intrinsics,
)


@dataclass(frozen=True)
class Observation:
    capture_id: str
    object_top: np.ndarray
    normalized_target: np.ndarray


def fit(
    values: list[Observation], initial: np.ndarray, minimum: int = 5
) -> tuple[np.ndarray, list[Observation], dict[str, Any]]:
    if len(values) < minimum:
        raise RuntimeError(f"need >= {minimum} simultaneous views, got {len(values)}")
    seed = np.r_[Rotation.from_matrix(initial[:3, :3]).as_rotvec(), initial[:3, 3]]

    def optimize(selected: list[Observation], start: np.ndarray) -> np.ndarray:
        def residual(parameters: np.ndarray) -> np.ndarray:
            top_from_target = np.eye(4)
            top_from_target[:3, :3] = Rotation.from_rotvec(parameters[:3]).as_matrix()
            top_from_target[:3, 3] = parameters[3:]
            target_from_top = np.linalg.inv(top_from_target)
            parts: list[float] = []
            for item in selected:
                points = item.object_top @ target_from_top[:3, :3].T + target_from_top[:3, 3]
                if np.any(points[:, 2] <= 0.1):
                    parts.extend(np.full(points.shape[0] * 2, 100.0))
                    continue
                projected = points[:, :2] / points[:, 2:3]
                parts.extend(((projected - item.normalized_target) / 0.0025).ravel())
            rotation_delta = (
                Rotation.from_rotvec(parameters[:3]) * Rotation.from_rotvec(seed[:3]).inv()
            )
            parts.extend(rotation_delta.as_rotvec() / math.radians(8.0))
            parts.extend((parameters[3:] - seed[3:]) / 0.20)
            return np.asarray(parts)

        lower = np.r_[np.full(3, -np.inf), initial[:3, 3] - 0.40]
        upper = np.r_[np.full(3, np.inf), initial[:3, 3] + 0.40]
        result = least_squares(
            residual,
            np.clip(start, lower, upper),
            bounds=(lower, upper),
            loss="soft_l1",
            f_scale=1.0,
            max_nfev=2400,
        )
        if not result.success:
            raise RuntimeError(result.message)
        if np.any(np.minimum(result.x[3:] - lower[3:], upper[3:] - result.x[3:]) < 0.002):
            raise RuntimeError("translation solution reached the search boundary")
        return result.x

    active = list(values)
    current = seed.copy()
    history = []
    for iteration in range(6):
        current = optimize(active, current)
        top_from_target = np.eye(4)
        top_from_target[:3, :3] = Rotation.from_rotvec(current[:3]).as_matrix()
        top_from_target[:3, 3] = current[3:]
        target_from_top = np.linalg.inv(top_from_target)
        errors = []
        for item in active:
            points = item.object_top @ target_from_top[:3, :3].T + target_from_top[:3, 3]
            projected = points[:, :2] / points[:, 2:3]
            errors.append(float(np.sqrt(np.mean(np.square(projected - item.normalized_target)))))
        array = np.asarray(errors)
        median = float(np.median(array))
        mad = float(np.median(np.abs(array - median)))
        cut = min(0.010, max(0.0035, median + 4.0 * mad))
        filtered = [item for item, error in zip(active, errors) if error <= cut]
        history.append(
            {
                "iteration": iteration + 1,
                "input": len(active),
                "output": len(filtered),
                "normalized_rms_cut": cut,
            }
        )
        if len(filtered) < minimum:
            raise RuntimeError(f"only {len(filtered)} views after robust gate")
        if len(filtered) == len(active):
            break
        active = filtered
    current = optimize(active, current)
    result = np.eye(4)
    result[:3, :3] = Rotation.from_rotvec(current[:3]).as_matrix()
    result[:3, 3] = current[3:]
    return (
        result,
        active,
        {
            "raw_views": len(values),
            "accepted_views": len(active),
            "discarded_views": len(values) - len(active),
            "rejection_history": history,
        },
    )


def bootstrap(values: list[Observation], initial: np.ndarray) -> tuple[np.ndarray, dict[str, Any]]:
    full, accepted, quality = fit(values, initial)
    candidates = []
    for fold in range(5):
        training = [item for index, item in enumerate(accepted) if index % 5 != fold]
        candidate, _kept, _quality = fit(training, initial, minimum=min(4, len(training)))
        candidates.append(candidate)
    neighbourhoods = [
        [
            index
            for index, other in enumerate(candidates)
            if transform_distance(candidate, other)[0] <= 0.10
            and transform_distance(candidate, other)[1] <= 1.0
        ]
        for candidate in candidates
    ]
    members = max(neighbourhoods, key=len)
    if len(members) / 5.0 < 0.60:
        raise RuntimeError(f"bootstrap dominant cluster {len(members)}/5")
    selected = [candidates[index] for index in members]
    consensus = np.eye(4)
    consensus[:3, 3] = np.median([item[:3, 3] for item in selected], axis=0)
    consensus[:3, :3] = Rotation.from_matrix([item[:3, :3] for item in selected]).mean().as_matrix()
    errors = [transform_distance(consensus, item) for item in selected]
    position = float(np.percentile([item[0] for item in errors], 95))
    angle = float(np.percentile([item[1] for item in errors], 95))
    tier = (
        "target_1mm_0.1deg"
        if position <= 0.001 and angle <= 0.1
        else "fallback_1cm_1deg"
        if position <= 0.01 and angle <= 1.0
        else "coarse_only"
    )
    quality.update(
        {
            "dominant_count": len(members),
            "dominant_fraction": len(members) / 5.0,
            "position_p95_m": position,
            "angle_p95_deg": angle,
            "achieved_tier": tier,
            "bootstrap_transforms": [matrix_summary(item) for item in candidates],
        }
    )
    return consensus, quality


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--data-root", type=Path, required=True)
    parser.add_argument("--annotations", type=Path, required=True)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--initial", type=Path, required=True)
    parser.add_argument("--intrinsics", type=Path, required=True)
    parser.add_argument("--reference", required=True)
    parser.add_argument("--target", required=True)
    parser.add_argument("--reference-transform", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.reference not in CAMERAS or args.target not in CAMERAS:
        raise SystemExit("unknown camera name")

    config = yaml.safe_load(args.config.read_text(encoding="utf-8"))
    initial_data = yaml.safe_load(args.initial.read_text(encoding="utf-8"))
    absolute = absolute_transforms(initial_data)
    top_from_kit = np.linalg.inv(absolute["velodyne_top_base_link"])
    initial_top_optical = {
        name: top_from_kit @ absolute[f"{name}/camera_link"] @ CAMERA_OPTICAL_TO_LINK
        for name in CAMERAS
    }
    intrinsics = {name: load_intrinsics(args.intrinsics / f"{name}.yaml") for name in CAMERAS}
    reference_data = yaml.safe_load(args.reference_transform.read_text(encoding="utf-8"))
    reference_frame = f"{args.reference}/camera_link"
    top_from_reference = (
        matrix_from_item(reference_data["transforms"][reference_frame]) @ CAMERA_OPTICAL_TO_LINK
    )
    allowed = set()
    with args.annotations.open(newline="", encoding="utf-8") as stream:
        for row in csv.DictReader(stream):
            if row.get("include_nonduplicate", "").lower() == "true":
                allowed.add(row["capture_id"])

    values: list[Observation] = []
    for index, capture_id in enumerate(sorted(allowed), 1):
        directory = args.data_root / capture_id
        manifest_path = directory / "manifest.json"
        if not manifest_path.is_file():
            continue
        poses = observations_for_capture(
            directory,
            json.loads(manifest_path.read_text(encoding="utf-8")),
            config["tag_board"],
            intrinsics,
            initial_top_optical,
        )
        references = [item for item in poses if item.camera == args.reference]
        targets = [item for item in poses if item.camera == args.target]
        choices = []
        for reference in references:
            for target in targets:
                common = sorted(set(reference.tag_ids) & set(target.tag_ids))
                if len(common) < 6:
                    continue
                gap = float(np.linalg.norm(reference.rough_center_top - target.rough_center_top))
                normal = angle_deg(reference.rough_normal_top, target.rough_normal_top)
                # Cameras are rigidly mounted close to their mechanical
                # seeds.  A wider gate admits a different identical-ID board
                # when several 900 mm targets share the scene.
                if gap <= 0.55 and normal <= 18.0:
                    choices.append((gap + 0.02 * normal, reference, target, common))
        if not choices:
            continue
        _score, reference, target, common = min(choices, key=lambda item: item[0])
        reference_by_id = {tag_id: i for i, tag_id in enumerate(reference.tag_ids)}
        target_by_id = {tag_id: i for i, tag_id in enumerate(target.tag_ids)}
        object_board = np.asarray(
            [reference.object_centres[reference_by_id[tag_id]] for tag_id in common]
        )
        normalized_target = np.asarray(
            [target.normalized_image_centres[target_by_id[tag_id]] for tag_id in common]
        )
        top_from_board = top_from_reference @ reference.transform_camera_from_board
        object_top = object_board @ top_from_board[:3, :3].T + top_from_board[:3, 3]
        values.append(Observation(capture_id, object_top, normalized_target))
        if index % 50 == 0:
            print(f"processed {index}/{len(allowed)} captures", flush=True)

    output: dict[str, Any] = {
        "schema_version": 1,
        "reference_frame": "velodyne_top_base_link",
        "method": "anchored_reference_camera_multiview_projection_bundle",
        "reference_camera": args.reference,
        "target_camera": args.target,
        "reference_transform_source": args.reference_transform.name,
        "corner_source": "fresh_redetection_from_each_saved_image",
    }
    try:
        optical, quality = bootstrap(values, initial_top_optical[args.target])
        output["transform"] = {
            "frame": f"{args.target}/camera_link",
            "parent": "velodyne_top_base_link",
            **matrix_summary(optical @ np.linalg.inv(CAMERA_OPTICAL_TO_LINK)),
            "source": "anchored_reference_camera_bundle",
        }
        output["quality"] = quality
    except RuntimeError as error:
        output["rejected"] = {"raw_views": len(values), "error": str(error)}
    args.output.write_text(
        yaml.safe_dump(output, allow_unicode=True, sort_keys=False), encoding="utf-8"
    )
    print(yaml.safe_dump(output.get("quality", output.get("rejected")), sort_keys=False))
    print(args.output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
