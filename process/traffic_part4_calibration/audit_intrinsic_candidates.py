#!/usr/bin/env python3
"""Compare traffic-camera intrinsic candidates on fresh Part-4 AprilGrid pixels."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys
from typing import Any

import cv2
import numpy as np
import yaml


HERE = Path(__file__).resolve().parent
JOINT = HERE.parent / "joint_extrinsic_station"
if str(JOINT) not in sys.path:
    sys.path.insert(0, str(JOINT))
from solve_joint_extrinsics import load_intrinsics, make_detector  # noqa: E402
from traffic_part4_calibrate import CAMERA, detector_result, part4_ids, stats  # noqa: E402


def radial_audit(
    camera: np.ndarray, distortion: np.ndarray, width: int, height: int
) -> dict[str, Any]:
    points = np.asarray(
        [
            [0, 0],
            [width - 1, 0],
            [width - 1, height - 1],
            [0, height - 1],
            [(width - 1) / 2, (height - 1) / 2],
        ],
        dtype=np.float64,
    ).reshape(-1, 1, 2)
    normalised = cv2.undistortPoints(points, camera, distortion).reshape(-1, 2)
    radius = np.linalg.norm(normalised, axis=1)
    finite = bool(np.isfinite(normalised).all())
    return {
        "finite_at_image_corners": finite,
        "undistorted_normalised_corner_radii": radius[:4].tolist(),
        "maximum_corner_radius": float(np.max(radius[:4])),
        "horizontal_fov_approx_deg": float(
            np.degrees(2.0 * np.arctan(width / (2.0 * camera[0, 0])))
        ),
    }


def main() -> int:
    base = HERE.parents[1]
    parser = argparse.ArgumentParser()
    parser.add_argument("--data-root", type=Path, default=base / "data/raw_extrinsic_data")
    parser.add_argument(
        "--part4-list",
        type=Path,
        default=base / "data/backup_manifests/20260903/part4_capture_groups.txt",
    )
    parser.add_argument(
        "--config", type=Path, default=HERE.parent / "joint_extrinsic_station/config.yaml"
    )
    parser.add_argument("--candidate", action="append", type=Path)
    parser.add_argument(
        "--output",
        type=Path,
        default=base / "result/traffic_part4_20260903/intrinsic_candidate_audit.json",
    )
    args = parser.parse_args()
    candidates = args.candidate or [
        base
        / "calibration/runtime_context_20260902/white_ws/calibration/camera_results"
        / f"traffic_light_front_{suffix}.yaml"
        for suffix in (
            "20260901_231630_372090284",
            "20260901_231923_564865757",
            "20260901_231925_129579029",
            "20260901_232705_372307319",
        )
    ]
    config = yaml.safe_load(args.config.read_text(encoding="utf-8"))
    board = config["tag_board"]
    detector = make_detector(board["dictionary"])
    output: dict[str, Any] = {
        "schema_version": 1,
        "method": "fresh_part4_aprilgrid_holdout_audit",
        "warning": "Lowest PnP residual alone does not justify a high-order distortion model.",
    }
    items = {}
    for candidate in candidates:
        intrinsic = load_intrinsics(candidate)
        errors, tags, accepted = [], [], []
        statuses = {}
        for capture_id in part4_ids(args.part4_list):
            capture = args.data_root / capture_id
            manifest = json.loads((capture / "manifest.json").read_text(encoding="utf-8"))
            relative = manifest["sensors"][CAMERA]["file"]
            image = cv2.imread(str(capture / relative), cv2.IMREAD_COLOR)
            result = detector_result(image, detector, board, intrinsic)
            statuses[result["status"]] = statuses.get(result["status"], 0) + 1
            if result["status"] == "accepted":
                errors.append(float(result["reprojection_rms_px"]))
                tags.append(int(result["tag_count"]))
                accepted.append(capture_id)
        camera, distortion, _model = intrinsic
        items[candidate.name] = {
            "path": str(candidate),
            "accepted_pnp_count": len(errors),
            "status_counts": statuses,
            "reprojection_rms_px": stats(errors),
            "tag_count": stats(tags),
            "accepted_capture_ids": accepted,
            "radial_model": radial_audit(camera, distortion, 320, 240),
            "K": camera.tolist(),
            "D": distortion.reshape(-1).tolist(),
        }
    output["candidates"] = items
    constrained = "traffic_light_front_20260901_232705_372307319.yaml"
    output["selection"] = {
        "selected": constrained,
        "reason": "Independent ChArUco fit uses fixed principal point/aspect ratio, zero tangential and fixed k3. The other candidates obtain marginally lower training residual with unstable high-order radial coefficients; Part-4 is an extrinsic set, not an independent intrinsic target set.",
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(output, ensure_ascii=False, indent=2), encoding="utf-8")
    print(
        json.dumps(
            {
                name: {
                    "count": item["accepted_pnp_count"],
                    "p95": item["reprojection_rms_px"]["p95"],
                }
                for name, item in items.items()
            },
            indent=2,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
