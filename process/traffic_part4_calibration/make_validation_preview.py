#!/usr/bin/env python3
"""Render a compact fresh-pixel validation contact sheet for Part 4."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

import cv2
import numpy as np
import yaml


HERE = Path(__file__).resolve().parent
JOINT = HERE.parent / "joint_extrinsic_station"
if str(JOINT) not in sys.path:
    sys.path.insert(0, str(JOINT))
from solve_joint_extrinsics import make_detector  # noqa: E402


def main() -> int:
    base = HERE.parents[1]
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--audit", type=Path, default=base / "result/traffic_part4_20260903/frame_audit.json"
    )
    parser.add_argument(
        "--config", type=Path, default=HERE.parent / "joint_extrinsic_station/config.yaml"
    )
    parser.add_argument("--data-root", type=Path, default=base / "data/raw_extrinsic_data")
    parser.add_argument(
        "--output",
        type=Path,
        default=base / "result/traffic_part4_20260903/fresh_pixel_validation.jpg",
    )
    args = parser.parse_args()
    payload = json.loads(args.audit.read_text(encoding="utf-8"))
    rows = payload["frames"]
    indices = np.linspace(0, len(rows) - 1, 12).round().astype(int)
    config = yaml.safe_load(args.config.read_text(encoding="utf-8"))
    detector = make_detector(config["tag_board"]["dictionary"])
    tiles = []
    for index in indices:
        row = rows[int(index)]
        image = cv2.imread(str(args.data_root / row["capture_id"] / row["image_file"]))
        corners, ids = detector(cv2.cvtColor(image, cv2.COLOR_BGR2GRAY))
        preview = image.copy()
        if ids is not None:
            cv2.aruco.drawDetectedMarkers(preview, corners, ids)
        colour = (40, 210, 40) if row.get("lidar_observations") else (30, 30, 235)
        label = (
            f"{row['capture_id'][9:]} tags={row.get('tag_count', 0)} "
            f"planes={len(row.get('lidar_observations', []))}"
        )
        cv2.rectangle(preview, (0, 0), (319, 22), (0, 0, 0), -1)
        cv2.putText(preview, label, (4, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.38, colour, 1, cv2.LINE_AA)
        tiles.append(preview)
    rows_image = [np.hstack(tiles[index : index + 3]) for index in range(0, 12, 3)]
    contact = np.vstack(rows_image)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    if not cv2.imwrite(str(args.output), contact, [cv2.IMWRITE_JPEG_QUALITY, 94]):
        raise RuntimeError("failed to write preview")
    print(args.output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
