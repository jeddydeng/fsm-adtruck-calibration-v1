#!/usr/bin/env python3
"""Fail-closed verification for the signed relative four-fisheye BEV package."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

import cv2
import numpy as np


CAMERAS = ("front", "right", "rear", "left")


def digest(path: Path) -> str:
    value = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            value.update(block)
    return value.hexdigest()


def arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--package", type=Path, required=True)
    parser.add_argument("--data-root", type=Path, required=True)
    parser.add_argument("--baseline", type=Path, required=True)
    return parser.parse_args()


def main() -> None:
    args = arguments()
    report = json.loads((args.package / "bev_calibration.json").read_text())
    assert report["status"] == "usable"
    assert report["deployment_allowed"] == 1
    assert report["deployment_scope"] == "BEV_relative_planar_mapping_only"
    assert report["absolute_vehicle_extrinsic_deployment_allowed"] == 0
    assert report["all_self_checks_passed"] is True
    assert all(report["self_checks"].values())
    assert report["selected_capture_count"] == 14
    assert report["selected_source_image_count"] == 4 * report["selected_capture_count"]

    for camera in CAMERAS:
        long_name = f"fisheye_{camera}"
        map_x = np.load(args.package / f"{long_name}_bev_map_x.npy")
        map_y = np.load(args.package / f"{long_name}_bev_map_y.npy")
        assert map_x.shape == (512, 512) and map_y.shape == (512, 512)
        assert map_x.dtype == np.float32 and map_y.dtype == np.float32
        assert np.isfinite(map_x).all() and np.isfinite(map_y).all()

        yaml_file = cv2.FileStorage(str(args.package / f"{camera}.yaml"), cv2.FILE_STORAGE_READ)
        assert yaml_file.isOpened()
        final_h = yaml_file.getNode("H_bev").mat()
        correction = yaml_file.getNode("C_bev_from_baseline").mat()
        assert final_h.shape == (3, 3) and correction.shape == (3, 3)
        assert abs(np.linalg.det(final_h)) > 1e-12
        assert abs(np.linalg.det(correction) - 1.0) < 1e-8
        baseline_file = cv2.FileStorage(
            str(args.baseline / f"{camera}.yaml"), cv2.FILE_STORAGE_READ
        )
        baseline_h = baseline_file.getNode("H_bev").mat()
        np.testing.assert_allclose(final_h, correction @ baseline_h, rtol=1e-8, atol=1e-8)
        yaml_file.release()
        baseline_file.release()

    source_lines = (args.package / "raw_image_sha256.txt").read_text().splitlines()
    assert len(source_lines) == report["selected_source_image_count"]
    for line in source_lines:
        expected, relative = line.split(maxsplit=1)
        assert digest(args.data_root / relative) == expected

    manifest_lines = (args.package / "manifest.sha256").read_text().splitlines()
    assert len(manifest_lines) >= 25
    for line in manifest_lines:
        expected, name = line.split(maxsplit=1)
        assert digest(args.package / name) == expected

    for name in (
        "four_camera_bev_preview.png",
        "strict14_bev_contact_sheet.jpg",
        "overlap_validation_contact_sheet.jpg",
        "bev_coverage_count.png",
    ):
        image = cv2.imread(str(args.package / name), cv2.IMREAD_UNCHANGED)
        assert image is not None and image.size > 0
    print(
        json.dumps(
            {
                "verified": True,
                "package_files_hashed": len(manifest_lines),
                "source_images_hashed": len(source_lines),
                "bev_deployment_allowed": 1,
                "vehicle_absolute_extrinsic_deployment_allowed": 0,
            },
            indent=2,
        )
    )


if __name__ == "__main__":
    main()
