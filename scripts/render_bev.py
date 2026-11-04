#!/usr/bin/env python3
"""Render four raw fisheye frames with saved destination-to-source ground maps."""

from __future__ import annotations

import argparse
from pathlib import Path

import cv2
import numpy as np

DIRECTIONS = ("front", "rear", "left", "right")


def compose(images: dict[str, np.ndarray], package: Path) -> np.ndarray:
    total = None
    weight_sum = None
    for direction in DIRECTIONS:
        image = images[direction]
        if image.shape[:2] != (1200, 1600):
            raise ValueError(f"{direction}: expected uncropped 1600x1200 image")
        map_x = np.load(package / f"fisheye_{direction}_bev_map_x.npy", allow_pickle=False)
        map_y = np.load(package / f"fisheye_{direction}_bev_map_y.npy", allow_pickle=False)
        if (
            map_x.shape != map_y.shape
            or not np.isfinite(map_x).all()
            or not np.isfinite(map_y).all()
        ):
            raise ValueError(f"{direction}: invalid remap arrays")
        valid = ((map_x >= 0) & (map_x < 1599) & (map_y >= 0) & (map_y < 1199)).astype(np.uint8)
        weight = cv2.distanceTransform(valid, cv2.DIST_L2, 3)
        weight = np.minimum(weight, 32.0) * valid
        warped = cv2.remap(
            image, map_x.astype(np.float32), map_y.astype(np.float32), cv2.INTER_LINEAR
        )
        if total is None:
            total = np.zeros_like(warped, dtype=np.float32)
            weight_sum = np.zeros_like(weight, dtype=np.float32)
        if total.shape != warped.shape:
            raise ValueError("All camera maps must share the same BEV canvas")
        total += warped * weight[..., None]
        weight_sum += weight
    return np.clip(total / np.maximum(weight_sum[..., None], 1e-6), 0, 255).astype(np.uint8)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--package", type=Path, required=True)
    for direction in DIRECTIONS:
        parser.add_argument(f"--{direction}", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    images = {}
    for direction in DIRECTIONS:
        image = cv2.imread(str(getattr(args, direction)))
        if image is None:
            parser.error(f"Cannot read {direction} image")
        images[direction] = image
    preview = compose(images, args.package)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    if not cv2.imwrite(str(args.output), preview):
        raise OSError(f"Cannot write {args.output}")
    print(f"BEV preview: {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
