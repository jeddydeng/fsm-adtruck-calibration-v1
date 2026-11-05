#!/usr/bin/env python3
"""Check runnable source, source dependencies, mapping consistency and camera models."""

from __future__ import annotations

import ast
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]


def main() -> int:
    sources = [ROOT / name for name in ("scripts", "process", "ros_ws/src", "drivers")]
    python_count = shell_count = 0
    for tree in sources:
        for path in tree.rglob("*.py"):
            ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
            python_count += 1
        for path in tree.rglob("*.sh"):
            subprocess.run(["bash", "-n", str(path)], check=True)
            shell_count += 1
    for path in ROOT.glob("*.sh"):
        subprocess.run(["bash", "-n", str(path)], check=True)
        shell_count += 1
    for path in (ROOT / "ros_ws/src").rglob("package.xml"):
        ET.parse(path)
        assert not (path.parent / "COLCON_IGNORE").exists(), f"Hidden package: {path.parent}"
    config = yaml.safe_load((ROOT / "config/black_vehicle/station.yaml").read_text())
    driver = yaml.safe_load((ROOT / "config/black_vehicle/rslidar.yaml").read_text())
    station_map = {
        x["topic"]: (x["msop_port"], x["difop_port"]) for x in config["lidars"]["sensors"]
    }
    driver_map = {
        x["ros"]["ros_send_point_cloud_topic"]: (
            x["driver"]["msop_port"],
            x["driver"]["difop_port"],
        )
        for x in driver["lidar"]
    }
    assert station_map == driver_map, "Station/driver lidar port mismatch"
    assert len(list((ROOT / "config/intrinsics").glob("*.yaml"))) == 11
    for name in ("front", "rear", "left", "right"):
        info = yaml.safe_load((ROOT / f"config/intrinsics/fisheye_{name}.yaml").read_text())
        assert (info["image_width"], info["image_height"]) == (1600, 1200)
        assert info["distortion_model"] == "equidistant"
        assert len(info["distortion_coefficients"]["data"]) == 4
        assert np.isfinite(info["camera_matrix"]["data"]).all()
    for axis in ("x", "y"):
        for name in ("front", "rear", "left", "right"):
            path = (
                ROOT
                / f"result/fisheye_bev_20260903/relative_usable/fisheye_{name}_bev_map_{axis}.npy"
            )
            remap = np.load(path, allow_pickle=False)
            assert remap.shape == (512, 512) and np.isfinite(remap).all()
    print(
        f"PASS: {python_count} Python sources, {shell_count} shells, 11 camera presets, 4 lidar mappings, 8 BEV maps"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
