#!/usr/bin/env python3
"""Estimate lower BPearl vertical translations from a shared garage floor.

The tool is deliberately limited to the Z translation.  It uses the roof lidar
floor plane as the reference, rejects walls/columns by plane normal, and reports
quality metrics before any calibration file is changed.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np
import yaml
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation


LOWER_LIDARS = {
    "lidar_front_left": "velodyne_front_left_base_link",
    "lidar_front_right": "velodyne_front_right_base_link",
    "lidar_rear": "velodyne_rear_base_link",
}


@dataclass(frozen=True)
class PlaneFit:
    normal: np.ndarray
    offset: float
    inliers: int
    candidates: int
    rms_m: float

    @property
    def tilt_deg(self) -> float:
        return float(np.degrees(np.arccos(np.clip(self.normal[2], -1.0, 1.0))))

    def z_at(self, x: float, y: float) -> float:
        return float(-(self.normal[0] * x + self.normal[1] * y + self.offset) / self.normal[2])


@dataclass(frozen=True)
class BroadPlaneFit:
    normal: np.ndarray
    offset: float
    inliers: int
    candidates: int
    rms_m: float
    extents_m: np.ndarray

    @property
    def distance_from_sensor_m(self) -> float:
        return abs(float(self.offset))


def read_binary_pcd_xyz(path: Path) -> np.ndarray:
    """Read XYZ from the binary PCD files written by the capture station."""
    with path.open("rb") as stream:
        header: dict[str, list[str]] = {}
        while True:
            line = stream.readline()
            if not line:
                raise ValueError(f"{path}: incomplete PCD header")
            text = line.decode("ascii").strip()
            if not text or text.startswith("#"):
                continue
            key, *values = text.split()
            header[key.upper()] = values
            if key.upper() == "DATA":
                if values != ["binary"]:
                    raise ValueError(f"{path}: only binary PCD is supported")
                break
        names = header["FIELDS"]
        sizes = [int(value) for value in header["SIZE"]]
        types = header["TYPE"]
        counts = [int(value) for value in header.get("COUNT", ["1"] * len(names))]
        points = int(header["POINTS"][0])
        formats: list[Any] = []
        offsets: list[int] = []
        item_size = 0
        type_codes = {
            ("F", 4): "<f4",
            ("F", 8): "<f8",
            ("U", 1): "u1",
            ("U", 2): "<u2",
            ("U", 4): "<u4",
            ("I", 1): "i1",
            ("I", 2): "<i2",
            ("I", 4): "<i4",
        }
        for size, scalar_type, count in zip(sizes, types, counts):
            scalar = np.dtype(type_codes[(scalar_type, size)])
            formats.append(scalar if count == 1 else (scalar, (count,)))
            offsets.append(item_size)
            item_size += size * count
        dtype = np.dtype(
            {"names": names, "formats": formats, "offsets": offsets, "itemsize": item_size}
        )
        values = np.frombuffer(stream.read(points * item_size), dtype=dtype, count=points)
    xyz = np.column_stack((values["x"], values["y"], values["z"])).astype(np.float64, copy=False)
    return xyz[np.isfinite(xyz).all(axis=1)]


def matrix_from_seed(item: dict[str, Any]) -> np.ndarray:
    matrix = np.eye(4)
    matrix[:3, :3] = Rotation.from_euler("xyz", item["rpy_deg"], degrees=True).as_matrix()
    matrix[:3, 3] = np.asarray(item["xyz_m"], dtype=float)
    return matrix


def fit_floor(points_top: np.ndarray, expected_z: float | None, seed: int) -> PlaneFit:
    radius = np.linalg.norm(points_top[:, :2], axis=1)
    mask = (radius >= 0.8) & (radius <= 12.0) & (points_top[:, 2] <= 0.25)
    if expected_z is not None:
        mask &= (points_top[:, 2] >= expected_z - 0.75) & (points_top[:, 2] <= expected_z + 0.45)
    else:
        # A lower bumper lidar still has a positive physical height.  With the
        # current seed translation (-0.3705 m), its floor must appear below
        # about -0.55 m in the roof frame.  Rejecting shallower planes prevents
        # the vehicle body, wheel-stop tops and rotation-induced wall planes
        # from winning merely because they contain many returns.
        mask &= (points_top[:, 2] >= -3.0) & (points_top[:, 2] <= -0.55)
    candidates = points_top[mask]
    if len(candidates) < 100:
        raise RuntimeError(f"ground ROI has only {len(candidates)} points")

    rng = np.random.default_rng(seed)
    # The garage contains broad walls, columns, pipes and wheel stops.  For
    # every XY cell keep the lowest return; the floor then dominates this lower
    # envelope while vertical and elevated structures contribute few cells.
    cells = np.floor(candidates[:, :2] / 0.22).astype(np.int32)
    _, inverse = np.unique(cells, axis=0, return_inverse=True)
    order = np.lexsort((candidates[:, 2], inverse))
    sorted_groups = inverse[order]
    first = np.r_[True, sorted_groups[1:] != sorted_groups[:-1]]
    sample = candidates[order[first]]
    if len(sample) > 9000:
        sample = sample[rng.choice(len(sample), 9000, replace=False)]
    best_count = 0
    best_plane: tuple[np.ndarray, float] | None = None
    threshold = 0.045
    for _ in range(1400):
        triple = sample[rng.choice(len(sample), 3, replace=False)]
        normal = np.cross(triple[1] - triple[0], triple[2] - triple[0])
        norm = np.linalg.norm(normal)
        if norm < 1e-8:
            continue
        normal /= norm
        if normal[2] < 0.0:
            normal = -normal
        # A floor can be slightly sloped, but walls, columns and most pipes
        # cannot pass this normal constraint.
        if normal[2] < np.cos(np.deg2rad(18.0)):
            continue
        offset = -float(normal @ triple[0])
        count = int(np.count_nonzero(np.abs(sample @ normal + offset) < threshold))
        if count > best_count:
            best_count = count
            best_plane = normal, offset
    if best_plane is None or best_count < 50:
        raise RuntimeError("no sufficiently large near-horizontal plane found")

    normal, offset = best_plane
    for _ in range(3):
        residual = np.abs(candidates @ normal + offset)
        inlier_points = candidates[residual < threshold]
        center = np.mean(inlier_points, axis=0)
        _, _, vectors_t = np.linalg.svd(inlier_points - center, full_matrices=False)
        normal = vectors_t[-1]
        if normal[2] < 0.0:
            normal = -normal
        offset = -float(normal @ center)
    residual = np.abs(candidates @ normal + offset)
    inlier_mask = residual < threshold
    inlier_residual = residual[inlier_mask]
    return PlaneFit(
        normal,
        offset,
        int(np.count_nonzero(inlier_mask)),
        len(candidates),
        float(np.sqrt(np.mean(np.square(inlier_residual)))),
    )


def fit_nearby_broad_plane(points_sensor: np.ndarray, seed: int) -> BroadPlaneFit:
    """Find the physical floor without trusting the old sensor rotation.

    The floor is the only broad plane whose perpendicular distance from the
    lower BPearl origin should be between 0.15 m and 1.60 m.  The garage walls
    and columns described for this site are 4 m or farther away, while vehicle
    body panels and wheel stops do not have a multi-metre two-dimensional span.
    """
    distance = np.linalg.norm(points_sensor, axis=1)
    candidates = points_sensor[(distance >= 0.7) & (distance <= 15.0)]
    if len(candidates) < 500:
        raise RuntimeError("not enough finite points for raw floor fitting")
    rng = np.random.default_rng(seed)
    sample = candidates
    if len(sample) > 12000:
        sample = sample[rng.choice(len(sample), 12000, replace=False)]
    neighbours = cKDTree(sample).query(sample, k=14, workers=-1)[1]
    best_score = -1.0
    best_plane: tuple[np.ndarray, float] | None = None
    threshold = 0.045
    for _ in range(4200):
        anchor = int(rng.integers(len(sample)))
        choices = rng.choice(neighbours[anchor, 1:], 2, replace=False)
        triple = sample[[anchor, int(choices[0]), int(choices[1])]]
        normal = np.cross(triple[1] - triple[0], triple[2] - triple[0])
        norm = np.linalg.norm(normal)
        if norm < 1e-8:
            continue
        normal /= norm
        offset = -float(normal @ triple[0])
        plane_distance = abs(offset)
        if not 0.15 <= plane_distance <= 1.60:
            continue
        inlier_mask = np.abs(sample @ normal + offset) < threshold
        count = int(np.count_nonzero(inlier_mask))
        if count < 80:
            continue
        inliers = sample[inlier_mask]
        center = np.mean(inliers, axis=0)
        _, _, vectors_t = np.linalg.svd(inliers - center, full_matrices=False)
        basis = vectors_t[:2]
        uv = (inliers - center) @ basis.T
        extents = np.diff(np.percentile(uv, [2.0, 98.0], axis=0), axis=0)[0]
        short, long = np.sort(extents)
        # Require a true 2-D plane and reward spatial coverage.  A long thin
        # wheel stop or pipe cannot outscore the garage floor.
        if short < 1.25 or long < 2.50:
            continue
        score = count * min(short, 8.0) * min(long, 12.0)
        if score > best_score:
            best_score = score
            best_plane = normal, offset
    if best_plane is None:
        raise RuntimeError("no broad plane at a physically plausible sensor height")

    normal, offset = best_plane
    for _ in range(3):
        residual = np.abs(candidates @ normal + offset)
        inliers = candidates[residual < threshold]
        center = np.mean(inliers, axis=0)
        _, _, vectors_t = np.linalg.svd(inliers - center, full_matrices=False)
        normal = vectors_t[-1]
        offset = -float(normal @ center)
    residual = np.abs(candidates @ normal + offset)
    inlier_mask = residual < threshold
    inliers = candidates[inlier_mask]
    center = np.mean(inliers, axis=0)
    _, _, vectors_t = np.linalg.svd(inliers - center, full_matrices=False)
    uv = (inliers - center) @ vectors_t[:2].T
    extents = np.diff(np.percentile(uv, [2.0, 98.0], axis=0), axis=0)[0]
    return BroadPlaneFit(
        normal=normal,
        offset=offset,
        inliers=int(np.count_nonzero(inlier_mask)),
        candidates=len(candidates),
        rms_m=float(np.sqrt(np.mean(np.square(residual[inlier_mask])))),
        extents_m=np.sort(extents),
    )


def lidar_file(capture: Path, name: str) -> Path:
    matches = sorted((capture / "lidars").glob(f"*_{name}.pcd"))
    if len(matches) != 1:
        raise RuntimeError(f"{capture}: expected one PCD for {name}, got {len(matches)}")
    return matches[0]


def analyse_capture(
    capture: Path, initial: dict[str, Any], seed: int
) -> dict[str, dict[str, float]]:
    roof_points = read_binary_pcd_xyz(lidar_file(capture, "lidar_top"))
    roof_floor = fit_floor(roof_points, expected_z=-1.742, seed=seed)
    result: dict[str, dict[str, float]] = {
        "lidar_top": {
            "ground_z_m": roof_floor.z_at(0.0, 0.0),
            "tilt_deg": roof_floor.tilt_deg,
            "inliers": float(roof_floor.inliers),
            "candidate_points": float(roof_floor.candidates),
            "rms_m": roof_floor.rms_m,
            "normal_x": float(roof_floor.normal[0]),
            "normal_y": float(roof_floor.normal[1]),
            "normal_z": float(roof_floor.normal[2]),
        }
    }
    for index, (name, frame) in enumerate(LOWER_LIDARS.items(), start=1):
        transform = matrix_from_seed(initial["transforms"][frame])
        raw = read_binary_pcd_xyz(lidar_file(capture, name))
        floor = fit_nearby_broad_plane(raw, seed=seed + index)
        normal_top_seed = transform[:3, :3] @ floor.normal
        if normal_top_seed[2] < 0.0:
            normal_top_seed = -normal_top_seed
            floor = BroadPlaneFit(
                -floor.normal,
                -floor.offset,
                floor.inliers,
                floor.candidates,
                floor.rms_m,
                floor.extents_m,
            )
        x, y = transform[:2, 3]
        sensor_height = floor.distance_from_sensor_m
        new_z = float(
            (
                sensor_height
                - roof_floor.normal[0] * x
                - roof_floor.normal[1] * y
                - roof_floor.offset
            )
            / roof_floor.normal[2]
        )
        old_z = float(transform[2, 3])
        tilt_from_roof = float(
            np.degrees(np.arccos(np.clip(normal_top_seed @ roof_floor.normal, -1.0, 1.0)))
        )
        result[name] = {
            "sensor_height_m": sensor_height,
            "new_translation_z_m": new_z,
            "height_correction_m": new_z - old_z,
            "tilt_deg": tilt_from_roof,
            "inliers": float(floor.inliers),
            "candidate_points": float(floor.candidates),
            "rms_m": floor.rms_m,
            "normal_x": float(normal_top_seed[0]),
            "normal_y": float(normal_top_seed[1]),
            "normal_z": float(normal_top_seed[2]),
            "raw_normal_x": float(floor.normal[0]),
            "raw_normal_y": float(floor.normal[1]),
            "raw_normal_z": float(floor.normal[2]),
            "short_extent_m": float(floor.extents_m[0]),
            "long_extent_m": float(floor.extents_m[1]),
        }
    return result


def main() -> int:
    base = Path(__file__).resolve().parent
    parser = argparse.ArgumentParser()
    parser.add_argument("captures", nargs="+", type=Path)
    parser.add_argument("--initial", type=Path, default=base / "initial_extrinsics.yaml")
    args = parser.parse_args()
    initial = yaml.safe_load(args.initial.read_text(encoding="utf-8"))
    corrections: dict[str, list[float]] = {name: [] for name in LOWER_LIDARS}
    print(
        "capture,sensor,height_or_ground_z_m,new_translation_z_m,correction_m,"
        "tilt_deg,normal_in_roof_seed,raw_normal,plane_extents_m,"
        "inliers,candidates,rms_m"
    )
    for capture_index, capture in enumerate(args.captures):
        result = analyse_capture(capture.resolve(), initial, 9400 + capture_index * 10)
        roof = result["lidar_top"]
        print(
            f"{capture.name},lidar_top,{roof['ground_z_m']:.5f},,,"
            f"{roof['tilt_deg']:.3f},"
            f"[{roof['normal_x']:.5f} {roof['normal_y']:.5f} {roof['normal_z']:.5f}],"
            f"[],"
            f"[],"
            f"{int(roof['inliers'])},"
            f"{int(roof['candidate_points'])},{roof['rms_m']:.5f}"
        )
        for name in LOWER_LIDARS:
            item = result[name]
            corrections[name].append(item["height_correction_m"])
            print(
                f"{capture.name},{name},{item['sensor_height_m']:.5f},"
                f"{item['new_translation_z_m']:.5f},"
                f"{item['height_correction_m']:+.5f},{item['tilt_deg']:.3f},"
                f"[{item['normal_x']:.5f} {item['normal_y']:.5f} {item['normal_z']:.5f}],"
                f"[{item['raw_normal_x']:.5f} {item['raw_normal_y']:.5f} {item['raw_normal_z']:.5f}],"
                f"[{item['short_extent_m']:.3f} {item['long_extent_m']:.3f}],"
                f"{int(item['inliers'])},{int(item['candidate_points'])},"
                f"{item['rms_m']:.5f}"
            )
    print("summary,sensor,median_correction_m,max_deviation_m")
    for name, values in corrections.items():
        median = float(np.median(values))
        deviation = float(np.max(np.abs(np.asarray(values) - median)))
        print(f"summary,{name},{median:+.5f},{deviation:.5f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
