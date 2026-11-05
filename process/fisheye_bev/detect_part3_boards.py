#!/usr/bin/env python3
"""Fresh-pixel AprilGrid detection for the Part-3 fisheye BEV dataset.

Four physical boards use the same 0..29 tag IDs.  A single call to
estimatePoseBoard therefore cannot be used.  This program first separates the
repeated boards with a multi-model homography RANSAC, then estimates one
fisheye PnP pose for every board instance.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

import cv2
import numpy as np
import yaml


CAMERAS = ("fisheye_front", "fisheye_right", "fisheye_rear", "fisheye_left")
COLS, ROWS = 6, 5
TAG_SIZE_M = 0.108
GAP_X_M, GAP_Y_M = 0.0304, 0.065
OUTER_M, ACTIVE_M = 0.900, 0.800
CORNER_ROTATION_QUARTERS = 2


@dataclass
class Marker:
    index: int
    tag_id: int
    corners_px: np.ndarray

    @property
    def centre(self) -> np.ndarray:
        return self.corners_px.mean(axis=0)

    @property
    def mean_side_px(self) -> float:
        return float(
            np.mean(np.linalg.norm(self.corners_px - np.roll(self.corners_px, 1, axis=0), axis=1))
        )


def marker_object_corners(tag_id: int) -> np.ndarray:
    """Return board-frame corners, matching the printed 180-degree rotation."""
    row, col = divmod(tag_id, COLS)
    pitch_x = TAG_SIZE_M + GAP_X_M
    pitch_y = TAG_SIZE_M + GAP_Y_M
    half = TAG_SIZE_M / 2.0
    x = (col - (COLS - 1) / 2.0) * pitch_x
    y = (row - (ROWS - 1) / 2.0) * pitch_y
    corners = np.asarray(
        [
            [x - half, y + half, 0.0],
            [x + half, y + half, 0.0],
            [x + half, y - half, 0.0],
            [x - half, y - half, 0.0],
        ],
        np.float64,
    )
    return np.roll(corners, CORNER_ROTATION_QUARTERS, axis=0)


def load_intrinsics(path: Path) -> tuple[np.ndarray, np.ndarray, tuple[int, int]]:
    data = yaml.safe_load(path.read_text())
    if data.get("distortion_model") != "equidistant":
        raise ValueError(f"{path}: expected equidistant distortion model")
    size = (int(data["image_width"]), int(data["image_height"]))
    if min(size) <= 0:
        raise ValueError(f"{path}: invalid image dimensions {size}")
    K = np.asarray(data["camera_matrix"]["data"], np.float64).reshape(3, 3)
    D = np.asarray(data["distortion_coefficients"]["data"], np.float64).reshape(4, 1)
    return K, D, size


def detect_markers(image: np.ndarray) -> list[Marker]:
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_16h5)
    parameters = cv2.aruco.DetectorParameters()
    parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    detector = cv2.aruco.ArucoDetector(dictionary, parameters)
    corners, ids, _ = detector.detectMarkers(image)
    if ids is None:
        return []
    result = []
    for index, (tag_id, corner) in enumerate(zip(ids.reshape(-1), corners)):
        tag_id = int(tag_id)
        if 0 <= tag_id < COLS * ROWS:
            result.append(Marker(index, tag_id, np.asarray(corner, np.float64).reshape(4, 2)))
    return result


def apply_homography(H: np.ndarray, points: np.ndarray) -> np.ndarray:
    points_h = np.c_[points, np.ones(len(points))]
    projected = points_h @ H.T
    return projected[:, :2] / projected[:, 2:3]


def assignment_for_homography(
    H: np.ndarray, markers: list[Marker], threshold_px: float
) -> tuple[list[Marker], float]:
    selected: list[Marker] = []
    errors: list[float] = []
    for tag_id in range(COLS * ROWS):
        predicted = apply_homography(H, marker_object_corners(tag_id)[:, :2])
        options = [marker for marker in markers if marker.tag_id == tag_id]
        if not options:
            continue
        candidate_errors = [
            float(np.sqrt(np.mean(np.sum((marker.corners_px - predicted) ** 2, axis=1))))
            for marker in options
        ]
        best = int(np.argmin(candidate_errors))
        if candidate_errors[best] <= threshold_px:
            selected.append(options[best])
            errors.append(candidate_errors[best])
    rms = float(np.sqrt(np.mean(np.square(errors)))) if errors else float("inf")
    return selected, rms


def fit_homography(markers: list[Marker]) -> np.ndarray | None:
    if not markers:
        return None
    source = np.concatenate([marker_object_corners(marker.tag_id)[:, :2] for marker in markers])
    target = np.concatenate([marker.corners_px for marker in markers])
    H, _ = cv2.findHomography(source, target, cv2.RANSAC, 3.0, maxIters=3000, confidence=0.999)
    return H


def separate_repeated_boards(
    markers: list[Marker], minimum_tags: int = 5, maximum_boards: int = 6
) -> list[list[Marker]]:
    # Spatial connected components are the primary separator.  A projective
    # homography is not globally valid on a 180-degree raw fisheye image and
    # tended to split the near edge of one physical board into two models.
    neighbours = [[] for _ in markers]
    for left in range(len(markers)):
        for right in range(left + 1, len(markers)):
            distance = float(np.linalg.norm(markers[left].centre - markers[right].centre))
            local_scale = max(markers[left].mean_side_px, markers[right].mean_side_px)
            if distance <= max(22.0, 3.80 * local_scale):
                neighbours[left].append(right)
                neighbours[right].append(left)
    components, visited = [], set()
    for start in range(len(markers)):
        if start in visited:
            continue
        stack, component = [start], []
        visited.add(start)
        while stack:
            current = stack.pop()
            component.append(markers[current])
            for neighbour in neighbours[current]:
                if neighbour not in visited:
                    visited.add(neighbour)
                    stack.append(neighbour)
        components.append(component)

    clean = []
    ambiguous = []
    for component in components:
        ids = [marker.tag_id for marker in component]
        if len(component) >= minimum_tags and len(set(ids)) == len(ids):
            clean.append(sorted(component, key=lambda marker: marker.tag_id))
        elif len(component) >= minimum_tags:
            ambiguous.extend(component)
    if not ambiguous:
        return sorted(clean, key=lambda group: -len(group))[:maximum_boards]

    # Rarely, two boards touch in the image and form one component.  Only that
    # ambiguous subset is passed through the slower repeated-model RANSAC.
    remaining = list(ambiguous)
    boards: list[list[Marker]] = []
    while len(remaining) >= minimum_tags and len(boards) < maximum_boards:
        scale = float(np.median([marker.mean_side_px for marker in remaining]))
        threshold = max(3.5, min(12.0, 0.30 * scale))
        best: tuple[int, float, list[Marker], np.ndarray] | None = None
        for marker in remaining:
            H = cv2.getPerspectiveTransform(
                marker_object_corners(marker.tag_id)[:, :2].astype(np.float32),
                marker.corners_px.astype(np.float32),
            )
            assigned, rms = assignment_for_homography(H, remaining, threshold)
            score = (len(assigned), -rms)
            if best is None or score > (best[0], -best[1]):
                best = (len(assigned), rms, assigned, H)
        if best is None or best[0] < minimum_tags:
            break
        assigned = best[2]
        for _ in range(4):
            H = fit_homography(assigned)
            if H is None:
                break
            refined, _ = assignment_for_homography(H, remaining, threshold)
            if {m.index for m in refined} == {m.index for m in assigned}:
                assigned = refined
                break
            assigned = refined
        if len(assigned) < minimum_tags:
            break
        boards.append(sorted(assigned, key=lambda marker: marker.tag_id))
        used = {marker.index for marker in assigned}
        remaining = [marker for marker in remaining if marker.index not in used]
    combined = clean + boards
    return sorted(combined, key=lambda group: -len(group))[:maximum_boards]


def estimate_pose(markers: list[Marker], K: np.ndarray, D: np.ndarray) -> dict | None:
    object_points = np.concatenate([marker_object_corners(marker.tag_id) for marker in markers])
    image_points = np.concatenate([marker.corners_px for marker in markers])
    normalized = cv2.fisheye.undistortPoints(image_points.reshape(-1, 1, 2), K, D).reshape(-1, 2)
    ok, rvec, tvec, inliers = cv2.solvePnPRansac(
        object_points,
        normalized,
        np.eye(3),
        None,
        flags=cv2.SOLVEPNP_ITERATIVE,
        reprojectionError=0.012,
        iterationsCount=300,
        confidence=0.999,
    )
    if not ok or inliers is None or len(inliers) < 4 * 4:
        return None
    ok, rvec, tvec = cv2.solvePnP(
        object_points[inliers.reshape(-1)],
        normalized[inliers.reshape(-1)],
        np.eye(3),
        None,
        rvec,
        tvec,
        True,
        flags=cv2.SOLVEPNP_ITERATIVE,
    )
    if not ok or float(tvec.reshape(3)[2]) <= 0:
        return None
    projected, _ = cv2.fisheye.projectPoints(object_points.reshape(1, -1, 3), rvec, tvec, K, D)
    residual = np.linalg.norm(projected.reshape(-1, 2) - image_points, axis=1)
    rms = float(np.sqrt(np.mean(residual**2)))
    p95 = float(np.percentile(residual, 95))
    if rms > 2.5 or p95 > 4.0:
        return None
    rotation, _ = cv2.Rodrigues(rvec)
    transform = np.eye(4)
    transform[:3, :3] = rotation
    transform[:3, 3] = tvec.reshape(3)
    return {
        "tag_count": len(markers),
        "tag_ids": [marker.tag_id for marker in markers],
        "centre_px": np.mean(image_points, axis=0).round(4).tolist(),
        "rms_px": rms,
        "median_px": float(np.median(residual)),
        "p95_px": p95,
        "T_camera_optical_from_board": transform.round(10).tolist(),
    }


def image_for_camera(capture: Path, camera: str) -> Path | None:
    files = sorted((capture / "cameras").glob(f"*_{camera}.png"))
    return files[0] if len(files) == 1 else None


def annotate(image: np.ndarray, groups: Iterable[list[Marker]]) -> np.ndarray:
    output = image.copy()
    colours = (
        (0, 255, 0),
        (0, 170, 255),
        (255, 80, 0),
        (255, 0, 255),
        (0, 255, 255),
        (255, 255, 0),
    )
    for board_index, group in enumerate(groups):
        colour = colours[board_index % len(colours)]
        all_corners = np.concatenate([marker.corners_px for marker in group])
        hull = cv2.convexHull(all_corners.astype(np.float32)).astype(np.int32)
        cv2.polylines(output, [hull], True, colour, 6, cv2.LINE_AA)
        centre = tuple(np.mean(all_corners, axis=0).astype(int))
        cv2.putText(
            output,
            f"board {board_index}: {len(group)} tags",
            centre,
            cv2.FONT_HERSHEY_SIMPLEX,
            0.9,
            colour,
            3,
            cv2.LINE_AA,
        )
    return output


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--data-root", type=Path, required=True)
    parser.add_argument("--group-list", type=Path, required=True)
    parser.add_argument("--intrinsics-root", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--preview-count", type=int, default=12)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    preview_dir = args.output / "detection_previews"
    preview_dir.mkdir(exist_ok=True)

    capture_ids = [
        line.strip() for line in args.group_list.read_text().splitlines() if line.strip()
    ]
    intrinsics = {
        camera: load_intrinsics(args.intrinsics_root / camera / "camera_info.yaml")
        for camera in CAMERAS
    }
    preview_indices = set(
        np.linspace(
            0, len(capture_ids) - 1, min(args.preview_count, len(capture_ids)), dtype=int
        ).tolist()
    )
    records, summary_rows = [], []
    raw_index = []
    raw_hashes = []
    for capture_index, capture_id in enumerate(capture_ids):
        capture = args.data_root / capture_id
        for camera in CAMERAS:
            path = image_for_camera(capture, camera)
            if path is None:
                records.append(
                    {
                        "capture_id": capture_id,
                        "camera": camera,
                        "error": "missing_or_ambiguous_image",
                        "boards": [],
                    }
                )
                continue
            image = cv2.imread(str(path), cv2.IMREAD_COLOR)
            if image is None:
                records.append(
                    {
                        "capture_id": capture_id,
                        "camera": camera,
                        "error": "unreadable_image",
                        "boards": [],
                    }
                )
                continue
            K, D, expected_size = intrinsics[camera]
            if (image.shape[1], image.shape[0]) != expected_size:
                records.append(
                    {
                        "capture_id": capture_id,
                        "camera": camera,
                        "error": "resolution_mismatch",
                        "boards": [],
                    }
                )
                continue
            markers = detect_markers(image)
            groups = separate_repeated_boards(markers)
            poses = [pose for group in groups if (pose := estimate_pose(group, K, D)) is not None]
            record = {
                "capture_id": capture_id,
                "camera": camera,
                "source_file": str(path.relative_to(args.data_root)),
                "fresh_pixel_detection": True,
                "detected_marker_count": len(markers),
                "boards": poses,
            }
            records.append(record)
            raw_index.append(record["source_file"])
            digest = hashlib.sha256()
            with path.open("rb") as image_stream:
                for chunk in iter(lambda: image_stream.read(1024 * 1024), b""):
                    digest.update(chunk)
            raw_hashes.append(f"{digest.hexdigest()}  {record['source_file']}")
            summary_rows.append(
                {
                    "capture_id": capture_id,
                    "camera": camera,
                    "markers": len(markers),
                    "boards": len(poses),
                    "best_board_tags": max([p["tag_count"] for p in poses], default=0),
                    "best_rms_px": min([p["rms_px"] for p in poses], default=float("nan")),
                }
            )
            if capture_index in preview_indices:
                cv2.imwrite(
                    str(preview_dir / f"{capture_id}_{camera}.jpg"),
                    annotate(image, groups),
                    [cv2.IMWRITE_JPEG_QUALITY, 90],
                )

    with (args.output / "board_detections.jsonl").open("w") as stream:
        for record in records:
            stream.write(json.dumps(record, ensure_ascii=False) + "\n")
    with (args.output / "detection_summary.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(summary_rows[0]))
        writer.writeheader()
        writer.writerows(summary_rows)
    (args.output / "raw_image_index.txt").write_text("\n".join(raw_index) + "\n")
    (args.output / "raw_image_sha256.txt").write_text("\n".join(raw_hashes) + "\n")
    (args.output / "source_capture_groups.txt").write_text("\n".join(capture_ids) + "\n")

    camera_summary = {}
    for camera in CAMERAS:
        rows = [row for row in summary_rows if row["camera"] == camera]
        detected = [row for row in rows if row["boards"]]
        rms = [row["best_rms_px"] for row in detected]
        camera_summary[camera] = {
            "images": len(rows),
            "images_with_board": len(detected),
            "board_instances": int(sum(row["boards"] for row in rows)),
            "median_best_reprojection_px": float(np.median(rms)) if rms else None,
            "p95_best_reprojection_px": float(np.percentile(rms, 95)) if rms else None,
        }
    report = {
        "schema_version": 1,
        "method": "fresh pixels; repeated-ID multi-homography separation; fisheye PnP",
        "capture_groups": len(capture_ids),
        "board": {
            "dictionary": "DICT_APRILTAG_16h5",
            "columns": COLS,
            "rows": ROWS,
            "outer_size_m": [OUTER_M, OUTER_M],
            "active_size_m": [ACTIVE_M, ACTIVE_M],
            "tag_size_m": TAG_SIZE_M,
            "horizontal_gap_m": GAP_X_M,
            "vertical_gap_m": GAP_Y_M,
            "same_ids_on_all_physical_boards": True,
        },
        "cameras": camera_summary,
    }
    (args.output / "DETECTION_REPORT.json").write_text(
        json.dumps(report, ensure_ascii=False, indent=2) + "\n"
    )
    print(json.dumps(report, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
