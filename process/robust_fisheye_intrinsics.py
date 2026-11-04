#!/usr/bin/env python3
"""Robust ChArUco calibration for the 1600x1200 vehicle fisheye cameras.

OpenCV's fisheye initializer can reject a complete data set when only a few
partial-board views are poorly conditioned.  This tool finds a deterministic,
well-spread seed, rejects views by independent pose/reprojection error, refits
the inliers, and reports bootstrap parameter stability.  It never installs the
result automatically.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

import cv2
import numpy as np
import yaml
from scipy.optimize import least_squares


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--input-dir", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--camera-name", required=True)
    parser.add_argument("--seed-attempts", type=int, default=36)
    parser.add_argument("--seed-views", type=int, default=28)
    parser.add_argument("--max-view-error-px", type=float, default=0.9)
    parser.add_argument("--random-seed", type=int, default=20260903)
    parser.add_argument(
        "--reference-report",
        type=Path,
        action="append",
        default=[],
        help="Accepted equidistant report(s) from identical camera hardware; "
        "used only to initialise robust per-camera fitting.",
    )
    return parser.parse_args()


def charuco_board() -> tuple[Any, Any, np.ndarray]:
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_100)
    if hasattr(cv2.aruco, "CharucoBoard_create"):
        board = cv2.aruco.CharucoBoard_create(12, 8, 0.070, 0.050, dictionary)
        points = np.asarray(board.chessboardCorners, dtype=np.float32)
    else:
        board = cv2.aruco.CharucoBoard((12, 8), 0.070, 0.050, dictionary)
        points = np.asarray(board.getChessboardCorners(), dtype=np.float32)
    return dictionary, board, points


def marker_components(marker_corners: list[np.ndarray]) -> list[list[int]]:
    """Separate multiple identical ChArUco boards visible in one image.

    The capture area contains several boards made from the same dictionary and
    IDs.  Feeding all detections to ``interpolateCornersCharuco`` silently
    combines equal IDs from different physical planes and produces unusable
    intrinsics.  Markers on one board form a connected grid: neighbouring
    marker centres are about 1.4 marker widths apart.  A scale-adaptive graph
    therefore separates boards even when their apparent sizes differ.
    """
    if not marker_corners:
        return []
    centers = []
    scales = []
    for corners in marker_corners:
        points = np.asarray(corners, dtype=np.float64).reshape(4, 2)
        centers.append(np.mean(points, axis=0))
        edges = np.linalg.norm(points - np.roll(points, 1, axis=0), axis=1)
        scales.append(max(float(np.median(edges)), 1.0))
    centers_array = np.asarray(centers)
    unseen = set(range(len(marker_corners)))
    components: list[list[int]] = []
    while unseen:
        seed = unseen.pop()
        component = [seed]
        queue = [seed]
        while queue:
            current = queue.pop()
            linked = []
            for candidate in unseen:
                distance = float(np.linalg.norm(centers_array[current] - centers_array[candidate]))
                # Allow one missing grid marker but do not bridge distant boards.
                if distance <= 3.15 * max(scales[current], scales[candidate]):
                    linked.append(candidate)
            for candidate in linked:
                unseen.remove(candidate)
                component.append(candidate)
                queue.append(candidate)
        components.append(sorted(component))
    return sorted(components, key=len, reverse=True)


def best_charuco_observation(
    gray: np.ndarray,
    dictionary: Any,
    board: Any,
) -> tuple[np.ndarray | None, np.ndarray | None, int]:
    marker_corners, marker_ids, _ = cv2.aruco.detectMarkers(gray, dictionary)
    if marker_ids is None or len(marker_ids) < 2:
        return None, None, 0
    candidates: list[tuple[int, float, np.ndarray, np.ndarray]] = []
    for component in marker_components(marker_corners):
        if len(component) < 2:
            continue
        corners_subset = [marker_corners[index] for index in component]
        ids_subset = marker_ids[np.asarray(component, dtype=int)]
        # A valid physical board cannot contain a marker ID twice.  A duplicate
        # means the adaptive graph bridged two boards, so reject that component.
        if len(np.unique(ids_subset)) != len(ids_subset):
            continue
        count, corners, ids = cv2.aruco.interpolateCornersCharuco(
            corners_subset, ids_subset, gray, board
        )
        if corners is None or ids is None or int(count) < 20:
            continue
        hull = cv2.convexHull(corners.reshape(-1, 2).astype(np.float32))
        candidates.append((int(count), float(cv2.contourArea(hull)), corners, ids))
    if not candidates:
        return None, None, len(marker_components(marker_corners))
    # Prefer corner count first and apparent area second.  This selects one
    # well-observed board rather than mixing the several boards in the garage.
    _, _, corners, ids = max(candidates, key=lambda item: (item[0], item[1]))
    return corners, ids, len(candidates)


def detect_views(input_dir: Path) -> tuple[list[dict[str, Any]], tuple[int, int], list[str]]:
    if (input_dir / "images").is_dir():
        input_dir = input_dir / "images"
    dictionary, board, board_points = charuco_board()
    views: list[dict[str, Any]] = []
    rejected: list[str] = []
    image_size: tuple[int, int] | None = None
    paths = sorted(
        path for path in input_dir.iterdir() if path.suffix.lower() in {".png", ".jpg", ".jpeg"}
    )
    for path in paths:
        gray = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
        if gray is None:
            rejected.append(path.name)
            continue
        size = (gray.shape[1], gray.shape[0])
        if image_size is None:
            image_size = size
        if size != image_size:
            raise RuntimeError(f"mixed resolution: {path.name} is {size}, expected {image_size}")
        corners, ids, board_candidates = best_charuco_observation(gray, dictionary, board)
        if corners is None or ids is None:
            rejected.append(path.name)
            continue
        image = corners.reshape(-1, 2).astype(np.float64)
        objects = board_points[ids.reshape(-1)].reshape(-1, 3).astype(np.float64)
        normalized = image / np.asarray(size, dtype=np.float64)
        hull = cv2.convexHull(normalized.astype(np.float32))
        area = max(float(cv2.contourArea(hull)), 1.0e-8)
        covariance = np.cov(normalized, rowvar=False)
        eigenvalues = np.maximum(np.linalg.eigvalsh(covariance), 0.0)
        shape_ratio = float(eigenvalues[0] / max(eigenvalues[-1], 1.0e-12))
        center = np.mean(normalized, axis=0)
        span = np.ptp(normalized, axis=0)
        quality = float(len(image) * np.sqrt(area) * np.sqrt(max(shape_ratio, 1.0e-6)))
        views.append(
            {
                "path": path,
                "object": objects,
                "image": image,
                "board_candidates": board_candidates,
                "center": center,
                "span": span,
                "area": area,
                "shape_ratio": shape_ratio,
                "quality": quality,
            }
        )
    if image_size is None:
        raise RuntimeError("no readable calibration images")
    if len(views) < 30:
        raise RuntimeError(f"only {len(views)} valid ChArUco views; at least 30 required")
    return views, image_size, rejected


def fish_arrays(
    views: list[dict[str, Any]], indices: list[int]
) -> tuple[list[np.ndarray], list[np.ndarray]]:
    objects = [views[index]["object"].reshape(1, -1, 3) for index in indices]
    images = [views[index]["image"].reshape(1, -1, 2) for index in indices]
    return objects, images


def calibrate(
    views: list[dict[str, Any]],
    indices: list[int],
    size: tuple[int, int],
    focal: float,
    camera_guess: np.ndarray | None = None,
    distortion_guess: np.ndarray | None = None,
) -> tuple[float, np.ndarray, np.ndarray]:
    objects, images = fish_arrays(views, indices)
    if camera_guess is None:
        camera = np.asarray(
            [[focal, 0.0, size[0] / 2.0], [0.0, focal, size[1] / 2.0], [0.0, 0.0, 1.0]],
            dtype=np.float64,
        )
    else:
        camera = camera_guess.astype(np.float64).copy()
    distortion = (
        np.zeros((4, 1), dtype=np.float64)
        if distortion_guess is None
        else distortion_guess.reshape(4, 1).astype(np.float64).copy()
    )
    flags = (
        cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC
        | cv2.fisheye.CALIB_FIX_SKEW
        | cv2.fisheye.CALIB_USE_INTRINSIC_GUESS
    )
    rms, camera, distortion, _, _ = cv2.fisheye.calibrate(
        objects,
        images,
        size,
        camera,
        distortion,
        flags=flags,
        criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_COUNT, 180, 1.0e-10),
    )
    return float(rms), camera, distortion.reshape(-1)


def project_error(view: dict[str, Any], camera: np.ndarray, distortion: np.ndarray) -> float:
    pose = estimate_pose(view, camera, distortion, refine=True)
    if pose is None:
        return float("inf")
    rvec, tvec = pose
    projected, _ = cv2.fisheye.projectPoints(
        view["object"].reshape(1, -1, 3), rvec, tvec, camera, distortion
    )
    delta = projected.reshape(-1, 2) - view["image"]
    return float(np.sqrt(np.mean(np.sum(delta * delta, axis=1))))


def estimate_pose(
    view: dict[str, Any], camera: np.ndarray, distortion: np.ndarray, *, refine: bool = True
) -> tuple[np.ndarray, np.ndarray] | None:
    normalized = cv2.fisheye.undistortPoints(view["image"].reshape(-1, 1, 2), camera, distortion)
    ok, rvec, tvec = cv2.solvePnP(
        view["object"], normalized, np.eye(3), None, flags=cv2.SOLVEPNP_ITERATIVE
    )
    if not ok or float(tvec[2, 0]) <= 0.0:
        return None
    if refine:
        initial = np.concatenate((rvec.reshape(3), tvec.reshape(3)))

        def residual(parameters: np.ndarray) -> np.ndarray:
            projected, _ = cv2.fisheye.projectPoints(
                view["object"].reshape(1, -1, 3),
                parameters[:3].reshape(3, 1),
                parameters[3:].reshape(3, 1),
                camera,
                distortion,
            )
            return (projected.reshape(-1, 2) - view["image"]).reshape(-1)

        try:
            solution = least_squares(
                residual,
                initial,
                loss="soft_l1",
                f_scale=0.7,
                max_nfev=80,
                ftol=1.0e-10,
                xtol=1.0e-10,
                gtol=1.0e-10,
            )
            if solution.success and float(solution.x[5]) > 0.0:
                rvec = solution.x[:3].reshape(3, 1)
                tvec = solution.x[3:].reshape(3, 1)
        except (ValueError, cv2.error):
            pass
    return rvec, tvec


def alternating_calibrate(
    views: list[dict[str, Any]],
    indices: list[int],
    size: tuple[int, int],
    focal: float,
    camera_guess: np.ndarray | None = None,
    distortion_guess: np.ndarray | None = None,
    cycles: int = 5,
) -> tuple[float, np.ndarray, np.ndarray, list[int]]:
    """Alternate planar pose estimation with robust 8-parameter optimisation."""
    if camera_guess is None:
        camera = np.asarray(
            [[focal, 0.0, size[0] / 2.0], [0.0, focal, size[1] / 2.0], [0.0, 0.0, 1.0]],
            dtype=np.float64,
        )
    else:
        camera = camera_guess.astype(np.float64).copy()
    distortion = (
        np.asarray([0.03, -0.01, 0.0, 0.0], dtype=np.float64)
        if distortion_guess is None
        else distortion_guess.reshape(-1).astype(np.float64).copy()
    )
    used = list(indices)
    for _ in range(cycles):
        poses: list[tuple[int, np.ndarray, np.ndarray]] = []
        for index in used:
            pose = estimate_pose(views[index], camera, distortion)
            if pose is not None:
                poses.append((index, pose[0], pose[1]))
        if len(poses) < min(20, max(12, int(len(indices) * 0.70))):
            raise RuntimeError("too few valid planar poses for fisheye optimisation")
        used = [item[0] for item in poses]
        initial = np.asarray(
            [camera[0, 0], camera[1, 1], camera[0, 2], camera[1, 2], *distortion.tolist()],
            dtype=np.float64,
        )
        lower = np.asarray(
            [0.15 * size[0], 0.15 * size[0], 0.30 * size[0], 0.25 * size[1], -1.0, -1.0, -0.5, -0.5]
        )
        upper = np.asarray(
            [1.20 * size[0], 1.20 * size[0], 0.70 * size[0], 0.75 * size[1], 1.0, 1.0, 0.5, 0.5]
        )
        initial = np.clip(initial, lower + 1.0e-8, upper - 1.0e-8)

        def residual(parameters: np.ndarray) -> np.ndarray:
            trial_camera = np.asarray(
                [
                    [parameters[0], 0.0, parameters[2]],
                    [0.0, parameters[1], parameters[3]],
                    [0.0, 0.0, 1.0],
                ],
                dtype=np.float64,
            )
            trial_distortion = parameters[4:8]
            chunks = []
            for index, rvec, tvec in poses:
                projected, _ = cv2.fisheye.projectPoints(
                    views[index]["object"].reshape(1, -1, 3),
                    rvec,
                    tvec,
                    trial_camera,
                    trial_distortion,
                )
                chunks.append((projected.reshape(-1, 2) - views[index]["image"]).reshape(-1))
            return np.concatenate(chunks)

        solution = least_squares(
            residual,
            initial,
            bounds=(lower, upper),
            loss="soft_l1",
            f_scale=0.7,
            x_scale="jac",
            max_nfev=100,
            ftol=1.0e-9,
            xtol=1.0e-9,
            gtol=1.0e-9,
        )
        camera = np.asarray(
            [
                [solution.x[0], 0.0, solution.x[2]],
                [0.0, solution.x[1], solution.x[3]],
                [0.0, 0.0, 1.0],
            ],
            dtype=np.float64,
        )
        distortion = solution.x[4:8]
    final_errors = np.asarray([project_error(views[index], camera, distortion) for index in used])
    rms = float(np.sqrt(np.mean(final_errors * final_errors)))
    return rms, camera, distortion, used


def sane(camera: np.ndarray, distortion: np.ndarray, size: tuple[int, int]) -> bool:
    fx, fy = float(camera[0, 0]), float(camera[1, 1])
    cx, cy = float(camera[0, 2]), float(camera[1, 2])
    return (
        0.15 * size[0] < fx < 1.2 * size[0]
        and 0.15 * size[0] < fy < 1.2 * size[0]
        and 0.75 < fx / fy < 1.25
        and 0.30 * size[0] < cx < 0.70 * size[0]
        and 0.25 * size[1] < cy < 0.75 * size[1]
        and np.all(np.isfinite(distortion))
        and float(np.max(np.abs(distortion))) < 5.0
    )


def diverse_sample(
    views: list[dict[str, Any]], pool: list[int], count: int, rng: np.random.Generator
) -> list[int]:
    features = np.asarray(
        [
            [
                views[i]["center"][0],
                views[i]["center"][1],
                np.sqrt(views[i]["area"]),
                views[i]["span"][0],
                views[i]["span"][1],
            ]
            for i in pool
        ],
        dtype=np.float64,
    )
    scale = np.std(features, axis=0)
    features /= np.where(scale > 1.0e-6, scale, 1.0)
    quality_order = sorted(range(len(pool)), key=lambda j: views[pool[j]]["quality"], reverse=True)
    selected = [quality_order[int(rng.integers(0, min(8, len(quality_order))))]]
    while len(selected) < min(count, len(pool)):
        distances = np.min(
            np.linalg.norm(
                features[:, None, :] - features[np.asarray(selected)][None, :, :], axis=2
            ),
            axis=1,
        )
        distances[np.asarray(selected)] = -1.0
        # Randomise among the farthest candidates to produce independent seeds.
        candidates = np.argsort(distances)[-min(8, len(pool) - len(selected)) :]
        selected.append(int(candidates[int(rng.integers(0, len(candidates)))]))
    return sorted(pool[index] for index in selected)


def reference_seed(
    paths: list[Path], size: tuple[int, int]
) -> tuple[np.ndarray, np.ndarray] | None:
    cameras: list[np.ndarray] = []
    distortions: list[np.ndarray] = []
    for path in paths:
        payload = json.loads(path.read_text(encoding="utf-8"))
        if payload.get("model") != "equidistant":
            raise RuntimeError(f"reference is not equidistant: {path}")
        if tuple(payload.get("resolution", [])) != tuple(size):
            raise RuntimeError(f"reference resolution mismatch: {path}")
        camera = np.asarray(payload["camera_matrix"], dtype=np.float64)
        distortion = np.asarray(payload["distortion"], dtype=np.float64).reshape(-1)
        if camera.shape != (3, 3) or distortion.shape != (4,):
            raise RuntimeError(f"invalid reference matrices: {path}")
        cameras.append(camera)
        distortions.append(distortion)
    if not cameras:
        return None
    camera = np.mean(cameras, axis=0)
    camera[2] = [0.0, 0.0, 1.0]
    return camera, np.mean(distortions, axis=0)


def find_candidate(
    views: list[dict[str, Any]], size: tuple[int, int], args: argparse.Namespace
) -> tuple[np.ndarray, np.ndarray, list[int], dict[str, Any]]:
    rng = np.random.default_rng(args.random_seed)
    scores = np.asarray([view["quality"] for view in views])
    threshold = float(np.quantile(scores, 0.20))
    pool = [
        index
        for index, view in enumerate(views)
        if view["quality"] >= threshold and len(view["image"]) >= 24
    ]
    if len(pool) < args.seed_views:
        pool = list(np.argsort(scores)[-max(args.seed_views, 30) :])
    focal_guesses = [0.26 * size[0], 0.30 * size[0], 0.34 * size[0], 0.40 * size[0], 0.50 * size[0]]
    candidates: list[dict[str, Any]] = []
    shared_seed = reference_seed(args.reference_report, size)
    if shared_seed is not None:
        seed_camera, seed_distortion = shared_seed
        seed_errors = np.asarray(
            [project_error(view, seed_camera, seed_distortion) for view in views]
        )
        ordered = np.argsort(seed_errors)
        for count in [12, 16, 20, 24, 30, 40, 50, 60, 70]:
            indices = [
                int(index)
                for index in ordered[: min(count, len(ordered))]
                if np.isfinite(seed_errors[index])
            ]
            if len(indices) < 12:
                continue
            try:
                rms, camera, distortion = calibrate(
                    views, indices, size, float(seed_camera[0, 0]), seed_camera, seed_distortion
                )
            except cv2.error:
                continue
            if rms > 2.0 or not sane(camera, distortion, size):
                continue
            errors = np.asarray([project_error(view, camera, distortion) for view in views])
            finite = errors[np.isfinite(errors)]
            if len(finite) < 30:
                continue
            accepted_count = int(np.sum(errors <= args.max_view_error_px))
            candidates.append(
                {
                    "rms": rms,
                    "camera": camera,
                    "distortion": distortion,
                    "indices": indices,
                    "probe_median": float(np.median(finite)),
                    "probe_p95": float(np.quantile(finite, 0.95)),
                    "accepted_count": accepted_count,
                    "method": "identical_hardware_reference_ransac",
                }
            )
    for attempt in range(args.seed_attempts):
        indices = diverse_sample(views, pool, args.seed_views, rng)
        focal = focal_guesses[attempt % len(focal_guesses)]
        try:
            rms, camera, distortion = calibrate(views, indices, size, focal)
        except cv2.error:
            continue
        if rms > 2.0 or not sane(camera, distortion, size):
            continue
        probe = diverse_sample(views, list(range(len(views))), min(60, len(views)), rng)
        errors = np.asarray([project_error(views[index], camera, distortion) for index in probe])
        if not np.all(np.isfinite(errors)):
            continue
        candidates.append(
            {
                "rms": rms,
                "camera": camera,
                "distortion": distortion,
                "indices": indices,
                "probe_median": float(np.median(errors)),
                "probe_p95": float(np.quantile(errors, 0.95)),
            }
        )
    if not candidates:
        # OpenCV may reject an otherwise useful set because a single planar
        # homography is ill-conditioned.  Alternate robust intrinsic fitting
        # with per-view poses instead of disabling all validation.
        for attempt, focal in enumerate(focal_guesses):
            indices = diverse_sample(views, pool, min(max(args.seed_views, 32), len(pool)), rng)
            try:
                rms, camera, distortion, indices = alternating_calibrate(
                    views, indices, size, focal
                )
            except (cv2.error, RuntimeError, ValueError) as error:
                print(f"alternating seed f={focal:.1f} failed: {error}", flush=True)
                continue
            print(
                f"alternating seed f={focal:.1f}: rms={rms:.4f}, "
                f"K=({camera[0, 0]:.2f},{camera[1, 1]:.2f},"
                f"{camera[0, 2]:.2f},{camera[1, 2]:.2f}), "
                f"D={distortion.tolist()}, sane={sane(camera, distortion, size)}",
                flush=True,
            )
            if rms > 2.0 or not sane(camera, distortion, size):
                continue
            probe = diverse_sample(views, list(range(len(views))), min(60, len(views)), rng)
            errors = np.asarray(
                [project_error(views[index], camera, distortion) for index in probe]
            )
            if not np.all(np.isfinite(errors)):
                continue
            candidates.append(
                {
                    "rms": rms,
                    "camera": camera,
                    "distortion": distortion,
                    "indices": indices,
                    "probe_median": float(np.median(errors)),
                    "probe_p95": float(np.quantile(errors, 0.95)),
                    "method": "alternating_robust",
                }
            )
    if not candidates:
        raise RuntimeError("no physically plausible fisheye seed converged")
    best = min(
        candidates,
        key=lambda item: (
            -int(item.get("accepted_count", 0)),
            item["probe_median"],
            item["probe_p95"],
            item["rms"],
        ),
    )
    return (
        best["camera"],
        best["distortion"],
        best["indices"],
        {
            "eligible_pool": len(pool),
            "converged_candidates": len(candidates),
            "seed_rms_px": best["rms"],
            "seed_probe_median_px": best["probe_median"],
            "seed_probe_p95_px": best["probe_p95"],
            "method": best.get("method", "opencv_fisheye"),
        },
    )


def matrix_block(values: np.ndarray) -> dict[str, Any]:
    array = np.asarray(values)
    return {
        "rows": int(array.shape[0]),
        "cols": int(array.shape[1]),
        "data": [float(value) for value in array.reshape(-1)],
    }


def save_preview(
    path: Path,
    views: list[dict[str, Any]],
    camera: np.ndarray,
    distortion: np.ndarray,
    size: tuple[int, int],
) -> None:
    picks = np.linspace(0, len(views) - 1, 6, dtype=int)
    tiles = []
    new_camera = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(
        camera, distortion, size, np.eye(3), balance=0.45
    )
    map1, map2 = cv2.fisheye.initUndistortRectifyMap(
        camera, distortion, np.eye(3), new_camera, size, cv2.CV_16SC2
    )
    for index in picks:
        image = cv2.imread(str(views[int(index)]["path"]))
        corrected = cv2.remap(image, map1, map2, cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
        pair = np.hstack((image, corrected))
        tiles.append(cv2.resize(pair, (800, 300), interpolation=cv2.INTER_AREA))
    preview = np.vstack(tiles)
    cv2.imwrite(str(path), preview, [cv2.IMWRITE_JPEG_QUALITY, 88])


def save_reprojection_preview(
    path: Path,
    views: list[dict[str, Any]],
    indices: list[int],
    camera: np.ndarray,
    distortion: np.ndarray,
) -> None:
    """Render measured (green) and reprojected (red) corners for visual QA."""
    if not indices:
        return
    picks = np.asarray(indices, dtype=int)[
        np.linspace(0, len(indices) - 1, min(6, len(indices)), dtype=int)
    ]
    tiles = []
    for index in picks:
        view = views[int(index)]
        image = cv2.imread(str(view["path"]))
        pose = estimate_pose(view, camera, distortion, refine=True)
        if image is None or pose is None:
            continue
        projected, _ = cv2.fisheye.projectPoints(
            view["object"].reshape(1, -1, 3), pose[0], pose[1], camera, distortion
        )
        predicted = projected.reshape(-1, 2)
        measured = view["image"]
        error = float(np.sqrt(np.mean(np.sum((predicted - measured) ** 2, axis=1))))
        for observed, estimate in zip(measured, predicted):
            observed_point = tuple(np.rint(observed).astype(int))
            estimate_point = tuple(np.rint(estimate).astype(int))
            cv2.circle(image, observed_point, 3, (0, 255, 0), -1, lineType=cv2.LINE_AA)
            cv2.circle(image, estimate_point, 3, (0, 0, 255), 1, lineType=cv2.LINE_AA)
            cv2.line(image, observed_point, estimate_point, (0, 255, 255), 1, lineType=cv2.LINE_AA)
        cv2.putText(
            image,
            f"RMS {error:.3f}px",
            (24, 48),
            cv2.FONT_HERSHEY_SIMPLEX,
            1.0,
            (0, 255, 255),
            2,
            lineType=cv2.LINE_AA,
        )
        tiles.append(cv2.resize(image, (640, 480), interpolation=cv2.INTER_AREA))
    if tiles:
        while len(tiles) < 6:
            tiles.append(np.zeros_like(tiles[0]))
        preview = np.vstack((np.hstack(tiles[:3]), np.hstack(tiles[3:6])))
        cv2.imwrite(str(path), preview, [cv2.IMWRITE_JPEG_QUALITY, 92])


def main() -> int:
    args = parse_args()
    views, size, detection_rejected = detect_views(args.input_dir)
    camera, distortion, seed_indices, search = find_candidate(views, size, args)
    initial_errors = np.asarray([project_error(view, camera, distortion) for view in views])
    inliers = [
        index
        for index, error in enumerate(initial_errors)
        if np.isfinite(error) and error <= args.max_view_error_px
    ]
    if len(inliers) < 30:
        inliers = np.argsort(initial_errors)[: min(50, len(views))].tolist()
    # Limit to a diverse, well-conditioned final set.  More near-duplicate views
    # add computation but do not add observability.
    rng = np.random.default_rng(args.random_seed + 1)
    final_indices = diverse_sample(views, inliers, min(70, len(inliers)), rng)
    try:
        final_rms, final_camera, final_distortion = calibrate(
            views, final_indices, size, float(camera[0, 0]), camera, distortion
        )
        if sane(final_camera, final_distortion, size):
            camera, distortion = final_camera, final_distortion
        else:
            raise RuntimeError("OpenCV refit left physical parameter bounds")
    except (cv2.error, RuntimeError):
        try:
            final_rms, camera, distortion, final_indices = alternating_calibrate(
                views, final_indices, size, float(camera[0, 0]), camera, distortion
            )
        except (cv2.error, RuntimeError, ValueError):
            final_rms = float("nan")
            final_indices = seed_indices
    errors = np.asarray([project_error(view, camera, distortion) for view in views])
    accepted_indices = [
        index
        for index, error in enumerate(errors)
        if np.isfinite(error) and error <= args.max_view_error_px
    ]

    bootstrap: list[dict[str, Any]] = []
    base = accepted_indices if len(accepted_indices) >= 30 else final_indices
    for fold in range(5):
        shuffled = np.asarray(base, dtype=int).copy()
        np.random.default_rng(args.random_seed + 100 + fold).shuffle(shuffled)
        train = sorted(shuffled[: max(24, int(len(shuffled) * 0.8))].tolist())
        held = sorted(set(base) - set(train))
        try:
            try:
                fold_rms, fold_camera, fold_distortion = calibrate(
                    views, train, size, float(camera[0, 0]), camera, distortion
                )
            except cv2.error:
                fold_rms, fold_camera, fold_distortion, train = alternating_calibrate(
                    views, train, size, float(camera[0, 0]), camera, distortion, cycles=4
                )
            if not sane(fold_camera, fold_distortion, size):
                continue
            held_errors = np.asarray(
                [project_error(views[i], fold_camera, fold_distortion) for i in held]
            )
            finite_held = held_errors[np.isfinite(held_errors)]
            if len(finite_held) < max(3, int(len(held_errors) * 0.90)):
                continue
            bootstrap.append(
                {
                    "fold": fold,
                    "train_views": len(train),
                    "held_views": len(held),
                    "train_rms_px": fold_rms,
                    "held_median_px": float(np.median(finite_held)),
                    "held_p95_px": float(np.quantile(finite_held, 0.95)),
                    "held_pose_failures": int(len(held_errors) - len(finite_held)),
                    "camera_matrix": fold_camera.tolist(),
                    "distortion": fold_distortion.tolist(),
                }
            )
        except (cv2.error, RuntimeError, ValueError):
            continue

    all_points = np.vstack([view["image"] for view in views])
    coverage = {
        "x_min_fraction": float(np.min(all_points[:, 0]) / size[0]),
        "x_max_fraction": float(np.max(all_points[:, 0]) / size[0]),
        "y_min_fraction": float(np.min(all_points[:, 1]) / size[1]),
        "y_max_fraction": float(np.max(all_points[:, 1]) / size[1]),
    }
    matrices = np.asarray([item["camera_matrix"] for item in bootstrap])
    focal_spread_pct = (
        float(
            np.max(
                np.ptp(matrices[:, (0, 1), (0, 1)], axis=0)
                / np.asarray([camera[0, 0], camera[1, 1]])
                * 100.0
            )
        )
        if len(matrices)
        else float("inf")
    )
    principal_spread_px = (
        float(np.max(np.ptp(matrices[:, (0, 1), (2, 2)], axis=0)))
        if len(matrices)
        else float("inf")
    )
    held_p95_max = (
        max(float(item["held_p95_px"]) for item in bootstrap) if bootstrap else float("inf")
    )
    accepted_errors = errors[accepted_indices] if accepted_indices else errors
    finite_errors = errors[np.isfinite(errors)]
    if not len(finite_errors):
        raise RuntimeError("all final per-view pose evaluations failed")
    quality_pass = bool(
        len(accepted_indices) >= 30
        and len(bootstrap) >= 4
        and float(np.median(accepted_errors)) <= 0.65
        and float(np.quantile(accepted_errors, 0.95)) <= args.max_view_error_px
        and held_p95_max <= 1.2
        and focal_spread_pct <= 2.0
        and principal_spread_px <= 15.0
        and coverage["x_min_fraction"] < 0.18
        and coverage["x_max_fraction"] > 0.82
        and coverage["y_min_fraction"] < 0.18
        and coverage["y_max_fraction"] > 0.82
    )

    output = args.output_dir / args.camera_name
    output.mkdir(parents=True, exist_ok=True)
    projection = np.zeros((3, 4), dtype=float)
    projection[:, :3] = camera
    yaml_data = {
        "image_width": size[0],
        "image_height": size[1],
        "camera_name": args.camera_name,
        "camera_matrix": matrix_block(camera),
        "distortion_model": "equidistant",
        "distortion_coefficients": matrix_block(distortion.reshape(1, -1)),
        "rectification_matrix": matrix_block(np.eye(3)),
        "projection_matrix": matrix_block(projection),
    }
    (output / "camera_info.yaml").write_text(
        yaml.safe_dump(yaml_data, sort_keys=False), encoding="utf-8"
    )
    report = {
        "schema_version": 1,
        "camera_name": args.camera_name,
        "input_directory": str(args.input_dir),
        "resolution": list(size),
        "model": "equidistant",
        "quality_pass": quality_pass,
        "target": {
            "type": "charuco",
            "squares": [12, 8],
            "square_length_m": 0.070,
            "marker_length_m": 0.050,
            "dictionary": "DICT_5X5_100",
        },
        "images": {
            "read": len(views) + len(detection_rejected),
            "detected": len(views),
            "detection_rejected": detection_rejected,
            "final_fit": len(final_indices),
            "accepted_by_error": len(accepted_indices),
        },
        "search": search,
        "final_fit_rms_px": (float(final_rms) if np.isfinite(final_rms) else None),
        "all_view_error_px": {
            "median": float(np.median(finite_errors)),
            "p95": float(np.quantile(finite_errors, 0.95)),
            "maximum": float(np.max(finite_errors)),
            "pose_failures": int(len(errors) - len(finite_errors)),
        },
        "accepted_view_error_px": {
            "median": float(np.median(accepted_errors)),
            "p95": float(np.quantile(accepted_errors, 0.95)),
            "maximum": float(np.max(accepted_errors)),
        },
        "accepted_images": [views[index]["path"].name for index in accepted_indices],
        "rejected_by_error": {
            views[index]["path"].name: (
                float(errors[index]) if np.isfinite(errors[index]) else None
            )
            for index in range(len(views))
            if index not in accepted_indices
        },
        "coverage": coverage,
        "camera_matrix": camera.tolist(),
        "distortion": distortion.tolist(),
        "bootstrap": bootstrap,
        "stability": {
            "successful_folds": len(bootstrap),
            "focal_range_percent": (focal_spread_pct if np.isfinite(focal_spread_pct) else None),
            "principal_point_range_px": (
                principal_spread_px if np.isfinite(principal_spread_px) else None
            ),
            "maximum_held_out_p95_px": (held_p95_max if np.isfinite(held_p95_max) else None),
        },
    }
    (output / "report.json").write_text(
        json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8"
    )
    save_preview(output / "undistortion_preview.jpg", views, camera, distortion, size)
    save_reprojection_preview(
        output / "reprojection_preview.jpg", views, accepted_indices, camera, distortion
    )
    print(
        json.dumps(
            {
                "camera_name": args.camera_name,
                "quality_pass": quality_pass,
                "detected": len(views),
                "accepted": len(accepted_indices),
                "median_px": report["accepted_view_error_px"]["median"],
                "p95_px": report["accepted_view_error_px"]["p95"],
                "bootstrap_folds": len(bootstrap),
                "focal_spread_percent": (
                    focal_spread_pct if np.isfinite(focal_spread_pct) else None
                ),
                "principal_spread_px": (
                    principal_spread_px if np.isfinite(principal_spread_px) else None
                ),
                "output": str(output),
            },
            ensure_ascii=False,
        )
    )
    return 0 if quality_pass else 3


if __name__ == "__main__":
    raise SystemExit(main())
