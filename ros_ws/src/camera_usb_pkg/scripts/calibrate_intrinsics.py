#!/usr/bin/env python3
"""Calibrate one camera from checkerboard or ChArUco images."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import re

import cv2
import numpy as np


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--input-dir", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--camera-name", required=True)
    parser.add_argument("--target", choices=("charuco", "checkerboard"), default="charuco")
    parser.add_argument("--squares-x", type=int, default=12)
    parser.add_argument("--squares-y", type=int, default=8)
    parser.add_argument("--square-length-m", type=float, default=0.070)
    parser.add_argument("--marker-length-m", type=float, default=0.050)
    parser.add_argument("--dictionary", default="DICT_5X5_100")
    parser.add_argument("--minimum-corners", type=int, default=20)
    parser.add_argument("--board-cols", type=int, help="checkerboard internal columns")
    parser.add_argument("--board-rows", type=int, help="checkerboard internal rows")
    parser.add_argument("--square-size-m", type=float, help="checkerboard square size")
    parser.add_argument("--model", choices=("pinhole", "fisheye"), default="pinhole")
    parser.add_argument("--rational-model", action="store_true")
    parser.add_argument("--fix-principal-point", action="store_true")
    parser.add_argument("--fix-aspect-ratio", action="store_true")
    parser.add_argument("--zero-tangent-distortion", action="store_true")
    parser.add_argument("--fix-k3", action="store_true")
    parser.add_argument(
        "--disable-fisheye-condition-check",
        action="store_true",
        help="diagnostic only; validate parameter stability before accepting",
    )
    parser.add_argument("--max-view-error-px", type=float, default=0.8)
    parser.add_argument("--minimum-views", type=int, default=20)
    parser.add_argument(
        "--maximum-views",
        type=int,
        default=100,
        help="deterministic upper bound before robust calibration",
    )
    return parser.parse_args()


def make_charuco(args: argparse.Namespace):
    if not hasattr(cv2, "aruco") or not hasattr(cv2.aruco, args.dictionary):
        raise SystemExit(f"OpenCV ArUco dictionary is unavailable: {args.dictionary}")
    dictionary_id = getattr(cv2.aruco, args.dictionary)
    if hasattr(cv2.aruco, "getPredefinedDictionary"):
        dictionary = cv2.aruco.getPredefinedDictionary(dictionary_id)
    else:
        dictionary = cv2.aruco.Dictionary_get(dictionary_id)
    if hasattr(cv2.aruco, "CharucoBoard_create"):
        board = cv2.aruco.CharucoBoard_create(
            args.squares_x,
            args.squares_y,
            args.square_length_m,
            args.marker_length_m,
            dictionary,
        )
    else:
        board = cv2.aruco.CharucoBoard(
            (args.squares_x, args.squares_y),
            args.square_length_m,
            args.marker_length_m,
            dictionary,
        )
    return dictionary, board


def board_corner_points(board) -> np.ndarray:
    if hasattr(board, "getChessboardCorners"):
        return np.asarray(board.getChessboardCorners(), dtype=np.float32)
    return np.asarray(board.chessboardCorners, dtype=np.float32)


def detect_charuco(gray: np.ndarray, dictionary, board):
    marker_corners, marker_ids, _ = cv2.aruco.detectMarkers(gray, dictionary)
    if marker_ids is None or len(marker_ids) < 2:
        return None, None
    count, corners, ids = cv2.aruco.interpolateCornersCharuco(
        marker_corners, marker_ids, gray, board
    )
    if corners is None or ids is None or int(count) < 1:
        return None, None
    return corners.astype(np.float32), ids.reshape(-1).astype(np.int32)


def matrix_block(values: np.ndarray) -> str:
    flat = ", ".join(f"{float(value):.12g}" for value in values.reshape(-1))
    rows, cols = values.shape
    return f"  rows: {rows}\n  cols: {cols}\n  data: [{flat}]"


def calibrate_pinhole(
    objects: list[np.ndarray],
    images: list[np.ndarray],
    size: tuple[int, int],
    flags: int,
) -> tuple[float, np.ndarray, np.ndarray, list[np.ndarray], list[np.ndarray], np.ndarray]:
    result = cv2.calibrateCameraExtended(objects, images, size, None, None, flags=flags)
    rms, camera, distortion, rvecs, tvecs, _, _, per_view = result
    return float(rms), camera, distortion.reshape(-1), rvecs, tvecs, per_view.reshape(-1)


def calibrate_fisheye(
    objects: list[np.ndarray],
    images: list[np.ndarray],
    size: tuple[int, int],
    check_condition: bool,
) -> tuple[float, np.ndarray, np.ndarray, list[np.ndarray], list[np.ndarray], np.ndarray]:
    fish_objects = [item.reshape(1, -1, 3).astype(np.float64) for item in objects]
    fish_images = [item.reshape(1, -1, 2).astype(np.float64) for item in images]
    camera = cv2.initCameraMatrix2D(
        [item.astype(np.float32) for item in objects],
        [item.astype(np.float32) for item in images],
        size,
        1.0,
    )
    camera = np.asarray(camera, dtype=np.float64)
    camera[0, 1] = 0.0
    distortion = np.zeros((4, 1), dtype=np.float64)
    flags = (
        cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC
        | cv2.fisheye.CALIB_FIX_SKEW
        | cv2.fisheye.CALIB_USE_INTRINSIC_GUESS
    )
    if check_condition:
        flags |= cv2.fisheye.CALIB_CHECK_COND
    rms, camera, distortion, rvecs, tvecs = cv2.fisheye.calibrate(
        fish_objects,
        fish_images,
        size,
        camera,
        distortion,
        flags=flags,
        criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_COUNT, 150, 1.0e-9),
    )
    errors = []
    for object_points, image_points, rvec, tvec in zip(fish_objects, fish_images, rvecs, tvecs):
        projected, _ = cv2.fisheye.projectPoints(object_points, rvec, tvec, camera, distortion)
        delta = projected.reshape(-1, 2) - image_points.reshape(-1, 2)
        errors.append(float(np.sqrt(np.mean(np.sum(delta * delta, axis=1)))))
    return float(rms), camera, distortion.reshape(-1), rvecs, tvecs, np.asarray(errors)


def view_condition_score(image_points: np.ndarray, size: tuple[int, int]) -> float:
    points = image_points.reshape(-1, 2).astype(np.float64)
    if len(points) < 4:
        return 0.0
    normalized = points / np.asarray(size, dtype=np.float64)
    covariance = np.cov(normalized, rowvar=False)
    eigenvalues = np.linalg.eigvalsh(covariance)
    if eigenvalues[-1] <= 1.0e-12:
        return 0.0
    spread_ratio = max(0.0, float(eigenvalues[0] / eigenvalues[-1]))
    hull = cv2.convexHull(normalized.astype(np.float32))
    normalized_area = max(0.0, float(cv2.contourArea(hull)))
    return spread_ratio * np.sqrt(normalized_area)


def main() -> int:
    args = parse_args()
    if args.minimum_views < 8 or args.max_view_error_px <= 0.0:
        raise SystemExit("minimum views must be >=8 and error threshold must be positive")
    if args.maximum_views < args.minimum_views:
        raise SystemExit("maximum views must be greater than or equal to minimum views")
    if args.rational_model and args.model == "fisheye":
        raise SystemExit("--rational-model cannot be combined with --model fisheye")
    pinhole_constraints = {
        "fix_principal_point": args.fix_principal_point,
        "fix_aspect_ratio": args.fix_aspect_ratio,
        "zero_tangent_distortion": args.zero_tangent_distortion,
        "fix_k3": args.fix_k3,
    }
    if args.model == "fisheye" and any(pinhole_constraints.values()):
        raise SystemExit("pinhole constraint flags cannot be combined with --model fisheye")
    pinhole_flags = cv2.CALIB_RATIONAL_MODEL if args.rational_model else 0
    if args.fix_principal_point:
        pinhole_flags |= cv2.CALIB_FIX_PRINCIPAL_POINT
    if args.fix_aspect_ratio:
        pinhole_flags |= cv2.CALIB_FIX_ASPECT_RATIO
    if args.zero_tangent_distortion:
        pinhole_flags |= cv2.CALIB_ZERO_TANGENT_DIST
    if args.fix_k3:
        pinhole_flags |= cv2.CALIB_FIX_K3

    dictionary = board = charuco_points = None
    checker_template = None
    target_report: dict[str, object]
    if args.target == "charuco":
        if args.squares_x < 3 or args.squares_y < 3:
            raise SystemExit("ChArUco board must have at least 3x3 squares")
        if not 0.0 < args.marker_length_m < args.square_length_m:
            raise SystemExit("marker length must be positive and smaller than square length")
        if args.minimum_corners < 8:
            raise SystemExit("minimum ChArUco corners must be >=8")
        dictionary, board = make_charuco(args)
        charuco_points = board_corner_points(board)
        target_report = {
            "type": "charuco",
            "squares": [args.squares_x, args.squares_y],
            "square_length_m": args.square_length_m,
            "marker_length_m": args.marker_length_m,
            "dictionary": args.dictionary,
            "minimum_corners_per_view": args.minimum_corners,
        }
    else:
        if not args.board_cols or not args.board_rows or not args.square_size_m:
            raise SystemExit("checkerboard requires --board-cols, --board-rows and --square-size-m")
        checker_template = np.zeros((args.board_cols * args.board_rows, 3), np.float32)
        checker_template[:, :2] = np.mgrid[0 : args.board_cols, 0 : args.board_rows].T.reshape(
            -1, 2
        )
        checker_template *= args.square_size_m
        target_report = {
            "type": "checkerboard",
            "internal_corners": [args.board_cols, args.board_rows],
            "square_size_m": args.square_size_m,
        }

    paths = sorted(
        path
        for path in args.input_dir.iterdir()
        if path.suffix.lower() in {".png", ".jpg", ".jpeg"}
    )
    objects: list[np.ndarray] = []
    images: list[np.ndarray] = []
    accepted_paths: list[Path] = []
    corner_counts: list[int] = []
    condition_scores: list[float] = []
    detection_rejected: list[str] = []
    image_size: tuple[int, int] | None = None
    checker_flags = cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_EXHAUSTIVE | cv2.CALIB_CB_ACCURACY
    for path in paths:
        gray = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
        if gray is None:
            detection_rejected.append(path.name)
            continue
        size = (gray.shape[1], gray.shape[0])
        if image_size is None:
            image_size = size
        if size != image_size:
            raise SystemExit(f"mixed image resolutions: {path} is {size}, expected {image_size}")

        if args.target == "charuco":
            corners, ids = detect_charuco(gray, dictionary, board)
            if corners is None or ids is None or len(ids) < args.minimum_corners:
                detection_rejected.append(path.name)
                continue
            objects.append(charuco_points[ids].reshape(-1, 3).astype(np.float32))
            images.append(corners.reshape(-1, 1, 2).astype(np.float32))
            corner_counts.append(len(ids))
        else:
            found, corners = cv2.findChessboardCornersSB(
                gray, (args.board_cols, args.board_rows), flags=checker_flags
            )
            if not found or corners is None:
                detection_rejected.append(path.name)
                continue
            objects.append(checker_template.copy())
            images.append(corners.astype(np.float32))
            corner_counts.append(len(corners))
        accepted_paths.append(path)
        condition_scores.append(view_condition_score(images[-1], image_size))

    if image_size is None or len(objects) < args.minimum_views:
        raise SystemExit(
            f"only {len(objects)} valid {args.target} views detected; "
            f"at least {args.minimum_views} are required"
        )

    detected_images_before_sampling = len(objects)
    sampling_rejected: list[str] = []
    if len(objects) > args.maximum_views:
        selected = np.linspace(0, len(objects) - 1, num=args.maximum_views, dtype=np.int64).tolist()
        selected_set = set(selected)
        sampling_rejected = [
            path.name for index, path in enumerate(accepted_paths) if index not in selected_set
        ]
        objects = [objects[index] for index in selected]
        images = [images[index] for index in selected]
        accepted_paths = [accepted_paths[index] for index in selected]
        corner_counts = [corner_counts[index] for index in selected]
        condition_scores = [condition_scores[index] for index in selected]

    kept = list(range(len(objects)))
    outlier_rejected: list[str] = []
    conditioning_rejected: list[str] = []
    maximum_iterations = len(kept) - args.minimum_views + 12
    for iteration in range(maximum_iterations):
        selected_objects = [objects[index] for index in kept]
        selected_images = [images[index] for index in kept]
        try:
            if args.model == "fisheye":
                calibration = calibrate_fisheye(
                    selected_objects,
                    selected_images,
                    image_size,
                    not args.disable_fisheye_condition_check,
                )
            else:
                calibration = calibrate_pinhole(
                    selected_objects, selected_images, image_size, pinhole_flags
                )
        except cv2.error as error:
            if args.model != "fisheye":
                raise
            if len(kept) <= args.minimum_views:
                raise SystemExit(
                    "fisheye calibration is still ill-conditioned at the minimum "
                    f"view count ({args.minimum_views})"
                ) from error
            match = re.search(r"input array (\d+)", str(error))
            if match and int(match.group(1)) < len(kept):
                bad_position = int(match.group(1))
            else:
                bad_position = min(
                    range(len(kept)),
                    key=lambda position: condition_scores[kept[position]],
                )
            removed = kept.pop(bad_position)
            conditioning_rejected.append(accepted_paths[removed].name)
            if iteration == maximum_iterations - 1:
                raise SystemExit(
                    "fisheye calibration remained ill-conditioned after view filtering"
                ) from error
            continue
        rms, camera, distortion, rvecs, tvecs, per_view = calibration
        bad_positions = [
            index for index, error in enumerate(per_view) if float(error) > args.max_view_error_px
        ]
        if (
            not bad_positions
            or len(kept) <= args.minimum_views
            or iteration == maximum_iterations - 1
        ):
            break
        removal_count = min(
            len(bad_positions),
            max(1, int(np.ceil(len(kept) * 0.10))),
            len(kept) - args.minimum_views,
        )
        remove_positions = set(
            sorted(bad_positions, key=lambda index: float(per_view[index]), reverse=True)[
                :removal_count
            ]
        )
        removed = [kept[index] for index in sorted(remove_positions)]
        outlier_rejected.extend(accepted_paths[index].name for index in removed)
        kept = [
            original_index
            for position, original_index in enumerate(kept)
            if position not in remove_positions
        ]

    if camera[0, 0] <= 0.0 or camera[1, 1] <= 0.0:
        raise SystemExit("calibration returned an invalid focal length")
    projection = np.zeros((3, 4), dtype=np.float64)
    projection[:, :3] = camera
    rectification = np.eye(3, dtype=np.float64)
    model_name = (
        "equidistant"
        if args.model == "fisheye"
        else ("rational_polynomial" if args.rational_model else "plumb_bob")
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        f"image_width: {image_size[0]}\n"
        f"image_height: {image_size[1]}\n"
        f"camera_name: {args.camera_name}\n"
        f"camera_matrix:\n{matrix_block(camera)}\n"
        f"distortion_model: {model_name}\n"
        f"distortion_coefficients:\n{matrix_block(distortion.reshape(1, -1))}\n"
        f"rectification_matrix:\n{matrix_block(rectification)}\n"
        f"projection_matrix:\n{matrix_block(projection)}\n",
        encoding="utf-8",
    )
    used_names = [accepted_paths[index].name for index in kept]
    report = {
        "camera_name": args.camera_name,
        "model": model_name,
        "pinhole_constraints": pinhole_constraints,
        "opencv_calibration_flags": int(pinhole_flags) if args.model == "pinhole" else None,
        "fisheye_condition_check_enabled": (
            args.model != "fisheye" or not args.disable_fisheye_condition_check
        ),
        "resolution": list(image_size),
        "target": target_report,
        "rms_reprojection_error_px": rms,
        "maximum_view_error_px": float(np.max(per_view)),
        "per_view_error_px": {name: float(error) for name, error in zip(used_names, per_view)},
        "corners_per_used_image": {
            accepted_paths[index].name: corner_counts[index] for index in kept
        },
        "used_images": used_names,
        "detection_rejected": detection_rejected,
        "detected_images_before_sampling": detected_images_before_sampling,
        "sampling_rejected": sampling_rejected,
        "conditioning_rejected": conditioning_rejected,
        "outlier_rejected": outlier_rejected,
        "camera_matrix": camera.tolist(),
        "distortion": distortion.tolist(),
    }
    report_path = args.output.with_suffix(".report.json")
    report_path.write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(
        f"wrote {args.output} and {report_path}; RMS={rms:.4f}px, "
        f"max={float(np.max(per_view)):.4f}px, views={len(kept)}, "
        f"sampled={len(objects)}/{detected_images_before_sampling}, "
        f"conditioned_out={len(conditioning_rejected)}, "
        f"outliers={len(outlier_rejected)}"
    )
    if float(np.max(per_view)) > args.max_view_error_px:
        print("WARNING: maximum retained view error exceeds the requested threshold")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
