"""Check printed-board geometry and OpenCV compatibility without real imagery."""

from pathlib import Path
import importlib.util
import sys

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location(
    "robust_fisheye_intrinsics", ROOT / "process/robust_fisheye_intrinsics.py"
)
INTRINSICS = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = INTRINSICS
SPEC.loader.exec_module(INTRINSICS)


def test_charuco_dimensions_and_full_board_detection() -> None:
    dictionary, board, points = INTRINSICS.charuco_board()
    assert points.shape == (77, 3)
    assert np.allclose(points.min(axis=0), [0.07, 0.07, 0])
    assert np.allclose(points.max(axis=0), [0.77, 0.49, 0])
    image = board.generateImage((1200, 800), marginSize=20)
    corners, ids, candidates = INTRINSICS.best_charuco_observation(image, dictionary, board)
    assert corners is not None and len(ids) == 77 and candidates == 1


def test_empty_image_does_not_create_calibration_observation() -> None:
    dictionary, board, _ = INTRINSICS.charuco_board()
    corners, ids, candidates = INTRINSICS.best_charuco_observation(
        np.full((1200, 1600), 255, np.uint8), dictionary, board
    )
    assert corners is None and ids is None and candidates == 0
