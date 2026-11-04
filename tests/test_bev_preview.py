from pathlib import Path

import numpy as np
import pytest

from render_bev import compose, DIRECTIONS

ROOT = Path(__file__).resolve().parents[1]
PACKAGE = ROOT / "result/fisheye_bev_20260903/relative_usable"


def test_remap_preserves_constant_ground_color() -> None:
    images = {name: np.full((1200, 1600, 3), 100, np.uint8) for name in DIRECTIONS}
    preview = compose(images, PACKAGE)
    assert preview.shape == (512, 512, 3)
    assert np.count_nonzero(preview[..., 0]) > 100000
    assert preview.max() <= 100
    assert preview[preview > 0].min() >= 99


def test_rejects_wrong_input_resolution() -> None:
    images = {name: np.zeros((600, 800, 3), np.uint8) for name in DIRECTIONS}
    with pytest.raises(ValueError, match="1600x1200"):
        compose(images, PACKAGE)
