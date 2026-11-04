from __future__ import annotations

import numpy as np

from detect_part3_boards import (
    ACTIVE_M,
    COLS,
    ROWS,
    Marker,
    marker_object_corners,
    separate_repeated_boards,
)


def test_aprilgrid_active_dimensions_are_exact() -> None:
    corners = np.concatenate([marker_object_corners(tag_id) for tag_id in range(COLS * ROWS)])
    span = np.ptp(corners[:, :2], axis=0)
    np.testing.assert_allclose(span, [ACTIVE_M, ACTIVE_M], atol=1e-12)


def test_spatial_grouping_keeps_two_repeated_id_boards_separate() -> None:
    markers = []
    index = 0
    for offset in (np.asarray([100.0, 200.0]), np.asarray([900.0, 250.0])):
        for tag_id in range(COLS * ROWS):
            object_corners = marker_object_corners(tag_id)[:, :2]
            pixel_corners = object_corners * 180.0 + offset
            markers.append(Marker(index, tag_id, pixel_corners))
            index += 1
    groups = separate_repeated_boards(markers)
    assert sorted(len(group) for group in groups) == [30, 30]
    assert all(len({marker.tag_id for marker in group}) == 30 for group in groups)
