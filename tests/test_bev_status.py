from pathlib import Path

import cv2
import numpy as np

from optimize_relative_bev import write_opencv_yaml


def test_failed_quality_is_not_published_as_usable(tmp_path: Path) -> None:
    data = {
        "K": np.eye(3).tolist(),
        "D": [0, 0, 0, 0],
        "T_vehicle_from_camera_optical": np.eye(4).tolist(),
        "K_rectified": np.eye(3).tolist(),
        "intrinsic_rms_px": 1.0,
        "intrinsic_accepted_p95_px": 2.0,
    }
    metrics = {"reprojection_median_px": 1.0, "reprojection_p95_px": 2.0}
    output = tmp_path / "front.yaml"
    write_opencv_yaml(output, "fisheye_front", data, metrics, np.eye(3), np.eye(3), False)
    document = cv2.FileStorage(str(output), cv2.FILE_STORAGE_READ)
    assert document.getNode("deployment_allowed").real() == 0
    assert document.getNode("bev_status").string() == "failed_self_check"
    document.release()
