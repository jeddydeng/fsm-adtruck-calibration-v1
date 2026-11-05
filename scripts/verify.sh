#!/usr/bin/env bash
set -euo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/env.sh"
if [[ -r "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash" ]]; then
  calibration_source_ros
fi
cd "${CALIBRATION_ROOT}"
"${CALIBRATION_PYTHON}" scripts/audit_repository.py
"${CALIBRATION_PYTHON}" -m ruff check scripts tests process ros_ws/src --output-format concise
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 "${CALIBRATION_PYTHON}" -m pytest
if "${CALIBRATION_PYTHON}" -c 'import rclpy' 2>/dev/null; then
  "${CALIBRATION_PYTHON}" process/joint_extrinsic_station/joint_capture_station.py --check \
    --config "${CALIBRATION_STATION_CONFIG}" --initial-extrinsics "${CALIBRATION_INITIAL_EXTRINSICS}"
fi
echo "Repository verification passed. Hardware acceptance remains a separate step."
