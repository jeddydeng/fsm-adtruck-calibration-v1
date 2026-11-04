#!/usr/bin/env bash
set -euo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/env.sh"
export CALIBRATION_NO_OVERLAY=1
calibration_source_ros
cd "${CALIBRATION_ROOT}/ros_ws"
# Use distribution CMake and compiler before unrelated /usr/local toolchains.
export PATH="/usr/bin:/bin:${PATH}"
# Colcon and ament use the Ubuntu system Python, matching the installed ROS ABI.
/usr/bin/colcon build --symlink-install --cmake-force-configure --parallel-workers "${BUILD_WORKERS:-2}" \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON \
  -DPython3_EXECUTABLE=/usr/bin/python3 -DPYTHON_EXECUTABLE=/usr/bin/python3
