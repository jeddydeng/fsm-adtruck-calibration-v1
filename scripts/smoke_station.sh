#!/usr/bin/env bash
set -euo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/env.sh"
calibration_source_ros
export ROS_DOMAIN_ID=193
export ROS_LOCALHOST_ONLY=1
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
unset CYCLONEDDS_URI
unset DISPLAY WAYLAND_DISPLAY
exec "${CALIBRATION_PYTHON}" "${CALIBRATION_ROOT}/scripts/smoke_station.py"
