#!/usr/bin/env bash
# Optional non-calibration sensors. CAN is receive-only.
set -euo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/env.sh"
calibration_source_ros
case "${1:-}" in
  imu) shift; exec ros2 launch imu_com_pkg imu_com_launch.py "$@" ;;
  gnss) shift; exec ros2 launch ins_com_pkg all_ins_com_launch.py "$@" ;;
  ultrasonic) shift; exec ros2 launch ultrasonic_radar12 ultrasonic_radar12_launch.py "$@" ;;
  can-rx) shift; exec ros2 launch chassis_can_pkg chassis_can_launch.py "$@" enable_tx:=false allow_arm_with_mechanical_estop:=false ;;
  lidar) shift; exec ros2 launch white_calibration_bringup calibration_lidars.launch.py "$@" ;;
  *) echo "Usage: $0 {imu|gnss|ultrasonic|can-rx|lidar} [launch arguments]" >&2; exit 2 ;;
esac
