#!/usr/bin/env bash
# Read-only source comparison after the vehicle is powered on.
set -euo pipefail
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
vehicle="${1:-fsm-lab@100.96.222.112}"
echo "Dry-run comparison; >f means vehicle file differs from this repository."
echo "Packaging intentionally changes paths and public-header installation; review diffs."
for package in camera_usb_pkg chassis_can_pkg imu_com_pkg ins_com_pkg rslidar_sdk \
  sensor_driver_msgs ultrasonic_radar12 white_sensor_kit_description; do
  echo "Package: ${package}"
  rsync -nrc --itemize-changes --exclude=.git --exclude=__pycache__ \
    -e 'ssh -S none -o BatchMode=yes -o ConnectTimeout=8' \
    "${vehicle}:white_ws/src/${package}/" "${root}/ros_ws/src/${package}/"
done
echo "Inspect solver and Pi differences separately before adopting live changes."
