#!/usr/bin/env bash
set -eo pipefail
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
[[ -r "$root/ros_ws/install/setup.bash" ]] || { echo 'Run scripts/build.sh first.' >&2; exit 1; }
source "$root/ros_ws/install/setup.bash"
exec ros2 launch "$root/drivers/black_vehicle/calibration_lidars.launch.py"
