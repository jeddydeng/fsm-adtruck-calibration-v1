#!/usr/bin/env bash
# Proposed replacement for Autoware_sh/start_camera.sh after review.
set -euo pipefail
project="${BLACK_CALIBRATION_ROOT:-$HOME/projects/fsm-adtruck-calibration-v1}"
[[ -f "$project/drivers/black_vehicle/start_gmsl_ros.sh" ]] || {
  echo 'Set BLACK_CALIBRATION_ROOT to the deployed black_vehicle checkout.' >&2
  exit 1
}
exec bash "$project/drivers/black_vehicle/start_gmsl_ros.sh" "$@"
