#!/usr/bin/env bash
# Source this file from an entry point; keep every path relative to this checkout.
CALIBRATION_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export CALIBRATION_ROOT
export PYTHONNOUSERSITE=1
if [[ -r "${CALIBRATION_ROOT}/config/local.env" ]]; then
  set -a
  source "${CALIBRATION_ROOT}/config/local.env"
  set +a
fi
if [[ -z "${ROS_DISTRO:-}" && -r /etc/os-release ]]; then
  case "$(. /etc/os-release; echo "${VERSION_ID}")" in
    24.04) export ROS_DISTRO=jazzy ;;
    22.04) export ROS_DISTRO=humble ;;
  esac
fi
export CALIBRATION_PYTHON="${CALIBRATION_PYTHON:-${CALIBRATION_ROOT}/.venv/bin/python}"
export CALIBRATION_DATA_ROOT="${CALIBRATION_DATA_ROOT:-${CALIBRATION_ROOT}/data}"
export PI_STREAM_BASE_URL="${PI_STREAM_BASE_URL:-http://127.0.0.1:8086}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-92}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
export ROS_AUTOMATIC_DISCOVERY_RANGE="${ROS_AUTOMATIC_DISCOVERY_RANGE:-SUBNET}"
if [[ ! -x "${CALIBRATION_PYTHON}" ]]; then
  echo "Run ./install.sh first: missing ${CALIBRATION_PYTHON}" >&2
  return 1
fi
export PATH="${CALIBRATION_ROOT}/.venv/bin:${PATH}"

calibration_source_ros() {
  local setup="/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"
  [[ -r "${setup}" ]] || { echo "ROS setup missing: ${setup}" >&2; return 1; }
  set +u
  source "${setup}"
  if [[ "${CALIBRATION_NO_OVERLAY:-0}" != 1 && -r "${CALIBRATION_ROOT}/ros_ws/install/setup.bash" ]]; then
    source "${CALIBRATION_ROOT}/ros_ws/install/setup.bash"
  fi
  set -u
}
