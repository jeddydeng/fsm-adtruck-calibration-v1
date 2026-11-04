#!/usr/bin/env bash
set -euo pipefail

source "$(dirname "${BASH_SOURCE[0]}")/env.sh"
workspace_dir="${CALIBRATION_ROOT}/ros_ws"
config_file="${workspace_dir}/src/camera_usb_pkg/config/camera_devices.yaml"
capture_script="${workspace_dir}/src/camera_usb_pkg/scripts/capture_calibration_images.py"
stream_tool="${CALIBRATION_ROOT}/drivers/pi_capture/camera_stream.sh"
duration_sec="${CAMERA_CAPTURE_DURATION_SEC:-1200}"
target_samples="${CAMERA_TARGET_SAMPLES:-70}"
max_duration_sec=1200
capture_root="${CAMERA_CAPTURE_ROOT:-${CALIBRATION_DATA_ROOT}/camera_data}"
red='\033[31m'
green='\033[32m'
reset='\033[0m'
previous_stream_mode=""

restore_stream_mode() {
  local original_status=$?
  local restore_status=0
  if [[ -n "${previous_stream_mode}" ]]; then
    if ! "${stream_tool}" mode "${previous_stream_mode}" >/dev/null; then
      echo -e "${red}WARNING: failed to restore Pi camera mode ${previous_stream_mode}.${reset}" >&2
      restore_status=1
    fi
  fi
  if (( original_status == 0 && restore_status != 0 )); then
    return "${restore_status}"
  fi
  return "${original_status}"
}

trap restore_stream_mode EXIT

echo "Select one camera for ChArUco intrinsic capture:"
echo "  1  video_rear_left   (left rear)"
echo "  2  video_front_left  (left front)"
echo "  3  video_front_right (right front)"
echo "  4  video_rear_right  (right rear)"
echo "  5  video_rear_mid    (rear center)"
echo "  6  video_front_mid   (front center)"
echo "  7  fisheye_front     (Pi front)"
echo "  8  fisheye_rear      (Pi rear)"
echo "  9  fisheye_left      (Pi left)"
echo "  0  fisheye_right     (Pi right)"
echo "  -  traffic_light_front (ROS 2 compressed, 320x240 at 5 Hz)"
echo -n "Press camera key [0-9 or -]: "
IFS= read -r -n 1 selection
echo

if [[ ! "${selection}" =~ ^[0-9]$ && "${selection}" != "-" ]]; then
  echo -e "${red}WARNING: select one digit from 0 through 9, or - for the traffic-light camera.${reset}" >&2
  exit 2
fi
if [[ ! "${duration_sec}" =~ ^[0-9]+$ ]] || \
   (( duration_sec < 1 || duration_sec > max_duration_sec )); then
  echo -e "${red}WARNING: CAMERA_CAPTURE_DURATION_SEC must be 1-${max_duration_sec}.${reset}" >&2
  exit 2
fi
if [[ ! -x "${capture_script}" ]]; then
  echo "Camera capture program is missing or not executable: ${capture_script}" >&2
  exit 1
fi
if [[ -z "${DISPLAY:-}" && -z "${WAYLAND_DISPLAY:-}" ]]; then
  echo -e "${red}WARNING: no desktop display is available; run this in the vehicle desktop terminal.${reset}" >&2
  exit 1
fi

capture_source_args=()
if [[ "${selection}" =~ ^[1-6]$ ]]; then
  if [[ ! -r "${config_file}" ]]; then
    echo "Camera configuration is missing: ${config_file}" >&2
    exit 1
  fi
  camera_index=$((selection - 1))
  camera_name="$(python3 -c \
    'import sys,yaml; d=yaml.safe_load(open(sys.argv[1])); i=int(sys.argv[2]); print(next(x["camera_name"] for x in d["cameras"] if x["index"]==i))' \
    "${config_file}" "${camera_index}")"
  capture_source_args=(
    --camera-index "${camera_index}"
    --config-file "${config_file}"
    --width 640
    --height 480
  )
elif [[ "${selection}" == "-" ]]; then
  ros_setup="/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"
  ros_topic="${TRAFFIC_LIGHT_CAMERA_TOPIC:-/sensing/camera/traffic_light_front/image_raw/compressed}"
  if [[ ! -r "${ros_setup}" ]]; then
    echo -e "${red}WARNING: ROS 2 Jazzy setup is missing: ${ros_setup}.${reset}" >&2
    exit 1
  fi
  set +u
  # shellcheck disable=SC1091
  source "${ros_setup}"
  if [[ -r "${workspace_dir}/install/setup.bash" ]]; then
    # shellcheck disable=SC1091
    source "${workspace_dir}/install/setup.bash"
  fi
  set -u

  camera_name="traffic_light_front"
  export ROS_DOMAIN_ID="${WHITE_ROS_DOMAIN_ID:-${ROS_DOMAIN_ID:-92}}"
  export RMW_IMPLEMENTATION="${TRAFFIC_LIGHT_RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
  unset ROS_LOCALHOST_ONLY || true
  export ROS_AUTOMATIC_DISCOVERY_RANGE="${TRAFFIC_LIGHT_DISCOVERY_RANGE:-SUBNET}"
  # The normal vehicle profile is deliberately loopback-only. Intrinsic capture
  # must discover the Raspberry Pi on the sensor LAN. A dedicated profile can
  # still be supplied explicitly when multicast is unavailable.
  if [[ -n "${TRAFFIC_LIGHT_CYCLONEDDS_URI:-}" ]]; then
    export CYCLONEDDS_URI="${TRAFFIC_LIGHT_CYCLONEDDS_URI}"
  fi
  capture_source_args=(
    --ros-compressed-topic "${ros_topic}"
    --camera-name "${camera_name}"
    --width 320
    --height 240
    --fps 5
  )
  echo "ROS 2 topic: ${ros_topic}"
  echo "Expected stream: 320x240 at 5 Hz; ROS_DOMAIN_ID=${ROS_DOMAIN_ID}."
else
  if [[ ! -x "${stream_tool}" ]]; then
    echo "Pi camera stream tool is missing: ${stream_tool}" >&2
    exit 1
  fi
  case "${selection}" in
    7) fisheye_source="front"; camera_name="fisheye_front" ;;
    8) fisheye_source="rear";  camera_name="fisheye_rear" ;;
    9) fisheye_source="left";  camera_name="fisheye_left" ;;
    0) fisheye_source="right"; camera_name="fisheye_right" ;;
  esac
  previous_stream_mode="$(
    "${stream_tool}" status | python3 -c \
      'import json,sys; mode=json.load(sys.stdin).get("mode"); print(mode if mode else "bev")'
  )"
  case "${previous_stream_mode}" in
    bev|raw-all|front|rear|left|right) ;;
    *)
      echo -e "${red}WARNING: invalid Pi camera mode: ${previous_stream_mode}.${reset}" >&2
      previous_stream_mode=""
      exit 1
      ;;
  esac
  "${stream_tool}" mode "${fisheye_source}" >/dev/null
  capture_source_args=(
    --stream-url "${PI_STREAM_BASE_URL}/api/stream/mjpeg?source=${fisheye_source}&fps=10"
    --camera-name "${camera_name}"
    --width 0
    --height 0
  )
  echo "Pi stream mode: ${previous_stream_mode} -> ${fisheye_source}; it will be restored on exit."
fi

output_dir="${capture_root}/${camera_name}_$(date +%Y%m%d_%H%M%S)"

echo "A preview window will open for at most ${duration_sec}s (20-minute hard limit)."
echo "Press q/w/e/a/s/d/z/x/c in that window to stop early."
set +e
python3 "${capture_script}" \
  "${capture_source_args[@]}" \
  --output-dir "${output_dir}" \
  --squares-x 12 \
  --squares-y 8 \
  --square-length-m 0.070 \
  --marker-length-m 0.050 \
  --dictionary DICT_5X5_100 \
  --target-samples "${target_samples}" \
  --minimum-corners 20 \
  --max-duration-sec "${duration_sec}"
status=$?
set -e

if (( status != 0 )); then
  echo -e "${red}WARNING: capture ended with status ${status}; review ${output_dir}.${reset}" >&2
  exit "${status}"
fi
echo -e "${green}============================================================${reset}"
echo -e "${green}  CAMERA CAPTURE COMPLETE: ${camera_name}${reset}"
echo -e "${green}  DATA: ${output_dir}${reset}"
echo -e "${green}============================================================${reset}"
