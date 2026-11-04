#!/usr/bin/env bash
set -euo pipefail

source "$(dirname "${BASH_SOURCE[0]}")/env.sh"
workspace_dir="${CALIBRATION_ROOT}/ros_ws"
calibrator="${workspace_dir}/src/camera_usb_pkg/scripts/calibrate_intrinsics.py"
selection="${1:-}"
session_dir="${2:-}"
model="${3:-}"

if [[ ( ! "${selection}" =~ ^[0-9]$ && "${selection}" != "-" ) || -z "${session_dir}" ]]; then
  echo "Usage: $0 CAMERA_KEY[0-9|-] SESSION_DIRECTORY [pinhole|fisheye]" >&2
  exit 2
fi

case "${selection}" in
  1) camera_name="video_rear_left" ;;
  2) camera_name="video_front_left" ;;
  3) camera_name="video_front_right" ;;
  4) camera_name="video_rear_right" ;;
  5) camera_name="video_rear_mid" ;;
  6) camera_name="video_front_mid" ;;
  7) camera_name="fisheye_front" ;;
  8) camera_name="fisheye_rear" ;;
  9) camera_name="fisheye_left" ;;
  0) camera_name="fisheye_right" ;;
  -) camera_name="traffic_light_front" ;;
esac

if [[ -z "${model}" ]]; then
  case "${selection}" in
    0|7|8|9) model="fisheye" ;;
    *) model="pinhole" ;;
  esac
fi
if [[ "${model}" != "pinhole" && "${model}" != "fisheye" ]]; then
  echo "Model must be pinhole or fisheye." >&2
  exit 2
fi
if [[ -d "${session_dir}/images" ]]; then
  image_dir="${session_dir}/images"
else
  image_dir="${session_dir}"
fi
if [[ ! -d "${image_dir}" ]]; then
  echo "Image directory does not exist: ${image_dir}" >&2
  exit 1
fi

result_dir="${CALIBRATION_ROOT}/output/intrinsics"
# All four 1600x1200, approximately 180-degree cameras use the validated
# robust equidistant pipeline, including the left camera.
if [[ "${model}" == fisheye ]]; then
  exec "${CALIBRATION_PYTHON}" "${CALIBRATION_ROOT}/process/robust_fisheye_intrinsics.py" \
    --input-dir "${image_dir}" \
    --output-dir "${result_dir}/${camera_name}_$(date +%Y%m%d_%H%M%S)" \
    --camera-name "${camera_name}"
fi
# Nanoseconds prevent concurrent validation runs for one camera from
# overwriting each other when they start within the same second.
output="${result_dir}/${camera_name}_$(date +%Y%m%d_%H%M%S_%N).yaml"
extra_args=()
if [[ "${CAMERA_RATIONAL_MODEL:-0}" == "1" ]]; then
  if [[ "${model}" == "fisheye" ]]; then
    echo "CAMERA_RATIONAL_MODEL=1 is incompatible with fisheye mode." >&2
    exit 2
  fi
  extra_args+=(--rational-model)
fi
if [[ "${selection}" == "-" ]]; then
  if [[ "${model}" != "pinhole" ]]; then
    echo "The traffic-light camera stable profile requires the pinhole model." >&2
    exit 2
  fi
  if [[ "${CAMERA_RATIONAL_MODEL:-0}" == "1" ]]; then
    echo "CAMERA_RATIONAL_MODEL=1 is not supported by the traffic-light stable profile." >&2
    exit 2
  fi
  # At 320x240 the unconstrained five-coefficient solution varied substantially
  # across 40/60/80-view subsets. These physical constraints produced the best
  # four-fold stability without materially increasing held-out error.
  extra_args+=(
    --fix-principal-point
    --fix-aspect-ratio
    --zero-tangent-distortion
    --fix-k3
  )
fi
if [[ "${CAMERA_FISHEYE_DISABLE_CONDITION_CHECK:-0}" == "1" ]]; then
  if [[ "${model}" != "fisheye" ]]; then
    echo "CAMERA_FISHEYE_DISABLE_CONDITION_CHECK=1 requires fisheye mode." >&2
    exit 2
  fi
  extra_args+=(--disable-fisheye-condition-check)
fi

python3 "${calibrator}" \
  --input-dir "${image_dir}" \
  --output "${output}" \
  --camera-name "${camera_name}" \
  --target charuco \
  --squares-x 12 \
  --squares-y 8 \
  --square-length-m 0.070 \
  --marker-length-m 0.050 \
  --dictionary DICT_5X5_100 \
  --minimum-corners 20 \
  --minimum-views 20 \
  --maximum-views "${CAMERA_CALIBRATION_MAX_VIEWS:-100}" \
  --max-view-error-px 0.8 \
  --model "${model}" \
  "${extra_args[@]}"

echo "Calibration result (not yet installed as the live CameraInfo): ${output}"
echo "Review the JSON quality report and undistorted images before replacing camera_info YAML."
