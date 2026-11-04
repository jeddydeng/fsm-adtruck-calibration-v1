#!/usr/bin/env bash
# Foreground transport receivers for intrinsic capture or manual diagnostics.
set -euo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/env.sh"
gateway="${CALIBRATION_ROOT}/drivers/pi_gateway"
case "${1:-}" in
  fisheye)
    exec "${CALIBRATION_PYTHON}" "${gateway}/app/w102_monitor.py" \
      --gateway-ip "${PI_GATEWAY_IP:-192.168.1.186}" --headless --web \
      --data-port 5610 --control-port 5611 --web-bind 127.0.0.1 --web-port 8086 \
      --capture-program "${gateway}/capture/capture_cameras.py" \
      --capture-output "${CALIBRATION_ROOT}/output/pi_capture" \
      --voice-cache "${CALIBRATION_ROOT}/.runtime/voice-cache"
    ;;
  traffic)
    calibration_source_ros
    exec "${CALIBRATION_PYTHON}" "${gateway}/traffic_camera/telephoto_ros2_receiver.py" \
      --source-ip "${PI_GATEWAY_IP:-192.168.1.186}"
    ;;
  *) echo "Usage: $0 {fisheye|traffic}; stop with Ctrl+C" >&2; exit 2 ;;
esac
