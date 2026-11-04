#!/usr/bin/env bash
set -euo pipefail
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
exec "${CALIBRATION_PYTHON:-${root}/.venv/bin/python}" \
  "${root}/drivers/pi_capture/camera_stream_client.py" \
  --base-url "${PI_STREAM_BASE_URL:-http://127.0.0.1:8086}" "$@"
