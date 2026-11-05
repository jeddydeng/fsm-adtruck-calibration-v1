#!/usr/bin/env bash
set -euo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/env.sh"
calibration_source_ros
exec "${CALIBRATION_PYTHON}" "${CALIBRATION_ROOT}/scripts/validate_black_capture.py"
