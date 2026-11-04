#!/usr/bin/env bash
set -euo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/scripts/env.sh"
calibration_source_ros
exec "${CALIBRATION_PYTHON}" "${CALIBRATION_ROOT}/scripts/station_manager.py" start "$@"
