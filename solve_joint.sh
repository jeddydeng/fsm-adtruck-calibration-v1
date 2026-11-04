#!/usr/bin/env bash
set -euo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/scripts/env.sh"
mode="${1:-}"
[[ $# -gt 0 ]] && shift
case "${mode}" in
  fixed-boards) program=part12_joint_extrinsics/part12_joint_calibrate.py ;;
  multiday) program=part12_joint_extrinsics/multiday_hierarchical_refine.py ;;
  traffic) program=traffic_part4_calibration/traffic_part4_calibrate.py ;;
  *) echo "Usage: $0 {fixed-boards|multiday|traffic} [solver arguments]; see docs/OPERATIONS_EN.md" >&2; exit 2 ;;
esac
exec "${CALIBRATION_PYTHON}" "${CALIBRATION_ROOT}/process/${program}" "$@"
