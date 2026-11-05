#!/usr/bin/env bash
set -euo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/scripts/env.sh"
mode="${1:-}"
[[ $# -gt 0 ]] && shift
if [[ "${mode}" == targets || ( "${mode}" == fixed-boards && "${CALIBRATION_STATION_CONFIG:-}" == config/black_vehicle/station.yaml ) ]]; then
  exec "${CALIBRATION_PYTHON}" "${CALIBRATION_ROOT}/process/joint_extrinsic_station/solve_joint_extrinsics.py" \
    --config "${CALIBRATION_ROOT}/${CALIBRATION_STATION_CONFIG:-process/joint_extrinsic_station/config.yaml}" \
    --initial "${CALIBRATION_ROOT}/${CALIBRATION_INITIAL_EXTRINSICS:-process/joint_extrinsic_station/initial_extrinsics.yaml}" \
    --data-root "${CALIBRATION_DATA_ROOT}/raw_extrinsic_data" \
    --output "${CALIBRATION_OUTPUT_ROOT:-${CALIBRATION_ROOT}/output}/black_joint_extrinsics.yaml" "$@"
fi
case "${mode}" in
  fixed-boards) program=part12_joint_extrinsics/part12_joint_calibrate.py ;;
  multiday) program=part12_joint_extrinsics/multiday_hierarchical_refine.py ;;
  traffic) program=traffic_part4_calibration/traffic_part4_calibrate.py ;;
  *) echo "Usage: $0 {fixed-boards|multiday|traffic} [solver arguments]; see docs/OPERATIONS_EN.md" >&2; exit 2 ;;
esac
exec "${CALIBRATION_PYTHON}" "${CALIBRATION_ROOT}/process/${program}" "$@"
