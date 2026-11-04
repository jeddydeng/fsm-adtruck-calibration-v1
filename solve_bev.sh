#!/usr/bin/env bash
set -euo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/scripts/env.sh"
mode="${1:-}"
[[ $# -gt 0 ]] && shift
case "${mode}" in
  detect) program=detect_part3_boards.py ;;
  absolute) program=build_bev_package.py ;;
  relative) program=optimize_relative_bev.py ;;
  verify) program=verify_relative_bev.py ;;
  *) echo "Usage: $0 {detect|absolute|relative|verify} [arguments]; add --help for options" >&2; exit 2 ;;
esac
exec "${CALIBRATION_PYTHON}" "${CALIBRATION_ROOT}/process/fisheye_bev/${program}" "$@"
