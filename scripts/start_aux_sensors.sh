#!/usr/bin/env bash
set -euo pipefail
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
case "${1:-}" in
  lidar) exec bash "$root/drivers/black_vehicle/start_lidars.sh" ;;
  gmsl) exec bash "$root/drivers/black_vehicle/start_gmsl_ros.sh" ;;
  *) echo "Usage: $0 {lidar|gmsl}. Pi fisheyes use scripts/start_pi_receiver.sh." >&2; exit 2 ;;
esac
