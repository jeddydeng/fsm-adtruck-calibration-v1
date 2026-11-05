#!/usr/bin/env bash
# Run as cityu02; sudo is requested only for hardware initialization.
set -euo pipefail
owner="${SUDO_USER:-$USER}"
owner_home="$(getent passwd "$owner" | cut -d: -f6)"
script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
packaged_sdk="$script_dir/vendor/senyun_pcie_v4l2/bash"
legacy_sdk="$owner_home/workspace/senyun/pcie_v4l2_sdk_ubuntu_SHAb57105dd-20240913_def/pcie_v4l2_sdk_ubuntu_SHAb57105dd-20240312/pcie_v4l2_sdk/bash"
if [[ -d "$packaged_sdk" ]]; then default_sdk="$packaged_sdk"; else default_sdk="$legacy_sdk"; fi
sdk="${GMSL_SDK_BASH:-$default_sdk}"
force=0
case "${1:-}" in
  --force) force=1 ;;
  --help|-h) echo 'Usage: gmsl_camera_init.sh [--force]'; echo 'Initializes card0, UYVY 1920x1080, internal trigger 10 Hz; does not start capture.'; exit 0 ;;
  '') ;;
  *) echo "Unknown option: $1" >&2; exit 2 ;;
esac
[[ -d "$sdk" ]] || { echo "SDK missing: $sdk" >&2; exit 1; }
sdk="$(cd "$sdk" && pwd)"
if (( EUID != 0 && force == 0 )) && [[ -e /run/black-vehicle-gmsl-card0.initialized && -c /dev/xdma0_user ]]; then
  ready=1
  for n in {0..7}; do [[ -c /dev/video$n && -r /dev/video$n && -w /dev/video$n ]] || ready=0; done
  if (( ready )); then
    echo 'Card0 is already initialized and accessible in this boot.'
    exit 0
  fi
fi
for file in load_modules.sh pcie_init_card0.sh common_fun.sh ../xdma_v4l2/xdma.ko ../tools/reg_rw ../tools/video_rw ../tools/serdes_cfg; do
  [[ -f "$sdk/$file" ]] || { echo "Required SDK file missing: $file" >&2; exit 1; }
done
if (( EUID != 0 )); then
  args=(); (( force == 0 )) || args+=(--force)
  exec sudo env GMSL_SDK_BASH="$sdk" bash "$(readlink -f "$0")" "${args[@]}"
fi
exec 9>/run/lock/black-vehicle-gmsl-init.lock
flock -n 9 || { echo 'Another initialization is running.' >&2; exit 1; }
shopt -s nullglob
devices=(/dev/video{0..7})
if ((${#devices[@]})) && fuser "${devices[@]}" >/dev/null 2>&1; then
  echo 'Camera devices are in use. Stop camera capture before initialization.' >&2
  exit 1
fi
stamp=/run/black-vehicle-gmsl-card0.initialized
if [[ -e "$stamp" && -c /dev/xdma0_user && -c /dev/video0 && "$force" == 0 ]]; then
  echo 'Card0 already initialized in this boot. Use --force to reinitialize when idle.'
  exit 0
fi
kernel="$(uname -r)"
vermagic="$(modinfo -F vermagic "$sdk/../xdma_v4l2/xdma.ko")"
[[ "${vermagic%% *}" == "$kernel" ]] || { echo "Kernel/module mismatch: $kernel / $vermagic" >&2; exit 1; }
log_dir=/var/log/black-vehicle-gmsl
mkdir -p "$log_dir"
log="$log_dir/init-$(date +%Y%m%d-%H%M%S).log"
exec > >(tee -a "$log") 2>&1
echo "SDK=$sdk; log=$log; initializing card0"
cd "$sdk" # Vendor scripts use relative tool/module paths.
rm -f "$stamp"
timeout 90 bash ./load_modules.sh
[[ -c /dev/xdma0_user ]] || { echo 'Missing /dev/xdma0_user after module load.'; exit 1; }
timeout 180 bash ./pcie_init_card0.sh
udevadm settle --timeout=15
for n in {0..7}; do
  [[ -c /dev/video$n ]] || { echo "Missing device /dev/video$n"; exit 1; }
  # Grant this login access without making devices world writable.
  command -v setfacl >/dev/null && setfacl -m "u:$owner:rw" "/dev/video$n"
  v4l2-ctl -d "/dev/video$n" --get-fmt-video
done
touch "$stamp"
echo 'Card0 initialized: eight device nodes. Connected cameras must be verified by reading frames.'
echo 'SDK settings: UYVY, 1920x1080, internal trigger 10 Hz, GMSL2F (3G).'
