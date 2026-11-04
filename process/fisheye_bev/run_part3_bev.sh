#!/usr/bin/env bash
set -euo pipefail

source "$(dirname "${BASH_SOURCE[0]}")/../../scripts/env.sh"
ROOT="${CALIBRATION_ROOT}"
PYTHON="${CALIBRATION_PYTHON}"
PROCESS="$ROOT/process/fisheye_bev"
DATA="$ROOT/data/raw_extrinsic_data"
# The broad backup interval contains 130 frames in which the garage background
# moves relative to the cameras.  The final 14 frames are the actual stationary
# vehicle / three radial board-position BEV sequence (3 + 6 + 5 captures).
CAPTURE_GROUP_LIST="${BEV_GROUP_LIST:-$PROCESS/part3_bev_selected_capture_groups.txt}"
INTRINSICS="$ROOT/result/fisheye_intrinsics_1600x1200_20260903_v2"
OUTPUT="${2:-$ROOT/result/fisheye_bev_20260903/final}"

if [[ $# -lt 1 ]]; then
  echo "用法: $0 <C任务签发的camera_relative_to_roof_lidar.yaml> [输出目录]" >&2
  echo "拒绝用机械初值静默生成可部署BEV参数。" >&2
  exit 2
fi

EXTRINSICS="$1"
if [[ ! -f "$EXTRINSICS" ]]; then
  echo "外参文件不存在: $EXTRINSICS" >&2
  exit 2
fi

mkdir -p "$OUTPUT"

"$PYTHON" "$PROCESS/detect_part3_boards.py" \
  --data-root "$DATA" \
  --group-list "$CAPTURE_GROUP_LIST" \
  --intrinsics-root "$INTRINSICS" \
  --output "$OUTPUT" \
  --preview-count 12

"$PYTHON" "$PROCESS/build_bev_package.py" \
  --data-root "$DATA" \
  --detections "$OUTPUT/board_detections.jsonl" \
  --intrinsics-root "$INTRINSICS" \
  --camera-extrinsics "$EXTRINSICS" \
  --output "$OUTPUT"

(
  cd "$OUTPUT"
  find . -type f ! -name manifest.sha256 -print0 | sort -z | xargs -0 shasum -a 256 > manifest.sha256
)

echo "Part3 fisheye BEV 标定包已生成：$OUTPUT"
