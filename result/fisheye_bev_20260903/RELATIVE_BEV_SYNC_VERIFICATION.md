# Relative four-fisheye BEV sync verification

Date: 2026-09-03 (Asia/Shanghai)

## Decision split

- Relative ground-plane BEV: `deployment_allowed=1`, `status=usable`.
- Deployment scope: `BEV_relative_planar_mapping_only`.
- Absolute camera-to-vehicle R/t: `deployment_allowed=0`, `review_only_not_signed`.
- Autoware was not modified.

## Mac source

- Result: `/Users/djhuai/fsm-adtruck/calibration-white/result/fisheye_bev_20260903/relative_usable`
- Tools: `/Users/djhuai/fsm-adtruck/calibration-white/process/fisheye_bev`
- Exact inputs: 14 capture groups / 56 fresh 1600×1200 source images.
- `verify_relative_bev.py`: passed.
- Pytest: 2 passed.
- Result manifest: 26/26 files passed SHA-256.
- Source image binding: 56/56 source images passed SHA-256.
- Process manifest: 8/8 files passed SHA-256.

## Vehicle isolated copy

- Result: `/home/fsm-lab/white_ws/calibration/offline_results/fisheye_bev_20260903/relative_usable`
- Tools: `/home/fsm-lab/white_ws/calibration/offline_tools/fisheye_bev`
- Result manifest: 26/26 files passed SHA-256 on the vehicle.
- Process manifest: 8/8 files passed SHA-256 on the vehicle.
- Result README SHA-256 on both machines:
  `c7447820c35f7ab73bdb4f8ca8a2ab52e981f08a356d1d925eede4dd635283ea`.

The vehicle copy is intentionally outside Autoware runtime configuration. No CAN,
vehicle control, Autoware launch, or sensor TF deployment was performed.
