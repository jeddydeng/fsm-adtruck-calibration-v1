# Traffic-light camera Part-4 calibration

This directory contains an offline-only and fail-closed workflow for the
2026-09-03 traffic-light camera captures.  It never edits a vehicle.

Run with the workspace Python environment:

```bash
../.venv/bin/python traffic_part4_calibrate.py
../.venv/bin/python audit_intrinsic_candidates.py
../.venv/bin/python make_validation_preview.py
```

Inputs are the authoritative 45 capture IDs in
`data/backup_manifests/20260903/part4_capture_groups.txt`.  The image detector
always starts from the saved PNG; cached UI detections are not trusted.

The camera-to-lidar fitter first searches each raw PCD independently.  A sparse
plane is retained only if a second lidar agrees with its plane equation.  A
dense roof-lidar observation may stand alone.  Three lower-lidar transforms are
explicitly treated as frozen fallback anchors.  Resampling is performed by
whole capture so observations from the same timestamp cannot leak into both
training and validation.

The generated result is review-only unless every quality gate in
`traffic_part4_result.yaml` is true.  A failed gate means no Autoware update.
