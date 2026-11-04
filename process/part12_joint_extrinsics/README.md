# Part 1/2 fixed-board moving-vehicle extrinsic calibration

This folder handles the 2026-09-03 Part 1 and Part 2 acquisitions.  The four
boards were stationary and the vehicle moved, so the older temporal background
subtraction pipeline must not be used.

The program:

1. re-detects AprilTag16h5 corners from every saved image;
2. uses the accepted 1600×1200 OpenCV `equidistant` fisheye intrinsics;
3. rejects camera/lidar pairs more than 100 ms apart;
4. fits board-sized planes directly in each raw PCD without ground removal;
5. jointly solves lidar-plane and simultaneous camera-camera constraints with
   the roof 32-line lidar fixed as the reference;
6. applies the hard 10 cm outlier rule, deterministic resampling, and an
   independent Part 1 versus Part 2 check;
7. writes review-only results.  It never edits Autoware or starts vehicle code.

Run with the pinned environment:

```bash
../.venv/bin/python part12_joint_calibrate.py --extract --max-rounds 200
```

An existing observation cache may be reused by omitting `--extract`.

## Multi-day hierarchical verification

`multiday_hierarchical_refine.py` combines the 2026-09-02 stationary-vehicle,
moving-board data with the 2026-09-03 fixed-board, moving-vehicle data.  It
keeps the previously repeatable three blind-zone lidars, four side USB
cameras, and left/right fisheye cameras frozen.  Only `video_front_mid`,
`video_rear_mid`, `fisheye_front`, and `fisheye_rear` are optimized.

The loop uses a 500-round hard limit, a strict 1 mm / 0.1 degree gate through
round 200, and the requested 1 cm / 1 degree fallback afterwards.  Five
consecutive passing rounds stop the bootstrap early.  Passing bootstrap
repeatability alone never authorizes deployment: cross-day agreement and
minimum independent-capture support must also pass.

```bash
../.venv/bin/python multiday_hierarchical_refine.py --extract-day1 --max-rounds 500
```

The September 2 images are re-detected using the intrinsics stored with each
capture, including the old 640×480 fisheye model.  AprilTag centres are used
for those files because their historical four-corner winding convention is
incompatible with the current board generator.  Camera pairs involving the
known unreliable traffic-light stream are explicitly excluded.  Raw captures
are never modified.

The current decision and exact evidence counts are documented in
`../../result/part12_multiday_20260903/C_VALIDATION_REPORT.md`.
