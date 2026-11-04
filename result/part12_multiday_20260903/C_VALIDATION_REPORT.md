# Task C multi-day extrinsic validation report

## Decision

**REVIEW ONLY — DO NOT DEPLOY.**  No Autoware, active sensor transform, map,
CAN, or vehicle-control file was changed by this calibration run.

The corrected hierarchical run stopped at round 205 after five consecutive
rounds met the post-200 fallback repeatability gate.  It did **not** pass the
independent cross-day and observability gates, so numerical loop stability is
not treated as physical accuracy.

## Inputs and immutable anchors

- 2026-09-02: 99 relevant stationary-vehicle captures; one board was moved
  only between captures.  Capture-local intrinsics were used and raw pixels
  were re-detected.
- 2026-09-03 Part 1/2: 152 fixed-board, moving-vehicle captures.
- Reference frame: `velodyne_top_base_link`.
- Frozen anchors: front-left/front-right/rear Bpearl lidars, four side USB
  cameras, and left/right fisheye cameras.  Their previous transforms were not
  changed, so this rejected run cannot degrade them.
- Optimized nodes: front-middle USB, rear-middle USB, front fisheye, rear
  fisheye.

## Cleaning and pairwise checks

- Raw data was not modified.  All rejection occurs in derived observations.
- Pixel detections were solved again rather than trusting live UI metadata.
- Camera/lidar observations with implausible seed distance, plane angle,
  centre gap, or dynamic timestamp skew were excluded.
- September 2 static snapshots retain measured camera skew but do not apply a
  dynamic 100 ms rejection gate.
- The static-day extractor produced 95 camera/lidar plane edges and 12 direct
  front-middle ↔ front-fisheye edges.
- September 3 contributed the rear-middle ↔ rear-fisheye overlap.  Weak/weak
  edges were coupled by alternating joint optimization instead of being
  silently discarded.
- Any pair involving the previously frozen/unreliable traffic-light stream was
  excluded from Task C.

## Final quality

| Sensor | Plane edges | Direct pair edges | Independent captures | Bootstrap P95 position | Bootstrap P95 angle | Cross-day difference | Status |
|---|---:|---:|---:|---:|---:|---:|---|
| front-middle USB | 225 | 14 | 115 | 7.119 mm | 0.1018° | 54.46 mm / 0.522° | review only |
| rear-middle USB | 6 | 3 | 7 | 8.457 mm | 0.2438° | 25.89 mm / 1.518° | review only |
| front fisheye | 83 | 22 | 32 | 6.274 mm | 0.1803° | 19.50 mm / 1.311° | review only |
| rear fisheye | 32 | 3 | 18 | 7.618 mm | 0.2220° | unavailable: day 1 has only 3 plane edges | review only |

The run met the relaxed bootstrap gate at round 205 but none of the four nodes
met all physical acceptance gates.  The front pair contains multiple relative
pose modes (`dominant_fraction` 0.286 and 0.364), consistent with identical-ID
board association ambiguity and planar/depth sensitivity.  The rear-middle
camera has fewer than eight independent usable captures.  The rear fisheye has
more plane observations, but lacks a solvable second-day-independent pose and
is dominated by similar rear-facing plane geometry.

## Why the additional data did not fix the four weak cameras

More frames do not add all six degrees of freedom when most boards have nearly
parallel normals.  Plane-only evidence strongly constrains orientation and
normal distance, but weakly constrains translation along the plane.  Repeated
captures of the same pose therefore reduce random noise without removing the
geometric ambiguity.  Rolling-shutter timing, ToF edge returns, partial tags,
and identical board IDs create secondary modes; the robust solver rejects
them instead of averaging them into a plausible-looking but wrong result.

## Minimum recapture needed

- Rear-middle USB: at least 20 accepted static snapshots.  In each, rear-middle
  and one already stable rear-left/rear-right camera must see the same board.
  Use 3–6 m distance, at least three board heights, and yaw/pitch changes of
  roughly ±20–35 degrees.
- Rear fisheye: at least 20 accepted snapshots shared with rear-left or
  rear-right, including board centres near image centre and all four image
  quadrants.  Avoid using only a vertical board at one height.
- Front-middle/front fisheye: 12–20 shared snapshots with front-left or
  front-right, with one uniquely identifiable board in the overlap.  If four
  identical boards remain in view, temporarily cover three or assign distinct
  tag-ID ranges.
- For every capture: park the vehicle and board, wait at least one second, then
  capture.  Do not click repeatedly without changing pose.  Prefer a hardware
  or common ROS timestamp trigger; otherwise keep the measured source stamps.

## Reproduction and audit files

- `multiday_hierarchical_result.yaml`: final 205-round review-only candidate.
- `rounds.csv`: machine-readable per-round convergence summary.
- `day1_observations.json`: fresh static-day detections and gate counters.
- `attempt1_no_static_pair_edges.yaml`: preserved earlier 500-round attempt
  before weak/weak pair coupling; retained for audit, never for deployment.
- `../../process/part12_joint_extrinsics/MULTIDAY_ITERATION_LOG.md`: all 205
  corrected rounds.
