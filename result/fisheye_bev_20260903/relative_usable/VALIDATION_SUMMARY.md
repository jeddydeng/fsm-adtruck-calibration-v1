# Four-fisheye relative BEV validation

- BEV status: **USABLE**
- BEV deployment_allowed: **1** (relative planar mapping only)
- Absolute vehicle R/t: **review only / not deployable**
- Data: final 14 Part3 captures only; 56 fresh 1600x1200 fisheye images
- Map: 512x512, 0.015 m/px; metric scale held fixed
- Combined 0.75–3.50 m annulus coverage: 97.66%
- Two-or-more-camera overlap: 60.52%

## Seam residuals

| overlap | retained/raw | retained median | retained P95 | all P95 |
|---|---:|---:|---:|---:|
| front<->right | 195/195 | 0.738 px | 1.837 px | 1.837 px |
| right<->rear | 56/60 | 1.771 px | 3.868 px | 5.298 px |
| rear<->left | 260/260 | 1.684 px | 2.798 px | 2.798 px |
| left<->front | 113/118 | 0.162 px | 1.170 px | 1.407 px |

## Per-camera reprojection errors

| camera | intrinsic RMS | accepted intrinsic-view P95 | Part3 board-pose median | Part3 board-pose P95 |
|---|---:|---:|---:|---:|
| front | 0.361 px | 0.734 px | 1.370 px | 2.064 px |
| right | 0.497 px | 0.688 px | 1.231 px | 1.398 px |
| rear | 0.520 px | 0.840 px | 1.689 px | 1.901 px |
| left | 0.454 px | 0.758 px | 1.140 px | 1.296 px |

All reported usable transforms preserve scale exactly. The residual is an image-space
overlap measure; it is not a millimetre claim. At 15 mm/px, 5 px equals 75 mm.
Raised objects can disagree because a single ground homography cannot remove parallax.

## Coordinate and matrix direction

- Vehicle: x forward, y left, z up.
- BEV: u right = -vehicle y; v down = -vehicle x; vehicle origin=(256,256).
- `C_bev_from_baseline`: `p_final_bev ~ C * p_baseline_bev`.
- `H_bev`: `p_bev ~ H * p_rectified`; never apply it to raw equidistant pixels.
- Runtime path: apply the saved `*_bev_map_x.npy`/`*_bev_map_y.npy` directly with `cv2.remap`.

## Absolute-pose warning

The supplied R/t values are retained only for traceability. Cross-session calibration did
not sign them, and the rear fisheye absolute height showed a material conflict. This package
therefore authorises only surround-view ground-plane rendering, not Autoware sensor TF.
