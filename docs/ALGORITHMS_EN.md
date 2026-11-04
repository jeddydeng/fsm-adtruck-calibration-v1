# Algorithms and coordinate contracts

## Intrinsics

The ordinary/traffic camera solver detects ChArUco corners, calibrates a pinhole model,
rejects high-error views and exports ROS CameraInfo. The robust fisheye solver uses
an equidistant projection, deterministic seed trials, independent pose/reprojection
filtering and refitting, then held-out/bootstrap stability checks. Duplicate board
IDs are split by physical marker components rather than combined into one plane.
The fixed-point camera geometry is 12x8, 70/50 mm. Board parameters are code constants
in the original solvers; config/boards.yaml documents their contract. Changing that
file alone does not redesign the detector. Change and test every consumer together.

## Extrinsics

The roof lidar is the reference. Static-scene capture records complete raw PCD plus
images and source/receive timestamps. Preview plane matching is only guidance.
Offline algorithms re-detect tag corners, associate physical boards using seeds,
fit local lidar planes, use camera-camera common-board constraints and optimize
robust residuals. Fixed-board/moving-vehicle and stationary-vehicle/moving-board
models are distinct. The latter permits temporal background reasoning; the former
does not. Candidate association errors can lock an optimizer into a precise wrong pose.

The multiday historical solver uses robust resampling, a 10 cm conflict rule,
majority consistency and a finite iteration cap. Bootstrap scatter is repeatability,
not independent absolute accuracy. Front/rear central-camera and rear-fisheye
weak modes remain explicitly labeled in archived reports; repackaging does not
certify them. Source timestamp validity, geometric observability and deployment
status must be reviewed separately.

## BEV

Ground-board detection uses the fisheye model and repeated-grid segmentation.
Absolute-candidate generation combines lens projection, camera pose and the ground
plane into destination-to-source maps. Relative optimization adjusts image-plane
SE(2) alignment with fixed scale, checks all four seams and ring coverage, and
exports a status limited to BEV_relative_planar_mapping_only.
Raised objects have unavoidable parallax under a single-ground-plane mapping.

## Matrix directions

- Vehicle: x forward, y left, z up. Optical: x right, y down, z forward.
- T_parent_child means p_parent = T_parent_child * p_child, homogeneous column vectors.
- A camera_link frame is not an optical frame; use CAMERA_OPTICAL_TO_LINK explicitly.
- Initial solver YAML xyz_m is metres; rpy_deg is fixed XYZ degrees.
- ROS sensor-kit YAML roll/pitch/yaw is radians. Do not copy the numbers without conversion.
- p_vehicle = R_vehicle_camera * p_camera_optical + t_vehicle_camera_m.
- p_bev ~ H_bev * p_rectified, with homogeneous normalization.
- remap arrays map each BEV output pixel to raw distorted input coordinates.
- Existing kit YAML places BPearl numeric offsets beneath a top-level key for legacy
  compatibility, but sensor_tree.xacro attaches those sensors to the roof lidar.
  Read the TF tree, not the YAML indentation alone, to determine the parent.

## Reference material

- [OpenCV ChArUco calibration](https://docs.opencv.org/4.x/da/d13/tutorial_aruco_calibration.html)
- [OpenCV fisheye projection](https://docs.opencv.org/4.x/db/d58/group__calib3d__fisheye.html)
- [ROS frame conventions, REP 103](https://www.ros.org/reps/rep-0103.html)
- [ROS Jazzy native installation](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)
