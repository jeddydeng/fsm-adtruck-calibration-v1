# Vehicle source audit — November 5, 2026

## Scope and method

Read-only SSH inspection of the online vehicle IPC, comparing the packaged
repository at 0b253a5 with white_ws/src, white_ws/calibration and the Pi capture,
sender and receiver source copies stored on the IPC. No vehicle source/configuration
was changed; no hardware driver, Autoware, CAN transmitter or arm service was started.
No recorded images, point clouds or bags were downloaded. Small source/configuration
snapshots and file hashes were used for comparison.

Python AST comparisons distinguish formatting-only changes from algorithm changes.
YAML values and SHA-256 checks distinguish parameter or numeric-map differences.
The Pi sender check covers the copy on the IPC, not a separately connected live Pi.
This is not a full white_ws/Autoware backup or a run-time sensor acceptance.

## Findings and disposition

| Component | Result | Disposition |
|---|---|---|
| ROS drivers and sensor-kit configuration | 147 shared source/config files: 142 byte-identical, 3 Python AST-equivalent, 2 different before synchronization | Preserve packaging header-install fix; synchronize the IMU parameter file below |
| white_calibration_bringup | 3 new repository files, absent on vehicle | Keep independent sensing-only bringup; do not overwrite vehicle white_bringup |
| 11 camera intrinsic presets | All match their corresponding deployed accepted results; the four fisheyes match v2 1600x1200 equidistant files | No change |
| Deployed sensor-kit static transforms | sensor_kit_calibration.yaml and sensors_calibration.yaml match | No change |
| Joint capture/solver | Common capture implementation is based on the same source; repository retains path/API, board-corner and fitting safeguards | Do not replace it with the older deployed solver |
| Offline solver variants | 20 files in repository joint_extrinsic_station are not at that vehicle station path; part12 offline solvers originate from the Mac analysis workflow | Preserve Mac-derived solvers; absence is not a reason to delete them |
| Robust fisheye intrinsic and validation programs | Both Python AST-equivalent | No change |
| Intrinsic launch script | Vehicle's older selection 9 still chooses pinhole; repository chooses robust equidistant for all four fisheyes | Retain repository fix; vehicle script not modified in this audit |
| Traffic extrinsic tools | 3 Python files AST-equivalent | No change |
| BEV tools | 4 Python files AST-equivalent; relative optimizer and launch wrapper retain documented packaging fixes | No rollback |
| Relative BEV parameters/maps | 4 YAML files and all 8 NPY remap hashes match | No change |
| IPC Pi gateway | 18 shared source/config files byte-identical | No change |
| IPC copy of Pi sender | 31 shared source/config files byte-identical | No change |
| Pi runtime physical-center BEV | Four sender YAMLs match vehicle 512 exports; all 4 XML.GZ map hashes match | No change |
| Pi capture SDK/helpers | 4 shared source files identical; 2 launcher/client differences are repository-relative paths | Keep portable paths |

Joint solver differences include OpenCV DetectorParameters compatibility, the
printed board's two-quarter corner-index rotation, configurable ground/visible extent,
and rejection of board fits far from camera predictions. The Mac research version
already contained several of these improvements before packaging. The capture config
also uses loopback binding, repository paths and the standalone lidar launch package.
These differences are intentional, not evidence that the vehicle is more up to date.

The IPC also stores 1024 and 1200 physical-center BEV exports, in addition to the 512
export already packaged with the Pi sender. They change output grid size/scale, not
camera calibration. They were inventoried but not added or made defaults by this audit.
The 512 sender export uses 0.021 m/pixel and vehicle-center origin; the historical
relative solver package uses 0.015 m/pixel. Do not interchange their homographies/maps.

## Approved IMU update

Only ros_ws/src/white_sensor_kit_description/config/imu_corrector.param.yaml was
synchronized from the vehicle. The vehicle file was last modified September 17.
Its own comments describe September 12 H30 stationary measurements: 452,782 samples
over 75 minutes. The raw measurements were not recalculated in this audit.

| ROS parameter | Previous repository value | Vehicle value now adopted |
|---|---:|---:|
| angular_velocity_offset_x | 0 | -0.0027668841826491096 |
| angular_velocity_offset_y | 0 | -0.0014558748082802262 |
| angular_velocity_offset_z | 0 | -0.0020824572465917425 |
| angular_velocity_stddev_xx | 0.03 | 0.003 |
| angular_velocity_stddev_yy | 0.03 | 0.003 |
| angular_velocity_stddev_zz | 0.03 | 0.003 |

Angular rates use rad/s; imu_corrector subtracts the raw-frame offsets. This is a
configuration synchronization, not a new IMU extrinsic calibration or an accuracy
claim. Different IMU hardware or mounting requires fresh validation.

## Publication and verification

The owner explicitly approved public visibility while retaining original license
declarations. Anonymous GitHub API access confirmed private=false, visibility=public
on November 5. Credential-pattern inspection found no SSH private keys or GitHub
tokens in the tracked files; this limited check is not a formal security audit.
Existing Proprietary notices remain; public access does not relicense those components.

Cloud Ubuntu 24.04/Jazzy installation, build and synthetic capture passed for 0b253a5:
[GitHub Actions evidence](https://github.com/jeddydeng/fsm-adtruck-calibration-v1/actions/runs/41946023120).
See VALIDATION.md for software checks and remaining physical-hardware acceptance.
