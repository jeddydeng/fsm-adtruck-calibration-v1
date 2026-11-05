# Operator and agent runbook

This is the authoritative runbook for the repackaged repository. Module README files
and reports under result/ describe historical acquisitions; their old absolute paths
and deployment decisions are not current installation instructions.

## 1. Scope and installation

The three workflows are (A) camera intrinsics, (B) multi-lidar/camera extrinsics,
and (C) four-fisheye ground mapping and BEV composition. Camera capture is 6 USB
MJPEG cameras, 4 network fisheyes and 1 ROS CompressedImage traffic camera.
Four raw PointCloud2 streams are captured without ground removal.

Deployment target: native Ubuntu 24.04, ROS 2 Jazzy, system Python 3.12.
Compatibility target: Ubuntu 22.04, ROS 2 Humble, system Python 3.10.
Offline solvers support Python 3.10–3.12 on Linux/macOS. The camera GUI requires a
desktop display. Live capture requires Linux /dev and ROS; macOS is not a driver target.

```bash
git clone git@github.com:jeddydeng/fsm-adtruck-calibration-v1.git
cd fsm-adtruck-calibration-v1
./install.sh                 # native apt + venv + ROS source build + checks
./install.sh --check         # repeat software checks only
# Offline workstation, no sudo/ROS installation:
./install.sh --python-only
```

Do not run install.sh itself as root. It invokes sudo for apt, rosdep initialization
and adding the current user to video/dialout. Log out/in for group changes.
The script does not modify network addresses, BIOS, systemd startup, CAN state,
Autoware files, or the existing white_ws. It obtains ROS's official apt-source
package if ROS is absent. Internet and sudo access are required for --full.
The pinned Python dependencies go in .venv; ROS Python extensions come from the
installed distribution via --system-site-packages. Never install a different
Python interpreter into a ROS venv. PYTHONNOUSERSITE prevents unrelated user packages
from influencing execution. No Docker is required.

Source installation commands are in scripts/build.sh. Use BUILD_WORKERS=2 on
memory-limited computers. Compilation artifacts remain under ros_ws/{build,install,log}.
If CMake/compiler/ROS distribution changes, use a fresh checkout/build directory.

## 2. Configuration and sensor identity

Copy config/local.env.example to config/local.env for overrides. This is trusted
shell syntax and must not contain untrusted downloaded code. Paths resolve from
the repository, not the current working directory.

The existing vehicle's sensor LAN is 192.168.1.0/24; IPC 192.168.1.102, Pi
192.168.1.186. Set the dedicated sensor NIC manually without a default gateway.
The installer intentionally cannot choose a NIC on a different computer.
Check ip route get 192.168.1.186 and the packet destination of each lidar.
Do not assume lidar source-IP order corresponds to physical position.

| Physical lidar | Model | MSOP / DIFOP | PointCloud2 topic |
|---|---|---|---|
| Roof | RSHELIOS 32-line | 6699 / 7788 | /sensing/lidar/top/pointcloud_raw |
| Front-left blind-zone | RSBPV4 | 6702 / 7791 | /sensing/lidar/front_left/pointcloud_raw |
| Front-right blind-zone | RSBPV4 | 6700 / 7789 | /sensing/lidar/front_right/pointcloud_raw |
| Rear blind-zone | RSBPV4 | 6701 / 7790 | /sensing/lidar/rear/pointcloud_raw |

Edit ros_ws/src/rslidar_sdk/config/config.yaml and the matching entries in
process/joint_extrinsic_station/config.yaml together. scripts/verify.sh checks
their agreement. Driver transforms remain identity; TF supplies extrinsics.
Stored PointCloud2 field layout includes timestamps and must not be silently
converted into an XYZ-only cloud before archival.

USB mapping is in ros_ws/src/camera_usb_pkg/config/camera_devices.yaml and the
station's usb_capture block. Use v4l2-ctl --list-devices, /dev/v4l/by-path and
udevadm info to confirm physical identities. Cameras without unique serials require
fixed USB socket placement; identical models cannot be reliably ordered by arbitrary
hotplug sequence. Update both mappings after moving sockets, and verify actual images.

| Key | Name | Default acquisition |
|---|---|---|
| 1 | video_rear_left | 640x480 MJPEG |
| 2 | video_front_left | 640x480 MJPEG |
| 3 | video_front_right | 640x480 MJPEG |
| 4 | video_rear_right | 640x480 MJPEG |
| 5 | video_rear_mid | 640x480 MJPEG |
| 6 | video_front_mid | 640x480 MJPEG |
| 7 | fisheye_front | 1600x1200, equidistant |
| 8 | fisheye_rear | 1600x1200, equidistant |
| 9 | fisheye_left | 1600x1200, equidistant |
| 0 | fisheye_right | 1600x1200, equidistant |
| - | traffic_light_front | 320x240 at 5 Hz, ROS compressed |

The six ordinary cameras are direct V4L2 captures, not six permanent ROS publishers.
Traffic topic: /sensing/camera/traffic_light_front/image_raw/compressed.
Corresponding calibration topic: /sensing/camera/traffic_light_front/camera_info.
Default ROS domain 92, CycloneDDS. To subscribe across computers, use SUBNET discovery
on both endpoints and allow DDS multicast; remove inherited loopback-only DDS profiles
only in the relevant terminal. The scripts preserve an explicitly supplied DDS URI.

## 3. Pi transport and optional sensors

drivers/pi_sender contains the complete existing Raspberry Pi sender release,
including four-camera V4L2/BEV C++ sources, the traffic-camera sender, protocol,
parameters, service templates and its own deployment guide. Install it on the Pi
only, using drivers/pi_sender/docs/INSTALLATION.md, running its commands from
drivers/pi_sender inside this checkout instead of a separate FSM-Pi-001 root.
Its historical vehicle-I/O and
fan services are not part of the IPC calibration installer.

drivers/pi_gateway contains the IPC W102 receiver and traffic ROS bridge. The root
start_joint.sh starts its camera gateway only if the configured local HTTP API is
unavailable. It uses data/control UDP ports 5610/5611 and HTTP 127.0.0.1:8086.
Set CALIBRATION_AUTO_GATEWAY=0 to manage it externally.
For a Pi that sends traffic JPEG over the supplied UDP protocol, set
CALIBRATION_AUTO_TRAFFIC=1 to start the 5620/5621 receiver and ROS publisher.
Default is 0, because an existing independent ROS publisher may already supply it.
Do not start two publishers/receivers for one physical stream.

For intrinsic capture, start the receiver beforehand when it is not an existing service:
```bash
./scripts/start_pi_receiver.sh fisheye
# separate terminal, only if no traffic ROS publisher exists:
./scripts/start_pi_receiver.sh traffic
```

These foreground processes stop with Ctrl+C. The receiver's voice UI is historical
optional functionality; voice assets and the unrelated production service installer
are omitted. Use the root installer and wrappers for calibration.

All additional vehicle sensor sources are included. They are optional for these
three calibration workflows and are not started by start_joint.sh:
```bash
./scripts/start_aux_sensors.sh imu
./scripts/start_aux_sensors.sh gnss
./scripts/start_aux_sensors.sh ultrasonic
./scripts/start_aux_sensors.sh can-rx
./scripts/start_aux_sensors.sh lidar
```

Run one in each terminal; stop with Ctrl+C. Device arguments are ROS launch
arguments, e.g. IMU_ComName:=/dev/serial/by-id/... and device:=... for ultrasonic.
CAN setup is an operator prerequisite; the can-rx wrapper enforces enable_tx=false.
There is no autonomy/arming entry point in this repository.

## 4. Workflow A: camera intrinsics

Board: ChArUco 12 columns x 8 rows, square length 0.070 m, marker length 0.050 m,
DICT_5X5_100. The active grid is 0.840 x 0.560 m on a 0.900 x 0.620 m sheet.
Do not substitute Tag16h5 in this workflow.

```bash
./start_intrinsics.sh
# Press one key from the table above.
./solve_intrinsics.sh 2 /absolute/path/to/video_front_left_session
./solve_intrinsics.sh 7 /absolute/path/to/fisheye_front_session
```

The preview displays detection feedback and captures diverse frames from the stream.
Press q/w/e/a/s/d/z/x/c in the image window, or Ctrl+C, to finish.
The hard maximum is 1200 s. CAMERA_CAPTURE_DURATION_SEC can shorten it.
CAMERA_TARGET_SAMPLES controls the capture target; the default is 70.
For a robust fisheye session, set CAMERA_TARGET_SAMPLES=180 and aim for at least
100 accepted observations. Video duration alone does not establish diversity.

Move a flat, rigid board slowly through center, corners, edges, multiple distances
and approximately 15–45 degree tilts. Keep shutter blur low and lock resolution,
focus and digital crop. Approximate useful distance for a 0.9 m board and ordinary
45–120 degree lenses is 0.8–3 m, but determine it by full-board visibility and
adequate pixel size; do not treat distance as a calibration constraint.
For 180-degree fisheyes deliberately include the peripheral image, while preserving
detectable tag size. Resizing/cropping a stream invalidates its old intrinsics.

USB/traffic solve uses calibrate_intrinsics.py. Four fisheyes always use
process/robust_fisheye_intrinsics.py (OpenCV equidistant, four coefficients).
The old left-fisheye pinhole fallback has been removed from the unified entry point.
Traffic's small 320x240 image uses constrained pinhole fitting.
Outputs are under output/intrinsics; no live CameraInfo is overwritten.

Inspect reprojection overlays, full-frame coverage, per-view error, held-out error,
and stability across independent subsets. Low training RMS alone is insufficient.
Fisheye reports record quality_pass and bootstrap/held-out diagnostics.
Existing accepted presets are under config/intrinsics; archive your previous
parameters before replacing them with reviewed new results.

## 5. Workflow B: joint lidar/camera extrinsics

```bash
./start_joint.sh
# browser: http://127.0.0.1:8765
./stop.sh
# Alternative view, four lidars plus only the traffic camera:
./traffic_cam_start.sh
./stop.sh
```

The page uses green/red sensor panels, tag detections and frame freshness.
The save button remains usable with missing sensors. Missing streams are skipped,
recorded in manifest.json and warned at bounded intervals. A green panel is only a
capture aid; wall/column false positives must be rejected during offline fitting.
Lidar detects board geometry, not an AprilTag ID.

Use four identical 0.9 x 0.9 m Tag16h5 boards, each with 6x5 tags (IDs 0–29).
Black tag edge 0.108 m, horizontal gap 0.0304 m, vertical gap 0.065 m; active grid
0.8 x 0.8 m, 0.05 m margins. ID origin bottom-left. Printed marker corner rotation
is documented in config/boards.yaml. Repeated IDs are not unique physical-board IDs.

For static-vehicle acquisitions, place front/left/right boards around 2–5 m away
and the rear board high enough to clear the vehicle's roof-lidar blind region.
Keep front and side boards visible to the corresponding 45-degree BPearls.
Move one board at a time, varying position and orientation. Keep everything static
for capture and record which board moved. Preserve independent test poses.

For the fixed-board moving-vehicle solver, keep all four boards fixed during each
part, move the vehicle to front/rear/oblique/side views, stop before every capture,
and record separate part1/part2 lists when board spacing changes. Never apply
static-background subtraction across a moving-vehicle sequence.

Each click writes data/raw_extrinsic_data/<timestamp>/ with cameras/*.png,
lidars/*.pcd, intrinsics/*.yaml, manifest.json and config/initial-transform snapshots.
Set CALIBRATION_DATA_ROOT to the 2 TB disk when needed. Raw files stay out of Git.
Capture is software latest-frame matching, not hardware synchronization.
The saved per-stream timestamps and freshness remain essential even for static work.

Arrange a new dataset base as follows:
```text
dataset/
  data/raw_extrinsic_data/<capture_id>/{manifest.json,cameras/,lidars/,intrinsics/}
  data/backup_manifests/20260903/part1_capture_groups.txt
  data/backup_manifests/20260903/part2_capture_groups.txt
  result/camera_intrinsics/<ordinary_or_traffic_name>.yaml
  result/fisheye_intrinsics_1600x1200_20260903_v2/<fisheye_name>/camera_info.yaml
```

Those dated folder names are the preserved v1 dataset schema, not acquisition-date
requirements. Each list contains one capture directory name per line. Copy actual
session intrinsics into that base; never infer a 1600x1200 model by scaling a cropped
640x480 calibration. Historical lists in this repository are reference metadata.
Do not reuse them for unrelated new acquisitions.

Example fresh solve (explicit paths avoid mixing cached observations):
```bash
./solve_joint.sh fixed-boards --base /absolute/dataset \
  --config process/joint_extrinsic_station/config.yaml \
  --seed result/extrinsics/all_sensors_relative_to_roof_lidar.yaml \
  --observations output/joint/observations.json \
  --output output/joint/extrinsics.yaml \
  --rounds-log output/joint/rounds.md --extract --max-rounds 500
```

The seed is an initialization, not ground truth. This solver re-detects image
corners and fits board planes in raw PCD. Its default camera/lidar timestamp gate
is 100 ms; use --max-sync-ms explicitly only for demonstrably static captures.
The multiday solver is a historical specialized refinement that freezes a subset
of the September vehicle sensors; it is not a generic estimator for a remounted kit.
Use ./solve_joint.sh multiday --help and read its module README before using it.
For standalone garage-structure lidar refinement:
```bash
.venv/bin/python process/joint_extrinsic_station/refine_lidar_environment.py \
  --data-root /absolute/dataset/data/raw_extrinsic_data \
  --initial process/joint_extrinsic_station/initial_extrinsics.yaml \
  --output output/lidar_environment.yaml --max-captures 24
```

Other moving-board graph/bundle variants are retained in process/joint_extrinsic_station
for audit and research; choose them from their --help and source docstrings.
Do not run all historical rounds blindly. A 500-round limit is a termination guard,
not a precision guarantee. A stable wrong solution must still fail independent checks.

Traffic-specific extrinsics use ./solve_joint.sh traffic --help, with explicit
--data-root, --part4-list, --intrinsic and --output arguments. Re-detect saved pixels;
exclude repeated/frozen imagery and inappropriate moving scenes. Keep static-fit
results and dynamic timestamp validity as separate evidence.

## 6. Workflow C: fisheye extrinsics and BEV

Use the same joint capture page, with four ground boards near the vehicle corners.
Keep the vehicle fixed, capture the initial board position, move all four boards
radially outward about 0.45 m, capture again, then repeat. Include common ground
texture between adjacent cameras. Record each capture group in a text file.
The v1 relative optimizer and its scale validator are specialized to exactly
14 groups, ordered as 3 initial + 6 first-move + 5 second-move observations.
For a new dataset using this entry point, follow that same grouping and move each
board by approximately 0.45 m in BOTH longitudinal and lateral components (about
0.636 m diagonally). Otherwise adapt radial_stage_validation and its checks first.
The detection and absolute-candidate tools can process other group counts, but the
relative usability validator must not be treated as generic.

All four inputs must be the full, uncropped 1600x1200 raw fisheye images.
The following steps create a new absolute-candidate package and then a separately
validated relative ground mapping:
```bash
./solve_bev.sh detect \
  --data-root /absolute/dataset/data/raw_extrinsic_data \
  --group-list /absolute/selected_groups.txt \
  --intrinsics-root result/fisheye_intrinsics_1600x1200_20260903_v2 \
  --output output/bev/detections

./solve_bev.sh absolute \
  --data-root /absolute/dataset/data/raw_extrinsic_data \
  --detections output/bev/detections/board_detections.jsonl \
  --intrinsics-root result/fisheye_intrinsics_1600x1200_20260903_v2 \
  --camera-extrinsics result/extrinsics/camera_relative_to_roof_lidar.yaml \
  --output output/bev/baseline

./solve_bev.sh relative \
  --data-root /absolute/dataset/data/raw_extrinsic_data \
  --selected /absolute/selected_groups.txt \
  --raw-index output/bev/detections/raw_image_index.txt \
  --baseline output/bev/baseline --output output/bev/relative

./solve_bev.sh verify --package output/bev/relative \
  --data-root /absolute/dataset/data/raw_extrinsic_data --baseline output/bev/baseline
```

For a new rig, replace all intrinsic/extrinsic paths above with reviewed new ones.
The baseline must contain valid metric ground remaps; relative optimization corrects
seams with scale fixed and cannot identify a wrong metric scale or global vehicle pose.
Never label an absolute R/t valid just because the seam optimizer passed.

Deliver K/D, camera model and resolution, R/t with direction, H_bev, maps, board
geometry, input hashes, per-camera error, reprojection/grid overlays and four-way preview.
result/fisheye_bev_20260903/relative_usable retains the historical usable planar maps.
The map is 512x512 at 0.015 m/pixel; u points right (-vehicle y), v down (-vehicle x).
Use scripts/render_bev.py for a deterministic preview from four new raw images.
H_bev acts on rectified pinhole pixels; it must never be applied directly to distorted
fisheye pixels. cv2.remap with *_bev_map_x/y.npy is the preferred runtime path.

## 7. Shutdown, diagnostics and acceptance

stop.sh terminates only process groups recorded by this checkout, after checking
Linux process start ticks to guard against PID reuse. The station stops first so
the SDK can restore the previous Pi mode. Existing gateway/ROS publishers are reused
and left running. Foreground intrinsic/auxiliary commands stop in their own terminal.
Logs: run_logs/. Runtime ownership records: .runtime/. Both are Git-ignored.

If the browser does not open over SSH, use a tunnel:
```bash
ssh -L 8765:127.0.0.1:8765 user@ipc
```
Then open http://127.0.0.1:8765 locally. Default bind is loopback; only set
CALIBRATION_BIND_HOST=0.0.0.0 on a trusted network when direct access is needed.
If a USB camera is busy, stop the other process that owns it before capture.
If fisheye HTTP fails, inspect Pi sender, ports, IP route and selected stream mode.
If traffic image is absent, check ROS domain, QoS/discovery and whether the UDP bridge
or a native ROS publisher is expected. Do not launch both.

Before accepting on the vehicle: compare source hashes with the live white_ws,
build on Jazzy, confirm 15 fresh streams and actual physical mapping, save one full
capture, inspect each PNG/PCD/manifest and test unplug/replug on a parked vehicle.
Review all calibration residuals on independent images/poses. Software checks on an
unconnected workstation do not establish live sensor reliability or metric accuracy.
Use docs/VALIDATION.md to distinguish completed tests from pending hardware checks.
