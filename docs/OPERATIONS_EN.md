# Black vehicle operator runbook

This branch targets the black vehicle (对坐黑车), not the reference-vehicle presets.
The authoritative step-by-step guide is [BLACK_VEHICLE.md](BLACK_VEHICLE.md).

- IPC: cityu02@100.93.32.20; checkout: ~/projects/fsm-adtruck-calibration-v1.
- Ubuntu 22.04 / ROS 2 Humble / Python 3.10, isolated .venv, ROS domain 0.
- Four RoboSense lidars: settings and existing Autoware initial poses under config/black_vehicle.
- Six GMSL cameras: video0,3,4,5,6,7; 1920x1080 UYVY, card0 internal trigger 10 Hz.
- Four Pi187 fisheyes: native 800x600 MJPEG, transmitted at 10 Hz. Do not resize to 1600x1200 or install another vehicle's intrinsics.
- The fifteenth traffic-light-camera slot and every corresponding workflow remain available. It is optional and not installed; absence never gates saving the other 14 streams.

Install the Python dependencies using ./install.sh --python-only, then ./scripts/build.sh and ./install.sh --check when ROS is already installed. Full installation is for a dedicated fresh Ubuntu machine; the shared IPC reuses its existing ROS installation.

Entry points:

```bash
./start_intrinsics.sh
./solve_intrinsics.sh 1 /absolute/GMSL/session
./solve_intrinsics.sh 7 /absolute/fisheye/session
./start_joint.sh
./solve_joint.sh fixed-boards --data-root /absolute/captures --initial /absolute/black_initial.yaml --output /absolute/result.yaml
./solve_bev.sh detect --help
./solve_bev.sh absolute --help
./solve_bev.sh relative --help
./solve_bev.sh verify --help
./traffic_cam_start.sh
./stop.sh
```

Intrinsic keys 1..6 select GMSL0,3,4,5,6,7; 7/8/9/0 select front/rear/left/right fisheyes. '-' retains the optional traffic camera. CAMERA_CAPTURE_HEADLESS=1 enables bounded capture over SSH. No real target board means insufficient calibration views (exit 2), not a software crash or a completed calibration.

Keep ChArUco geometry, AprilGrid geometry, solver algorithms, motion protocol and quality thresholds unchanged. Fisheye image dimensions are read from CameraInfo throughout the BEV workflow. The relative BEV task retains the original 14 distinct captures and three-stage board movement protocol.

Hardware acceptance: ./scripts/validate_black_capture.sh. Synthetic solver acceptance: ./scripts/validate_black_workflows.sh. Synthetic outputs are explicitly marked, excluded from Git and never installed as vehicle calibration. Historical result/ artifacts remain regression references only.

The GMSL card0 supplier SDK snapshot is included under drivers/black_vehicle/vendor; the approved kernel module is preserved byte-for-byte. Other kernels require an appropriate supplier-supported module, not an implicit module replacement.
