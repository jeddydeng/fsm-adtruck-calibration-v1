# Validation record — 2026-11-05

## Completed checks

| Environment | Check | Result |
|---|---|---|
| Ubuntu 22.04.5, ROS 2 Humble, Python 3.10.12 | Native colcon source build | 12 packages finished; no build errors |
| Same Linux workstation | Repository pytest suite | 22 passed |
| Same Linux workstation | Capture station config/import check | 4 lidars + 11 cameras accepted |
| Same Linux workstation | Isolated ROS capture smoke test | Passed; 4 PCD + 1 PNG, manifest, stale-data skip and shutdown |
| Same Linux workstation | Seven extrinsic/BEV solver CLI entry points | All --help commands exited successfully |
| macOS, Python 3.11.15 | Offline dependency installation and pytest | 15 passed; 7 Linux/ROS-only tests explicitly skipped |
| Both | Syntax, fatal lint and configuration audit | 84 Python sources, 28 shell scripts, 11 camera presets, 4 lidar mappings, 8 BEV maps checked |

The 22 tests include ChArUco geometry and synthetic detection, AprilGrid pose,
lidar-target geometry, transform direction, BEV map/render/status checks, PID
ownership and Pi protocol CRC/out-of-order/atomic-save tests. They do not establish
real-world calibration accuracy. Raw calibration data was not reprocessed in this
packaging task.

The ROS smoke test uses domain 193, localhost discovery, temporary synthetic data
and HTTP port 18765. It disables all hardware auto-start paths. It checks preservation
of 32 points per PCD, the five-file manifest, expiration after publishers stop,
partial capture with missing inputs and an empty managed-process record after stop.
No chassis CAN output or arm service is used. Re-run only while this checkout is idle:

```bash
./scripts/verify.sh
./scripts/smoke_station.sh  # Linux + built ROS overlay only
```

The workstation had pre-existing ROS/system development dependencies. The Python
installer and source build were exercised, but this is **not** a fresh-OS full apt
installation acceptance. Vendored RoboSense C++ generated missing-field-initializer
warnings; they were not build failures. CycloneDDS reported loopback multicast
disabled in the isolated smoke test; message delivery nevertheless passed.

## Defects found and fixed during packaging

- Public serial-helper headers were installed outside the package-scoped include
  path expected by the generated ROS interface target.
- A leftover COLCON_IGNORE hid the required autoware_sensing_msgs package in the
  standalone workspace.
- Capture-station AprilTag initialization used the removed OpenCV factory API.
  It now supports the installed OpenCV 4.10 constructor and has a regression test.
- One historical fisheye entry path selected a pinhole model. All four fisheye keys
  now select the robust equidistant solver.
- Per-camera BEV YAML could claim deployment allowed despite a failed aggregate
  quality gate. The statuses now agree, including insufficient board-motion data.
- Hard-coded workstation paths and duplicate production installers were removed
  from authoritative entry points. Existing external services are preserved on stop.

## Pending acceptance / limits

1. Vehicle source comparison: vehicle computer was unavailable during packaging.
   Start with scripts/compare_vehicle_snapshot.sh after power-on; also compare live
   calibration and Pi sources before accepting any newer differences.
2. Native Ubuntu 24.04 / Jazzy fresh-machine installation and build. A GitHub Actions
   workflow is supplied, but an unobserved or not-yet-run workflow is not a pass.
3. Live hardware: correct physical camera/lidar mapping; 15 fresh streams; Pi sender
   build/deployment; full capture; unplug/replug; timestamp validity; long-run behavior.
4. New calibration quality: independent held-out images and geometric measurements.
   Historical result labels remain historical. Seam quality does not prove absolute
   vehicle-frame pose accuracy. No millimeter-level accuracy is promised by tests.
5. BEV relative validator remains specialized to the documented 14-group, 3/6/5-stage
   acquisition. Different datasets require adapting that validator before acceptance.
6. Sensor drivers are included; no Autoware runtime or autonomous-driving acceptance
   is implied. Optional voice/vehicle-I/O UI is outside calibration acceptance.

Original Mac backups and vehicle configuration were not modified. Source manifests
exclude build outputs, virtual environments, runtime logs and original imagery/bags.
