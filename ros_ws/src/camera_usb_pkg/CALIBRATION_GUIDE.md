# Six-camera direct-capture calibration workflow

The six cameras must be calibrated separately at their final 640x480 MJPEG mode. Normal
operation does not publish any of these six observation cameras to ROS. Collect all six
directly from their stable `physical_device` or `by-id` entries without starting a ROS camera
driver. `video_front_mid` now follows exactly the same lifecycle as the other five cameras and
is not used for Autoware traffic-light recognition. Do not change focus, zoom, crop or
resolution after calibration.

The measured ChArUco board is 12 x 8 squares, 70 mm square length, 50 mm marker length and
`DICT_5X5_100`. For each camera, stop the normal camera launch and collect at least 50--70
sharp, diverse views. Move the board through the centre, all four edges and corners; vary
distance, roll, pitch and yaw. Hand-holding is allowed, but pause briefly at each pose and
avoid glare, flex, rolling-shutter motion and blur.

```bash
cd ~/white_ws
./calibration/run_camera_intrinsic_capture.sh

./calibration/calibrate_camera_intrinsics.sh 6 \
  ~/white_ws/calibration/camera_data/video_front_mid_<time>
```

Assuming the quoted FOV is horizontal and the 840 x 560 mm active pattern is landscape, use
these centre/fronto-parallel near, middle and far distances: 45 deg: 1.5/2.3/4.1 m; 75 deg:
0.8/1.2/2.2 m; 100 deg: 0.5/0.8/1.4 m; 120 deg: 0.35/0.55/1.0 m. Increase distance about
10--30 percent for tilted edge/corner views. Use `--model fisheye` only for a true fisheye
lens. Use `CAMERA_RATIONAL_MODEL=1` for a very wide pinhole lens only when held-out error
improves consistently; extra coefficients can overfit a small data set.

The generated JSON report records every retained image and its reprojection error.  The tool's
default 0.8 px per-view rejection limit is an engineering screening limit, not an OpenCV or ROS
guarantee.  Prefer an RMS below roughly 0.3--0.5 px and inspect undistorted straight lines near
all image edges.  Re-capture the data if the board does not cover the field of view.

The current Raspberry Pi left stream is a measured exception: despite the physical fisheye
lens, repeated OpenCV `equidistant` solves were ill-conditioned. The 640x480 transmitted stream
passed the 100/60-view stability checks with `plumb_bob`, so selection `9` defaults to the
pinhole solver. This decision applies to the transmitted stream, not to an uncropped raw sensor
mode. Selections `7`, `8`, and `0` continue to default to `equidistant`.

Autoware traffic-light recognition is disabled by default until the dedicated 5 Hz Raspberry
Pi front camera is integrated. That camera must use the independent
`traffic_light_front` namespace and must not reuse `video_front_mid` CameraInfo.

After intrinsics, calibrate extrinsics in this order:

1. Verify base-to-top-lidar height/roll/pitch on level ground.
2. Refine lidar-to-lidar transforms from overlapping static structure.
3. Run Autoware CalibrationTools camera-lidar calibration for every camera.  The expected saved
   transform is `sensor_kit_base_link` to `video_<position>/camera_link`.
4. Validate by projecting lidar edges into images at several distances, then update
   `white_sensor_kit_description/config/sensor_kit_calibration.yaml`.

Primary references:

- ROS 2 monocular calibration: https://docs.ros.org/en/ros2_packages/jazzy/api/camera_calibration/doc/tutorial_mono.html
- OpenCV camera calibration: https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html
- Autoware lidar-camera calibration: https://autowarefoundation.github.io/autoware-documentation/main/tutorials/integrating-autoware/creating-vehicle-and-sensor-model/calibrating-sensors/lidar-camera-calibration/
- TIER IV CalibrationTools (Ubuntu 24.04/Jazzy supported): https://github.com/tier4/CalibrationTools
