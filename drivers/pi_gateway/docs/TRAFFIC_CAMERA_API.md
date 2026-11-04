# 交通灯摄像头 ROS 2 接口

`fsm-pi-master-traffic-camera.service` 每秒向树莓派发送租约心跳。只有租约有效时，树莓派才传送第五颗长焦/交通灯摄像头，固定为 320×240、JPEG、5 Hz。工控机收到完整 JPEG 后逐帧发布图像和相同时间戳的标定信息。

## ROS 2 话题

| 话题 | 类型 |
|---|---|
| `/sensing/camera/traffic_light_front/image_raw/compressed` | `sensor_msgs/msg/CompressedImage` |
| `/sensing/camera/traffic_light_front/camera_info` | `sensor_msgs/msg/CameraInfo` |

共同参数：

- `frame_id`: `traffic_light_front/camera_optical_link`
- QoS: KEEP_LAST，depth 5，BEST_EFFORT，VOLATILE
- 图像 `format`: `bgr8; jpeg compressed bgr8`
- `header.stamp`: 树莓派该帧实际采集时间；CameraInfo 与图像完全一致
- 内参文件：`traffic_camera/calibration.json`

当前内参为 `fx=fy=650.836043691`、`cx=159.5`、`cy=119.5`，`plumb_bob`，畸变 `[-0.1011836722, 0.0401139710885, 0, 0, 0]`。

## 使用

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=92
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 topic hz /sensing/camera/traffic_light_front/image_raw/compressed
ros2 topic echo /sensing/camera/traffic_light_front/camera_info --once
python3 ~/FSM-Pi-Master/traffic_camera/capture_one_ros2.py \
  --output ~/traffic-light.jpg
```

本机诊断接口：

```text
GET http://127.0.0.1:8090/status
GET http://127.0.0.1:8090/image.jpg
```

ROS DDS 发现范围固定为 `LOCALHOST`，避免与树莓派跨网，也避免影响车上其他 ROS 2 域。若其他本机程序必须订阅，需使用同一 `ROS_DOMAIN_ID=92` 和兼容 Sensor Data QoS。

