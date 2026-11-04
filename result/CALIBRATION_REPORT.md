# 小白车传感器标定结果（Mac 离线处理）

## 结论

所有数值均以车顶主激光雷达为参考，x 向前、y 向左、z 向上。本结果包区分数值重复性与绝对真值；没有量具/全站仪证据时不宣称毫米级绝对精度。

竖装 BPearl 接近欧拉角奇异位形；文件同时提供 RPY、四元数和 4×4 矩阵，程序应优先读取四元数/矩阵。

### 激光雷达

- `velodyne_front_left_base_link`：fallback_1cm_1deg，p95 重复性 7.01 mm / 0.333°。
- `velodyne_front_right_base_link`：fallback_1cm_1deg，p95 重复性 7.56 mm / 0.207°。
- `velodyne_rear_base_link`：fallback_1cm_1deg，p95 重复性 3.65 mm / 0.138°。

### 摄像头外参

- `video_front_left/camera_link`：fallback_1cm_1deg，7.47 mm / 0.108°。
- `video_front_mid/camera_link`：requires_recapture，26.13 mm / 0.063°。
- `video_front_right/camera_link`：fallback_1cm_1deg，7.45 mm / 0.054°。
- `video_rear_left/camera_link`：fallback_1cm_1deg，5.34 mm / 0.084°。
- `video_rear_mid/camera_link`：requires_recapture，23.27 mm / 0.522°。
- `video_rear_right/camera_link`：fallback_1cm_1deg，8.32 mm / 0.091°。
- `fisheye_front/camera_link`：requires_recapture，23.01 mm / 0.064°。
- `fisheye_left/camera_link`：fallback_1cm_1deg，2.95 mm / 0.049°。
- `fisheye_right/camera_link`：fallback_1cm_1deg，5.14 mm / 0.063°。
- `fisheye_rear/camera_link`：requires_recapture，无有效 bootstrap。
- `traffic_light_front/camera_link`：requires_recapture，71.82 mm / 0.698°。

## 必须补采

- 交通灯相机：图像文件是新的，但采集时角点缓存陈旧；离线重检测后，5 Hz 与 USB 相机的同板尺度/同步检查仍失败。
- 后中相机、后鱼眼：只有 7 个悬空后板位姿，平面法的最小奇异值过低，平移不可完全观测。
- 前中与前鱼眼：二者相对关系稳定，但相对主雷达存在约 2–3 cm 共同平移弱模态。

## 文件

- `camera_intrinsics/`：11 颗相机 ROS 2 CameraInfo YAML 与原始报告。
- `extrinsics/all_sensors_relative_to_roof_lidar.yaml`：统一参考坐标结果。
- `extrinsics/autoware_sensor_kit_candidate.yaml`：转换到 sensor_kit_base_link 的候选值，含部署阻断清单。
- `lidar_intrinsics/`：雷达型号/端口及 DIFOP 工厂校正说明。
