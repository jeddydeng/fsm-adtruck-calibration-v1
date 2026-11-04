# 四鱼眼 BEV 标定（Part 3）

## 边界与输入

本目录只处理 2026-09-03 的 Part 3 数据，不修改 Autoware、不启动 CAN，也不
与车辆通信。备份时按长时间段初分的 Part 3 有 144 组，但逐帧核验发现前 130 组
车库柱线和停车线会相对相机移动，不符合“车不动、四角板径向移动”的 BEV 采集描述。
最后 14 组与描述严格吻合，并由两个明显的人工移动停顿分成 `3 + 6 + 5` 组：

- 初始位置：`15:36:27`–`15:36:32`，3 组；
- 各板向外约 0.45 m：`15:37:51`–`15:38:18`，6 组；
- 再向外约 0.45 m：`15:39:20`–`15:39:27`，5 组。

因此 BEV 求解严格使用 `part3_bev_selected_capture_groups.txt` 中的 14 组、56 张
1600×1200 鱼眼原图；144 组广义备份清单仍原样保留，不删除任何原始数据。

四块 900×900 mm Tag16h5 板使用相同的 ID 0–29，不能把 tag ID 当成物理板编号。
`detect_part3_boards.py` 每次重新读取保存的 PNG，用重复模型分割同 ID 的多个物理板，
再用 equidistant 模型求板姿态。它不采用采集网页缓存的角点。

四路内参必须来自已验收的
`result/fisheye_intrinsics_1600x1200_20260903_v2`，模型为 OpenCV `equidistant`，
不能换成旧的 640×480/800×600 参数，也不能按普通 `plumb_bob` 使用。

## 运行

Mac 上已验证的 Python 环境为：

```text
/Users/djhuai/fsm-adtruck/calibration-white/process/.venv/bin/python
```

相对 BEV 可独立于失败的绝对车体外参验收。下面的命令只使用最后 14 组 Part 3
图片，在现有度量地面映射上做四接缝闭环、尺度固定的 SE(2) 优化：

```bash
/Users/djhuai/fsm-adtruck/calibration-white/process/.venv/bin/python \
  optimize_relative_bev.py \
  --data-root ../../data/raw_extrinsic_data \
  --selected part3_bev_selected_capture_groups.txt \
  --raw-index ../../result/fisheye_bev_20260903/selected14_analysis/raw_image_index.txt \
  --baseline ../../result/fisheye_bev_20260903/review_only_c200 \
  --output ../../result/fisheye_bev_20260903/relative_usable
```

相对包只有在四条接缝特征数/P95、采集跨度、环带覆盖、多相机重叠、尺度、H、remap、
预览和哈希全部通过时才写入 `deployment_allowed: 1`，且 scope 固定为
`BEV_relative_planar_mapping_only`。绝对车体 R/t 始终单独保留为不允许部署。

如将来 Part 1/2 签发了可信的“顶雷达←camera_link”结果，可另外生成绝对外参复核包：

```bash
./run_part3_bev.sh /absolute/path/to/camera_relative_to_roof_lidar.yaml
```

程序输出：

- 四路 OpenCV YAML，含 K、D、R、t、H；
- 每路从原始 equidistant 图直接生成 BEV 的 `*_bev_map_x.npy` / `*_bev_map_y.npy`；
- 原始图、去畸变图、单路 BEV、四路拼接预览；
- 原图上的 1 m 车辆地面网格外参验证图；
- 新鲜像素分板结果、重投影误差、跨相机同板中心差和 SHA-256 清单。

## 尺寸

| 项目 | 数值 |
|---|---:|
| 板外尺寸 | 0.900 × 0.900 m |
| 有效 AprilGrid | 0.800 × 0.800 m |
| 排列 | 6 列 × 5 行，ID 0–29 |
| Tag 黑色外框 | 0.108 m |
| 横向净间距 | 0.0304 m |
| 纵向净间距 | 0.065 m |
| 四边留白 | 0.050 m |

横向：`6×0.108 + 5×0.0304 = 0.800 m`；纵向：
`5×0.108 + 4×0.065 = 0.800 m`。

## 坐标系与矩阵方向

- 车辆坐标 `sensor_kit_base_link`：x 向前、y 向左、z 向上，米。
- 相机 optical 坐标：x 向右、y 向下、z 向前，米。
- `R_vehicle_camera`、`t_vehicle_camera_m` 的定义严格为：
  `p_vehicle = R_vehicle_camera · p_camera_optical + t_vehicle_camera_m`。
- BEV 画布 512×512、0.015 m/px，车辆原点为 `(u,v)=(256,256)`；
  u 向右对应车辆 −y，v 向下对应车辆 −x。因此覆盖车辆前后、左右各 3.84 m。
- `H_bev` 的定义为：`p_bev ~ H_bev · p_rectified`。

## 必须区分 H 与鱼眼 remap

单个 3×3 单应矩阵只适用于**已经去畸变的虚拟针孔平面**。原始约 180°
equidistant 鱼眼图像到地面不是单应关系，禁止把 `H_bev` 直接乘原始像素。
运行时推荐直接加载预计算 `*_bev_map_x.npy` / `*_bev_map_y.npy`，调用
`cv2.remap(raw_fisheye, map_x, map_y, cv2.INTER_LINEAR)`。这个映射已把鱼眼模型、
外参、地面平面和 BEV 坐标一次性合并，速度和可靠性均优于逐帧重新计算。

## 可观测性说明

Part 3 的平放板能高质量校正每颗相机相对地面的 roll、pitch、相机高度 z，并验证
跨相机同板重合；单纯平面数据不能独立确定车辆平面内的 x、y、yaw。因此这三个量
必须继承 Part 1/2 的多传感器联合外参，最终签发前必须完成交叉验证。

不过，四路相对地面 BEV 可以由公共地面纹理和闭环接缝独立验收：front 相机只作为
画布规范的 gauge，其他三路仅做尺度固定的平移/旋转修正。这样得到的 H/remap 可用于
环视拼接，但不能反向宣称车辆绝对 R/t 已经可信。输出文件把这两层状态分开保存。
