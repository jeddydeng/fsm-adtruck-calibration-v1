# 车辆 Autoware 上车就绪性验收报告

审计日期：2026-09-03  
审计方式：Mac 标定结果检查 + 工控机只读 SSH 检查  
安全约束：未启动 Autoware、未调用 arm 服务、未下发 CAN 帧、未修改活动配置。

## 结论

**当前不能直接进入 Autoware 自动驾驶。**

Ubuntu 24.04、ROS 2 Jazzy、Autoware 1.9.0、`white_ws`、地图文件、CAN
恢复服务和双层解锁门均已存在，基础软件栈具备继续调试的条件。但是最新实车
启动存在被 `start.sh` 漏检的进程崩溃，GNSS 到 Autoware 的 QoS 链路不通；
活动外参、车辆几何、交通灯地图语义和实车控制标定也未达到闭环条件。

在以下 P0 全部解决前，不应执行 `./arm_autonomy.sh`。

## 已确认通过

- Ubuntu 24.04.4 LTS；Python 3.12.3；ROS 2 Jazzy。
- Autoware 1.9.0，commit
  `10718787ba6e28f038a0cb29ff99cc627b5abfd2`。
- `white-can0.service` 为 enabled/active；`can0` UP、500000 bit/s、
  ERROR-ACTIVE，bus-off/error-passive 为 0。
- 本次审计读取到 CAN RX 4,381,986 帧、TX 0 帧，确认没有发送控制数据。
- CAN 发送层为 10 ms 周期（100 Hz）；启动后仍保持两层 DISARMED，必须通过
  车辆静止、驻车、反馈新鲜、无故障和急停状态等门禁后才能解锁。
- `DG_testmap` 的 OSM、PCD 和 projector 文件存在；历史规划仿真曾完成路线。

## P0：自动驾驶前必须完成

### 1. 修复交通灯融合节点崩溃与启动假成功

最新完整日志中 `traffic_light_multi_camera_fusion_node` 以 exit code `-6`
退出：

```text
parameter_value_from failed for parameter 'camera_namespaces':
No parameter value set
```

`start.sh` 的 `critical_processes` 没有包含交通灯 bridge、检测和 fusion 节点，
所以进程已经崩溃时脚本仍可能报告启动成功。

证据：

- `/home/fsm-lab/white_ws/run_logs/white_stack_20260903_131253.log:559`
- `/home/fsm-lab/white_ws/run_logs/white_stack_20260903_131253.log:564`
- `/home/fsm-lab/white_ws/start.sh:239`
- `/home/fsm-lab/white_ws/src/white_bringup/launch/autoware_vehicle.launch.py`

必须给有相机模式传入非空 namespace；无交通灯相机时明确禁用整条流水线；
正式启动 bridge，并把图像、CameraInfo、检测、fusion 及其输出话题加入健康门。

### 2. 修复 GNSS QoS 断链

GNSS 驱动的 SensorDataQoS 与 `gnss_poser` 订阅可靠性不兼容。最新日志对
fix 和 orientation 均明确报告 `No messages will be sent`。当前健康监控只看
原始串口文本，不能证明 `/sensing/gnss/pose` 可用。

证据：

- `/home/fsm-lab/white_ws/run_logs/white_stack_20260903_131253.log:469`
- `/home/fsm-lab/white_ws/run_logs/white_stack_20260903_131253.log:475`
- `/home/fsm-lab/white_ws/src/ins_com_pkg/src/wtrtk_gnss_node.cpp`
- `/home/fsm-lab/white_ws/src/white_sensor_kit_launch/launch/gnss.launch.xml`

必须统一发布/订阅 QoS，并把处理后的 pose、covariance、fixed 状态和频率纳入
健康检查；之后在室外 RTK fixed 环境验证坐标、航向和协方差。

### 3. 部署可信外参

活动 `sensor_kit_calibration.yaml` 中：

- 三颗 BPearl 明确标注为仍需标定的粗略种子；
- `video_front_left/right`、`video_rear_left/right/mid` 外参全部为零；
- `video_front_mid` 只是 provisional seed；
- 鱼眼和交通灯相机尚未形成已验收的活动 TF。

证据：

- `/home/fsm-lab/white_ws/src/white_sensor_kit_description/config/sensor_kit_calibration.yaml`
- `../part12_multiday_20260903/C_VALIDATION_REPORT.md`
- `../traffic_part4_20260903/REPORT.md`

任务 C、D、E 的结果均被质量门拒绝，必须补采并通过跨日、共视、点云重叠和
重投影验证后，才允许原子化写入活动 TF。

### 4. 实测车辆几何

活动文件明确写着 `Recovered ... Re-measure before the first closed-course`
。当前轴距 1.55 m、轮距 0.87 m、轮半径 0.25 m、前后悬 0.65/0.60 m、
车高 2.40 m 和最大转角 0.5061 rad 均需针对当前白车实测。

证据：

- `/home/fsm-lab/white_ws/src/white_vehicle_description/config/vehicle_info.param.yaml`

### 5. 补齐本车纵向、制动和转向标定

`white_ws` 中未发现车辆专用 accel/brake/steering map、死区、延迟或滞回
拟合表。Autoware 源码里的 default、simulator 和 calibrator 示例不能当作本车
真值。当前车辆接口仍主要使用固定线性换算系数。

证据：

- `/home/fsm-lab/white_ws/src/fsm_vehicle_interface/config/vehicle_interface.param.yaml`
- `/home/fsm-lab/white_ws/src/white_vehicle_launch/launch/vehicle_interface.launch.xml`

### 6. 完成定位初始化和地图对齐

最新启动中 `map` 与 `base_link` 不在同一 TF 树；GNSS pose 未到达，NDT/EKF
也没有形成持续的 `map → base_link`。必须验证地图尺度、方向和高程，完成手动
或 GNSS 初始位姿，检查 NDT score/迭代/协方差，并验证静止及行驶恢复。

证据：

- `/home/fsm-lab/white_ws/run_logs/white_stack_20260903_131253.log`

### 7. 补全当前版本的启动健康门

启动成功不能只检查进程名。至少还要检查：

- 四雷达原始频率、时间戳和融合点云；
- 校正后的 IMU；
- GNSS pose/fixed 状态；
- `base_link` 到所有传感器 TF；
- 初始化后的 `map → base_link`；
- 交通灯 image/CameraInfo/detection/fusion；
- `/vehicle/status/*`、规划轨迹、控制命令；
- 两层 armed 必须为 false；
- 任一关键节点退出时立即启动失败或自动 disarm。

## 交通灯专项

当前交通灯链路不可用：bridge 未确认进入正式 bringup、namespace 为空、fusion
已崩溃、健康门默认不要求交通灯相机，而且 `DG_testmap` 中
`traffic_light`、`regulatory_element`、`stop_line` 的匹配计数均为 0。

任务 E 的候选也被拒绝：18 个鲁棒内点仍有平面距离 P95 328.5 mm、法向
P95 4.289°、bootstrap 平移 P95 412.3 mm、角度 P95 4.779°。如果封闭测试
路线没有交通灯，应明确关闭整条模块，不能保留“启动后崩溃”的状态。

## P1：低速闭环前完成

1. 精标 IMU 方向、零偏、温漂、协方差，以及 GNSS 杆臂、双天线航向和时间偏差。
2. 最终外参部署后验证四雷达地面/柱墙重合、车体裁剪、BPearl 地面误检和运动补偿。
3. 保持 DISARMED 静态运行 15–30 分钟，验证节点、话题、TF、定位和规划。
4. 做 CAN、USB、雷达、相机冻结、命令/反馈超时、急停和遥控抢占故障注入。
5. 先架空轮验证挡位、轮速和转向符号，再做 0.2 m/s、0.5 m/s 封闭场地标定。

## 推荐验收顺序

1. 修复交通灯崩溃和 GNSS QoS，并补健康门。
2. 补采 C/E 数据，部署通过独立质量门的外参。
3. 实测车辆几何；精标 IMU/GNSS。
4. 补地图交通灯、停止线和监管元素，或在无灯封闭测试中明确禁用交通灯模块。
5. 静态全栈验收并建立稳定 `map → base_link`。
6. 架空轮验证控制方向和安全联锁。
7. 标定速度、加速、制动、转向、延迟和滞回。
8. 故障注入后，依次进行 0.2 m/s 直线、0.5 m/s 直线、转弯和停车。
9. 最后才允许学校道路或停车场路线自动驾驶。

## 验收矩阵

| 项目 | 状态 |
|---|---|
| Ubuntu 24.04 / ROS 2 Jazzy | 通过 |
| Autoware 1.9.0 源码安装 | 通过 |
| white_ws 编译 | 通过 |
| CAN 恢复、500 kbit/s、双层门禁 | 基础通过；实车联锁待验收 |
| 地图文件存在 | 通过 |
| 地图交通灯语义 | 不通过 |
| 四雷达/相机活动外参 | 不通过 |
| IMU/GNSS 精标 | 不通过 |
| GNSS 到 Autoware QoS | 不通过 |
| 交通灯启动链和融合节点 | 不通过 |
| 车辆几何 | 不通过 |
| 实车纵横向控制标定 | 不通过 |
| 定位初始化和 `map → base_link` | 不通过 |
| 当前配置静态全栈验收 | 不通过 |
| 实车低速闭环 | 未执行 |
| 可直接自动驾驶 | **否** |

本次只读审计没有修改活动 Autoware、`white_ws` 配置或 CAN 控制状态。
