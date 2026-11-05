# fsm-adtruck-calibration-v1 · 对坐黑车

`black_vehicle` 是**对坐黑车**的传感器驱动和三组标定工具分支：相机内参、四雷达/相机联合外参、四鱼眼外参与 BEV 拼接。使用 Ubuntu 22.04、ROS 2 Humble、Python 3.10；驱动和算法独立于完整 Autoware 运行。

## 部署位置与传感器

黑车工控机：`cityu02@100.93.32.20`。当前项目位于 `~/projects/fsm-adtruck-calibration-v1`。2 TB NVMe 当前未在线，项目使用系统盘独立目录。

| 设备 | 黑车配置 |
|---|---|
| 四颗雷达 | top RSHELIOS_16P；left/right RSHELIOS；rear RSE1；端口、初始外参见 `config/black_vehicle/` |
| 六颗 GMSL | `/dev/video0,3,4,5,6,7`，UYVY 1920×1080，10 Hz；video1/2 为未接相机的空通道 |
| 四颗鱼眼 | Pi `192.168.1.187`，MJPEG **800×600**，原图传输 10 Hz；保持原生分辨率，不拉伸为 1600×1200 |
| 红绿灯相机 | **未安装，保留第 15 个逻辑槽、采集入口和求解流程**；缺席不阻塞其他 14 路采集 |

GMSL 暂按通道编号命名，不猜测其物理方位。鱼眼网关复用工控机已有 `http://127.0.0.1:8086` 服务。现场鱼眼方位：front=ID03、rear=ID02、left=ID01、right=ID04。

## 环境与一键初始化

```bash
cd ~/projects/fsm-adtruck-calibration-v1
./install.sh --python-only    # 已有 ROS 的机器：仅建立项目隔离 Python 环境
./scripts/build.sh           # 编译当前分支所需的三个 ROS 包
./install.sh --check
```

全新专用 Ubuntu 可使用 `./install.sh --full`。共享工控机复用已有 ROS，不修改其 APT 源或其他工作空间。

```bash
~/fsm-adtruck/Autoware_sh/gmsl_camera_init.sh
```

初始化脚本进入现场 SDK 的 `bash` 目录，顺序运行 `load_modules.sh`、`pcie_init_card0.sh`；包含内核匹配、并发/占用检查和当前用户访问权限设置。设备存在不代表相机在线，已按逐路读帧确认六个有效通道。

正常 Autoware 仍使用 `~/fsm-adtruck/Autoware_sh/start.sh`。标定时不必启动完整 Autoware；程序复用已存在的雷达发布者，未启动时使用本项目编译的雷达驱动。标定入口不启动底盘控制。

## 三组标定

| 工作 | 采集入口 | 求解入口 |
|---|---|---|
| 相机内参 | `./start_intrinsics.sh` | `./solve_intrinsics.sh 1 /绝对路径/采集目录` |
| 雷达/相机联合外参 | `./start_joint.sh`，浏览器 `http://127.0.0.1:8765` | `./solve_joint.sh fixed-boards --data-root /绝对路径/数据目录 --initial /绝对路径/初始外参.yaml` |
| 四鱼眼外参与 BEV | 同一联合采集页面，布置四角地面板 | `./solve_bev.sh detect --help`、`absolute --help`、`relative --help`、`verify --help` |

内参按键 1～6 对应 GMSL 通道 **0、3、4、5、6、7**；7/8/9/0 对应前/后/左/右鱼眼；`-` 保留红绿灯相机入口，未安装时明确跳过。内参预览按 q/w/e/a/s/d/z/x/c 或 Ctrl+C 结束。`./stop.sh` 停止本项目后台采集并恢复原鱼眼图传模式。

独立 GMSL ROS 图像发布：`./drivers/black_vehicle/start_gmsl_ros.sh`，与直接 V4L2 标定采集互斥。不要让两者同时占用相机。

详细操作、输入输出和红绿灯相机后续接入见 [黑车使用说明](docs/BLACK_VEHICLE.md)。模拟闭环：`./scripts/validate_black_workflows.sh`；模拟结果带显式标记，不会覆盖现场标定文件。

## 配置与数据

- `config/vehicle.env`：黑车默认环境；`config/local.env`：本机覆盖项。
- `config/black_vehicle/`：真实设备、雷达端口、现有 Autoware 初始外参、独立相机内参目录。
- `drivers/black_vehicle/`：GMSL 初始化/ROS 发布、本项目雷达启动。
- `drivers/pi_capture/`、`drivers/pi_gateway/`、`drivers/pi_sender/`：鱼眼协议、SDK 与配套树莓派源代码。Pi sender 不在工控机启动。
- `ros_ws/src/`：仅保留 rslidar_sdk、rslidar_msg、相机采集/未来红绿灯桥接包；未使用的辅助车辆驱动已移除。
- `data/`：新采集；`output/`：新求解及模拟报告。二者不提交 Git。
- `result/`：历史算法回归参考；**不是黑车标定结果，不会安装为黑车参数**。

ChArUco：12×8，格长 70 mm、标记 50 mm，DICT_5X5_100。联合/BEV 靶板：900×900 mm、Tag16h5，详见 `config/boards.yaml`。

相机内参、初始外参必须来自本车相同分辨率的测量。当前四雷达位姿是现有 Autoware 初值；黑车尚无实拍标定板数据。模拟数据只验证软件可执行性和数值正确性，不能替代实车标定精度验收。
