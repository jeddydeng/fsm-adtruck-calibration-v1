# fsm-adtruck-calibration-v1

小白车三项标定工具的独立源码仓库：**相机内参、四激光雷达/相机联合外参、四鱼眼外参与 BEV 拼接**。
无需安装 Autoware 即可采集与计算。生产目标为 Ubuntu 24.04 / ROS 2 Jazzy / Python 3.12；
Ubuntu 22.04 / Humble / Python 3.10 用于兼容验证。Mac 可运行离线计算。

## 安装

```bash
git clone https://github.com/jeddydeng/fsm-adtruck-calibration-v1.git
cd fsm-adtruck-calibration-v1
./install.sh
```

安装器配置系统依赖、ROS、独立 Python 环境并编译驱动；不会启动传感器或车辆。
仅离线计算：`./install.sh --python-only`。详细环境见 `requirement.txt` 和 `environment/apt.txt`。
新电脑先按英文手册核对 USB 物理端口、雷达网口和树莓派地址。

## 三项工作怎么启动

| 工作 | 采集 / 运行 | 计算 |
|---|---|---|
| 相机内参 | `./start_intrinsics.sh` | `./solve_intrinsics.sh 7 /绝对路径/采集目录` |
| 雷达/相机联合外参 | `./start_joint.sh`，网页 `http://127.0.0.1:8765` | `./solve_joint.sh fixed-boards --help` |
| 鱼眼外参 / BEV | 同一联合采集网页，四角地面板采集 | `./solve_bev.sh detect --help`、`absolute`、`relative` |

内参按键：`1`左后、`2`左前、`3`右前、`4`右后、`5`正后、`6`正前、
`7`前鱼眼、`8`后鱼眼、`9`左鱼眼、`0`右鱼眼、`-`交通灯。
预览窗口中按 q/w/e/a/s/d/z/x/c 任一个结束，最长 20 分钟。
仅交通灯外参采集：`./traffic_cam_start.sh`；后台采集统一用 `./stop.sh` 停止。
内参采集在前台运行，使用上述按键或 Ctrl+C 结束。

三套完整、可复制的计算命令和数据组织方式见 **[英文操作手册](docs/OPERATIONS_EN.md)**。
算法步骤与矩阵方向见 [算法说明](docs/ALGORITHMS_EN.md)。

## 数据与目录

- `ros_ws/src/`：四雷达、USB 相机、IMU、GNSS、超声波、CAN 驱动及消息、TF。
- `drivers/`：树莓派五相机发送端、工控机接收端、鱼眼 SDK。
- `process/`：三套算法和既有数据校验工具；`scripts/`：统一运行与安装管理。
- `config/`：11 颗相机内参、标定板和环境配置；`result/`：历史参数及质量报告。
- `data/`：采集输入，默认不入 Git；`output/`：新计算结果，默认不入 Git。

ChArUco：12×8 格，格长 70 mm，内部标记 50 mm，DICT_5X5_100。
联合外参板：900×900 mm，Tag16h5、6×5 个 Tag，精确间距见 `config/boards.yaml`。
四鱼眼统一使用 1600×1200、equidistant 模型。

离线或检测失败不阻止其他传感器保存。网页绿/红框为质量提示，原始 PCD 保留全部点。
三套默认入口不启动底盘控制；可选 CAN 入口固定为只接收。
BEV 的相对地面拼接可用性与绝对车辆外参精度分开记录，历史候选不能自动视为精标真值。

**本版基于 2026-09-02～04 备份，已于 2026-11-05 只读核对实车源码，并同步车上最新 IMU 零偏参数。**
详细一致项、保留的修复及范围见 [源码核对报告](docs/VEHICLE_SOURCE_AUDIT_20261105.md)。
验证范围及未完成项见 [VALIDATION.md](docs/VALIDATION.md)。
本次已完成 Humble 下 12 个 ROS 包源码编译、22 项测试及模拟 ROS 采集闭环；
Ubuntu 24.04/Jazzy 的 GitHub 云端完整安装、编译及模拟采集检查也已通过；
实车 15 路传感器运行验收仍待进行。公开仓库保留各组件原许可声明，
`Proprietary` 组件未因公开而改为开源许可，使用或再分发前请核实相应授权。
代码采用清晰分层、单一职责、显式异常、受控资源生命周期等规约，详见
[代码规范](docs/CODING_STANDARD.md)；不声称取得阿里巴巴认证。
