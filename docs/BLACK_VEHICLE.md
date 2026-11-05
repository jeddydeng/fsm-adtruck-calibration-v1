# 对坐黑车操作说明

## 已部署环境

工控机 `cityu02@100.93.32.20`，Ubuntu 22.04/Humble，项目 `~/projects/fsm-adtruck-calibration-v1`、分支 `black_vehicle`。Python 依赖只安装在项目 `.venv`，三个 ROS 包编译在 `ros_ws`，不依赖旧车驱动目录。ROS_DOMAIN_ID=0，CycloneDDS，localhost。

树莓派 `fsm-pi-002@192.168.1.187` 保持 MJPEG 800×600；四相机原生支持 1920×1080、1280×720、800×600、640×480，不支持1600×1200。此次不改其分辨率、服务和临时 BEV 参数。工控机复用8086本地网关，采集时切换原图模式，退出恢复。

## 传感器启动

正常 Autoware 沿用 `~/fsm-adtruck/Autoware_sh/start.sh`。只做标定：

```bash
cd ~/projects/fsm-adtruck-calibration-v1
./start_joint.sh
# 结束
./stop.sh
```

入口初始化 GMSL，复用已有雷达发布者或启动本分支编译的雷达及其初始 TF；不启动完整 Autoware/底盘控制。已存在部分雷达发布者时不重复绑定端口，应先确认相应进程和网络状态。

GMSL单独初始化：`~/fsm-adtruck/Autoware_sh/gmsl_camera_init.sh`。先从 SDK `bash/` 运行 `load_modules.sh`，再运行 `pcie_init_card0.sh`；设置为1920×1080/UYVY/10 Hz。`--force`只在停止相机采集后使用。本仓库包含已验收的card0 SDK快照和内核模块，优先从项目vendor目录加载；逐文件哈希见PROVENANCE.json。原现场SDK作为备份保留。

GMSL ROS发布：`./drivers/black_vehicle/start_gmsl_ros.sh` 或 `~/fsm-adtruck/Autoware_sh/start_camera.sh`。六路压缩图像话题为 `/sensing/camera/gmsl_videoN/image_raw/compressed`，N=0,3,4,5,6,7。该模式与直接相机标定采集互斥，切换前停止原采集。

## A. 内参

```bash
./start_intrinsics.sh
# 1～6：GMSL0、3、4、5、6、7；7/8/9/0：前后左右鱼眼；-：保留红绿灯入口
./solve_intrinsics.sh 1 /绝对路径/GMSL会话
./solve_intrinsics.sh 7 /绝对路径/鱼眼会话
```

板：12×8 ChArUco、70/50 mm、DICT_5X5_100。GMSL原生1920×1080，鱼眼原生800×600。保持足够的距离、倾角和边缘覆盖，不能用拉伸分辨率规避质量门限。

输出位于 `output/intrinsics/`；审核各相机报告后，把对应 CameraInfo YAML 按相机名保存到 `config/black_vehicle/intrinsics/`，不要复制其他车辆内参。需用于 BEV 的四鱼眼还需保留各自 `camera_info.yaml` 与 `report.json` 的子目录结构。

SSH无预览可用 `CAMERA_CAPTURE_HEADLESS=1`；`CAMERA_CAPTURE_DURATION_SEC=秒数`控制时长（最多1200秒）。无标定板时仍可录像并正常退出，不能据此求得有效内参。

## B. 四雷达/相机联合外参

采集站保留15个槽，当前14路有数据。点击保存会跳过离线设备；manifest同时记录 `missing_optional_sensors` 和 `complete_required_sensor_set`。四雷达初始值来自本车 Autoware，单位米/弧度；相机初始值应按实际安装测量后填入单独的 initial_extrinsics.yaml。GMSL物理方位在未确认前按设备编号命名。

```bash
./solve_joint.sh fixed-boards \
  --data-root /绝对路径/联合采集根目录 \
  --initial /绝对路径/本车初始外参.yaml \
  --output /绝对路径/新求解结果.yaml
```

黑车 fixed-boards 使用仓库原有的通用靶板联合求解器，按配置中的设备名和真实雷达 frame 读取数据；原有 Part1/2、multiday 和交通灯算法代码保留。历史数据专用算法仍可从其原模块按显式参数运行，不默认套用历史参数到黑车。求解器中的相机内参来自会话快照或黑车独立内参目录。

## C. 四鱼眼外参与 BEV

沿用四角地面板和原有算法。14组数据按初始3组、第一次向外移动6组、第二次5组组织；两次板中心的X/Y分量分别移动±0.45 m。分辨率改为按CameraInfo读取800×600，不改变板尺寸、优化器和质量门限。

```bash
./solve_bev.sh detect --data-root /采集根目录 --group-list /14组列表.txt \
  --intrinsics-root /四鱼眼内参根目录 --output /检测结果目录
./solve_bev.sh absolute --data-root /采集根目录 --detections /检测结果目录/board_detections.jsonl \
  --intrinsics-root /四鱼眼内参根目录 --camera-extrinsics /本车相机外参.yaml \
  --vehicle-roof-xyz-rpy 2,0,2.5,0,0,0 --output /BEV基线目录
./solve_bev.sh relative --data-root /采集根目录 --selected /14组列表.txt \
  --raw-index /检测结果目录/raw_image_index.txt --baseline /BEV基线目录 --output /相对BEV目录
./solve_bev.sh verify --data-root /采集根目录 --package /相对BEV目录 --baseline /BEV基线目录
```

输出包含四路512×512目标到源remap、相对平面拼接质量、源图像哈希和预览。地面相对拼接通过不等于绝对车辆外参已验收；保持原有部署门限。

## 红绿灯相机预留

`traffic_light_front` 的 ROS 订阅、独立页面 `./traffic_cam_start.sh`、内参按键 `-`、`./solve_joint.sh traffic` 与相关驱动全部保留。当前配置 `enabled: true, installed: false, required: false`，不启动不存在的树莓派第五相机，也不等待它就绪。仅交通灯页面仍可保存现有四雷达；普通联合采集可保存其余14路。恢复图像发布者后同一槽自动接收数据，无需更换采集/标定算法。

以后安装时设置 `installed: true`，按实际硬件配置话题及发送/接收链路，必要时设 `CALIBRATION_AUTO_TRAFFIC=1`；相机内参、安装位姿与独立交通灯流程仍需单独验证。不要把当前Pi的硬件能力开关当成已经安装相机。

## 验证

```bash
./install.sh --check
./scripts/smoke_station.sh           # 隔离域193，模拟未来交通灯发布者
./scripts/validate_black_capture.sh  # 实机14路 + 红绿灯空位
./scripts/validate_black_workflows.sh # 三组模拟求解，输出显式标注SYNTHETIC
```

模拟内参使用渲染的ChArUco图像；联合外参使用渲染AprilGrid图像与独立生成的雷达观测；BEV使用合成纹理原图、已知地面板观测测试数值流程，同时运行检测入口。模拟结果不写入 `config/black_vehicle/intrinsics` 或现场 Autoware，也不作为实车参数提交。
