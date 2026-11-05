# 对坐黑车驱动

当前分支收齐四雷达、六GMSL、四鱼眼所需启动入口及源代码，保留未来红绿灯相机的接收/采集/标定流程。

- `calibration_lidars.launch.py`、`start_lidars.sh`：本项目已编译的雷达驱动及黑车初始TF。未编译时明确提示构建，不再隐式依赖其他工作空间。
- `gmsl_camera_init.sh`：优先使用本目录 `vendor/senyun_pcie_v4l2` 的已验收SDK快照，按 load_modules → pcie_init_card0 顺序初始化，内核不匹配时停止。
- `gmsl_ros.py`、`start_gmsl_ros.sh`：有效通道0、3、4、5、6、7的ROS压缩图像发布，与标定直接采集互斥。
- `start_camera.sh`：工控机 `~/fsm-adtruck/Autoware_sh/start_camera.sh` 的统一项目入口。

厂商快照仅保留card0必需脚本、三个硬件工具、相关供应商源码和已确认的 `xdma.ko`。模块内核为 `5.15.105-0515105-generic`，SHA256为 `e9843ad35f891c6cf406f5b5ea075e12435bb9d5a67ef1c65462a37edcc5c95d`。原有加密/二进制供应商文件原样保留，没有解密、重编译或更换已验收版本。来源与逐文件哈希见 `vendor/senyun_pcie_v4l2/PROVENANCE.json`。不保留其他采集卡、构建临时对象或初始化日志。

Pi采集/传输代码在兄弟目录pi_sender、pi_gateway、pi_capture；已有187服务继续使用800×600，未重配树莓派。未安装红绿灯相机不影响14路采集，未来接入可沿用现有槽位。

详细启动、内外参和BEV使用方式见 [BLACK_VEHICLE.md](../../docs/BLACK_VEHICLE.md)。实测与模拟范围见 [BLACK_VALIDATION.md](../../docs/BLACK_VALIDATION.md)。
