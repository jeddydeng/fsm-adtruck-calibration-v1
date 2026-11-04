# FSM-Pi-001 车载树莓派正式程序

这是车辆 `192.168.1.186` 上正式运行版本的可重复部署包。它只包含生产功能、当前参数和 512×512 正式标定，不包含测试、压力测试、分辨率基准、历史备份、运行日志、抓拍图片、构建缓存或调试二进制。

## 生产功能

- 四颗鱼眼相机固定采集 1600×1200 MJPEG，以 10 Hz 输出 512×512 BEV；掉线方向用黑色填充。
- 断电重启后始终回到四路 BEV；运行中可切换 BEV、四路原图或任意单路，并支持一次性抓取四路原图。
- 两路 CAN FD 控制器固定为 `can10/can11`、500 kbit/s、经典 CAN、内核 `listen-only`；程序只接收、不发送。
- 读取 W102 底盘状态，保留 USB-RS485 LED 转发和脱离工控机工作的优先级语音播报。
- 25°C 停扇、60°C 满速，中间线性 PWM；控制失败时满速保护。
- 第五颗交通灯相机 320×240、5 Hz、按工控机租约发送；工控机不在线时停止发送。
- 五个 systemd 服务开机自启动并带自动重启和资源上限。

## 已验证基线

- Raspberry Pi 5，aarch64
- Raspberry Pi OS 64-bit / Debian 13
- 内核 `6.18.34+rpt-rpi-2712`
- OpenCV 4.10（最低构建要求 4.5）
- 微雪双路隔离 CAN FD HAT，Mode B 设备树覆盖

Debian 12/其他内核可能可以构建，但尚未按本车硬件验收。目标板必须配有支持 `pwmfan` hwmon 的风扇；没有该接口时风扇服务会满速保护后报错重启。

## 新机最快安装

```bash
git clone <你的仓库地址> FSM-Pi-001
cd FSM-Pi-001
./scripts/inventory_hardware.sh
sudo ./scripts/install.sh --user fsm-pi-001
sudo ./scripts/configure_network.sh --apply
sudo reboot
```

重启后执行：

```bash
cd FSM-Pi-001
sudo ./scripts/verify.sh
```

只有验证结果中两路 CAN 均显示 `kernel listen-only` 后，才允许接入车辆 CAN。若新机的 USB 路径不同，先按[安装说明](docs/INSTALLATION.md)和[参数说明](docs/CONFIGURATION.md)修改，再启动服务。

## 目录

- `app/`：生产源代码、原始语音及唯一的 512 BEV 标定
- `config/`：车辆 186 当前参数样例、udev 与 watchdog 配置
- `drivers/`：当前验证过的 Mode B 设备树覆盖
- `systemd/`：不写死用户名和安装路径的服务模板
- `scripts/`：安装、配置、硬件盘点、验收和卸载
- `docs/`：新机安装、参数和运维手册

当前版本见 [`VERSION`](VERSION)，文件完整性见 [`MANIFEST.sha256`](MANIFEST.sha256)。本仓库未执行任何 Git 初始化、提交或上传操作。
