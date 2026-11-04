# 新树莓派安装说明

## 1. 硬件准备

1. Raspberry Pi 5 与可靠的官方规格供电；PCIe USB 扩展板使用独立 30 W 供电且不得向树莓派倒灌。
2. 四颗 1600×1200 MJPEG 鱼眼相机；其物理方向必须与配置中的前、后、左、右一致。
3. 微雪双路隔离 CAN FD HAT，跳线设为 Mode B；总线两端按车辆拓扑配置 120 Ω 终端电阻。
4. FTDI FT232R USB-RS485，当前序列号 `A9IM4UBG`，9600 bit/s。
5. USB 音频设备，当前 ALSA 目标 `hw:CARD=Audio,DEV=0`。
6. USB2 交通灯相机 LRCP H960P；固定 320×240 MJPEG、5 Hz。
7. 连接到 PWM 风扇控制器的散热风扇。

安装期间先断开车辆 CAN。CAN 只读验收通过后再接线。

## 2. 系统准备

安装 Raspberry Pi OS 64-bit（已验证 Debian 13），创建服务用户：

```bash
sudo adduser fsm-pi-001
sudo usermod -aG video,audio,dialout fsm-pi-001
```

用该用户克隆仓库并记录设备路径：

```bash
git clone <你的仓库地址> FSM-Pi-001
cd FSM-Pi-001
./scripts/inventory_hardware.sh | tee hardware-inventory.txt
```

`hardware-inventory.txt` 只用于现场核对，不应提交 Git。

## 3. 安装生产程序

```bash
sudo ./scripts/install.sh --user fsm-pi-001
```

默认安装到 `/opt/fsm-pi-001`。其他路径可用 `--install-dir /绝对路径`，服务模板会自动替换，不依赖固定用户名或家目录。脚本会：

- 安装编译和运行依赖；
- 使用 `-DBUILD_TESTING=OFF` 构建 Release 目标；
- 安装 Mode B 设备树覆盖及 CAN/RS485 udev 规则；
- 安装五项服务、60 秒硬件 watchdog 和当前车辆参数；
- 启用开机自启动，但默认不在本次启动强行启动硬件服务。

更新已有安装时默认保留已修改的三个运行配置。只有明确需要恢复车辆 186 样例时才使用 `--force-config`。

## 4. 对齐新机硬件路径

比较硬件盘点与 `/opt/fsm-pi-001/config/gateway.env`、`config/telephoto.env`、`state/vehicle-io.json`。可用统一配置入口：

```bash
sudo ./scripts/configure.sh \
  --peer-ip 192.168.1.102 \
  --camera-front /dev/v4l/by-path/... \
  --camera-rear /dev/v4l/by-path/... \
  --camera-left /dev/v4l/by-path/... \
  --camera-right /dev/v4l/by-path/... \
  --telephoto-device /dev/v4l/by-id/... \
  --serial-device /dev/serial/by-id/... \
  --audio-output hw:CARD=Audio,DEV=0
```

不要用 `/dev/video0` 一类会随启动变化的短名称。摄像头方向须通过四路原图逐一确认。

若更换 FTDI 适配器，还应把 `config/81-fsm-pi-rs485.rules` 中的序列号改成 `udevadm info` 读取到的新值，然后重新运行安装脚本。

## 5. 固定地址

网络切换会立即中断 SSH，因此建议在本机屏幕/键盘或带外控制台执行：

```bash
sudo ./scripts/configure_network.sh --apply
```

默认值为 `eth0`、`192.168.1.186/24`、网关 `192.168.1.1`、工控机 `192.168.1.102`。不同网络用命令参数覆盖；先运行 `--help` 查看。

## 6. 重启和验收

```bash
sudo reboot
```

重新登录后：

```bash
cd FSM-Pi-001
sudo ./scripts/verify.sh
```

必须满足：

- 五项服务均 enabled/active；
- `can10` 和 `can11` 均为 `LISTEN-ONLY`，500000 bit/s；
- 四个正式标定 YAML 与四个 512 remap 文件存在；
- 四颗鱼眼设备路径全部存在；
- 车辆 I/O API 在本机 8089 响应；
- 固件未报告欠压/降频历史，或已查清历史位来源。

交通灯相机和 RS485 未接入时只报告 warning。其他 failure 必须解决后才能接车。

## 7. 工控机联合测试

工控机安装配套 `FSM-Pi-Master` 后，确认：

1. 512×512 BEV 持续约 10 Hz，四路掩码为 `15/15`；
2. 页面显示 CAN 状态、SOC、转角、车速、档位、灯光和控制模式；
3. 四路原图单次抓拍和连续 raw-all 均可用，切换后可恢复 BEV；
4. 交通灯相机 ROS 2 压缩图像与 CameraInfo 均为 5 Hz；
5. 倒车/左转/右转/自动驾驶/遥控驾驶语音优先级和间隔正确；
6. RS485 LED 数据持续更新。

## 8. 卸载

`sudo ./scripts/uninstall.sh` 停用并删除服务，但保留应用和配置。只有确认不再需要恢复参数时才执行 `sudo ./scripts/uninstall.sh --purge`。设备树覆盖默认保留，避免卸载脚本意外改变启动硬件。
