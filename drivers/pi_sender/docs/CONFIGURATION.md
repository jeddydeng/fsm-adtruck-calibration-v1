# 参数与接口说明

安装目录由 `/etc/fsm-pi-001/install.conf` 记录。以下用 `/opt/fsm-pi-001` 表示默认目录。

## 当前车辆 186 参数

| 项目 | 当前值 |
|---|---|
| 树莓派 / 工控机 | `192.168.1.186` / `192.168.1.102` |
| BEV UDP / 控制 UDP | `5610` / `5611` |
| 四路输入 | 1600×1200 MJPEG |
| BEV 输出 | 512×512，10 Hz，JPEG 质量 75 |
| 启动模式 | `bev`，每次进程/整机重启恢复 |
| BEV 比例 | 0.021 m/px，车辆物理中心 `(256,256)` |
| 可视范围 | 中心起前后左右各 5.376 m |
| 车辆遮罩 | 长 4.35 m，宽 1.53 m |
| CAN | `can10/can11`，500 kbit/s，listen-only，stale 500 ms |
| RS485 | FT232R `A9IM4UBG`，9600 bit/s，每 0.5 s |
| 语音 | 原始录音，音量/上限均 80%，未静音 |
| 交通灯相机 | UDP 5620/控制 5621，320×240，5 Hz，租约 3 s |
| 风扇 | ≤25°C PWM 0；≥60°C PWM 255；中间线性 |

四路物理映射保存在 `config/gateway.env`，当前为：

- 前：树莓派第二个板载 USB3 控制器
- 后：树莓派另一个板载 xHCI 控制器
- 左：PCIe USB HAT 端口 4
- 右：PCIe USB HAT 端口 2

设备路径是本车实际枚举结果，新硬件必须重新盘点，不能根据端口描述盲抄。

## 配置文件

- `config/gateway.env`：相机、BEV、发送端口、CAN 和热保护
- `config/telephoto.env`：第五颗相机及租约发送
- `state/vehicle-io.json`：CAN stale、RS485、语音源/内容/间隔/音量（运行时可由工控机页面写入）

`scripts/configure.sh` 可安全修改常用硬件路径、对端 IP 和音量。高级参数可直接编辑配置后重启对应服务。

语音优先级固定为：倒车 > 左转 > 右转 > 自动驾驶 > 遥控驾驶。每项只有 `interval_s`，没有与“频率”叠加的第二参数。`source=original` 使用 `app/vehicle-io/original-audio` 中原车录音；工控机页面下发的自定义音频写入运行缓存，不属于发布包。

车辆 I/O HTTP API：

- `GET http://192.168.1.186:8089/api/status`
- `GET http://192.168.1.186:8089/api/config`
- `POST /api/config`：下发配置及音频资产
- `POST /api/mute`：`{"muted":true}` 或 `false`
- `POST /api/original-test`：测试原始录音

写操作只接受本机或 `192.168.1.102`。完整页面操作由配套 `FSM-Pi-Master` 提供。

## BEV 与原图接口

二进制协议见 [`app/bev/PROTOCOL.md`](../app/bev/PROTOCOL.md)。运行控制不会写回启动配置。常用模式：

- `bev`：四路融合，生产默认；
- `raw-all`：四路原图连续 10 Hz；
- `front/rear/left/right`：指定一路；
- `CAPTURE_RAW`：不改变 BEV 的情况下，一次性获取最新四路原图。

只有来自配置中 `PEER_IP` 的控制包会被接受；UDP 本身没有加密，车辆网段必须隔离。

## CAN 数据

W102 接收 ID：`0x352/0x354/0x356/0x358/0x360/0x362/0x364`。解析内容包括控制模式、档位、车速/加速度、驻车、前轮转角、制动、SOC/电流/电压、灯光和故障。两个独立进程均只创建接收 SocketCAN 套接字；系统启动脚本额外强制内核 listen-only。

如果 `verify.sh` 未确认 LISTEN-ONLY，立即断开车辆 CAN，不要依赖应用层“不主动发送”的假设。

## 标定约束

唯一生产标定位于 `app/bev/calibration/fisheye_bev_20260903_physical_center`。映射针对 1600×1200 原图和 512×512 输出预计算；不能只改分辨率数字。报告明确仅允许相对平面 BEV 映射，未使用未经批准的绝对车辆外参。
