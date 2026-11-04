# 运行维护

## 服务

```bash
systemctl status fsm-pi-camera-gateway
systemctl status fsm-pi-can-interfaces
systemctl status fsm-pi-vehicle-io
systemctl status fsm-pi-linear-fan
systemctl status fsm-pi-telephoto-sender
```

配置修改后只重启相关服务。所有服务都设置 `Restart=always`；断电重启后相机网关回到 `bev`，语音不依赖工控机连接，交通灯流只有收到工控机租约才发包。

## 快速诊断

```bash
sudo ./scripts/verify.sh
./scripts/inventory_hardware.sh
journalctl -u fsm-pi-camera-gateway -n 100 --no-pager
journalctl -u fsm-pi-vehicle-io -n 100 --no-pager
```

发布包不携带历史日志。systemd journal 由操作系统管理。

## 常见问题

- 四路掩码不是 `15/15`：先检查独立供电、USB 路径和 MJPEG 1600×1200 支持，再看相机服务日志。
- CAN 无数据：检查 HAT Mode B、SPI 覆盖、500 kbit/s、终端电阻和接线；保持 listen-only。
- 中文原始语音正常而自定义语音异常：切回 `source=original`；不要改动 MP3 文件编码。
- 音量过大：`volume_percent` 实际输出永远受 `max_volume_percent` 上限约束；紧急时调用 mute。
- 红灯/数据停止：先查供电、`vcgencmd get_throttled`、温度及系统 journal；硬件 watchdog 为 60 秒。
- 第五颗相机不发：这是租约设计；先启动工控机接收程序，而不是持续空发 UDP。

## 更新

拉取新版本后再次运行 `install.sh`。默认保留本机运行配置；标定或车辆协议升级必须作为明确版本变更并重新验收。不要把 `build/`、日志、压力测试输出、抓拍图或配置备份提交仓库。
