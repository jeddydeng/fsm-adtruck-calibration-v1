# 故障排查

## 网页打不开

```bash
systemctl status fsm-pi-master-monitor.service
curl -v http://127.0.0.1:8086/api/status
journalctl -u fsm-pi-master-monitor.service -n 100 --no-pager
```

若日志显示地址已被占用，检查旧的 `w102-ipc-monitor-186.service` 是否还在运行。只在确认迁移后执行 `./install.sh --replace-legacy`。

## 网页打开但树莓派离线

```bash
ip route get 192.168.1.186
ping -c 3 192.168.1.186
curl http://192.168.1.186:8089/api/status
```

路由的 `src` 必须是 `192.168.1.102`。检查网线、车载路由器、树莓派供电和工控机车内网口静态地址。树莓派重启后，工控机接收器会自动识别新的 `boot_id` 并请求最新缓存帧。

## BEV 有黑色方向或缺相机

网页查看四颗相机的 online、age 和 fresh mask。完整掩码是 15。先检查对应 USB 接头/有源 Hub 供电，再查看 FSM-Pi-001 服务日志。工控机只接收成品，不应在这里修改摄像头设备映射。

## 交通灯页面有图但 ROS 2 无话题

```bash
systemctl status fsm-pi-master-traffic-camera.service
curl http://127.0.0.1:8090/status
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=92
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 topic list | grep traffic_light_front
```

订阅者必须使用同一 Domain ID。图像和 CameraInfo 使用 BEST_EFFORT Sensor Data QoS；可靠 QoS 不匹配时可能看不到数据。

## 中文神经语音无法生成

```bash
cd ~/FSM-Pi-Master/voice
sha256sum --check SHA256SUMS
~/FSM-Pi-Master/voice-env/bin/python -c 'import piper, onnxruntime; print("OK")'
ffmpeg -filters 2>/dev/null | grep rubberband
```

如果 `.onnx` 只有几十或几百字节，克隆到的是 Git LFS 指针而不是真模型。重新获取大文件后运行 `./install.sh`。原系统录音由树莓派直接播放，不依赖工控机 Piper。

## 抓拍或录制后模式未恢复

正常退出会恢复原模式；强制杀死调用程序可能来不及恢复。执行：

```bash
~/FSM-Pi-Master/scripts/camera_stream.sh mode bev
```

树莓派每次重新通电也会回到 BEV 默认模式。

