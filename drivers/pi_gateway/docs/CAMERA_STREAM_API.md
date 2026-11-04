# 四鱼眼/BEV 调用接口

工控机监控服务默认只在 `127.0.0.1:8086` 提供 HTTP 接口，树莓派的启动默认模式始终是四路融合 BEV。客户端临时切换后应恢复之前的模式。

## 命令行

```bash
cd ~/FSM-Pi-Master
./scripts/camera_stream.sh status
./scripts/camera_stream.sh mode raw-all
./scripts/camera_stream.sh snapshot --source all
./scripts/camera_stream.sh record --source all --duration 60 --fps 10
./scripts/camera_stream.sh mode bev
```

抓拍写入 `runtime/capture-image/`，录像写入 `runtime/record-video/`。`snapshot` 和 `record` 默认会记住原模式并在结束后恢复；只有显式添加 `--keep-mode` 才保持新模式。

## HTTP

```text
GET  /api/stream/status
POST /api/stream/mode                 JSON: {"mode":"raw-all"}
GET  /api/stream/frame/bev.jpg
GET  /api/stream/frame/front.jpg
GET  /api/stream/frame/rear.jpg
GET  /api/stream/frame/left.jpg
GET  /api/stream/frame/right.jpg
GET  /api/stream/mjpeg?source=front&fps=10
```

允许的模式为 `bev`、`raw-all`、`front`、`rear`、`left`、`right`。原始鱼眼图为 1600×1200 JPEG，目标频率 10 Hz；BEV 为 512×512。HTTP 状态中的 `frame_id`、`age_s`、`fps` 应用于判断新鲜度，不要把重复获取同一 JPEG 当作新采集帧。

## Python SDK

```python
import sys
sys.path.insert(0, "/home/USER/FSM-Pi-Master/capture")
from fisheye_raw_sdk import FisheyeRawReceiver

with FisheyeRawReceiver("http://127.0.0.1:8086", fps=10) as receiver:
    frames = receiver.read_all(timeout_s=2, max_age_s=0.5)
    front_bgr = frames["front"].bgr
    print(front_bgr.shape, frames["front"].fps, frames["front"].age_s)
```

`FisheyeRawReceiver.start()` 切换到 `raw-all` 并并行接收四路；`close()`/上下文退出在没有其他调用者抢占模式时恢复启动前模式。每路 `RawFrame` 含方向、序号、接收时间、测得帧率和 OpenCV BGR 数组。

## 可靠性约束

- 不要多个程序同时频繁切换模式。
- 处理线程不要阻塞 SDK 的四个接收线程；耗时算法应复制帧后进入自己的队列。
- 连接丢失后，网页接收器会持续 HELLO 并接受 Pi 最新缓存帧；SDK MJPEG 线程会重连。
- 工控机程序停止后，树莓派交通灯相机租约会停止发送；BEV 主通道由监控服务恢复时自动续接。

