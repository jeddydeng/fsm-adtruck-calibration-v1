# 四路鱼眼原始直传模式与 Python SDK

## 模式定义

模式名称为 `raw-all`。启用后，树莓派停止 BEV 融合计算，把前、后、左、右
四个物理方向的画面作为四条互相独立的 JPEG 图像流发送到工控机，每路目标频率
10 Hz。模式只在当前运行周期有效；树莓派或摄像头服务重启后仍恢复默认 `bev`。

当前接口参数：

| 项目 | 值 |
|---|---|
| 方向名称 | `front`、`rear`、`left`、`right` |
| 单路分辨率 | 640×480，运行中固定 |
| 单路目标频率 | 10 Hz |
| 网络编码 | JPEG，质量 75 |
| 图像处理 | 不进行去畸变、BEV、光照补偿或画面融合 |
| 丢帧策略 | 只保留最新完整帧；UDP 分片不完整或 CRC 错误时丢弃整帧 |
| 相机缺失 | 对应方向停止更新并报告不可用，另外三路继续发送 |

这里的“原始”指原始鱼眼视角，而不是相机传感器的 Bayer RAW。摄像头提供 MJPEG，
现有统一采集进程将其解码为 BGR 后按质量 75 编码为独立 JPEG；没有改变几何视角。

## 启用和恢复

```bash
/home/fsm-lab/Pi-capture/camera_stream.sh mode raw-all
/home/fsm-lab/Pi-capture/camera_stream.sh status
/home/fsm-lab/Pi-capture/camera_stream.sh mode bev
```

也可以使用本机 HTTP：

```text
POST http://127.0.0.1:8086/api/stream/mode  {"mode":"raw-all"}
GET  http://127.0.0.1:8086/api/stream/status
```

状态接口的 `sources` 字段分别给出每一路的 `available`、`frame_id`、`age_s`、
`fps` 和传输 `flags`。后续程序应同时检查 `available=true`、`age_s < 0.5`，不能
把很久以前的缓存帧当成当前画面。

## Python 图像处理 SDK

SDK 安装位置：

`/home/fsm-lab/Pi-capture/fisheye_raw_sdk.py`

下面的程序会切换到四路原图模式，并发接收四路最新 BGR 图像；退出上下文时，
只有在没有其他调用者改变模式的情况下，才恢复进入程序前的模式。

```python
import sys
sys.path.insert(0, "/home/fsm-lab/Pi-capture")

from fisheye_raw_sdk import FisheyeRawReceiver

with FisheyeRawReceiver(fps=10) as cameras:
    while True:
        frames = cameras.read_all(timeout_s=1.0, max_age_s=0.5)

        front_bgr = frames["front"].bgr
        rear_bgr = frames["rear"].bgr
        left_bgr = frames["left"].bgr
        right_bgr = frames["right"].bgr

        # 在这里调用后续识别、保存或录像算法。
        # sequence: 工控机SDK内每路递增序号
        # received_unix_ns: 工控机收到并解码该帧的系统时间
        print({name: (item.sequence, item.fps, item.age_s)
               for name, item in frames.items()})
```

读取某一路的下一张新图：

```python
last_sequence = 0
while True:
    frame = cameras.read("front", after_sequence=last_sequence, timeout_s=1)
    last_sequence = frame.sequence
    process(frame.bgr)
```

`read_all()` 返回的是四颗自由运行 USB 摄像头各自“最新”的一帧，不代表硬件同步
曝光。如果后续算法要求亚毫秒级严格同步，必须增加支持外触发的摄像头和同步线；
软件不能把四颗普通 USB 摄像头变成硬件同步相机。

## 无 SDK 的单路接口

最新完整 JPEG：

```text
GET /api/stream/frame/front.jpg
GET /api/stream/frame/rear.jpg
GET /api/stream/frame/left.jpg
GET /api/stream/frame/right.jpg
```

连续 MJPEG：

```text
GET /api/stream/mjpeg?source=front&fps=10
GET /api/stream/mjpeg?source=rear&fps=10
GET /api/stream/mjpeg?source=left&fps=10
GET /api/stream/mjpeg?source=right&fps=10
```

这些地址仅在工控机本机开放。慢速消费者不会反压树莓派，也不会累积旧帧。
