# 186 树莓派鱼眼摄像头图传调用说明

## 1. 工作方式

树莓派始终采集四颗摄像头，但任意时刻只发布一种输出模式：

输出模式是整台树莓派当前进程的全局运行状态，多个程序同时调用时以最后一条成功指令为准。录像任务运行期间，其他程序不要再切换模式。模式不会持久化；树莓派断电、重启或摄像头服务重启后一定返回 `bev`。

| 模式 | 说明 | 树莓派负载与网络流量 |
|---|---|---|
| `bev` | 四路标定、光照调整并融合为 BEV | 默认模式，约 10 Hz |
| `raw-all` | 前、后、左、右四路鱼眼原始视角分别以目标 10 Hz 直传 | 停止 BEV 计算，网络流量最高 |
| `front` | 只发送前摄像头 | 最低延迟、较低流量 |
| `rear` | 只发送后摄像头 | 最低延迟、较低流量 |
| `left` | 只发送左摄像头 | 最低延迟、较低流量 |
| `right` | 只发送右摄像头 | 最低延迟、较低流量 |

树莓派启动配置 `gateway.env` 固定使用 `DEFAULT_OUTPUT_MODE=bev`。接口切换只影响当前运行进程，不会修改该文件。

树莓派长期保存的是四颗摄像头的内外参。当前服务从以下目录读取：

`/home/fsm-pi-001/Pi-5-image/bev/calibration/provisional/`

文件分别为 `front-provisional.yaml`、`rear-provisional.yaml`、`left-provisional.yaml` 和 `right-provisional.yaml`。后续完成正式标定时，应替换或更新这些标定文件，而不是保存图传模式。

工控机服务地址为 `http://127.0.0.1:8086`。这些 HTTP 接口仅绑定工控机回环地址，供工控机本机程序调用，不向车外网络开放。

面向后续图像算法的四路并行 Python SDK、数据字段及可靠性约定见
`FISHEYE_RAW_SDK.md`。SDK 位于 `/home/fsm-lab/Pi-capture/fisheye_raw_sdk.py`。

## 2. 查询与切换模式

查询当前模式和各图像源的最新帧状态：

```bash
curl -s http://127.0.0.1:8086/api/stream/status
```

切换为四路原图：

```bash
curl -s -X POST http://127.0.0.1:8086/api/stream/mode \
  -H 'Content-Type: application/json' \
  -d '{"mode":"raw-all"}'
```

切回融合 BEV：

```bash
curl -s -X POST http://127.0.0.1:8086/api/stream/mode \
  -H 'Content-Type: application/json' \
  -d '{"mode":"bev"}'
```

将 `mode` 替换为 `front`、`rear`、`left` 或 `right` 即可只发送指定摄像头。接口收到树莓派对当前运行状态的确认后返回成功。

前端页面顶部也提供同样的模式选择框和“切换画面”按钮。

## 3. 获取最新 JPEG

```text
GET /api/stream/frame/bev.jpg
GET /api/stream/frame/front.jpg
GET /api/stream/frame/rear.jpg
GET /api/stream/frame/left.jpg
GET /api/stream/frame/right.jpg
GET /api/stream/frame/raw-all.jpg
```

`raw-all.jpg` 是工控机为了浏览而临时排列的 2×2 预览；UDP 中收到的仍是四张独立原始 JPEG，并没有在树莓派上融合。

`GET /api/stream/status` 的每个 `sources` 条目还包含当前 `frame_id`、`age_s`、
`fps`、`flags` 和 `available`，适合程序判断该方向是不是最新有效画面。

保存前摄像头最新帧：

```bash
curl -f http://127.0.0.1:8086/api/stream/frame/front.jpg \
  -o front.jpg
```

请求尚未收到的图像源时返回 HTTP 503。程序应先切换到对应单路模式，或切换到 `raw-all` 后再分别读取四路。

## 4. 低延迟 MJPEG 接口

```text
GET /api/stream/mjpeg?source=front&fps=10
GET /api/stream/mjpeg?source=rear&fps=10
GET /api/stream/mjpeg?source=left&fps=10
GET /api/stream/mjpeg?source=right&fps=10
GET /api/stream/mjpeg?source=bev&fps=10
```

接收端只保留每个图像源的最新完整帧，慢速调用方不会让旧帧积压。树莓派摄像头侧使用 V4L2 单缓冲，UDP 侧使用分片、帧 CRC 和启动编号；因此发生丢包时会丢弃不完整帧，而不会显示破损图像。

OpenCV 示例：

```python
import cv2

url = "http://127.0.0.1:8086/api/stream/mjpeg?source=front&fps=10"
capture = cv2.VideoCapture(url)
while True:
    ok, frame = capture.read()
    if not ok:
        break
    # frame 是前鱼眼摄像头的最新 BGR 图像
capture.release()
```

## 5. 工控机命令行工具

工具位置：

`/home/fsm-lab/Pi-capture/camera_stream.sh`

查询状态：

```bash
/home/fsm-lab/Pi-capture/camera_stream.sh status
```

切换模式：

```bash
/home/fsm-lab/Pi-capture/camera_stream.sh mode raw-all
/home/fsm-lab/Pi-capture/camera_stream.sh mode bev
```

保存四张当前原图：

```bash
/home/fsm-lab/Pi-capture/camera_stream.sh snapshot --source all
```

文件保存到 `/home/fsm-lab/Pi-capture/capture-image/`，命名示例：

`20260901_143012_123456_front.jpg`

同时录制四颗鱼眼摄像头 60 秒：

```bash
/home/fsm-lab/Pi-capture/camera_stream.sh record \
  --source all --duration 60 --fps 10
```

视频保存到 `/home/fsm-lab/Pi-capture/record-video/`。工具会先记住当前模式，临时切换到所需模式，结束后自动恢复。添加 `--keep-mode` 可在录制结束后保留原图模式。

录制前摄像头 10 分钟：

```bash
/home/fsm-lab/Pi-capture/camera_stream.sh record \
  --source front --duration 600 --fps 10
```

## 6. Python 调用模式接口

```python
import json
from urllib.request import Request, urlopen

request = Request(
    "http://127.0.0.1:8086/api/stream/mode",
    data=json.dumps({"mode": "front"}).encode("utf-8"),
    headers={"Content-Type": "application/json"},
    method="POST",
)
with urlopen(request, timeout=3) as response:
    result = json.load(response)
print(result)
```

## 7. UDP 控制协议预留接口

需要绕过工控机 HTTP 服务时，可直接向树莓派 `192.168.1.186:5611/UDP` 发送 W102 v1 控制包：

- 命令号：`6`（设置输出模式）
- `stream_mask`：BEV=`0x8000`，前=`0x0001`，后=`0x0002`，左=`0x0004`，右=`0x0008`，四路=`0x000F`
- 控制来源必须是工控机 `192.168.1.102`，其他源地址会被树莓派忽略。
- 树莓派以 `CONTROL_JSON` 返回确认，其中包含 `accepted`、`output_mode`、`output_stream_mask` 和 `nonce`。

优先推荐 HTTP 接口，因为它已处理 UDP 分片、CRC、超时、确认和错误信息。

## 8. 与一次性四路标定抓拍的区别

原有“一键保存四路原图”使用命令号 `5`，只请求一次四路最新帧，不改变持续输出模式。它仍然适合摄像头标定。持续查看或录像请使用本说明中的模式切换和 MJPEG 接口。
