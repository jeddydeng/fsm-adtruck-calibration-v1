# Pi-capture 四路鱼眼相机即时抓图

该程序在工控机上运行，通过 UDP 向 `192.168.1.186:5611` 请求树莓派
返回四颗鱼眼相机的最新原始 JPEG。它不会开启持续四路原图传输，也不会改变
现有 BEV 和状态数据的 10 Hz 发送。

## 使用方法

```bash
cd /home/fsm-lab/Pi-capture
./capture_all.sh
```

成功后会一次保存四个文件到：

```text
/home/fsm-lab/Pi-capture/capture-image/
```

文件名示例：

```text
20260823_143015_327481_front.jpg
20260823_143015_327481_rear.jpg
20260823_143015_327481_left.jpg
20260823_143015_327481_right.jpg
```

四个文件使用同一个精确到微秒的本地时间戳，表示同一次请求。只有四张图全部通过分片和
整图 CRC 校验后才会保存；若某颗相机离线，不会生成真假混合的一组文件，程序
会列出缺失的英文方向并返回退出码 2。

可选参数：

```bash
./capture_all.sh --timeout 2 --retries 5
CAPTURE_OUTPUT_DIR=/data/calibration ./capture_all.sh
```

程序只使用 Python 3 标准库，不需要 ROS、OpenCV 或 Conda 环境。树莓派只接受
来自固定工控机 IP `192.168.1.102` 的控制请求。

当前 186 树莓派服务中的方向映射固定为：流 0=`FRONT`、流 1=`REAR`、
流 2=`LEFT`、流 3=`RIGHT`。如果以后调整相机物理插口，必须同时更新树莓派
网关配置中的四个 `CAMERA_*` 路径，不能只改文件名。

## 测试

```bash
cd /home/fsm-lab/Pi-capture
python3 -m unittest discover -s tests -v
```
