# port_camera_publisher

把相机图像采集下来，转成 ROS 2 图像消息发布到 `/video`。

主线是 **Intel RealSense D435i**：走 RealSense 官方 SDK 封装（`realsense2_camera`），
彩色流映射到 `/video`，可选把对齐后的深度发到 `/video/depth`。
包里另外保留了一个通用采集节点（UVC 摄像头 / RTSP / HTTP-MJPEG / 视频文件）和一个
裸 TCP 端口收 JPEG 的节点，作为备选。

面向 **ROS 2 Jazzy**（Ubuntu 24.04，Python 3.12）编写。

## 内容

| 文件 | 用途 |
| --- | --- |
| `launch/d435i_video.launch.py` | **D435i 主入口**：官方 `realsense2_camera` → `/video` |
| `video_publisher` | 通用采集：UVC 相机索引 / `rtsp://` / `http://` / 视频文件 |
| `tcp_jpeg_receiver` | 对端往 TCP 端口推 JPEG 字节流时使用 |
| `check_video` | 订阅话题体检（帧率/分辨率/编码），可存一张图 |

输出话题：

| 话题 | 类型 | 说明 |
| --- | --- | --- |
| `/video` | `sensor_msgs/Image` | D435i 彩色图，`rgb8` |
| `/video/depth` | `sensor_msgs/Image` | 对齐到彩色的深度，`16UC1`，单位毫米（需 `enable_depth:=true`） |

QoS 是 `BEST_EFFORT`（相机类话题的标准做法）。订阅端也要用 best-effort，
`ros2 topic echo` 需加 `--qos-reliability best_effort`，否则收不到。

## 一、D435i 快速开始

```bash
# 1. 容器里装依赖（realsense2_camera + cv_bridge + python3-opencv）
bash /home/y/ros2-jazzy-docker/scripts/install-video-deps.sh

# 2. 进容器
bash /home/y/ros2-jazzy-docker/ros2.sh --shell

# 3. 构建
cd /home/y/ros2_ws
colcon build --packages-select port_camera_publisher --symlink-install
source install/setup.bash

# 4. 启动（1280x720@30 彩色 → /video）
ros2 launch port_camera_publisher d435i_video.launch.py
```

开深度并对齐到彩色：

```bash
ros2 launch port_camera_publisher d435i_video.launch.py \
    enable_depth:=true align_depth:=true
# /video        rgb8   彩色
# /video/depth 16UC1   深度（毫米，已对齐到彩色像素）
```

多台相机按序列号选：

```bash
ros2 launch port_camera_publisher d435i_video.launch.py serial_no:=123622270123
# 序列号从 rs-enumerate-devices 或 lsusb -d 8086:0b3a -v 里看
```

## 二、D435i 参数

| launch 参数 | 默认 | 说明 |
| --- | --- | --- |
| `color_profile` | `1280x720x30` | 彩色 `宽x高x帧率`，必须是 D435i 支持的组合 |
| `depth_profile` | `640x480x30` | 深度 `宽x高x帧率` |
| `enable_depth` | `false` | 是否开深度流 |
| `align_depth` | `false` | 深度对齐到彩色（开深度时才有意义） |
| `serial_no` | 空 | 空 = 自动选第一台 |
| `pointcloud` | `false` | 发布点云（需开深度） |
| `video_topic` | `/video` | 彩色输出话题 |
| `depth_topic` | `/video/depth` | 深度输出话题 |
| `base_frame` | `camera_link` | TF 基坐标系 |

D435i 支持的常用彩色组合（`宽x高x帧率`）：

```
1920x1080x30   1280x720x30   960x540x60   848x480x60
  640x480x60      640x360x60   424x240x60   320x240x60
```

深度常用：`1280x720x30`、`848x480x90`、`640x480x30/60/90`。

## 三、验证

```bash
# 帧率与分辨率统计，顺手存一张图
ros2 run port_camera_publisher check_video --ros-args -p duration:=5.0 -p save:=/tmp/f.jpg

# 直接看话题
ros2 topic hz /video
ros2 topic echo /video --qos-reliability best_effort --field encoding --once

# 有图形界面
ros2 run rqt_image_view rqt_image_view /video
ros2 run rviz2 rviz2   # Fixed Frame 设 camera_link，加 Image 显示 /video
```

`check_video` 期望看到约 30 fps、`1280x720 rgb8`、`frame_id=camera_color_optical_frame`。

### 在宿主机的 rqt 里看

容器是 `network_mode: host`，宿主机（Humble）可以直接订阅容器里的 `/video`：

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4   # 与容器 fastdds.xml 对齐，避开共享内存权限问题

ros2 topic list | grep video              # 先确认能发现话题
rqt_image_view                            # 左上角下拉选 /video
```

**rqt_image_view 里要把 Reliability 切到 Best Effort**，否则一直黑屏：相机话题是
`BEST_EFFORT`，默认的 Reliable 订阅接不上。

`ros2 topic list` 里没有 `/video`（跨发行版发现偶发失败）时，退回容器内跑 rqt，
窗口一样显示在宿主机桌面：

```bash
bash /home/y/ros2-jazzy-docker/ros2.sh --shell   # 首次需先跑一次 gui 模式生成 X11 授权
rqt_image_view
```

## 四、容器与 USB 注意事项

- **必须插 USB 3.0 口**（蓝色/SS 标识），用原装线。插 USB 2.0 会只剩个位数帧率，
  甚至 `RuntimeError: Couldn't resolve requests`。
- 确认容器能看到设备：

  ```bash
  lsusb -d 8086:0b3a          # D435i 的 VID:PID
  ls /dev/video*              # 彩色/深度都会注册 V4L2 节点
  rs-enumerate-devices        # librealsense 自带，能列出固件版本和支持的 profile
  ```

- `docker-compose.yml` 里已是 `privileged: true`，设备通常直接可见。若不行，
  在 `ros2` 服务下补：

  ```yaml
      devices:
        - /dev/bus/usb:/dev/bus/usb
  ```

  个别机器还需要在**宿主机**装 udev 规则后重新插拔（容器内的 udev 规则对宿主无效）：

  ```bash
  sudo cp /etc/udev/rules.d/99-realsense-librealsense2.rules /etc/udev/rules.d/ 2>/dev/null
  sudo udevadm control --reload-rules && sudo udevadm trigger
  ```

- 热插拔：容器 privileged 下一般可用；若拔插后节点不恢复，重启容器即可。

## 五、其它源（备选）

`video_publisher` 不依赖 RealSense，走 OpenCV：

```bash
# USB 摄像头（UVC，索引 0 = /dev/video0）
ros2 run port_camera_publisher video_publisher --ros-args -p source:=0

# RTSP 摄像头
ros2 launch port_camera_publisher video_publisher.launch.py \
    source:=rtsp://admin:12345@192.168.1.10:554/stream1

# HTTP-MJPEG
ros2 run port_camera_publisher video_publisher --ros-args -p source:=http://192.168.1.10:8080/video

# 视频文件（按源帧率回放，可当"假相机"联调）
ros2 run port_camera_publisher video_publisher --ros-args -p source:=/data/test.mp4

# 对端主动往 8080 端口推 JPEG（按 FFD8...FFD9 切帧）
ros2 launch port_camera_publisher tcp_jpeg_receiver.launch.py port:=8080
```

> D435i 也能被 OpenCV 当 UVC 打开（`source:=0`），但只能拿到彩色、拿不到深度，
> 且部分分辨率/帧率组合不可用。正经用还是走 `d435i_video.launch.py`。

`video_publisher` 常用参数：`source`、`topic`、`publish_rate`、`width`/`height`、
`camera_fps`、`fourcc`（USB 相机像素格式，720p 以上要用 `MJPG`）、`encoding`、
`publish_compressed`（同时发 `<topic>/compressed`）。

## 六、实现要点

- **D435i 走官方封装**：`realsense2_camera` 负责 profile 协商、时间戳、TF、
  IMU；我们只做话题重映射，不重复造轮子。
- **通用节点读写解耦**：读帧线程只保留最新帧，rclpy 定时器负责发布；相机卡顿、
  拔插不会堵住执行器，也不会把旧帧发出去，断流自动重连。
- **显式后端**：相机走 `CAP_V4L2`，网络/文件走 `CAP_FFMPEG`，避免 `CAP_ANY`
  误选 GStreamer 后端直接失败。
- **低延迟**：`CAP_PROP_BUFFERSIZE=1`、QoS depth 1。

## 七、踩坑

- **`No device connected`**：USB 口/线不行，或容器没映射设备；先 `lsusb -d 8086:0b3a`。
- **`Couldn't resolve requests`**：请求的分辨率/帧率组合该型号不支持，或 USB 带宽不够
  （同时开 1080p 彩色 + 深度常见），换 `640x480x30` 试。
- **帧率上不去**：USB 2.0 口、多相机共用一条总线、或 CPU 被别的进程占满。
- **订阅 `/video` 收不到**：QoS 不匹配，订阅端用 reliable 连不上 best-effort 的发布端。
- 宿主机 Python 若装过 numpy 2.x，会顶掉系统 numpy 导致 `import cv2` 报
  `numpy.core.multiarray failed to import`。容器内不受影响；在宿主机跑请用
  `PYTHONNOUSERSITE=1`。
