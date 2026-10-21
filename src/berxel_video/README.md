# berxel_video

博升（Berxel）iHawk 深度相机的 ROS 2 入口包：启动厂商驱动 `berxel_camera_ros2`，
把彩色/深度重映射到统一话题：

| 输出话题 | 类型 | 来源 |
| --- | --- | --- |
| `/video` | `sensor_msgs/Image` | `color/color_raw` |
| `/video/depth` | `sensor_msgs/Image` | `depth/depth_raw` |

相机还有 `color/camera_info`、`depth/camera_info`、`berxel_cloudpoint` 等原生话题，
保持原样不动。

## 依赖

1. **Berxel SDK**（`/home/y/berxel-sdk`），提供 `libBerxelHawk.so` 和头文件。
2. **厂商驱动包** `berxel_camera_ros2`，已拷贝到 `/home/y/ros2_ws/src/berxel_camera_ros2`。

驱动包通过环境变量 `BERXEL_SDK_LIBRARY` 找 SDK：

```bash
source /home/y/berxel-sdk/berxel_set_env.sh
```

不 source 的话，CMake 会退回到相对路径 `../../Include`，在 src 布局下找不到。

> 容器的 `HOME` 是 `/root`，所以**不要写 `~/berxel-sdk`**（会解析成 `/root/berxel-sdk`），
> 一律用绝对路径 `/home/y/berxel-sdk`。
>
> 容器只挂载了 `/home/y/ros2_ws` 和 `/home/y/berxel-sdk`（后者在 compose 里新加）。
> 如果还没重建过容器，SDK 的临时副本在 `/home/y/ros2_ws/berxel-sdk`，
> `source /home/y/ros2_ws/berxel-sdk/berxel_set_env.sh` 同样可用。

## udev 规则（只装一次，装在宿主机）

SDK 里的 `berxel-usb.rules` 必须在**宿主机**安装，容器内的 udev 对宿主无效：

```bash
# 宿主机
sudo cp /home/y/berxel-sdk/berxel-usb.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
# 然后拔插一次相机
```

不做这一步，容器里会报 `LIBUSB_ERROR_ACCESS` 或直接 `Not find device!`。

## 构建

```bash
# 容器内
source /home/y/berxel-sdk/berxel_set_env.sh
cd /home/y/ros2_ws
colcon build --packages-select berxel_camera_ros2 berxel_video
source install/setup.bash
```

## 运行

```bash
ros2 launch berxel_video berxel_video.launch.py
```

默认 `stream_flag=2`（Mix VGA，彩色+深度 640x400）、`stream_type=3`（彩色+深度）。

常用变体：

```bash
# 深度对齐到彩色
ros2 launch berxel_video berxel_video.launch.py enable_align:=true

# 只要彩色 1280x800
ros2 launch berxel_video berxel_video.launch.py stream_flag:=3 stream_type:=1

# 还要点云（发在 /berxel_cloudpoint）
ros2 launch berxel_video berxel_video.launch.py enable_pointcloud:=true

# 改输出话题
ros2 launch berxel_video berxel_video.launch.py video_topic:=/camera/color
```

## 参数

| 参数 | 默认 | 说明 |
| --- | --- | --- |
| `stream_flag` | 2 | 1 单流 / 2 Mix VGA(640x400) / 3 Mix HD(1280x800) / 4 Mix QVGA(320x200) |
| `stream_type` | 3 | 位掩码：1 彩色 / 2 深度 / 3 彩色+深度 / 4 泛光IR / 5 点光IR |
| `depth_fps` | 30 | 深度帧率 |
| `enable_align` | false | 深度对齐到彩色 |
| `enable_pointcloud` | false | 发点云 |
| `enable_denoise` | false | 深度降噪 |
| `video_topic` | `/video` | 彩色输出 |
| `depth_topic` | `/video/depth` | 深度输出 |
| `serial_number` | 空 | 多机时按序列号选 |

更多硬件参数（激光电流、曝光、增益、温度补偿）在 `config/berxel_ihawk.yaml` 里，
含义见厂商 SDK 的 `RosSDKNode/berxel_camera_ros2/README.MD`。

## 验证

```bash
ros2 topic list | grep -E "video|color|depth"
ros2 topic hz /video
ros2 run rqt_image_view rqt_image_view /video
```

## 排错

- **`Not find device!`**
  1. 容器里 `lsusb` 能看到 `0603:0009`；
  2. `/dev/bus/usb` 已挂载（compose 里那条 `- /dev/bus/usb:/dev/bus/usb`）；
  3. 宿主已装 udev 规则并拔插过。

- **`Load uvc driver failed` / 库找不到**
  `echo $LD_LIBRARY_PATH` 里应该有 `/home/y/berxel-sdk/libs`，没有就
  `source /home/y/berxel-sdk/berxel_set_env.sh`。

- **只出彩色、没有深度**
  检查 `stream_flag` 和 `stream_type` 是否匹配：只要 `stream_type=1`（纯彩色）时，
  深度本来就不会发布。

- **分辨率不是预期的 640x400**
  `stream_flag` 决定硬件模式，`color_width/color_height` 在 Mix 模式下由固件决定，
  改 yaml 里的分辨率不一定生效，优先换 `stream_flag`。

## 与 d435i 那套的关系

| 场景 | 用哪个 |
| --- | --- |
| Intel D435i | `port_camera_publisher/launch/d435i_video.launch.py`（官方 realsense2_camera） |
| Intel D435i、纯 C | `d435i_video_c`（rcl C API + librealsense C API） |
| 博升 iHawk | **本包**（厂商 `berxel_camera_ros2` + 话题重映射） |

三个包输出的话题一致（`/video`、`/video/depth`），下游节点不用改。
