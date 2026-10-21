# d435i_video_c

Intel RealSense D435i 图像发布节点，**纯 C 实现**：采集用 librealsense2 的 C API
（`rs2_*`），发布用 ROS 2 的 C 客户端库 `rcl` + rosidl 生成的 C 消息类型
（`sensor_msgs__msg__Image`）。不依赖 rclcpp / rclpy。

功能与 `port_camera_publisher` 里的 D435i 方案一致：

| 话题 | 类型 | 说明 |
| --- | --- | --- |
| `/video` | `sensor_msgs/Image` | 彩色，`bgr8` |
| `/video/depth` | `sensor_msgs/Image` | 深度，`16UC1`，毫米，可对齐到彩色 |

QoS 默认 `BEST_EFFORT` + `KEEP_LAST(1)`，与相机类话题惯例一致。

## 为什么是纯 C

`rcl` 在 Jazzy 里仍是官方 C 客户端库，`rosidl` 会为每条消息生成 C 结构体
（`sensor_msgs__msg__Image`），所以不用 C++ 也能直接发图像。代价是没有 ROS 参数
系统和自动 remap——参数通过命令行传，话题名用 `--topic` 指定。

## 依赖

```bash
# ROS 侧：rcl + sensor_msgs（容器里已有）
# RealSense 侧：librealsense2 的开发文件（头文件 + CMake config）
apt-get install -y librealsense2-dev          # Ubuntu 源
# 或者用 ROS 打包版（随 realsense2_camera 一起装的话已经有了）
apt-get install -y ros-jazzy-librealsense2
```

确认 CMake 能找到：

```bash
ls $(ros2 pkg prefix librealsense2)/share/librealsense2/cmake 2>/dev/null
# 或
find / -name "realsense2Config.cmake" 2>/dev/null
```

## 构建

```bash
cd /home/y/ros2_ws
colcon build --packages-select d435i_video_c
source install/setup.bash
```

## 运行

```bash
# 彩色 1280x720@30 → /video
ros2 run d435i_video_c d435i_video_c

# 彩色 + 对齐深度
ros2 run d435i_video_c d435i_video_c --depth=true --align=true

# 降分辨率 / 选相机 / 换话题
ros2 run d435i_video_c d435i_video_c --width 640 --height 480 --fps 30
ros2 run d435i_video_c d435i_video_c --serial=123622270123
ros2 run d435i_video_c d435i_video_c --topic /camera/color --depth-topic /camera/depth
```

或走 launch：

```bash
ros2 launch d435i_video_c d435i_video_c.launch.py
ros2 launch d435i_video_c d435i_video_c.launch.py enable_depth:=true align_depth:=true
ros2 launch d435i_video_c d435i_video_c.launch.py width:=640 height:=480
```

## 参数

| 参数 | 默认 | 说明 |
| --- | --- | --- |
| `--width` / `--height` | 1280 / 720 | 彩色分辨率 |
| `--fps` | 30 | 彩色帧率 |
| `--topic` | `/video` | 彩色话题 |
| `--depth=true` | false | 开深度流 |
| `--depth-width` / `--depth-height` | 640 / 480 | 深度分辨率 |
| `--depth-fps` | 30 | 深度帧率 |
| `--align=true` | false | 深度对齐到彩色 |
| `--depth-topic` | `/video/depth` | 深度话题 |
| `--serial=SN` | 空 | 多机时按序列号选，空 = 第一台 |
| `--frame-id` | `camera_link` | header.frame_id |
| `--reliable=true` | false | 改用 RELIABLE QoS |

布尔参数写成 `--depth=true`（等号），不带值时等同于 `true`。

## 实现要点

- **单线程流水**：`rs2_pipeline_wait_for_frames` 取帧 → 发布 → 循环。纯发布节点不需要
  executor，省掉 spin。
- **对齐**：`rs2_create_align(RS2_STREAM_COLOR)` + 帧队列（`rs2_start_processing_queue`）+
  `rs2_wait_for_frame`，把异步处理块变成同步取帧。
- **零拷贝之外的最小开销**：图像消息复用一块 buffer，只在分辨率变大时 `realloc`。
- **按流类型分发**：遍历 frameset，用 `rs2_get_stream_profile_data` 判断是
  `RS2_STREAM_COLOR` 还是 `RS2_STREAM_DEPTH`，不依赖帧在集合里的顺序。

## 与 Python 版的差异

| | `d435i_video.launch.py`（Python 包） | 本包（C） |
| --- | --- | --- |
| 驱动 | `realsense2_camera`（官方节点） | 直接调 librealsense C API |
| 参数 | ROS 参数 / launch | 命令行 |
| 额外话题 | 内参、TF、IMU、点云 | 只有彩色和深度 |
| 依赖 | `realsense2_camera` | `librealsense2-dev` |

要 TF、内参、点云、IMU，仍然建议用官方 `realsense2_camera`；本包适合只需要
图像流、又不想引入 C++ 节点和整套相机话题的场景。

## 排错

- **`没有找到 RealSense 设备`**：`lsusb -d 8086:0b3a` 确认设备；
  容器要映射 `/dev/bus/usb`；插 USB 3.0 口。
- **`配置彩色流 失败` / `启动管线 失败`**：分辨率帧率组合不支持，或 USB 带宽不足，
  用 `rs-enumerate-devices` 查支持的组合，降到 640x480@30 试。
- **编译时 `find_package(realsense2)` 失败**：装 `librealsense2-dev`，
  或把 ROS 版 librealsense2 的 cmake 目录加进 `CMAKE_PREFIX_PATH`。
