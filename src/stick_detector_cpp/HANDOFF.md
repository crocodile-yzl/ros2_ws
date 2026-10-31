# 挡车杆检测项目 — 交接手册（给下一个 AI / 接手者）

> 目标：把 Berxel iHawk100 深度相机的点云，用于识别地面上的 **Π 形门字挡车杆（800×200mm）**，
> 输出位姿与宽高，为避障 demo 提供"前方是否有障碍、在哪、多大"。
> 本文档是权威交接材料：读完它能独立接手、编译、运行、调试、继续开发。

---

## 0. 一句话现状

- 检测链路 **已完成并验证**：纯 C++17 实现，单帧 **~1.2ms**，仿真精确检出挡车杆（W=0.84m H=0.21m）。
- **唯一未完成的实机环节**：真实相机的 **深度↔点云一致性校准** 还没出结果（用户实机数据我这边容器收不到）。
- 相机驱动包 `berxel_camera_ros2` 是厂商 SDK 自带，已能跑通（深度流 + 点云话题）。

---

## 1. 环境与路径（全部固化，勿改）

| 项 | 值 |
|---|---|
| ROS 2 | Humble（`/opt/ros/humble`） |
| 工作区 | `~/ros2_ws`，**编译产物目录是 `build_y` / `install_y` / `log_y`**（不是默认 build/install） |
| 相机 SDK | `/home/y/berxel-sdk`（v2.0.115，x86-64 预编译库） |
| 相机驱动包 | `~/ros2_ws/src/berxel_camera_ros2`（厂商 SDK 自带，含 launch `berxel_camera_iHawk100.py`） |
| 检测包（生产） | `~/ros2_ws/src/stick_detector_cpp`（**本项目的核心，C++**） |
| 检测包（参考实现） | `~/ros2_ws/src/stick_detector`（Python，算法验证用，逻辑与 C++ 一一对应） |
| SDK 项目根 | `/home/y/berxel-sdk/RosSDKNode/berxel_camera_ros2`（含 .workbuddy 记忆与历史 WORKLOG） |

**关键约定**：
- 系统 Python 是 3.10（ROS 用）；`conda (base)` 是 3.14，会劫持 rqt 等工具 → 用 rqt 前必须 `conda deactivate`。
- 系统 numpy 是 2.2.6，**cv2 / scipy 直接崩**（`_ARRAY_API not found`）→ 所有算法必须 numpy/C++ 自实现，禁止引 cv2/scipy。
- 我的执行 shell 跑在**受限容器**：能 `lsusb` 看到相机、能 DDS 发现话题，但**无 `/dev/bus/usb`**（相机节点打不开）、
  **收不到真实点云数据**（FastDDS 走共享内存，容器不共享）。→ 相机必须由**用户本机终端**启动；实机排查只能靠用户贴日志。

### 编译命令（务必带 `--*-base *_y`）

```bash
source /opt/ros/humble/setup.bash
cd ~/ros2_ws
colcon --log-base log_y build --packages-select stick_detector_cpp \
       --build-base build_y --install-base install_y
```

**⚠️ 用户侧每次我重编后，用户终端必须重新 `source ~/ros2_ws/install_y/setup.bash`**（或开新终端），
否则报 `package 'xxx' not found`（install 的包索引在 source 时固定）。

---

## 2. 检测包文件清单（stick_detector_cpp）

```
src/stick_detector_cpp/
├── include/stick_detector_cpp/geometry.hpp   # 纯算法库（无 ROS）：CC、RANSAC、PCA、四元数
├── src/stick_detector_node.cpp               # 检测主节点（51 个参数）
├── src/calib_check_node.cpp                  # 深度↔点云校准诊断节点
├── src/cloud_sim_node.cpp                    # 仿真点云（含深度图、尺度误差注入）
├── config/stick_detector.yaml                # 全部参数（中文注释，可直接改）
├── launch/  (3 个：sim_test / stick_detect / camera_with_detector)
├── rviz/stick_detector.rviz                  # 蓝色底图 + 绿ROI + 红棍子叠加
└── README.md                                 # 完整文档（参数表/性能/排查，见 §6）
```

### 话题（两包完全同名，可互换）

| 话题 | 类型 | 说明 |
|---|---|---|
| 输入 `/berxel_cloudpoint` | PointCloud2 | 相机点云（需 **有序**，x/y/z/rgb FLOAT32） |
| `/depth/depth_raw` | Image 16UC1 | 深度图（**毫米**），校准用 |
| `/stick_detector/sticks` | PoseArray | 目标位姿（+x 轴=横杆方向） |
| `/stick_detector/markers` | MarkerArray | rviz：红线/圆柱/文字 + `roi_box` 线框 |
| `/stick_detector/stick_points` | PointCloud2 | 目标点云 |
| `/stick_detector/roi_cloud` | PointCloud2 | **ROI 裁剪后**点云（排查"ROI 有没有生效"用） |

---

## 3. 算法管线（读懂它就能继续开发）

```
parseCloud（PointCloud2 → 有序网格，stride 降采样，cloud_scale 尺度修正）
  → roi 盒子裁剪（x/y/z 范围）
  → ground_roi 地面高通：估地平面 → 只保留离地 hp_min..band_max 的点
       （地面/墙/天花板/行人整块消失，点数 -80%）
  → 连通分量（游程并查集，8连通+1px容差）
  → 每个团块：
       stick_enable 且细长比/长度/半径达标 → 长直棍
       否则 barrier_enable → detect_barrier：最高点∈[10,45]cm、最低点贴地、
           取顶部35%点 PCA 得"水平细长横杆"、横杆长=宽度∈[0.4,1.6] → 挡车杆
  → 发布
```

**地面估计三种模式**（`ground_plane_mode`）：
1. `auto`（默认）：RANSAC 全点估平面，每 `ground_plane_refresh` 帧刷新；
2. `manual`：`camera_height` + `camera_pitch_deg`（下俯为正）直接构造法向 `n=(0,cosθ,sinθ)`；
（注：iHawk100 无 IMU，故已移除 `ground_imu_enable` 外部 IMU 模式，仅保留 auto/manual。）

**挡车杆尺寸口径**：`W` 是顶杆**最外沿**长度。图纸 800mm 若是两端管**中心距**、管径 Ø40，实测 = 0.84m（两端各多一个管半径，恒定偏大 40mm）。

---

## 4. 关键参数速查（config/stick_detector.yaml，共 51 个）

| 参数 | 默认 | 说明 |
|---|---|---|
| `cloud_topic` | `/berxel_cloudpoint` | 输入 |
| `process_hz` | 10.0 | 处理帧率上限 |
| `downsample_stride` | 2 | 降采样步长 |
| `roi_enable` + `roi_x/y/z_min/max` | true | 3D 盒子裁剪 |
| `ground_roi_enable` | true | 地面带裁剪总开关 |
| `ground_plane_mode` | auto | auto / manual |
| `camera_height` / `camera_pitch_deg` | 0.80 / 15.0 | manual 用（下俯为正） |
| `ground_hp_enable` / `ground_hp_min` | true / 0.012 | 挖掉地面点，离地 1.2cm 算障碍 |
| `ground_band_max` | 0.30 | **必须 ≥0.30**（挡车杆高 200mm） |
| `cloud_scale` | 1.0 | 尺度修正（=校准节点给的 1/ratio） |
| `stick_enable` | false | 直棍检测（挡车杆场景关闭） |
| `barrier_enable` + `barrier_*` | true | 挡车杆判定 |
| `publish_roi_cloud` / `roi_box_marker` | true | ROI 可视化 |
| `debug_log` / `profile` | false | 日志 / 计时（launch 参数可传） |

---

## 5. 运行方式

```bash
# 仿真自测（无相机，推荐先跑这个确认链路）
ros2 launch stick_detector_cpp sim_test.launch.py

# 实机（相机 + 检测 + rviz 一条命令）
ros2 launch stick_detector_cpp camera_with_detector.launch.py
ros2 launch stick_detector_cpp camera_with_detector.launch.py rviz:=false

# 日志/计时（launch 参数，不是 --ros-args）
ros2 launch stick_detector_cpp camera_with_detector.launch.py debug_log:=true profile:=true

# 深度↔点云校准诊断
ros2 run stick_detector_cpp calib_check_node
```

仿真期望输出：`detected 1 target(s): #1 BARRIER W=0.84m H=0.21m c=(0.00,-0.08,2.66)`

---

## 6. 已知坑与经验（务必读，省大量时间）

1. **`np.linalg.svd` 默认 `full_matrices=True`** 曾让 RANSAC 单次 362ms（构造 N×N 矩阵）。
   已改用 3×3 协方差 `eigh` → 4.8ms。**Python 参考实现 geometry_core 也已是 eigh**，勿改回 svd。
2. **躺平面的棍子被平面剔除后只剩 3-5px 厚"背脊带"**：任何 3×3 腐蚀会清零残差 → 不要对残差做腐蚀；
   连通分量的 1px 容差不能去掉，否则棍子碎成 0.28m 片段。
3. **池化聚类**（2×2 多数投票 + dilate3）只用于初始大团块；**残差重聚类必须全分辨率**（薄带会被池化切碎）。
4. **C++ 编译坑**：`ament_target_dependencies(... Eigen3)` 不传 include，必须 `target_link_libraries(... Eigen3::Eigen)`；
   `std::max(declare_parameter<int>(...), 1)` 模板推导报 `max(long,int)`，先赋值再判；
   本环境 `sed -i` 偶发 "设备或资源忙"，改用 python 写文件。
5. **rclcpp 订阅回调**：用 `std::bind(&Class::cb, this, _1)` + 按值 `SharedPtr`（lambda const-ref 形式 Humble 编译不过）。
6. **launch 参数**：`ros2 launch ... debug_log:=true` 是 launch 参数写法；`--ros-args` 是错的（ros2 launch 不接）。
   加 launch 参数时要同时 `DeclareLaunchArgument` + `ParameterValue(..., value_type=bool)` + `from launch.substitutions import LaunchConfiguration`（这个 import 曾漏过一次）。
7. **rviz 看不到 ROI 是正常的**：ROI 只作用内部，原始 `/berxel_cloudpoint` 从不裁剪；要看效果开绿色 `ROICloud` + `roi_box`。
8. **相机 yaml**：需 `enable_ordered_pointcloud: true`（默认已是）；点云单位是**米**、深度图单位是**毫米**。
9. **性能基线**：C++ 1.0-1.3ms/帧（Python 10.2ms）；NUC 15 Pro 预估 1.5-2ms，毫无压力。
   真正的瓶颈是相机 30Hz 4MB 点云传输（建议 `depth_fps: 15`）和 rviz 渲染（NUC 上别开）。

---

## 7. 下一步（按优先级）

### P0：深度↔点云校准（当前卡在这里，最该做）
用户实机跑 `calib_check_node` 时**没出 `[calib]` 结果**（只有启动两行），已给校准节点加了诊断，
会打印三种之一：`size mismatch`（尺寸/坐标不对）/ `stamp gap`（时间戳不同步）/ `no valid pixel pairs`。
**先拿到这行诊断**，判定"点云烂"是：
- 尺度问题 → `cloud_scale` 一次修正；
- 内参/畸变问题（分块比值有方向性）→ 需重标相机内参，另一条路。

### P1：实机调参
拿到用户 `debug_log:=true` 的 `ROI: A -> B px (-NN%)` 与 `candidates(...)` 行，按真实数据定
地面阈值（`ground_plane_thresh`）与 ROI 范围。

### P2：避障输出（用户多次提到 demo）
把检测结果封装成避障友好的形式：`nav_msgs/OccupancyGrid` 或自定义"前方距离/横向区间/是否被挡"话题，
供下游运动规划直接消费。**尚未实现**。

### P3：若目标不止"挡车杆"
`stick_enable: true` 可同时检测长直棍（杆/管），判据已在（细长比≥5、长度∈[0.25,3]m、半径≤35mm）。

---

## 8. 记忆文件

- 项目记忆：`/home/y/berxel-sdk/RosSDKNode/berxel_camera_ros2/.workbuddy/memory/2026-10-27.md`（当日详细记录）
- 历史日志：同目录 `WORKLOG_2026-10-21.md`（相机驱动调试全过程）
- 本手册位置：`~/ros2_ws/src/stick_detector_cpp/HANDOFF.md`
