# stick_detector_cpp — 挡车杆 / 长直棍检测（C++ 版）

从 Berxel 相机的有序深度点云中分割并识别**贴地障碍物**：主目标是 **Π 形门字挡车杆（800×200mm）**，
同时保留长直棍（杆/管）检测能力（`stick_enable` 开关）。

纯 C++17 + Eigen + rclcpp，**无 Python 依赖**，单帧 **~1.2 ms**（640×400，stride=2）。

---

## 编译

```bash
source /opt/ros/humble/setup.bash
cd ~/ros2_ws
colcon --log-base log_y build --packages-select stick_detector_cpp \
       --build-base build_y --install-base install_y
source install_y/setup.bash
```

## 运行

```bash
# 1) 仿真自测（无相机；C++ 合成点云 + 检测）
ros2 launch stick_detector_cpp sim_test.launch.py
#   期望输出：
#   detected 1 target(s): #1 BARRIER W=0.84m H=0.21m c=(-0.00,-0.08,2.66)

# 2) 接真实相机（相机 + 检测 + rviz 一条命令）
ros2 launch stick_detector_cpp camera_with_detector.launch.py
ros2 launch stick_detector_cpp camera_with_detector.launch.py rviz:=false

# 3) 只跑检测（相机节点已在别处运行）
ros2 launch stick_detector_cpp stick_detect.launch.py
```

> 相机节点必须在能直接访问 USB 的本机终端启动（容器/受限 shell 无 `/dev/bus/usb`）。

## 输出

独立地面分割（相机节点已启动）：

```bash
ros2 run stick_detector_cpp ground_segment_node
rviz2 -d /home/y/ros2_ws/install_y/stick_detector_cpp/share/stick_detector_cpp/rviz/ground_segment.rviz
```

该节点默认保留光轴深度小于 3 米的有效点（`max_range` 可调），
按水平、正立的光学坐标系估计地面；下俯安装时用
`--ros-args -p camera_pitch_deg:=15.0` 指定角度。自动模式只拟合符合地面方向、
位于相机下方且至少占采样点 10% 的平面（`ground_min_inlier_ratio` 可调）。
没有有效地面时，`/ground/ground_cloud` 为空，所有有效点暂时发布到
`/ground/non_ground`，不发布新的地面方程。输出点云使用 Reliable QoS，
仅包含 xyz 字段，RViz 应使用 FlatColor；专用配置已设置原始点云、绿色地面与蓝色非地面。

| 话题 | 类型 | 说明 |
|---|---|---|
| `/stick_detector/sticks` | `geometry_msgs/PoseArray` | 目标位姿；orientation 的 +x 轴 = 横杆/棍轴方向 |
| `/stick_detector/markers` | `visualization_msgs/MarkerArray` | rviz：红色轴线（棍子另加实心圆柱）+ 文字（`barrier W/H` 或棍长） |
| `/stick_detector/stick_points` | `sensor_msgs/PointCloud2` | 分割出的目标点云（xyz，米） |
| `/stick_detector/roi_cloud` | `sensor_msgs/PointCloud2` | **ROI 裁剪后的点云**（rviz 里绿色显示，与原始蓝云对比即可看到裁剪效果） |
| `/stick_detector/markers` 的 `roi_box` | Marker(LINE_LIST) | ROI 盒子线框（绿），用来对照裁剪范围是否设对 |

日志：`detected 1 target(s): #1 BARRIER W=0.84m H=0.21m c=(x,y,z)`

## 算法流程

1. 解析 `PointCloud2` → 有序 `H×W` 网格（`downsample_stride` 降采样）
2. `roi_enable`：3D 盒子裁剪（x/y/z 范围）
3. `ground_roi_enable`：RANSAC 估计地面平面（`auto` 自动 / `manual` 用 `camera_height`+`camera_pitch_deg`，每 `ground_plane_refresh` 帧刷新）
4. `ground_hp_enable`：按离地高度做**高通**——只保留 `ground_hp_min .. ground_band_max`（默认 1.2–30cm），
   地板/墙/天花板/行人整块消失（实测点数 -80%），并跳过后续逐团块 RANSAC
5. 游程并查集连通分量（8 连通 + 1px 容差）
6. 每个团块：PCA → 细长比/长度/半径
   - `stick_enable` 且达标 → 长直棍
   - 否则 `barrier_enable`：最高点 ∈ [10,45]cm、最低点贴地、取顶部 35% 点 PCA 得到**水平细长横杆**，
     横杆长度=宽度 ∈ [0.4,1.6]m → 挡车杆
7. 发布 PoseArray / MarkerArray / 目标点云

## 参数（`config/stick_detector.yaml`）

| 参数 | 默认 | 说明 |
|---|---|---|
| `cloud_topic` | `/berxel_cloudpoint` | 输入点云 |
| `process_hz` | 10.0 | 处理频率上限 |
| `downsample_stride` | 2 | 像素降采样 |
| `max_range` | 5.0 | 深度上限（米） |
| `min_cluster_px` | 250 | 最小团块像素数（降采样后） |
| `roi_enable` / `roi_x/y/z_*` | true / 见 yaml | 3D 盒子裁剪 |
| `ground_roi_enable` | true | 地面高度带裁剪总开关 |
| `ground_plane_mode` | auto | `auto`=自动估地面；`manual`=用 height+pitch |
| `camera_height` / `camera_pitch_deg` | 0.80 / 15.0 | manual 模式参数（下俯为正） |
| `ground_plane_refresh` | 30 | 每 N 帧重估地面 |
| `ground_hp_enable` | true | 用地平面挖掉地面点 |
| `ground_hp_min` | 0.012 | 离地多少算障碍 |
| `ground_band_max` | 0.30 | 只看到离地 30cm（挡车杆高 200mm，**必须 ≥0.30**） |
| `stick_enable` | false | 目标只有挡车杆时关闭 |
| `min_length` / `max_length` / `min_elongation` / `max_radius` | 0.25/3.0/5.0/0.035 | 长棍判据 |
| `barrier_enable` | true | 挡车杆判定 |
| `barrier_width_min/max` | 0.40 / 1.60 | 宽度范围（米） |
| `barrier_height_min/max` | 0.10 / 0.45 | 高度范围（米） |
| `barrier_top_frac` | 0.65 | 取顶部 35% 点当横杆 |
| `barrier_min_elongation` | 3.0 | 横杆细长比下限 |
| `barrier_max_tilt` | 0.45 | 横杆与地面法向夹角余弦上限（越小越水平） |
| `publish_roi_cloud` | true | 发布 `/stick_detector/roi_cloud`（裁剪后点云，便于肉眼确认 ROI） |
| `roi_box_marker` | true | 发布 ROI 盒子线框 |
| `ground_plane_fallback_manual` | true | `auto` 估不到地面时回退到 `camera_height`/`camera_pitch_deg` 并告警一次 |
| `debug_log` | false | 打印 `ROI: A -> B px (-NN%)` 与候选团块指标 |
| `profile` | false | 每 5 帧打印各阶段耗时 |

## 性能（实测，640×400，stride=2）

| 阶段 | 耗时 |
|---|---|
| parse | 0.2 ms |
| mask + 地面高通 | <0.1 ms |
| 连通分量 | <0.1 ms |
| PCA + 分类 | 0.1 ms |
| 发布 | 0.1 ms |
| **合计** | **1.0–1.3 ms** |

对比 Python 版 10.2 ms（同一算法、同场景同结果），约 **8–10×** 加速；
NUC 15 Pro 上预估 1.5–2 ms/帧，30Hz 输入全处理也只是个位数 CPU 占用。

## 与 Python 版的关系

`stick_detector`（Python）是算法验证用的参考实现，逻辑与结果一致（BARRIER W=0.84m H=0.21m）。
生产/部署请使用本包 `stick_detector_cpp`。两包话题名完全兼容，可互换。

## 排查："看起来没有 ROI"

**ROI 只作用于内部处理，不会裁剪 rviz 里显示的原始 `/berxel_cloudpoint`。** 要"看到" ROI：

1. rviz 里打开 `ROICloud (裁剪后)`（绿色，来自 `/stick_detector/roi_cloud`）与 `roi_box` 线框；
   绿色点只有原始蓝云的一小部分，说明裁剪生效。
2. 或看日志：**launch 参数**（不是 `--ros-args`）

```bash
ros2 launch stick_detector_cpp camera_with_detector.launch.py debug_log:=true
ros2 launch stick_detector_cpp camera_with_detector.launch.py debug_log:=true profile:=true
```

```
ROI: 6935 -> 1431 px (-79%, ground removed)
```

若这里的 `-NN%` 接近 0，通常是**地面没估到**（地面点少/强反光/相机没拍到地面）：

- 日志里会先出现 `auto ground plane not found -> fallback to manual (...)`
- 此时把 `ground_plane_mode` 设为 `manual`，并填准 `camera_height`（离地高度，米）和
  `camera_pitch_deg`（下俯角，正数），ROI 立刻生效
- 也可以先用 `roi_enable` 的 x/y/z 盒子把范围收紧（`debug_log` 同样能看到压缩率）

## 深度↔点云校准（calib_check_node）

**目的**：量化"点云和深度图是否一致"，即透视/尺度误差到底多大。

```bash
ros2 run stick_detector_cpp calib_check_node
```

它逐像素比较 `depth_raw(mm)` 与点云 z，输出 **全局比值 + 4×4 分块比值**：

```
[calib] global ratio z_cloud/depth = 1.0000 (n=33382 px), suggested cloud_scale = 1.0000
      1.000  1.000  1.000  1.000
      ...
```

判读：
- `ratio ≈ 1.000`：深度→点云转换是一致的，"点云烂"不是尺度问题（查别的）；
- `ratio = k ≠ 1`：存在**系统尺度误差**，把 `1/k` 填到 `config/stick_detector.yaml` 的 `cloud_scale`，
  节点在解析时统一乘上该系数即可修正（已实测：0.79m 误测 → 0.84m 正确）；
- 各分块比值**差异很大且有方向性**（左小右大/中心偏）：是**内参/畸变**问题，不是单一尺度系数能修的，
  需要按相机内参重新标定（把 `calib_check` 的分块表发给我可进一步定位）。

## 独立地面分割（ground_segment_node）

想**单独验证/使用地面识别**（不跑完整检测）时：

```bash
ros2 run stick_detector_cpp ground_segment_node
```

输出：

| 话题 | 类型 | 说明 |
|---|---|---|
| `/ground/plane` | `shape_msgs/Plane` | 平面方程 `a·x+b·y+c·z+d=0`（coef[4]） |
| `/ground/ground_cloud` | PointCloud2 | 平面上的点（地面） |
| `/ground/non_ground` | PointCloud2 | 平面以上的点（障碍物） |

参数：`ground_plane_mode`(auto/manual)、`camera_height`、`camera_pitch_deg`、`ground_thresh`、
`max_slope_deg`(默认15°，法向与期望地面法向夹角超此值则拒绝，防止把墙面当地面)。

rviz 里把 `ground_cloud`（灰）和 `non_ground`（红）叠起来看，能直接判断地面分割质量。

## 已知限制

- 仅支持**有序**点云（organize 后按像素邻接聚类），需相机 yaml `enable_ordered_pointcloud: true`
- `auto` 地面估计假设"地面是场景最大平面"；地面点太少（强反光/纯黑地毯）时用 `manual`
- 半径/宽度是**外沿**尺寸：Ø40 管、中心距 800mm 时测得 0.84m（两端各多一个管半径）
