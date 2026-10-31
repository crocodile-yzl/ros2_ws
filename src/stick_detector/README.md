# stick_detector — 点云长棍分割与识别

从 Berxel 相机的有序点云中分割并识别**贴地障碍物**：主目标是 **Π 形门字挡车杆（800×200mm）**，
同时保留通用的长直棍（杆/管）检测能力（可用 `stick_enable` 开关）。
输出位姿 + 尺寸 + rviz 可视化 + 目标点云。

纯 `numpy` 实现（连通分量、RANSAC 平面、PCA 全部自研），不依赖 Open3D / PCL / cv2 / scipy。

---

## 快速开始

### 1. 仿真自测（无需相机）

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install_y/setup.bash
ros2 launch stick_detector sim_test.launch.py
```

终端会持续打印：

```
detected 2 stick(s): #1 L=0.40m r=22mm e=6 c=(0.30,0.14,0.96);
                     #2 L=0.88m r=18mm e=17 c=(-0.10,0.24,0.97)
```

仿真是 640×400 有序点云，按真实安装几何搭建：相机离地 0.80m、下俯 15°，
地板 + 墙面 + 挡车杆（800×200mm，管径 40mm，离地 200mm）+ 干扰箱子 + 干扰立柱。
验收标准：**只检出 1 个挡车杆（W≈0.84m H≈0.21m），箱子/立柱/墙面不报**：

```
detected 1 stick(s): #1 BARRIER W=0.84m H=0.21m c=(-0.00,-0.08,2.66)
```

### 2. 接真实相机

**一条命令同时起相机 + 检测 + rviz**（推荐，需在能直接访问相机的本机终端运行，容器/远程 shell 无 `/dev/bus/usb` 权限会报 "No Connected BerxelDevice"）：

```bash
ros2 launch stick_detector camera_with_detector.launch.py            # 带 rviz
ros2 launch stick_detector camera_with_detector.launch.py rviz:=false  # 不要 rviz
```

rviz 里默认就是：**蓝色点云底图 + 红色棍子（实心圆柱 + 轴线 + 长度文字）**，无需手动配置。

或分开启动：

```bash
# 终端 1：相机驱动（相机需已插好）
ros2 launch berxel_camera_ros2 berxel_camera_iHawk100.py

# 终端 2：识别节点
ros2 launch stick_detector stick_detect.launch.py

# 终端 3：可视化
rviz2 -d $(ros2 pkg prefix stick_detector)/share/stick_detector/rviz/stick_detector.rviz
```

> ⚠️ 两点坑（看不到点云的常见原因）：
> 1. 驱动自带 `berxel_default.rviz` 里点云显示项是 `Enabled: false`（写死的），要用本包提供的配置；
> 2. rviz 默认 Fixed Frame 是 `map`，必须改成 `berxel_camera/berxel_base_link`（本包配置已设好）。

**首次实机调参**：把 `config/stick_detector.yaml` 里 `debug_log` 改为 `true`，
每帧会打印所有候选团块的实测指标，与阈值对比即可判断该调什么：

```
candidates(3): L=0.86 r=18 e=17 | L=0.40 r=23 e=6 | L=0.12 r=41 e=1
[need L 0.25-3.0 e>=5.0 r<=35mm]
```

**前提**：相机 yaml 中 `enable_ordered_pointcloud: true`（默认已是 true）。
节点收到非有序点云会告警并跳过。

## 输出

| 话题 | 类型 | 说明 |
|---|---|---|
| `/stick_detector/sticks` | `geometry_msgs/PoseArray` | 每个目标的位姿：position=中心，orientation 的 +x 轴 = 棍轴/横杆方向 |
| `/stick_detector/markers` | `visualization_msgs/MarkerArray` | rviz：红色实心圆柱/轴线 + 文字（棍长 或 挡车杆 W/H） |
| `/stick_detector/stick_points` | `sensor_msgs/PointCloud2` | 分割出的棍子点云（xyz，米） |

日志每帧打印：长度 L、截面半径 r、细长比 e、中心坐标。

## 算法流程

1. 解析有序点云 → `H×W×3`，按 `downsample_stride` 降采样
2. 有效像素掩膜（NaN/超距过滤）→ **自研游程并查集** 8 连通分量聚类
3. 大团块做 **RANSAC 平面剔除**（关键：棍子贴着桌面/地面时，必须先把支撑平面去掉才能分离）
4. 残差像素重新聚类
5. 每个子团块 **PCA**：主轴、长度（p1~p99 鲁棒跨度）、半径（到轴 RMS 距离）、细长比 `sqrt(λ1/λ2)`
6. 三条同时满足判为长棍：`min_length ≤ L ≤ max_length`、`e ≥ min_elongation`、`r ≤ max_radius`
7. **核心修剪精化**：若首次判定失败，按首主轴裁出半径 `max_radius` 内的核心再拟合一次（应对棍子与其他物体粘连），但要求核心保留 ≥60% 点，避免把两个物体“修剪”成假棍

## 参数（`config/stick_detector.yaml`）

| 参数 | 默认 | 说明 |
|---|---|---|
| `cloud_topic` | `/berxel_cloudpoint` | 输入话题 |
| `process_hz` | 8.0 | 处理频率上限（掉帧策略，30fps 输入只处理约 8fps） |
| `downsample_stride` | 2 | 像素降采样；1=精度最高最慢 |
| `max_range` | 5.0 | 有效深度上限（米） |
| `min_cluster_px` | 250 | 最小团块像素数（降采样后），滤噪点 |
| `plane_remove` | true | 是否剔除主导平面 |
| `plane_thresh` | 0.012 | 平面内点距离阈值（米） |
| `plane_min_px` | 2500 | 触发平面剔除的最小团块 |
| `plane_min_ratio` | 0.35 | 平面内点占比阈值 |
| `min_length` | 0.25 | 棍长下限（米） |
| `max_length` | 3.0 | 棍长上限（米） |
| `min_elongation` | 5.0 | 细长比下限 |
| `max_radius` | 0.035 | 截面半径上限（米），也用作精化修剪半径 |
| `debug_log` | false | true 时每帧打印候选团块 L/r/细长比（首次调参用） |
| `profile` | false | true 时每 10 帧打印各阶段耗时 |
| `roi_enable` | true | 是否启用 3D 盒子 ROI 裁剪 |
| `roi_x_min/max` | -1.5 / 1.5 | 横向范围（米） |
| `roi_y_min/max` | -0.10 / 2.0 | **竖向范围（相机光学系 +y 向下）**：地面障碍场景把它卡在相机高度附近，直接丢掉天花板/高处杂物 |
| `roi_z_min/max` | 0.20 / 5.0 | 前后距离（米） |

**调参建议**
- 检测太灵敏（把桌腿、椅子框报成棍）→ 提高 `min_elongation`（8~12）、提高到 `min_length`
- 棍子漏检 → 降低 `min_elongation`；若棍子很细远，降低 `min_cluster_px`
- 棍子与背景粘连 → 确认 `plane_remove: true`，必要时微调 `plane_thresh`
- 目标尺寸已知 → 直接收紧 `min_length`/`max_length`/`max_radius` 三件套

## 挡车杆（Π 形门字杆）识别

目标若是 **800×200mm 的门字形挡车杆**，单纯的"长棍细长比"判据会把整团判成不细长（细长比只有 ~4）。
本包内置了专用判定 `barrier_enable`：

1. 用地平面算出高度：要求最低点贴地（h_min < 8cm）、最高点 h_max ∈ [barrier_height_min, barrier_height_max]
2. 取顶部 35% 点做 PCA → 必须是**水平、细长**的横杆（`barrier_min_elongation`、`barrier_max_tilt`）
3. 横杆长度即挡车杆宽度，必须落在 `barrier_width_min..max`

输出与长棍共用同一套话题（`/sticks`、`/markers`），日志形如：

```
detected 2 stick(s): #1 BARRIER W=0.84m H=0.21m c=(0.00,-0.08,2.66); #2 L=0.99m r=16mm e=22
```

注意两点：
- `ground_band_max` 必须 ≥ 0.30，否则 200mm 高的顶杆会被地面带裁掉（默认已 0.30）；
- 目标只有挡车杆时把 `stick_enable: false`（默认已是），避免把箱体边缘、立柱误判成直棍。

尺寸口径：`W` 是顶杆**最外沿**长度。若图纸 800mm 是两端管中心距，Ø40 管实测会得 0.84m（两端各多 20mm），
差值恒定、可直接减去；若 800mm 是外沿，则实测就是 0.80m。`H` 为最高点离地高度（含管半径）。

## 已知限制

- **半径是下界**：躺在地面/桌面上的棍，紧贴支撑面的下半部分会被平面剔除吃掉，实测半径略小于真实值（仿真中 20mm 测得 18mm）
- **偶发漏检/拆段**：两物体在像素上被噪声桥接时（罕见），该帧可能漏检；一根棍被噪声切断时可能报成两段（每段长度仍准确）
- **仅支持有序点云**：依赖像素邻接做快速聚类，无序点云会跳过并告警
- **单平面假设**：每个团块只剔除一个主导平面；场景中多个交错平面（如 L 形桌面）可能残留
- 需要目标在视野内且长度覆盖足够像素（远距离细棍像素太少会低于 `min_cluster_px`）

## 性能（实测）

640×400 点云、`downsample_stride: 2`、单线程 Python：

| 阶段 | 优化前 | 优化后 |
|---|---|---|
| 平面 RANSAC | 362 ms（`np.linalg.svd` 默认 `full_matrices=True` 生成 N×N 矩阵） | 4–5 ms（改 3×3 协方差特征分解） |
| 连通分量 | 23 ms | 2–3 ms（2× 多数投票池化 + 膨胀、内联并查、向量化根压缩） |
| 解析/掩膜/PCA/发布 | ~4 ms | ~5 ms |
| 地面高通（新增） | — | 去掉逐团块 RANSAC，点数 -85% |
| **单帧合计** | **~380 ms（2.6 fps）** | **10–12 ms（>80 fps）** |

**地面高通裁剪**（贴地障碍的杀手锏）：先用 RANSAC 估计地面平面（每 30 帧刷新一次，
`auto` 模式无需标定），只保留 `ground_hp_min..ground_band_max`（默认离地 1.2–15cm）的点。
仿真（相机 0.8m 高、下俯 15°、棍子离地 0–4cm @1.8m）实测：**11465 → 1671 px（-85%）**，
地板、墙面、天花板、行人全部消失，检测精度不变（0.99m / 1.93m）。

**NUC 15 Pro 部署评估**：该流程为纯 CPU numpy、无 CUDA 依赖，单线程为主。
Core Ultra 7 255H 单核性能与本机接近（约 0.85–0.95×），预估单帧 **12–15 ms**（地面高通开启时），
配合 `process_hz: 10` 可稳定 10 Hz 输出。注意两点：
1. 相机 yaml 把 `depth_fps: 30` 降到 `15`，可把 DDS 带宽从 ~120 MB/s 减半；
2. NUC 上不建议同时跑 rviz（渲染 25 万点很吃 iGPU），
   建议 rviz 只在调试机上看，或用 `stick_points`（点少）验证。
