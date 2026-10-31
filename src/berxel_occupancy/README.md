# P100R 固定相机概率占据实验

独立 C++17 ROS 2 包，试验 SCAN-Planner 所用的概率占据建图原理。
没有复制 SCAN-Planner 源码，没有调用 RANSAC，也没有接入导航规划器。

## 启动

关闭 HawkDepth 和其他占用 P100R 的相机驱动；原来的 ground_segment_node 不需要运行。

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install_y/setup.bash
source ~/berxel-sdk/berxel_set_env.sh
ros2 launch berxel_occupancy occupancy.launch.py
```

默认同时启动已有 P100R 驱动、占据节点和 RViz。
驱动使用已有 berxel_ihawk_100R.yaml，覆盖关闭点云、彩色输出与设备时间戳。
若已经启动相机驱动：

```bash
ros2 launch berxel_occupancy occupancy.launch.py start_camera:=false
```

无显示器测试可加 `rviz:=false`。配置文件可以复制修改后用 `params_file:=/绝对路径/occupancy.yaml` 传入。
单独启动：`ros2 run berxel_occupancy occupancy_node --ros-args --params-file /home/y/ros2_ws/src/berxel_occupancy/config/occupancy.yaml`。

## 算法与输入

- 输入 `/depth/depth_raw` 和 `/depth/camera_info`，支持 mono16 / 16UC1 毫米与 32FC1 米，支持两种字节序和图像行步长。
- 校验有限正焦距、主点、K、图像缓冲区、CameraInfo.width 和光学 frame 一致。Berxel 驱动目前把 CameraInfo.height 留为零、把高度填进 binning_x；因此以 Image 的尺寸为准，不对 K 再做 binning 缩放。
- 使用 K 针孔反投影，未实现额外畸变校正；如更换为有明显畸变的原始深度输入，应提供已校正且匹配内参的深度图。
- 相机固定离地 0.90m、向下俯视 34°；光学坐标 x 右、y 下、z 前转换为地图 x 前、y 左、z 上。光心地图位置 `(0,0,0.90)`。令 θ=34°，转换为 `x=cosθ*z-sinθ*y; y=-x_optical; z=0.90-sinθ*z-cosθ*y`。
- 默认 10cm 体素，20Hz 只处理最新一张新图，像素步长4；同一光学 frame 与时间戳的连续重复图不重复累积。中间帧会丢弃，不建立队列。
- 地图界限 x [-0.5,4.0)、y [-1.5,1.5)、z [-0.2,2.0) 米；光轴深度有效范围 [0.2,3.0] 米。
- 有界三维 DDA 射线：相机至有效端点之间的体素更新空闲，端点更新占据。有效点在地图外时，射线裁剪至地图界限，边界只更新空闲，不造出障碍。
- 零、NaN、Inf、过近、超过有效深度范围的返回全部跳过，**不会当成最大射线清空未知区域**。
- 每个体素每帧最多更新一次，有命中则优先命中。log-odds 命中 +0.85、未命中 -0.4，截断 [-2,3.5]；>=0.7 占据，<=-0.2 空闲；中间是已观测但不确定，未观测是未知。
- 已有障碍需要后续有效射线穿过才能清除；没有时间衰减。物体移走但露出的背景没有有效深度时，原占据会保留。

## 输出与观察

| 话题 | 内容 |
|---|---|
| `/occupancy/occupied` | 所有占据体素中心，包括地板，橙色 |
| `/occupancy/free` | 明确空闲体素，RViz 默认关闭以降低绘制负担 |
| `/occupancy/obstacles` | 中心离地 0.15～1.8m 的占据体素，红色 |
| `/occupancy/projection` | 高度带二维投影：100占据，0整列高度带均已观测为空闲，-1未知或不确定 |
| `/occupancy/processed_frames` | 已处理新深度图计数，用于检查是否更新 |

RViz 固定坐标系 `occupancy_map`，无需外部 TF。输出消息使用源深度图时间戳。
原始占据地图保留地面几何，只对障碍显示应用高度筛选；不是地面拟合或语义识别。
二维投影采用保守规则：一列高度带存在任何占据体素就为100；只有全部被纳入的高度体素均明确空闲才为0；包含未知或不确定则为-1。
未做机器人轮廓与障碍膨胀，不可直接作为安全导航代价地图。

## 限制和调优

相机在实验过程中必须固定。移动相机而不提供里程计/TF 会产生重影；本节点不消费动态位姿。
不做障碍膨胀、类别识别、运动预测或自动避障。
10cm 栅格和步长4可能遗漏细杆等小障碍；地面安装误差也会影响高度筛选。
CPU 是否下降必须实测比较，概率射线建图并不天然比单帧分割便宜。
降低 `update_rate`、增大 `pixel_stride`、减小地图范围可减负；同时影响时间与空间细节。
节点只在收到新图时发布地图，停流时 RViz 保留最后一次地图，查看帧计数可确认更新。

## 编译与测试

```bash
source /opt/ros/humble/setup.bash
cd ~/ros2_ws
/usr/bin/colcon --log-base log_y build --build-base build_y --install-base install_y --packages-select berxel_occupancy --cmake-args -DCMAKE_BUILD_TYPE=Release -DPython3_EXECUTABLE=/usr/bin/python3 -DPYTHON_EXECUTABLE=/usr/bin/python3
/usr/bin/colcon --log-base log_y test --build-base build_y --install-base install_y --packages-select berxel_occupancy
source install_y/setup.bash
ROS_DOMAIN_ID=147 /usr/bin/python3 src/berxel_occupancy/test/smoke_ros.py
```

单元测试包含 DDA、裁剪、清除、去重、命中优先、概率截断、未知、坐标变换与深度格式。
smoke_ros.py 使用合成数据、独立 DDS domain，不启动实体相机；检查消息元数据、实际 ROS 输出和旧图不重复更新。

## RViz 操作与相机位置

默认工具为 Move Camera，默认视角为 Orbit：鼠标放在右侧三维画面中，左键拖动旋转，中键拖动平移，滚轮缩放。按 Esc 回到默认工具；也可点击顶部 Move Camera。Views 面板中可选择 Orbit 并调整 Distance、Pitch、Yaw。

青色球是相机光心 `(0,0,camera_height)`（默认离地0.90m），标有 P100R camera；青色竖线连接光心和地面投影；绿色箭头表示实际参数指定的光轴方向（默认向前下俯34°）。地面 z=0，灰色网格每格0.5m。坐标轴 X 红色向前、Y 绿色向左、Z 蓝色向上。

橙色体素是测得的全部占据几何，包含地板；红色体素是离地0.15～1.8m的占据区域，并不是物体类别识别。青色球与绿色箭头仅为相机参考标记，不是测出的障碍。蓝色空闲空间默认不显示，以免遮挡。
