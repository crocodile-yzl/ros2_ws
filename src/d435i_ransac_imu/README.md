# d435i_ransac_imu

`d435i_ransac` 的 IMU 版本：在原有地面分割基础上，用 D435i 内置加速度计测得的重力方向
作为 RANSAC 的平面先验，取代写死的 `camera_pitch_deg`。

Intel RealSense D435i depth input with the `stick_detector_cpp` standalone ground RANSAC segmentation ported unchanged in its fitting and classification algorithm. The optional stick detector is not included: this package focuses on ground / non-ground obstacle points.

与原包 `d435i_ransac` 的差异：

| 文件 | 差异 |
| --- | --- |
| `src/ground_segment_node.cpp` | 新增 IMU 订阅、重力低通滤波、`prior_down()`；auto/manual 使用重力先验 |
| `src/d435i_cloud_node.cpp` | 同一进程内同时发布深度点云与 IMU（运动传感器直接回调取流） |
| `config/ransac.yaml` | 新增 `use_imu_prior`、`imu_topic`、`gravity_tau_sec` 等参数 |
| `launch/d435i_ransac_imu.launch.py` | 新增 `enable_imu`、`use_imu_prior` 启动参数 |

RANSAC 拟合、点云解析、发布逻辑与原包完全一致。

## Build

```bash
source /opt/ros/humble/setup.bash
cd ~/ros2_ws
/usr/bin/colcon --log-base log_y build --base-paths src --build-base build_y --install-base install_y --packages-select d435i_ransac_imu
source install_y/setup.bash
```

The machine needs librealsense2 development files (`librealsense2-dev`), its udev rules, and a USB 3 connection. The default depth profile is 640x480 at 30 Hz. The librealsense SDK computes organized points from Z16 depth; point coordinates are in metres in `camera_depth_optical_frame` (x right, y down, z forward). Points beyond 3 m or with invalid depth are NaN and excluded by the segmenter.

## Run

The launch starts the camera and segmentation. RViz is off by default:

```bash
ros2 launch d435i_ransac_imu d435i_ransac_imu.launch.py
ros2 launch d435i_ransac_imu d435i_ransac_imu.launch.py rviz:=true
ros2 launch d435i_ransac_imu d435i_ransac_imu.launch.py serial_no:=243522071034 mode:=manual
```

The camera publisher can also be run alone as `ros2 run d435i_ransac_imu d435i_cloud_node`; segmentation can consume an external compatible cloud with `ros2 run d435i_ransac_imu ground_segment_node --ros-args -p cloud_topic:=/d435i/points`.

Topics: `/d435i/points` (`sensor_msgs/PointCloud2`), `/ground/plane` (`shape_msgs/Plane`), `/ground/ground_cloud`, and `/ground/non_ground` (`sensor_msgs/PointCloud2`). Plane fitting refreshes every 30 frames; the per-frame split uses the latest accepted plane. Auto mode constrains the fitted normal to within 15 degrees of the assumed floor direction. Manual mode uses the configured height and pitch directly.

## Installation assumptions

The initial config assumes the camera optical origin is 0.90 m above level ground and pitched downward by 34 degrees. These values are starting assumptions inherited from the previous mounting setup, not measurements of the D435i installation. `camera_height` and `camera_pitch_deg` affect manual mode and constrain RANSAC direction in auto mode; recalibrate after changing mount position or tilt. RANSAC estimates only the visible ground plane, so it can fail when the floor is not visible or the floor is uneven.

Close realsense-viewer streaming before launch: only one process can own the depth stream. If no valid ground is found, non_ground contains unclassified valid points, not confirmed obstacles. Cached planes remain in use when a later fit fails, matching the original implementation; this is intended for a fixed camera mount, not pose tracking. ROS cloud timestamps are host publication time rather than hardware exposure timestamps.

## RViz ground coordinates

RViz now uses `d435i_ground`: X forward along the ground, Y left, Z up. The
origin is the camera's perpendicular projection onto the fitted floor. The XY
grid is at floor height; ground axes are red X, green Y, blue Z. A second set of
axes at the camera shows its optical convention (X right, Y down, Z forward).
The default Orbit view looks forward from behind and above the camera. Use the
Move Camera tool to rotate, pan and zoom.

The separate `ground_frame_node` publishes a 30 Hz TF from `d435i_ground` to
`camera_depth_optical_frame`. Before receiving a valid plane it uses the same
mount parameters as the segmenter, for example:

```bash
ros2 launch d435i_ransac_imu d435i_ransac_imu.launch.py rviz:=true camera_height:=0.9 camera_pitch_deg:=34.0
```

After a valid fit, it levels the display to the estimated floor, including roll.
The last valid transform remains cached if no new fit is available; malformed
planes are ignored. `/ground/plane` has no frame or timestamp fields and is
assumed to describe `camera_depth_optical_frame`. TF is published with current
ROS time so RViz can transform each incoming cloud. The ground frame is only a
local display aid, not odometry, localization or camera motion compensation.
Point coordinates, RANSAC fitting and obstacle classification remain unchanged.

## IMU gravity prior (`use_imu_prior`, default on)

The D435i has a built-in IMU. At rest the accelerometer measures specific force,
so its reading points **up**; negating it gives the ground normal directly.
`rs-enumerate-devices -c` reports an **identity rotation** between Depth and
Gyro/Accel (only a few mm of translation), so the value can be used in
`camera_depth_optical_frame` without any extrinsic conversion.

`ground_segment_node` subscribes to `imu_topic` (default `/d435i/accel/sample`),
low-pass filters the direction (`gravity_tau_sec`, default 0.3 s), rejects
samples outside `[imu_accel_min, imu_accel_max]`, and uses `-up` as the RANSAC
prior normal instead of the static `camera_pitch_deg` value. If no IMU sample
arrives within `imu_timeout_sec` (default 0.5 s) it silently falls back to the
config prior, so behaviour without an IMU is unchanged. `imu_pitch_offset_deg`
trims a residual extrinsic error. In `manual` mode `camera_height` still sets
`d`, only the normal comes from gravity.

`d435i_cloud_node` publishes the IMU itself (`enable_imu`, default true,
`imu_accel_fps`/`imu_gyro_fps` default 200/200). This is deliberate: librealsense
takes exclusive access to the USB device, so a separate IMU process fails with
`Device or resource busy`. One process must own the camera. Depth goes through
the point-cloud pipeline, the motion sensor is opened directly; both paths still
drop repeated hardware frame numbers before publishing.

Measured on this mount (D435I, fw 5.15.1.55): gravity gives pitch **19.7 deg**,
roll **-0.8 deg**, normal `(-0.014, 0.942, 0.336)`. The configured
`camera_pitch_deg: 34.0` is therefore off by **14.3 deg**, which is right at the
default `max_slope_deg: 15.0` boundary. A/B with `max_slope_deg:=5.0`:

| prior | result |
| --- | --- |
| IMU | plane `n=(-0.000 0.938 0.346) d=0.882 m`, ~27 000 ground points |
| config 34 deg | plane `n=(-0.019 0.873 0.487) d=0.996 m`, ~9 900 ground points, then fit fails |

With the wrong prior RANSAC latches onto a plane 5 deg off the real floor and
loses about two thirds of the ground points.

### One pipeline caveat: keep the IMU out of `rs2::config`

Enabling accel/gyro in the pipeline config looks natural but silently caps the
IMU at the depth rate: the pipeline's syncer only emits a frameset once every
enabled stream has a new frame, so a 200 Hz IMU came out at 30 Hz with most
samples dropped. `d435i_cloud_node` therefore streams depth through the pipeline
and opens the motion sensor directly (`sensor.open(...)` + `sensor.start(cb)`),
which keeps the IMU at its full rate. Measured on the host with `ros2 topic hz`:

```
/d435i/points        29.968      (full 30 Hz profile)
/d435i/accel/sample  202.359
/d435i/gyro/sample   200.081
```

When the same launch runs inside a sandboxed container the depth rate drops to
~21 Hz (node CPU stays at 16 %, so it is the USB/DDS path of the container, not
the node). Judge the real rate on the host.

End-to-end launch (`ros2 launch d435i_ransac_imu d435i_ransac_imu.launch.py`):

```
[d435i_cloud]: D435i depth 640x480@30, scale 0.001000 m/unit, topic /d435i/points
[d435i_cloud]: IMU: accel 200 Hz -> /d435i/accel/sample, gyro 200 Hz -> /d435i/gyro/sample
[ground_segmentation]: IMU gravity prior: pitch=19.5 deg, roll=-1.0 deg, n=(-0.017 0.943 0.333)
[ground_segmentation]: plane fit ok, prior=imu n=(-0.017 0.942 0.336)
[ground_segmentation]: ground: n=(-0.003 0.940 0.340) d=0.877 m, 28635 ground pts
```

Note that every process must be able to see `/dev/bus/usb` and `/dev/video*`; in
a session where `/dev` is overlaid by a tmpfs, run the nodes under
`unshare -m bash -c 'umount -l /dev; ...'`. Only one process at a time can own
the camera, so stop any other `d435i_cloud_node` before launching this one.


