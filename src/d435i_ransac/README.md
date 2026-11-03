# d435i_ransac

Intel RealSense D435i depth input with the `stick_detector_cpp` standalone ground RANSAC segmentation ported unchanged in its fitting and classification algorithm. The optional stick detector is not included: this package focuses on ground / non-ground obstacle points.

## Build

```bash
source /opt/ros/humble/setup.bash
cd ~/ros2_ws
/usr/bin/colcon --log-base log_y build --base-paths src --build-base build_y --install-base install_y --packages-select d435i_ransac
source install_y/setup.bash
```

The machine needs librealsense2 development files (`librealsense2-dev`), its udev rules, and a USB 3 connection. The default depth profile is 640x480 at 30 Hz. The librealsense SDK computes organized points from Z16 depth; point coordinates are in metres in `camera_depth_optical_frame` (x right, y down, z forward). Points beyond 3 m or with invalid depth are NaN and excluded by the segmenter.

## Run

The launch starts the camera and segmentation. RViz is off by default:

```bash
ros2 launch d435i_ransac d435i_ransac.launch.py
ros2 launch d435i_ransac d435i_ransac.launch.py rviz:=true
ros2 launch d435i_ransac d435i_ransac.launch.py serial_no:=243522071034 mode:=manual
```

The camera publisher can also be run alone as `ros2 run d435i_ransac d435i_cloud_node`; segmentation can consume an external compatible cloud with `ros2 run d435i_ransac ground_segment_node --ros-args -p cloud_topic:=/d435i/points`.

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
ros2 launch d435i_ransac d435i_ransac.launch.py rviz:=true camera_height:=0.9 camera_pitch_deg:=34.0
```

After a valid fit, it levels the display to the estimated floor, including roll.
The last valid transform remains cached if no new fit is available; malformed
planes are ignored. `/ground/plane` has no frame or timestamp fields and is
assumed to describe `camera_depth_optical_frame`. TF is published with current
ROS time so RViz can transform each incoming cloud. The ground frame is only a
local display aid, not odometry, localization or camera motion compensation.
Point coordinates, RANSAC fitting and obstacle classification remain unchanged.
