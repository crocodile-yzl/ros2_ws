# D435i 占据建图一体化启动：librealsense 原生采集 + 概率体素占据 + RViz
# 用法：
#   ros2 launch d435i_occupancy d435i_occupancy.launch.py
#   已在跑相机:     ros2 launch d435i_occupancy d435i_occupancy.launch.py start_camera:=false
#   深度拉满 90fps: ros2 launch d435i_occupancy d435i_occupancy.launch.py depth_fps:=90
#   无显示器:       ros2 launch d435i_occupancy d435i_occupancy.launch.py rviz:=false
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package = get_package_share_directory('d435i_occupancy')
    params = LaunchConfiguration('params_file')
    depth_fps = LaunchConfiguration('depth_fps')
    color_fps = LaunchConfiguration('color_fps')
    color = LaunchConfiguration('color')

    return LaunchDescription([
        # rs_camera_publisher 用系统 librealsense（/usr/lib），无需额外环境
        DeclareLaunchArgument(
            'start_camera', default_value='true',
            description='启动 librealsense 采集节点'),
        DeclareLaunchArgument(
            'rviz', default_value='true', description='启动 RViz2 显示体素地图'),
        DeclareLaunchArgument(
            'depth_fps', default_value='60',
            description='深度帧率：480x270 支持 60/90（90 需 USB3 且关 IMU）'),
        DeclareLaunchArgument(
            'color_fps', default_value='60',
            description='彩色帧率：<=960x540 档支持 60'),
        DeclareLaunchArgument(
            'color', default_value='true',
            description='是否开启彩色流（占据建图用不到，可关以省带宽）'),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(package, 'config', 'd435i_occupancy.yaml')),
        Node(
            package='d435i_occupancy', executable='rs_camera_publisher',
            name='rs_camera_publisher', condition=IfCondition(LaunchConfiguration('start_camera')),
            parameters=[{'depth_fps': depth_fps, 'color_fps': color_fps,
                         'enable_color': color}],
            output='screen'),
        Node(
            package='d435i_occupancy', executable='occupancy_node',
            parameters=[params], output='screen'),
        Node(
            package='rviz2', executable='rviz2',
            condition=IfCondition(LaunchConfiguration('rviz')),
            arguments=['-d', os.path.join(package, 'rviz', 'occupancy.rviz')]),
    ])
