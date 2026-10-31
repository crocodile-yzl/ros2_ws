"""Berxel camera + stick detector (+ optional rviz) in one launch.

  ros2 launch stick_detector camera_with_detector.launch.py            # 带 rviz
  ros2 launch stick_detector camera_with_detector.launch.py rviz:=false
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    cam_share = get_package_share_directory('berxel_camera_ros2')
    det_share = get_package_share_directory('stick_detector')
    det_params = os.path.join(det_share, 'config', 'stick_detector.yaml')
    rviz_cfg = os.path.join(det_share, 'rviz', 'stick_detector.rviz')

    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(cam_share, 'launch', 'berxel_camera_iHawk100.py')))

    detector = Node(package='stick_detector',
                    executable='stick_detector_node',
                    name='stick_detector',
                    parameters=[det_params],
                    output='screen')

    rviz = Node(package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['-d', rviz_cfg],
                condition=IfCondition(LaunchConfiguration('rviz')),
                output='screen')

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true',
                              description='是否同时启动 rviz2'),
        camera,
        detector,
        rviz,
    ])
