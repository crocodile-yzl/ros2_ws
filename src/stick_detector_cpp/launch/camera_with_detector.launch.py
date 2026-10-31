"""Berxel camera + C++ detector (+ optional rviz) in one launch."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    cam_share = get_package_share_directory('berxel_camera_ros2')
    my_share = get_package_share_directory('stick_detector_cpp')
    params = os.path.join(my_share, 'config', 'stick_detector.yaml')
    rviz_cfg = os.path.join(my_share, 'rviz', 'stick_detector.rviz')

    dbg = ParameterValue(LaunchConfiguration('debug_log'), value_type=bool)
    prf = ParameterValue(LaunchConfiguration('profile'), value_type=bool)
    return LaunchDescription([
        DeclareLaunchArgument('debug_log', default_value='false',
                              description='打印 ROI 压缩率与候选团块指标'),
        DeclareLaunchArgument('profile', default_value='false',
                              description='打印各阶段耗时'),

        DeclareLaunchArgument('rviz', default_value='true'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(cam_share, 'launch', 'berxel_camera_iHawk100.py'))),
        Node(package='stick_detector_cpp', executable='stick_detector_node',
             name='stick_detector', parameters=[params, {'debug_log': dbg, 'profile': prf}],
             output='screen'),
        Node(package='rviz2', executable='rviz2', name='rviz2',
             arguments=['-d', rviz_cfg],
             condition=IfCondition(LaunchConfiguration('rviz')),
             output='screen'),
    ])
