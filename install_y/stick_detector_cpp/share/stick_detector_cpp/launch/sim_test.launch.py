"""Simulation test: C++ synthetic cloud + C++ detector."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    params = os.path.join(get_package_share_directory('stick_detector_cpp'),
                          'config', 'stick_detector.yaml')
    dbg = ParameterValue(LaunchConfiguration('debug_log'), value_type=bool)
    prf = ParameterValue(LaunchConfiguration('profile'), value_type=bool)
    return LaunchDescription([
        DeclareLaunchArgument('debug_log', default_value='false',
                              description='打印 ROI 压缩率与候选团块指标'),
        DeclareLaunchArgument('profile', default_value='false',
                              description='打印各阶段耗时'),

        Node(package='stick_detector_cpp', executable='cloud_sim_node',
             name='berxel_cloud_sim', output='screen'),
        Node(package='stick_detector_cpp', executable='stick_detector_node',
             name='stick_detector', parameters=[params, {'debug_log': dbg, 'profile': prf}],
             output='screen'),
    ])
