"""Simulation test: synthetic cloud publisher + stick detector."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(get_package_share_directory('stick_detector'),
                          'config', 'stick_detector.yaml')
    return LaunchDescription([
        Node(package='stick_detector',
             executable='cloud_sim_node',
             name='berxel_cloud_sim',
             output='screen'),
        Node(package='stick_detector',
             executable='stick_detector_node',
             name='stick_detector',
             parameters=[params],
             output='screen'),
    ])
