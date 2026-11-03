import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    params = os.path.join(get_package_share_directory('d435i_ransac_imu'), 'config', 'ransac.yaml')
    mount = {name: ParameterValue(LaunchConfiguration(name), value_type=float)
             for name in ('camera_height', 'camera_pitch_deg')}
    return LaunchDescription([
        DeclareLaunchArgument('serial_no', default_value='', description='D435i serial number'),
        DeclareLaunchArgument('max_range', default_value='3.0'),
        DeclareLaunchArgument('mode', default_value='auto', description='auto RANSAC or manual plane'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('camera_height', default_value='0.9'),
        DeclareLaunchArgument('camera_pitch_deg', default_value='34.0'),
        DeclareLaunchArgument('enable_imu', default_value='true',
                              description='Publish IMU from the same pipeline (one process owns the camera)'),
        DeclareLaunchArgument('use_imu_prior', default_value='true',
                              description='Use gravity direction as the ground normal prior'),
        Node(package='d435i_ransac_imu', executable='d435i_cloud_node', name='d435i_cloud',
             parameters=[{'serial_no': ParameterValue(LaunchConfiguration('serial_no'), value_type=str),
                          'max_range': ParameterValue(LaunchConfiguration('max_range'), value_type=float),
                          'enable_imu': ParameterValue(LaunchConfiguration('enable_imu'), value_type=bool)}], output='screen'),
        Node(package='d435i_ransac_imu', executable='ground_segment_node', name='ground_segmentation',
             parameters=[params, mount, {'ground_plane_mode': LaunchConfiguration('mode'), 'max_range': ParameterValue(LaunchConfiguration('max_range'), value_type=float),
                                         'use_imu_prior': ParameterValue(LaunchConfiguration('use_imu_prior'), value_type=bool)}], output='screen'),
        Node(package='d435i_ransac_imu', executable='ground_frame_node',
             name='ground_visualization_frame', parameters=[mount], output='screen'),
        Node(package='rviz2', executable='rviz2', name='rviz2',
             arguments=['-d', os.path.join(get_package_share_directory('d435i_ransac_imu'),
                                           'rviz', 'ground_segment.rviz')],
             condition=IfCondition(LaunchConfiguration('rviz')), output='screen'),
    ])
