from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    package = get_package_share_directory('berxel_occupancy')
    driver = get_package_share_directory('berxel_camera_ros2')
    return LaunchDescription([
        DeclareLaunchArgument('start_camera', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('params_file', default_value=os.path.join(package, 'config', 'occupancy.yaml')),
        Node(package='berxel_camera_ros2', executable='berxel_camera_ros2', name='occupancy_camera',
             condition=IfCondition(LaunchConfiguration('start_camera')),
             parameters=[os.path.join(driver, 'config', 'berxel_ihawk_100R.yaml'),
                         {'enable_pointcloud': False, 'enable_color': False, 'enable_device_timestamp': False}], output='screen'),
        Node(package='berxel_occupancy', executable='occupancy_node',
             parameters=[LaunchConfiguration('params_file')], output='screen'),
        Node(package='rviz2', executable='rviz2', condition=IfCondition(LaunchConfiguration('rviz')),
             arguments=['-d', os.path.join(package, 'rviz', 'occupancy.rviz')]),
    ])
