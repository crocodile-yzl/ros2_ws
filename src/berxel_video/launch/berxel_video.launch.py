"""启动博升 iHawk 深度相机，把彩色/深度映射到 /video 与 /video/depth。

用法：
    ros2 launch berxel_video berxel_video.launch.py
    ros2 launch berxel_video berxel_video.launch.py enable_align:=true
    ros2 launch berxel_video berxel_video.launch.py stream_flag:=3 stream_type:=1   # 只要彩色 1280x800
    ros2 launch berxel_video berxel_video.launch.py video_topic:=/camera/color

底层是厂商驱动 berxel_camera_ros2，原生话题是 color/color_raw、depth/depth_raw，
这里重映射成 /video、/video/depth。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    default_cfg = os.path.join(
        get_package_share_directory('berxel_video'), 'config', 'berxel_ihawk.yaml')

    stream_flag = LaunchConfiguration('stream_flag')
    stream_type = LaunchConfiguration('stream_type')
    depth_fps = LaunchConfiguration('depth_fps')
    enable_align = LaunchConfiguration('enable_align')
    enable_pointcloud = LaunchConfiguration('enable_pointcloud')
    enable_denoise = LaunchConfiguration('enable_denoise')
    video_topic = LaunchConfiguration('video_topic')
    depth_topic = LaunchConfiguration('depth_topic')
    serial_number = LaunchConfiguration('serial_number')

    return LaunchDescription([
        DeclareLaunchArgument(
            'stream_flag', default_value='2',
            description='1 单流 / 2 Mix VGA(640x400) / 3 Mix HD(1280x800) / 4 Mix QVGA(320x200)'),
        DeclareLaunchArgument(
            'stream_type', default_value='3',
            description='位掩码：1 彩色 / 2 深度 / 3 彩色+深度 / 4 泛光IR / 5 点光IR'),
        DeclareLaunchArgument('depth_fps', default_value='30'),
        DeclareLaunchArgument(
            'enable_align', default_value='false',
            description='深度对齐到彩色（两路分辨率一致时才有意义）'),
        DeclareLaunchArgument('enable_pointcloud', default_value='false'),
        DeclareLaunchArgument('enable_denoise', default_value='false'),
        DeclareLaunchArgument('video_topic', default_value='/video'),
        DeclareLaunchArgument('depth_topic', default_value='/video/depth'),
        DeclareLaunchArgument(
            'serial_number', default_value='',
            description='多台相机时按序列号选，空 = 第一台'),
        Node(
            package='berxel_camera_ros2',
            executable='berxel_camera_ros2',
            name='berxel_camera_ros2',
            output='screen',
            parameters=[
                default_cfg,
                {
                    'stream_flag': ParameterValue(stream_flag, value_type=int),
                    'stream_type': ParameterValue(stream_type, value_type=int),
                    'depth_fps': ParameterValue(depth_fps, value_type=int),
                    'enable_align': ParameterValue(enable_align, value_type=bool),
                    'enable_pointcloud': ParameterValue(
                        enable_pointcloud, value_type=bool),
                    'enable_denoise': ParameterValue(
                        enable_denoise, value_type=bool),
                    'serial_number': ParameterValue(
                        serial_number, value_type=str),
                },
            ],
            remappings=[
                ('/color/color_raw', video_topic),
                ('/depth/depth_raw', depth_topic),
            ],
        ),
    ])
