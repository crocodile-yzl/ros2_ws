"""启动 video_publisher：从 USB 相机（默认）或 RTSP/HTTP 源采集图像发到 /video。

基线参数来自 config/video_publisher.yaml，launch 参数可覆盖，例如：
    ros2 launch port_camera_publisher video_publisher.launch.py                  # /dev/video0
    ros2 launch port_camera_publisher video_publisher.launch.py source:=1        # /dev/video1
    ros2 launch port_camera_publisher video_publisher.launch.py fourcc:=YUYV
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    default_params = os.path.join(
        get_package_share_directory('port_camera_publisher'),
        'config', 'video_publisher.yaml')

    source = LaunchConfiguration('source')
    topic = LaunchConfiguration('topic')
    frame_id = LaunchConfiguration('frame_id')
    publish_rate = LaunchConfiguration('publish_rate')
    width = LaunchConfiguration('width')
    height = LaunchConfiguration('height')
    camera_fps = LaunchConfiguration('camera_fps')
    fourcc = LaunchConfiguration('fourcc')
    encoding = LaunchConfiguration('encoding')
    publish_compressed = LaunchConfiguration('publish_compressed')

    return LaunchDescription([
        DeclareLaunchArgument(
            'source', default_value='0',
            description='USB 相机索引(0=/dev/video0)，也支持 rtsp:// http:// 和文件路径'),
        DeclareLaunchArgument(
            'topic', default_value='/video', description='图像话题名'),
        DeclareLaunchArgument(
            'frame_id', default_value='camera_link',
            description='图像消息的 frame_id'),
        DeclareLaunchArgument(
            'publish_rate', default_value='30.0',
            description='发布节拍(Hz)，源帧率更高时只发最新帧'),
        DeclareLaunchArgument(
            'width', default_value='1280',
            description='期望宽度，0 = 保持相机默认'),
        DeclareLaunchArgument(
            'height', default_value='720',
            description='期望高度，0 = 保持相机默认'),
        DeclareLaunchArgument(
            'camera_fps', default_value='30.0',
            description='相机采集帧率，0 = 不改驱动设置'),
        DeclareLaunchArgument(
            'fourcc', default_value='MJPG',
            description='USB 像素格式：MJPG（默认，高分辨率必需）/ YUYV'),
        DeclareLaunchArgument(
            'encoding', default_value='bgr8',
            description='图像编码：bgr8 / rgb8 / mono8'),
        DeclareLaunchArgument(
            'publish_compressed', default_value='true',
            description='是否同时发布 <topic>/compressed (JPEG)'),
        Node(
            package='port_camera_publisher',
            executable='video_publisher',
            name='video_publisher',
            output='screen',
            emulate_tty=True,
            parameters=[
                default_params,
                {
                    'source': ParameterValue(source, value_type=str),
                    'topic': ParameterValue(topic, value_type=str),
                    'frame_id': ParameterValue(frame_id, value_type=str),
                    'publish_rate': ParameterValue(
                        publish_rate, value_type=float),
                    'width': ParameterValue(width, value_type=int),
                    'height': ParameterValue(height, value_type=int),
                    'camera_fps': ParameterValue(
                        camera_fps, value_type=float),
                    'fourcc': ParameterValue(fourcc, value_type=str),
                    'encoding': ParameterValue(encoding, value_type=str),
                    'publish_compressed': ParameterValue(
                        publish_compressed, value_type=bool),
                },
            ],
        ),
    ])
