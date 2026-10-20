"""启动 tcp_jpeg_receiver：监听 TCP 端口，接收 JPEG 帧并发布到 /video。

基线参数来自 config/tcp_jpeg_receiver.yaml，launch 参数可覆盖，例如：
    ros2 launch port_camera_publisher tcp_jpeg_receiver.launch.py port:=9000
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
        'config', 'tcp_jpeg_receiver.yaml')

    bind_address = LaunchConfiguration('bind_address')
    port = LaunchConfiguration('port')
    topic = LaunchConfiguration('topic')
    frame_id = LaunchConfiguration('frame_id')
    jpeg_quality = LaunchConfiguration('jpeg_quality')

    return LaunchDescription([
        DeclareLaunchArgument(
            'bind_address', default_value='0.0.0.0',
            description='监听地址，0.0.0.0 表示所有网卡'),
        DeclareLaunchArgument(
            'port', default_value='8080', description='监听 TCP 端口'),
        DeclareLaunchArgument('topic', default_value='/video'),
        DeclareLaunchArgument('frame_id', default_value='camera_link'),
        DeclareLaunchArgument(
            'jpeg_quality', default_value='80',
            description='转发压缩图时的 JPEG 质量'),
        Node(
            package='port_camera_publisher',
            executable='tcp_jpeg_receiver',
            name='tcp_jpeg_receiver',
            output='screen',
            emulate_tty=True,
            parameters=[
                default_params,
                {
                    'bind_address': ParameterValue(
                        bind_address, value_type=str),
                    'port': ParameterValue(port, value_type=int),
                    'topic': ParameterValue(topic, value_type=str),
                    'frame_id': ParameterValue(frame_id, value_type=str),
                    'jpeg_quality': ParameterValue(
                        jpeg_quality, value_type=int),
                },
            ],
        ),
    ])
