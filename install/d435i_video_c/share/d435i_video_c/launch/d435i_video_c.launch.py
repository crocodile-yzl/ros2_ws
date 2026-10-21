"""启动 d435i_video_c（纯 C 实现的 RealSense D435i 图像发布节点）。

用法：
    ros2 launch d435i_video_c d435i_video_c.launch.py
    ros2 launch d435i_video_c d435i_video_c.launch.py enable_depth:=true align_depth:=true
    ros2 launch d435i_video_c d435i_video_c.launch.py width:=640 height:=480 fps:=30
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    width = LaunchConfiguration('width')
    height = LaunchConfiguration('height')
    fps = LaunchConfiguration('fps')
    enable_depth = LaunchConfiguration('enable_depth')
    align_depth = LaunchConfiguration('align_depth')
    depth_width = LaunchConfiguration('depth_width')
    depth_height = LaunchConfiguration('depth_height')
    depth_fps = LaunchConfiguration('depth_fps')
    video_topic = LaunchConfiguration('video_topic')
    depth_topic = LaunchConfiguration('depth_topic')
    frame_id = LaunchConfiguration('frame_id')
    serial = LaunchConfiguration('serial_no')
    reliable = LaunchConfiguration('reliable')

    return LaunchDescription([
        DeclareLaunchArgument('width', default_value='1280'),
        DeclareLaunchArgument('height', default_value='720'),
        DeclareLaunchArgument('fps', default_value='30'),
        DeclareLaunchArgument('enable_depth', default_value='false'),
        DeclareLaunchArgument('align_depth', default_value='false'),
        DeclareLaunchArgument('depth_width', default_value='640'),
        DeclareLaunchArgument('depth_height', default_value='480'),
        DeclareLaunchArgument('depth_fps', default_value='30'),
        DeclareLaunchArgument('video_topic', default_value='/video'),
        DeclareLaunchArgument('depth_topic', default_value='/video/depth'),
        DeclareLaunchArgument('frame_id', default_value='camera_link'),
        DeclareLaunchArgument('serial_no', default_value=''),
        DeclareLaunchArgument('reliable', default_value='false'),
        Node(
            package='d435i_video_c',
            executable='d435i_video_c',
            name='d435i_video_c',
            output='screen',
            arguments=[
                '--width', width,
                '--height', height,
                '--fps', fps,
                '--depth-width', depth_width,
                '--depth-height', depth_height,
                '--depth-fps', depth_fps,
                '--topic', video_topic,
                '--depth-topic', depth_topic,
                '--frame-id', frame_id,
                ['--serial=', serial],
                ['--depth=', enable_depth],
                ['--align=', align_depth],
                ['--reliable=', reliable],
            ],
        ),
    ])
