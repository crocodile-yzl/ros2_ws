"""启动 Intel RealSense D435i（官方 realsense2_camera），把彩色流映射到 /video。

依赖：ros-jazzy-realsense2-camera（RealSense 官方 ROS 2 封装，内含 librealsense2）

用法：
    ros2 launch port_camera_publisher d435i_video.launch.py
    ros2 launch port_camera_publisher d435i_video.launch.py color_profile:=1920x1080x30
    ros2 launch port_camera_publisher d435i_video.launch.py enable_depth:=true align_depth:=true
    ros2 launch port_camera_publisher d435i_video.launch.py serial_no:=123622270123

彩色图（BGR8/ RGB8）发到 /video；开深度后，对齐到彩色的深度（16UC1，毫米）
发到 /video/depth。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    color_profile = LaunchConfiguration('color_profile')
    depth_profile = LaunchConfiguration('depth_profile')
    enable_depth = LaunchConfiguration('enable_depth')
    align_depth = LaunchConfiguration('align_depth')
    serial_no = LaunchConfiguration('serial_no')
    pointcloud = LaunchConfiguration('pointcloud')
    video_topic = LaunchConfiguration('video_topic')
    depth_topic = LaunchConfiguration('depth_topic')
    base_frame = LaunchConfiguration('base_frame')

    return LaunchDescription([
        DeclareLaunchArgument(
            'color_profile', default_value='1280x720x30',
            description='彩色流 宽x高x帧率，必须是 D435i 支持的组合'),
        DeclareLaunchArgument(
            'depth_profile', default_value='640x480x30',
            description='深度流 宽x高x帧率'),
        DeclareLaunchArgument(
            'enable_depth', default_value='false',
            description='是否同时开深度流'),
        DeclareLaunchArgument(
            'align_depth', default_value='false',
            description='把深度对齐到彩色（enable_depth 为 true 时有效）'),
        DeclareLaunchArgument(
            'serial_no', default_value='',
            description='多台相机时按序列号选择，空 = 自动选第一台'),
        DeclareLaunchArgument(
            'pointcloud', default_value='false',
            description='是否发布点云（需要 enable_depth）'),
        DeclareLaunchArgument(
            'video_topic', default_value='/video',
            description='彩色图的输出话题'),
        DeclareLaunchArgument(
            'depth_topic', default_value='/video/depth',
            description='深度的输出话题'),
        DeclareLaunchArgument(
            'base_frame', default_value='camera_link',
            description='TF 基坐标系名'),
        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            name='camera',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'enable_color': True,
                'rgb_camera.color_profile': ParameterValue(
                    color_profile, value_type=str),
                'enable_depth': ParameterValue(enable_depth, value_type=bool),
                'depth_module.depth_profile': ParameterValue(
                    depth_profile, value_type=str),
                'align_depth.enable': ParameterValue(
                    align_depth, value_type=bool),
                'pointcloud.enable': ParameterValue(
                    pointcloud, value_type=bool),
                'enable_sync': True,
                'serial_no': ParameterValue(serial_no, value_type=str),
                # D435i 的 IMU 默认关掉，需要时自己加 enable_gyro/enable_accel
                'enable_gyro': False,
                'enable_accel': False,
                'enable_infra1': False,
                'enable_infra2': False,
                'base_frame_id': ParameterValue(base_frame, value_type=str),
            }],
            remappings=[
                ('/camera/color/image_raw', video_topic),
                ('/camera/aligned_depth_to_color/image_raw', depth_topic),
            ],
        ),
    ])
