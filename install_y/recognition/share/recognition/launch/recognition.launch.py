# recognition 一体化启动文件（相机 + 识别，单文件搞定）
# 用法：
#   真实 D435i（UVC 直采，默认）: ros2 launch recognition recognition.launch.py
#   指定其他相机索引:             ros2 launch recognition recognition.launch.py camera:=0
#   仿真自测（发本地图片）:       ros2 launch recognition recognition.launch.py sim:=true
import os

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.conditions import IfCondition, LaunchConfigurationNotEquals
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# vision_msgs 临时安装在 ~/.local/ros-deps（系统未装 ros-humble-vision-msgs）。
# launch 内自动补环境，用户终端无需手动 export。
# 正式 `sudo apt install ros-humble-vision-msgs` 后可删除这两行。
_LOCAL_DEPS = os.path.expanduser('~/.local/ros-deps')


def generate_launch_description():
    pkg_share = get_package_share_directory('recognition')
    default_cfg = os.path.join(pkg_share, 'config', 'recognition.yaml')
    default_img = os.path.expanduser(
        '/home/y/WorkBuddy/2026-11-07-11-19-51/d435i_rgb_awb.png')

    sim = LaunchConfiguration('sim')
    img = LaunchConfiguration('img')
    camera = LaunchConfiguration('camera')
    rviz = LaunchConfiguration('rviz')
    rqt = LaunchConfiguration('rqt')
    width = LaunchConfiguration('width')
    height = LaunchConfiguration('height')
    fps = LaunchConfiguration('fps')

    recognition_node = Node(
        package='recognition',
        executable='recognition_node',
        name='recognition_node',
        output='screen',
        parameters=[default_cfg],
    )

    # 仿真模式：循环发本地图片
    sim_source = Node(
        package='recognition',
        executable='sim_image_publisher',
        name='image_source',
        output='screen',
        condition=IfCondition(sim),
        parameters=[{
            'topic': '/video',
            'rate': 5.0,
            'image_file': img,
        }],
    )

    # 相机模式：直接打开 UVC 设备实时采集
    camera_source = Node(
        package='recognition',
        executable='sim_image_publisher',
        name='image_source',
        output='screen',
        condition=LaunchConfigurationNotEquals('camera', '-1'),
        parameters=[{
            'topic': '/video',
            'rate': 30.0,
            'width': width,
            'height': height,
            'fps': fps,
            'camera_index': camera,
        }],
    )

    # rqt_image_view 的脚本解释器是 #!/usr/bin/env python3，若终端激活了 conda
    # 会被 python3.14 劫持导致瞬间崩溃（rclpy pybind11 是 python3.10 编译的）。
    # 这里强制用系统 /usr/bin/python3 启动，对 conda 环境免疫。
    rqt_exe = os.path.join(
        get_package_prefix('rqt_image_view'), 'lib', 'rqt_image_view',
        'rqt_image_view')
    rqt_view = ExecuteProcess(
        cmd=['/usr/bin/python3', rqt_exe, '/recognition/annotated'],
        condition=IfCondition(rqt),
        additional_env={'QT_QPA_PLATFORMTHEME': ''},
        output='screen',
    )

    return LaunchDescription([
        SetEnvironmentVariable(
            'LD_LIBRARY_PATH',
            _LOCAL_DEPS + '/lib:' + os.environ.get('LD_LIBRARY_PATH', '')),
        SetEnvironmentVariable(
            'PYTHONPATH',
            _LOCAL_DEPS + '/local/lib/python3.10/dist-packages:' +
            os.environ.get('PYTHONPATH', '')),
        DeclareLaunchArgument(
            'sim', default_value='false',
            description='true=用本地图片模拟相机输入（自测用）'),
        DeclareLaunchArgument(
            'img', default_value=default_img,
            description='sim 模式下发送的图片路径'),
        DeclareLaunchArgument(
            'camera', default_value='6',
            description='UVC 相机索引，6=D435i RGB (/dev/video6)；设为 -1 关闭相机只用仿真'),
        DeclareLaunchArgument(
            'width', default_value='848',
            description='采集宽度。D435i RGB：<=960x540 支持 60fps，720p/1080p 上限 30fps'),
        DeclareLaunchArgument(
            'height', default_value='480',
            description='采集高度'),
        DeclareLaunchArgument(
            'fps', default_value='60',
            description='采集帧率（480p 档最高 60）'),
        DeclareLaunchArgument(
            'rviz', default_value='false',
            description='true=启动 RViz2 显示实时标注图'),
        DeclareLaunchArgument(
            'rqt', default_value='true',
            description='true=启动 rqt_image_view 查看实时标注图'),
        recognition_node,
        sim_source,
        camera_source,
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            condition=IfCondition(rviz),
            arguments=['-d', os.path.join(pkg_share, 'rviz', 'recognition.rviz')],
        ),
        rqt_view,
    ])
