import os

from glob import glob

from setuptools import setup

package_name = 'port_camera_publisher'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dev',
    maintainer_email='dev@example.com',
    description='从视频传输端口采集图像并发布到 /video 话题',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'video_publisher = port_camera_publisher.video_publisher_node:main',
            'tcp_jpeg_receiver = port_camera_publisher.tcp_jpeg_receiver_node:main',
            'check_video = port_camera_publisher.check_topic:main',
        ],
    },
)
