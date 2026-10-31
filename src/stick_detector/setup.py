from setuptools import find_packages, setup

package_name = 'stick_detector'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/stick_detect.launch.py',
            'launch/sim_test.launch.py',
            'launch/camera_with_detector.launch.py',
        ]),
        ('share/' + package_name + '/config', ['config/stick_detector.yaml']),
        ('share/' + package_name + '/rviz', ['rviz/stick_detector.rviz']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dev',
    maintainer_email='dev@example.com',
    description='Segment long stick-like objects from organized point clouds',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'stick_detector_node = stick_detector.stick_detector_node:main',
            'cloud_sim_node = stick_detector.cloud_sim_node:main',
        ],
    },
)
