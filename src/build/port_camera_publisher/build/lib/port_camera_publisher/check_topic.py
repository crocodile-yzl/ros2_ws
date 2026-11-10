#!/usr/bin/env python3
"""订阅图像话题做体检：看帧率、分辨率、编码，可选存一张图。

用法：
    ros2 run port_camera_publisher check_video --ros-args -p topic:=/video
    ros2 run port_camera_publisher check_video --ros-args -p save:=/tmp/frame.jpg
"""

import time

import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, Image

from port_camera_publisher.utils import as_bool, dynamic_descriptor


class TopicChecker(Node):

    def __init__(self) -> None:
        super().__init__('check_video')

        dyn = dynamic_descriptor()

        def declare(name, default):
            self.declare_parameter(name, default, dyn)

        declare('topic', '/video')
        declare('save', '')
        declare('reliable', False)
        declare('duration', 0.0)      # 0 = 一直运行

        topic = str(self.get_parameter('topic').value)
        self._save = str(self.get_parameter('save').value).strip()
        duration = float(self.get_parameter('duration').value)

        qos = QoSProfile(
            depth=10,
            history=HistoryPolicy.KEEP_LAST,
            reliability=(ReliabilityPolicy.RELIABLE
                         if as_bool(self.get_parameter('reliable').value)
                         else ReliabilityPolicy.BEST_EFFORT),
        )

        self._count = 0
        self._window_count = 0
        self._window_start = time.monotonic()
        self._last = None

        self.create_subscription(Image, topic, self._on_image, qos)
        self.create_subscription(
            CompressedImage, topic + '/compressed', self._on_compressed, qos)

        self.get_logger().info(f'订阅 {topic}，等待数据…')
        if duration > 0.0:
            self.create_timer(duration, self._on_finish)

    def _on_image(self, msg: Image) -> None:
        self._count += 1
        self._window_count += 1
        self._last = msg
        self._report_if_needed()

    def _on_compressed(self, msg: CompressedImage) -> None:
        if self._save:
            with open(self._save, 'wb') as fp:
                fp.write(msg.data)

    def _report_if_needed(self) -> None:
        now = time.monotonic()
        dt = now - self._window_start
        if dt < 1.0:
            return
        msg = self._last
        self.get_logger().info(
            f'{self._window_count / dt:5.1f} fps | '
            f'{msg.width}x{msg.height} {msg.encoding} | '
            f'frame_id={msg.header.frame_id!r} | 累计 {self._count} 帧')
        self._window_start = now
        self._window_count = 0

    def _on_finish(self) -> None:
        msg = self._last
        if msg is not None and self._save:
            frame = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                msg.height, msg.width, -1)
            if msg.encoding == 'rgb8':
                frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            elif frame.shape[2] == 1:
                frame = frame[:, :, 0]
            cv2.imwrite(self._save, frame)
            self.get_logger().info(f'已保存一帧到 {self._save}')
        elif msg is None:
            self.get_logger().warn('超时未收到任何图像帧')
        rclpy.shutdown()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TopicChecker()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
