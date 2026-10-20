#!/usr/bin/env python3
"""裸 TCP 端口接收 JPEG 图像，并发布到 ROS2 话题（默认 /video）。

适用场景：下位机 / 网络相机 / ESP32 等主动把 JPEG 字节流推到某个 TCP 端口，
而不是提供标准 RTSP 地址。本节点监听该端口，按 JPEG 的 SOI(FFD8)/EOI(FFD9)
标记切分帧，解码后发布。

支持一次连接持续推流，客户端断开后自动等待下一次连接；多客户端时只服务
当前连接。

发布话题：
    <topic>              sensor_msgs/Image
    <topic>/compressed   sensor_msgs/CompressedImage
"""

import socket
import threading
import time

import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, Image

from port_camera_publisher.utils import as_bool, dynamic_descriptor

SOI = b'\xff\xd8'
EOI = b'\xff\xd9'
RECV_SIZE = 65536


class TcpJpegReceiverNode(Node):
    """监听 TCP 端口，接收 JPEG 帧并转成 ROS 图像消息。"""

    def __init__(self) -> None:
        super().__init__('tcp_jpeg_receiver')

        dyn = dynamic_descriptor()

        def declare(name, default):
            self.declare_parameter(name, default, dyn)

        declare('bind_address', '0.0.0.0')
        declare('port', 8080)
        declare('topic', '/video')
        declare('frame_id', 'camera_link')
        declare('publish_rate', 30.0)
        declare('queue_size', 1)
        declare('reliable', False)
        declare('publish_compressed', True)
        declare('jpeg_quality', 80)
        declare('max_buffer_mb', 16)
        declare('log_every_n', 150)

        gp = self.get_parameter
        self._bind = str(gp('bind_address').value)
        self._port = int(gp('port').value)
        self._topic = str(gp('topic').value)
        self._frame_id = str(gp('frame_id').value)
        self._rate = max(float(gp('publish_rate').value), 0.1)
        self._compressed = as_bool(gp('publish_compressed').value)
        self._jpeg_quality = int(gp('jpeg_quality').value)
        self._max_buffer = max(int(gp('max_buffer_mb').value), 1) * 1024 * 1024
        self._log_every = max(int(gp('log_every_n').value), 1)

        reliability = (ReliabilityPolicy.RELIABLE
                       if as_bool(gp('reliable').value)
                       else ReliabilityPolicy.BEST_EFFORT)
        qos = QoSProfile(
            depth=max(int(gp('queue_size').value), 1),
            history=HistoryPolicy.KEEP_LAST,
            reliability=reliability,
        )

        self._pub = self.create_publisher(Image, self._topic, qos)
        self._pub_compressed = None
        if self._compressed:
            self._pub_compressed = self.create_publisher(
                CompressedImage, self._topic + '/compressed', qos)

        self._lock = threading.Lock()
        self._frame = None
        self._jpeg = None
        self._seq = 0
        self._published_seq = -1
        self._frame_count = 0
        self._fps_window_start = time.monotonic()
        self._stop_event = threading.Event()

        self._server = None
        self._thread = threading.Thread(
            target=self._server_loop, name='tcp_receiver', daemon=True)
        self._thread.start()

        self._timer = self.create_timer(1.0 / self._rate, self._publish_tick)
        self.get_logger().info(
            f'监听 tcp://{self._bind}:{self._port} -> topic={self._topic}')

    # ------------------------------------------------------------------
    @staticmethod
    def _split_jpegs(buf: bytes):
        """从缓冲区中切出完整 JPEG 帧，返回 (frames, remaining)。"""
        frames = []
        while True:
            start = buf.find(SOI)
            if start < 0:
                buf = b''
                break
            end = buf.find(EOI, start + 2)
            if end < 0:
                buf = buf[start:]      # 保留半帧，等后续数据
                break
            frames.append(buf[start:end + 2])
            buf = buf[end + 2:]
        return frames, buf

    def _handle_stream(self, conn: socket.socket, peer) -> None:
        self.get_logger().info(f'客户端已连接: {peer}')
        conn.settimeout(1.0)
        buf = b''
        while not self._stop_event.is_set():
            try:
                data = conn.recv(RECV_SIZE)
            except socket.timeout:
                continue
            except OSError as exc:
                self.get_logger().warn(f'接收异常: {exc}')
                break
            if not data:
                break

            buf += data
            if len(buf) > self._max_buffer:
                self.get_logger().warn(
                    f'缓冲区超过 {self._max_buffer // 1024 // 1024}MB 仍无完整帧，'
                    f'丢弃旧数据')
                buf = buf[-self._max_buffer // 4:]

            frames, buf = self._split_jpegs(buf)
            for jpg in frames:
                img = cv2.imdecode(
                    np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)
                if img is None:
                    continue
                with self._lock:
                    self._frame = img
                    self._jpeg = jpg
                    self._seq += 1
        self.get_logger().info(f'客户端断开: {peer}')

    def _server_loop(self) -> None:
        while not self._stop_event.is_set():
            try:
                self._server = socket.socket(
                    socket.AF_INET, socket.SOCK_STREAM)
                self._server.setsockopt(
                    socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                self._server.bind((self._bind, self._port))
                self._server.listen(1)
                self._server.settimeout(1.0)
                break
            except OSError as exc:
                self.get_logger().error(
                    f'绑定端口 {self._port} 失败: {exc}，2s 后重试')
                self._stop_event.wait(2.0)

        while not self._stop_event.is_set():
            try:
                conn, peer = self._server.accept()
            except socket.timeout:
                continue
            except OSError:
                break
            with conn:
                self._handle_stream(conn, peer)

        if self._server is not None:
            self._server.close()

    # ------------------------------------------------------------------
    def _publish_tick(self) -> None:
        with self._lock:
            if self._frame is None or self._seq == self._published_seq:
                return
            frame = self._frame
            jpeg = self._jpeg
            seq = self._seq
        self._published_seq = seq

        stamp = self.get_clock().now().to_msg()
        msg = Image()
        msg.header.stamp = stamp
        msg.header.frame_id = self._frame_id
        msg.height, msg.width = frame.shape[:2]
        msg.encoding = 'bgr8'
        msg.is_bigendian = 0
        msg.step = msg.width * 3
        msg.data = frame.tobytes()
        self._pub.publish(msg)

        if self._pub_compressed is not None:
            cmsg = CompressedImage()
            cmsg.header = msg.header
            cmsg.format = 'jpeg'
            if jpeg is not None and self._jpeg_quality >= 95:
                cmsg.data = jpeg            # 直接转发原始压缩数据
            else:
                ok, buf = cv2.imencode(
                    '.jpg', frame,
                    [cv2.IMWRITE_JPEG_QUALITY, self._jpeg_quality])
                if ok:
                    cmsg.data = buf.tobytes()
            if cmsg.data:
                self._pub_compressed.publish(cmsg)

        self._frame_count += 1
        if self._frame_count % self._log_every == 0:
            now = time.monotonic()
            fps = self._log_every / max(now - self._fps_window_start, 1e-6)
            self._fps_window_start = now
            self.get_logger().info(
                f'已发布 {self._frame_count} 帧, 实际 {fps:.1f} fps, '
                f'分辨率 {msg.width}x{msg.height}')

    def shutdown(self) -> None:
        self._stop_event.set()
        if self._server is not None:
            try:
                self._server.close()
            except OSError:
                pass
        if self._thread.is_alive():
            self._thread.join(timeout=2.0)
        self.get_logger().info('节点已停止')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TcpJpegReceiverNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
