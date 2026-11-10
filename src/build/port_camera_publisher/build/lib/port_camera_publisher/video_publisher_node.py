#!/usr/bin/env python3
"""从视频传输端口采集图像，发布到 ROS2 话题（默认 /video）。

支持 cv2.VideoCapture 能打开的任意源：
    rtsp://user:pass@192.168.1.10:554/stream1
    http://192.168.1.10:8080/video        (MJPEG over HTTP)
    0                                      (本地 USB 摄像头)
    /path/to/video.mp4

读帧放在独立线程，发布在定时器中完成，因此断流/卡顿不会阻塞 ROS 执行器；
断流后自动重连。

发布话题：
    <topic>              sensor_msgs/Image             （默认 bgr8）
    <topic>/compressed   sensor_msgs/CompressedImage   （开门可关）
"""

import os
import threading
import time

import cv2
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, Image

from port_camera_publisher.utils import as_bool, dynamic_descriptor


class VideoPublisherNode(Node):
    """采集端口图像 -> ROS 图像话题。"""

    def __init__(self) -> None:
        super().__init__('video_publisher')

        # ------------------------- 参数声明 -------------------------
        # 命令行 `-p source:=0` 里的 0 会被 ROS 解析成 INTEGER，固定类型声明会抛
        # InvalidParameterTypeException。统一用 dynamic_typing，下面读取时再转换。
        dyn = dynamic_descriptor()

        def declare(name, default):
            self.declare_parameter(name, default, dyn)

        declare('source', '0')                            # "0" = /dev/video0
        declare('topic', '/video')
        declare('frame_id', 'camera_link')
        declare('publish_rate', 30.0)                     # 发布节拍 Hz
        declare('width', 0)                               # 0 = 保持源分辨率
        declare('height', 0)
        declare('camera_fps', 0.0)                        # USB 相机采集帧率，0=不设
        declare('fourcc', '')                             # USB 像素格式，如 MJPG
        declare('encoding', 'bgr8')                       # bgr8 | rgb8 | mono8
        declare('reliable', False)                        # False -> BEST_EFFORT
        declare('queue_size', 1)
        declare('publish_compressed', True)
        declare('jpeg_quality', 80)
        declare('rtsp_transport', 'tcp')                  # tcp | udp
        declare('reconnect_delay', 2.0)                   # 秒
        declare('max_read_failures', 30)                  # 连续读失败多少次重连
        declare('log_every_n', 150)                       # 每 N 帧打一条状态日志

        gp = self.get_parameter
        self._source = str(gp('source').value).strip()
        self._topic = str(gp('topic').value)
        self._frame_id = str(gp('frame_id').value)
        self._rate = max(float(gp('publish_rate').value), 0.1)
        self._width = int(gp('width').value)
        self._height = int(gp('height').value)
        self._camera_fps = float(gp('camera_fps').value)
        self._fourcc = str(gp('fourcc').value).strip().upper()
        self._encoding = str(gp('encoding').value).lower()
        self._compressed = as_bool(gp('publish_compressed').value)
        self._jpeg_quality = int(gp('jpeg_quality').value)
        self._transport = str(gp('rtsp_transport').value).lower()
        self._reconnect_delay = float(gp('reconnect_delay').value)
        self._max_failures = max(int(gp('max_read_failures').value), 1)
        self._log_every = max(int(gp('log_every_n').value), 1)

        if self._encoding not in ('bgr8', 'rgb8', 'mono8'):
            self.get_logger().warn(
                f'不支持的 encoding={self._encoding}，回落为 bgr8')
            self._encoding = 'bgr8'

        if self._fourcc and len(self._fourcc) != 4:
            self.get_logger().warn(
                f'fourcc 必须是 4 个字符（如 MJPG），忽略 {self._fourcc!r}')
            self._fourcc = ''

        # ------------------------- QoS / 发布者 -------------------------
        reliability = (ReliabilityPolicy.RELIABLE
                       if as_bool(gp('reliable').value)
                       else ReliabilityPolicy.BEST_EFFORT)
        qos = QoSProfile(
            depth=max(int(gp('queue_size').value), 1),
            history=HistoryPolicy.KEEP_LAST,
            reliability=reliability,
        )

        self._bridge = CvBridge()
        self._pub = self.create_publisher(Image, self._topic, qos)
        self._pub_compressed = None
        if self._compressed:
            self._pub_compressed = self.create_publisher(
                CompressedImage, self._topic + '/compressed', qos)

        # ------------------------- 共享帧缓存 -------------------------
        self._lock = threading.Lock()
        self._frame = None
        self._seq = 0                # 读线程每采到一帧 +1
        self._published_seq = -1     # 已发布到第几帧，避免重复发同一帧
        self._stop_event = threading.Event()
        self._frame_count = 0
        self._fps_window_start = time.monotonic()

        # RTSP 强制走 TCP，避免 UDP 丢包导致的画面撕裂
        if self._source.lower().startswith('rtsp'):
            os.environ['OPENCV_FFMPEG_CAPTURE_OPTIONS'] = (
                f'rtsp_transport;{self._transport}')

        self._reader = threading.Thread(
            target=self._reader_loop, name='video_reader', daemon=True)
        self._reader.start()

        self._timer = self.create_timer(1.0 / self._rate, self._publish_tick)
        self.get_logger().info(
            f'节点已启动: source={self._source} -> topic={self._topic} '
            f'encoding={self._encoding} rate={self._rate:.1f}Hz '
            f'reliability={reliability.name}')

    # ------------------------------------------------------------------
    # 采集线程
    # ------------------------------------------------------------------
    def _open_capture(self) -> cv2.VideoCapture:
        """按源类型显式指定后端。

        CAP_ANY 在网络流/文件上有时会误选 GStreamer 后端并直接失败，
        因此网络地址与文件统一走 FFMPEG，摄像头索引走 V4L2，失败再回退。
        """
        src = int(self._source) if self._source.isdigit() else self._source
        if isinstance(src, int):
            # 摄像头只用 V4L2；回退 CAP_ANY 只会让 GStreamer 再刷一遍同样的错误
            backends = (cv2.CAP_V4L2,)
        else:
            backends = (cv2.CAP_FFMPEG, cv2.CAP_ANY)

        cap = cv2.VideoCapture(src, backends[0])
        for backend in backends[1:]:
            if cap.isOpened():
                break
            cap.release()
            cap = cv2.VideoCapture(src, backend)

        if not cap.isOpened():
            return cap

        # 只保留最新一帧，显著降低实时流延迟
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        if isinstance(src, int):
            # 720p 以上的 USB 相机若用未压缩 YUYV，带宽不够会掉到几帧；
            # 先协商 MJPG 再设分辨率（顺序反过来驱动常常不生效）。
            if self._fourcc:
                cap.set(cv2.CAP_PROP_FOURCC,
                        cv2.VideoWriter_fourcc(*self._fourcc))

        if self._width > 0 and self._height > 0:
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, self._width)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self._height)

        if isinstance(src, int) and self._camera_fps > 0:
            cap.set(cv2.CAP_PROP_FPS, self._camera_fps)

        return cap

    def _is_live_source(self) -> bool:
        """网络流或摄像头索引视为实时源；本地文件按源 FPS 回放。"""
        return (self._source.isdigit()
                or self._source.lower().startswith(
                    ('rtsp://', 'rtmp://', 'http://', 'https://', 'udp://')))

    def _reader_loop(self) -> None:
        live = self._is_live_source()
        while not self._stop_event.is_set():
            cap = self._open_capture()
            if not cap.isOpened():
                self.get_logger().error(
                    f'打开视频源失败: {self._source}，'
                    f'{self._reconnect_delay:.1f}s 后重试')
                cap.release()
                self._stop_event.wait(self._reconnect_delay)
                continue

            w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            src_fps = float(cap.get(cv2.CAP_PROP_FPS) or 0.0)
            # 本地文件读帧不受帧率约束，按源 FPS 节流后才能当"假相机"用
            file_fps = 0.0 if live else (src_fps if 0 < src_fps <= 240 else 0.0)
            fourcc = int(cap.get(cv2.CAP_PROP_FOURCC) or 0)
            fourcc_str = ''.join(
                chr((fourcc >> (8 * i)) & 0xFF) for i in range(4)).strip('\x00 ')
            detail = f'{w}x{h}'
            if src_fps > 0:
                detail += f' @{src_fps:.1f}fps'
            if fourcc_str and self._source.isdigit():
                detail += f' {fourcc_str}'
            if file_fps:
                detail += ' [按源速率回放]'
            self.get_logger().info(f'视频源已连接: {self._source} ({detail})')

            failures = 0
            next_frame_time = time.monotonic()
            while not self._stop_event.is_set():
                ok, frame = cap.read()
                if not ok or frame is None:
                    failures += 1
                    if failures >= self._max_failures:
                        log = (self.get_logger().warn if live
                               else self.get_logger().info)
                        log(f'连续 {failures} 次读取失败，准备重连')
                        break
                    time.sleep(0.01)
                    continue

                failures = 0
                with self._lock:
                    self._frame = frame
                    self._seq += 1

                if file_fps:
                    next_frame_time += 1.0 / file_fps
                    delay = next_frame_time - time.monotonic()
                    if delay > 0:
                        time.sleep(delay)
                    elif delay < -1.0:      # 落后太多就不追帧了
                        next_frame_time = time.monotonic()

            cap.release()
            if not self._stop_event.is_set():
                self._stop_event.wait(self._reconnect_delay)

    # ------------------------------------------------------------------
    # 发布
    # ------------------------------------------------------------------
    def _publish_tick(self) -> None:
        with self._lock:
            if self._frame is None or self._seq == self._published_seq:
                return
            frame = self._frame
            seq = self._seq
        self._published_seq = seq

        stamp = self.get_clock().now().to_msg()

        try:
            if self._encoding == 'rgb8':
                msg = self._bridge.cv2_to_imgmsg(
                    cv2.cvtColor(frame, cv2.COLOR_BGR2RGB), encoding='rgb8')
            elif self._encoding == 'mono8':
                gray = (frame if frame.ndim == 2
                        else cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY))
                msg = self._bridge.cv2_to_imgmsg(gray, encoding='mono8')
            else:
                msg = self._bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        except CvBridgeError as exc:
            self.get_logger().error(f'cv_bridge 转换失败: {exc}')
            return

        msg.header.stamp = stamp
        msg.header.frame_id = self._frame_id
        self._pub.publish(msg)

        if self._pub_compressed is not None:
            ok, buf = cv2.imencode(
                '.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, self._jpeg_quality])
            if ok:
                cmsg = CompressedImage()
                cmsg.header = msg.header
                cmsg.format = 'jpeg'
                cmsg.data = buf.tobytes()
                self._pub_compressed.publish(cmsg)

        self._frame_count += 1
        if self._frame_count % self._log_every == 0:
            now = time.monotonic()
            fps = self._log_every / max(now - self._fps_window_start, 1e-6)
            self._fps_window_start = now
            self.get_logger().info(
                f'已发布 {self._frame_count} 帧, 实际 {fps:.1f} fps, '
                f'分辨率 {msg.width}x{msg.height}')

    # ------------------------------------------------------------------
    def shutdown(self) -> None:
        self._stop_event.set()
        if self._reader.is_alive():
            self._reader.join(timeout=2.0)
        self.get_logger().info('节点已停止')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VideoPublisherNode()
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
