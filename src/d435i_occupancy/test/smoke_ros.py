#!/usr/bin/env python3
"""Synthetic DDS integration check. Does not open a camera."""
import os
import subprocess
import signal
import time
import struct
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from nav_msgs.msg import OccupancyGrid

def main():
    rclpy.init()
    node = Node('occupancy_smoke')
    image_pub = node.create_publisher(Image, '/test/depth', 10)
    info_pub = node.create_publisher(CameraInfo, '/test/camera_info', 10)
    received = []
    grids = []
    node.create_subscription(PointCloud2, '/occupancy/occupied', received.append, 10)
    node.create_subscription(OccupancyGrid, '/occupancy/projection', grids.append, 10)
    process = subprocess.Popen(['ros2', 'run', 'd435i_occupancy', 'occupancy_node', '--ros-args',
        '-p', 'depth_topic:=/test/depth', '-p', 'camera_info_topic:=/test/camera_info',
        '-p', 'pixel_stride:=1'], stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, start_new_session=True)
    def spin(seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.05)
    try:
        spin(2.)
        assert process.poll() is None, 'occupancy node did not start'
        info = CameraInfo()
        info.header.frame_id = 'test_optical'
        info.width = 4
        info.height = 0  # Actual Berxel driver bug; Image is authoritative.
        info.binning_x = 4
        info.k = [100., 0., 1.5, 0., 100., 1.5, 0., 0., 1.]
        image = Image()
        image.header.frame_id = 'test_optical'
        image.height = image.width = 4
        image.encoding = 'mono16'
        image.step = 8
        image.data = struct.pack('<16H', *([1000] * 16))
        for _ in range(5):
            info_pub.publish(info)
            spin(.1)
        # Mismatch must reject image without producing map output.
        image.header.frame_id = 'wrong_frame'
        image.header.stamp = node.get_clock().now().to_msg()
        image_pub.publish(image)
        spin(.4)
        assert not received, 'mismatched camera frames accepted'
        image.header.frame_id = 'test_optical'
        image.header.stamp = node.get_clock().now().to_msg()
        image_pub.publish(image)
        spin(.5)
        assert received and received[-1].width > 0, 'valid depth did not create occupied voxels'
        assert grids and 100 in grids[-1].data and -1 in grids[-1].data, 'projection loses occupied or unknown state'
        count = len(received)
        spin(.8)
        assert len(received) == count, 'stale buffered image integrated repeatedly'
        # Duplicate timestamp/frame must also not count as a new observation.
        image_pub.publish(image)
        spin(.4)
        assert len(received) == count, 'duplicate image integrated repeatedly'
        image.header.stamp = node.get_clock().now().to_msg()
        image.data = bytes(32)  # invalid zero depth must not clear evidence
        image_pub.publish(image)
        spin(.4)
        assert len(received) > count and received[-1].width == received[count - 1].width, 'invalid zero depth cleared occupancy'
        print('PASS ROS smoke: metadata guard, mono16 projection, unknown, stale/duplicate rejection, zero depth retained')
    finally:
        os.killpg(process.pid, signal.SIGTERM)
        try:
            output, _ = process.communicate(timeout=8)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            output, _ = process.communicate()
        print(output[-3000:])
        node.destroy_node()
        rclpy.shutdown()
if __name__ == '__main__':
    main()
