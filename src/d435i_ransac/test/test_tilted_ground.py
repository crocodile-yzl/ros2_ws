import os
import struct
import subprocess
import sys
import time
import unittest

import rclpy
from rclpy.qos import QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField
from shape_msgs.msg import Plane

EXECUTABLE = sys.argv.pop(1)


class GroundSegmentRegression(unittest.TestCase):
    def test_tilted_floor_and_obstacle(self):
        namespace = '/d435i_ransac_test_' + str(os.getpid())
        rclpy.init()
        node = rclpy.create_node('subscriber', namespace=namespace)
        received = {}
        subscriptions = []
        for name, kind in [('ground_cloud', PointCloud2), ('non_ground', PointCloud2),
                           ('plane', Plane)]:
            subscriptions.append(node.create_subscription(
                kind, namespace + '/ground/' + name,
                lambda msg, key=name: received.update({key: msg}), QoSProfile(depth=5)))
        publisher = node.create_publisher(PointCloud2, namespace + '/input', qos_profile_sensor_data)
        process = subprocess.Popen([
            EXECUTABLE, '--ros-args', '-r', '__ns:=' + namespace,
            '-p', 'cloud_topic:=' + namespace + '/input', '-p', 'camera_pitch_deg:=34.0',
            '-p', 'downsample_stride:=1', '-p', 'max_range:=3.0'],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        try:
            # Optical coordinates: y points down; the floor is 0.9 m below camera.
            points = []
            for row in range(30):
                for col in range(40):
                    z = 0.7 + row * 0.035
                    y = (0.9 - z * 0.559) / 0.829
                    x = (col - 20) * z * 0.004
                    points.append((x, y, z))
            floor_count = len(points)
            points.extend((x, y - 0.12, z) for x, y, z in points[::5])
            raw = b''.join(struct.pack('<fff', *p) for p in points)
            cloud = PointCloud2()
            cloud.header.frame_id = 'camera_depth_optical_frame'
            cloud.height = 1
            cloud.width = len(points)
            cloud.point_step = 12
            cloud.row_step = cloud.width * cloud.point_step
            cloud.fields = [PointField(name=n, offset=i * 4, datatype=7, count=1)
                            for i, n in enumerate(('x', 'y', 'z'))]
            cloud.data = raw
            deadline = time.monotonic() + 8
            while time.monotonic() < deadline and len(received) < 3:
                self.assertIsNone(process.poll(), 'node exited before publishing')
                cloud.header.stamp = node.get_clock().now().to_msg()
                publisher.publish(cloud)
                rclpy.spin_once(node, timeout_sec=0.05)
            self.assertEqual(set(received), {'ground_cloud', 'non_ground', 'plane'})
            self.assertGreater(received['plane'].coef[2], 0.5)
            ground = received['ground_cloud']
            nonground = received['non_ground']
            self.assertEqual(ground.header.frame_id, cloud.header.frame_id)
            self.assertGreaterEqual(ground.width, floor_count - 20)
            self.assertGreater(nonground.width, 100)
        finally:
            process.terminate()
            process.wait(timeout=5)
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    unittest.main()
