"""Exercise the real node with reliable subscribers and synthetic camera clouds."""
import os
import random
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


class GroundSegmentTest(unittest.TestCase):
    def check_cloud(self, wall=False, organized=False, manual=False, ceiling=False,
                    wall_only=False, noisy_wall=False):
        namespace = '/ground_test_' + str(os.getpid())
        rclpy.init()
        node = rclpy.create_node('subscriber', namespace=namespace)
        received = {}
        subscriptions = []
        for topic, message_type in [('ground_cloud', PointCloud2),
                                    ('non_ground', PointCloud2), ('plane', Plane)]:
            subscriptions.append(node.create_subscription(
                message_type, namespace + '/ground/' + topic,
                lambda msg, key=topic: received.update({key: msg}), QoSProfile(depth=5)))
        publisher = node.create_publisher(
            PointCloud2, namespace + '/input', qos_profile_sensor_data)
        process = subprocess.Popen([
            EXECUTABLE, '--ros-args', '-r', '__ns:=' + namespace,
            '-p', 'cloud_topic:=' + namespace + '/input',
            '-p', 'camera_pitch_deg:=0.0', '-p', 'downsample_stride:=1',
            '-p', 'max_range:=5.0',
            '-p', 'ground_plane_mode:=' + ('manual' if manual else 'auto')],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        try:
            # A slightly tilted floor with negative normal.z is valid for a
            # horizontal camera. The wall has four times as many points.
            floor = [(x * .05 - .5, .8 + (.005 * (1 + z * .06)), 1 + z * .06)
                     for x in range(20) for z in range(20)]
            obstacles = [(x, y - .2, z) for x, y, z in floor[::4]]
            points = floor + obstacles
            if wall:
                points += [(x * .04 - .8, y * .04 - .8, 3.)
                           for x in range(40) for y in range(40)]
            if ceiling:
                points += [(x * .04 - .8, -.4 + .005 * (1 + z * .06), 1 + z * .06)
                           for x in range(40) for z in range(40)]
            if wall_only:
                noise = random.Random(1234)
                points = [(x * .04 - .8, y * .04 - .8, 3.)
                          for x in range(40) for y in range(40)]
                if noisy_wall:
                    points = [(x * .02 - 1., y * .02 - 1.,
                               3. + noise.uniform(-.15, .15))
                              for x in range(100) for y in range(100)]
            cloud = PointCloud2()
            cloud.header.frame_id = 'test_camera_optical'
            cloud.height = 20 if organized else 1
            cloud.width = len(points) // cloud.height
            cloud.point_step = 12
            cloud.row_step = cloud.width * cloud.point_step + (16 if organized else 0)
            cloud.fields = [PointField(name=name, offset=i * 4, datatype=7, count=1)
                            for i, name in enumerate(('x', 'y', 'z'))]
            raw = b''.join(struct.pack('<fff', *p) for p in points)
            row_bytes = cloud.width * cloud.point_step
            cloud.data = b''.join(raw[i * row_bytes:(i + 1) * row_bytes] +
                                  bytes(cloud.row_step - row_bytes)
                                  for i in range(cloud.height))
            deadline = time.monotonic() + 8
            expected = {'ground_cloud', 'non_ground'}
            if not wall_only:
                expected.add('plane')
            while time.monotonic() < deadline and not expected <= set(received):
                self.assertIsNone(process.poll(), 'node exited before publishing')
                cloud.header.stamp = node.get_clock().now().to_msg()
                publisher.publish(cloud)
                rclpy.spin_once(node, timeout_sec=.05)
            self.assertEqual(set(received), expected, 'Missing point cloud output')
            if wall_only:
                self.assertEqual(received['ground_cloud'].width, 0)
                self.assertEqual(received['non_ground'].width, len(points))
                self.assertEqual(bytes(received['non_ground'].data), raw)
                return
            self.assertGreater(received['plane'].coef[1], .95)
            ground = received['ground_cloud']
            self.assertEqual(ground.header.frame_id, cloud.header.frame_id)
            self.assertGreaterEqual(ground.width, 390)
            xyz = list(struct.iter_unpack('<fff', bytes(ground.data)))
            self.assertGreater(sum(abs(y - .005 * z - .8) < .02 for x, y, z in xyz)
                               / len(xyz), .95, 'non-ground surface was labeled as ground')
            self.assertGreaterEqual(received['non_ground'].width, 100)
        finally:
            process.terminate()
            process.wait(timeout=5)
            node.destroy_node()
            rclpy.shutdown()

    def test_ground_beside_dominant_wall(self):
        self.check_cloud(wall=True)

    def test_horizontal_camera_normal_direction(self):
        self.check_cloud()

    def test_ground_is_below_camera(self):
        self.check_cloud(ceiling=True)

    def test_padded_organized_cloud_manual_mode(self):
        self.check_cloud(organized=True, manual=True)

    def test_visible_cloud_without_floor(self):
        self.check_cloud(wall_only=True)

    def test_noisy_wall_is_not_a_floor(self):
        self.check_cloud(wall_only=True, noisy_wall=True)


if __name__ == '__main__':
    unittest.main()
