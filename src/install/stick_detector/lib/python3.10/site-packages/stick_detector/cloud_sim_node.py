#!/usr/bin/env python3
"""Synthetic organized point cloud publisher for stick_detector testing.

Publishes a 640x400 organized cloud on /berxel_cloudpoint (same layout as
the Berxel driver: x,y,z,rgb float32, point_step 16, invalid = NaN).

Scene (camera optical frame: +z forward, +x right, +y down):
  - solid backdrop plane at z = 1.0
  - box (0.14 m cube) and sphere (r = 0.06 m) as distractors
  - stick A: L = 0.90 m, r = 0.02 m, lying horizontally
  - stick B: L = 0.40 m, r = 0.025 m, lying, angle ~ -12 deg in-plane
Expected: detector finds exactly 2 sticks, rejects box / sphere / plane.
"""
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField


def perp_basis(a):
    a = a / (np.linalg.norm(a) + 1e-12)
    tmp = np.array([1.0, 0.0, 0.0]) if abs(a[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
    e1 = np.cross(a, tmp)
    e1 /= np.linalg.norm(e1) + 1e-12
    e2 = np.cross(a, e1)
    return e1, e2


def cylinder(center, axis, length, radius, step=0.004, n_theta=64):
    center = np.asarray(center, dtype=float)
    axis = np.asarray(axis, dtype=float)
    t = np.arange(-length / 2, length / 2 + step, step)
    th = np.linspace(0.0, 2.0 * np.pi, n_theta, endpoint=False)
    T, TH = np.meshgrid(t, th, indexing='ij')
    e1, e2 = perp_basis(axis)
    pts = (center[None, None, :]
           + T[..., None] * axis[None, None, :]
           + radius * (np.cos(TH)[..., None] * e1 + np.sin(TH)[..., None] * e2))
    return pts.reshape(-1, 3)


def box(center, size, step=0.006):
    c = np.asarray(center, float)
    s = size / 2.0
    faces = []
    for axis, sign in ((0, 1), (0, -1), (1, 1), (1, -1), (2, 1), (2, -1)):
        g = np.arange(-s, s + step, step)
        A, B = np.meshgrid(g, g, indexing='ij')
        pts = np.tile(c, (A.size, 1))
        pts = pts.reshape(-1, 3)
        i1, i2 = [i for i in range(3) if i != axis]
        flat = pts.copy()
        flat[:, axis] += sign * s
        flat[:, i1] = A.ravel()
        flat[:, i2] = B.ravel()
        faces.append(flat)
    return np.vstack(faces)


def tube_g(p0, p1, r, step=0.008, n_theta=24):
    """Straight tube between two ground-coords points p0/p1 (x, s, h)."""
    p0 = np.asarray(p0, float)
    p1 = np.asarray(p1, float)
    axis = p1 - p0
    ln = np.linalg.norm(axis)
    axis = axis / max(ln, 1e-9)
    t = np.arange(0.0, ln + step, step)
    th = np.linspace(0.0, 2.0 * np.pi, n_theta, endpoint=False)
    tmp = np.array([0.0, 0.0, 1.0]) if abs(axis[2]) < 0.9 else np.array([1.0, 0.0, 0.0])
    e1 = np.cross(axis, tmp)
    e1 /= np.linalg.norm(e1) + 1e-12
    e2 = np.cross(axis, e1)
    T, TH = np.meshgrid(t, th, indexing="ij")
    return (p0[None, None, :] + T[..., None] * axis[None, None, :]
            + r * (np.cos(TH)[..., None] * e1 + np.sin(TH)[..., None] * e2)
            ).reshape(-1, 3)


def disk_g(cx, cs, r, h=0.008, step=0.006):
    """Flat base plate (flange) in ground coords."""
    gx = np.arange(-r, r + step, step)
    gs = np.arange(-r, r + step, step)
    X, S = np.meshgrid(gx, gs, indexing="ij")
    keep = (X ** 2 + S ** 2) <= r ** 2
    return np.stack([cx + X[keep], cs + S[keep], np.full(keep.sum(), h)], axis=1)


def sphere(center, radius, n=40):
    u = np.linspace(0.0, np.pi, n)
    v = np.linspace(0.0, 2.0 * np.pi, 2 * n)
    U, V = np.meshgrid(u, v, indexing='ij')
    pts = np.stack([np.sin(U) * np.cos(V), np.sin(U) * np.sin(V), np.cos(U)],
                   axis=-1).reshape(-1, 3) * radius
    return pts + np.asarray(center, float)


class CloudSimNode(Node):

    def __init__(self):
        super().__init__('berxel_cloud_sim')
        self.pub = self.create_publisher(PointCloud2, 'berxel_cloudpoint',
                                         qos_profile_sensor_data)
        self.timer = self.create_timer(0.1, self.publish)  # 10 Hz

        self.W, self.H = 640, 400
        self.fx = self.fy = 570.0
        self.cx, self.cy = 320.0, 200.0
        self.frame = 'berxel_camera/berxel_camera_depth_optical_frame'

        # ---- realistic road-block geometry -------------------------------
        # camera 0.80 m above the floor, pitched 15 deg down; the stick lies
        # on the floor (0..4 cm above it) 1.8 m ahead, spanning the path.
        H_CAM = 0.80
        PITCH = np.deg2rad(15.0)

        def g2c(x, s, h):
            """ground coords (x lateral, s forward, h above floor) -> camera."""
            y = (H_CAM - h) * np.cos(PITCH) - s * np.sin(PITCH)
            z = (H_CAM - h) * np.sin(PITCH) + s * np.cos(PITCH)
            return np.stack(np.broadcast_arrays(x, y, z), axis=-1)

        # floor
        gx = np.arange(-1.5, 1.51, 0.014)
        gs = np.arange(1.0, 3.21, 0.014)
        GX, GS = np.meshgrid(gx, gs, indexing='ij')
        floor = g2c(GX.ravel(), GS.ravel(), np.zeros(GX.size))

        # back wall (h 0..1.0 m at s = 3.2) -- cut away by the ground band
        gxw = np.arange(-1.5, 1.51, 0.018)
        ghw = np.arange(0.0, 1.01, 0.018)
        GXW, GHW = np.meshgrid(gxw, ghw, indexing='ij')
        wall = g2c(GXW.ravel(), np.full(GXW.size, 3.2), GHW.ravel())

        # distractors on the floor: small box 1.8 m ahead-left, pillar right
        bx = np.arange(-0.80, -0.60, 0.006)
        bs = np.arange(2.0, 2.20, 0.006)
        bh = np.arange(0.0, 0.201, 0.015)
        BX, BS, BH = np.meshgrid(bx, bs, bh, indexing='ij')
        box_pts = g2c(BX.ravel(), BS.ravel(), BH.ravel())          # solid block

        px = np.arange(0.70, 0.86, 0.005)
        ps = np.arange(2.5, 2.66, 0.005)
        ph = np.arange(0.0, 1.201, 0.03)
        PX, PS, PH = np.meshgrid(px, ps, ph, indexing='ij')
        pillar = g2c(PX.ravel(), PS.ravel(), PH.ravel())

        # ---- 挡车杆 (Π 形门字杆, 图 6: 800mm 宽 x 200mm 高, 管径 40mm) ----
        S0 = 2.6                      # 2.6 m ahead
        half, top = 0.40, 0.20        # 半宽 0.4 m, 顶杆中心离地 0.20 m
        tube_r = 0.02
        bar_pts = [tube_g((-half, S0, top), (half, S0, top), tube_r),      # 顶杆
                   tube_g((-half, S0, top), (-half, S0, tube_r), tube_r),  # 左腿
                   tube_g((half, S0, top), (half, S0, tube_r), tube_r),    # 右腿
                   disk_g(-half, S0, 0.055), disk_g(half, S0, 0.055)]
        bar = np.vstack(bar_pts)
        bar_cam = g2c(bar[:, 0], bar[:, 1], bar[:, 2])

        self.scene = np.vstack([floor, wall, box_pts, pillar, bar_cam])
        self.rng = np.random.default_rng(7)
        self.get_logger().info(
            f'sim scene ready: {len(self.scene)} pts (floor + wall + '
            'barrier 800x200mm @2.6m + box + pillar + wall)')

    def publish(self):
        P = self.scene + self.rng.normal(0.0, 0.003, self.scene.shape)
        x, y, z = P[:, 0], P[:, 1], P[:, 2]
        u = np.round(self.cx + self.fx * x / z).astype(np.int64)
        v = np.round(self.cy + self.fy * y / z).astype(np.int64)
        inb = ((z > 0.1) & (u >= 0) & (u < self.W) & (v >= 0) & (v < self.H))

        order = np.argsort(-z[inb])                 # far -> near, nearest last
        u, v, z = u[inb][order], v[inb][order], z[inb][order]
        pts = P[inb][order]

        img = np.full((self.H * self.W, 3), np.nan, dtype=np.float32)
        flat = v * self.W + u
        img[flat] = pts                              # nearest overwrite far

        data = np.zeros((self.H * self.W, 4), dtype=np.float32)
        data[:, :3] = img

        msg = PointCloud2()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame
        msg.height, msg.width = self.H, self.W
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step = 16
        msg.row_step = 16 * self.W
        msg.is_dense = False
        msg.data = data.tobytes()
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CloudSimNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
