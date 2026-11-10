#!/usr/bin/env python3
"""Long-stick detection node for organized point clouds.

Pipeline (per frame):
 1. parse PointCloud2 -> organized HxWx3 (stride-downsampled)
 2. connected components on the valid-pixel mask
 3. dominant-plane RANSAC removal per large blob (so sticks lying on a
    table/floor are separated from the background plane)
 4. re-cluster the residual pixels
 5. PCA per sub-cluster -> length / radius / elongation
 6. classify as "stick" and publish poses + markers + segmented points
"""
import time

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Point, Pose, PoseArray
from sensor_msgs.msg import PointCloud2, PointField
from visualization_msgs.msg import Marker, MarkerArray

from stick_detector.geometry_core import (connected_components, dilate3,
                                          ransac_plane, stick_features,
                                          quat_x_to, quat_z_to)


class StickDetectorNode(Node):

    def __init__(self):
        super().__init__('stick_detector')

        # ---- parameters ----
        self.declare_parameter('cloud_topic', '/berxel_cloudpoint')
        self.declare_parameter('process_hz', 8.0)
        self.declare_parameter('downsample_stride', 2)
        self.declare_parameter('max_range', 5.0)
        # ROI (camera optical frame: +x right, +y down, +z forward).
        # 地面障碍场景：y_min 卡在相机高度以下即可去掉天花板/高处杂物。
        self.declare_parameter('roi_enable', True)
        self.declare_parameter('roi_x_min', -1.5)
        self.declare_parameter('roi_x_max', 1.5)
        self.declare_parameter('roi_y_min', -0.10)
        self.declare_parameter('roi_y_max', 2.0)
        self.declare_parameter('roi_z_min', 0.20)
        self.declare_parameter('roi_z_max', 5.0)

        # ---- 地面高度带 ROI（地面障碍场景的核心裁剪）----
        # 长棍贴地放置时，只保留"离地 band_min..band_max 米"的点，
        # 天花板 / 墙 / 货架 / 行人 会被整块切掉。
        self.declare_parameter('ground_roi_enable', True)
        self.declare_parameter('ground_band_min', -0.03)   # 地面以下（噪点）
        self.declare_parameter('ground_band_max', 0.30)    # 地面以上 30cm（挡车杆高 20cm）
        self.declare_parameter('ground_hp_enable', True)   # 用地平面"挖掉"地面点
        self.declare_parameter('ground_hp_min', 0.012)     # 高于地面多少算障碍
        self.declare_parameter('ground_plane_mode', 'auto')  # auto | manual
        self.declare_parameter('camera_height', 0.80)      # manual 模式用
        self.declare_parameter('camera_pitch_deg', 0.0)    # manual 模式用（下俯为正）
        self.declare_parameter('ground_plane_refresh', 30)  # 每 N 帧重估地面
        self.declare_parameter('ground_plane_thresh', 0.02)
        self.declare_parameter('ground_plane_sample', 4000)
        self.declare_parameter('min_cluster_px', 250)
        self.declare_parameter('plane_remove', True)
        self.declare_parameter('plane_thresh', 0.012)
        self.declare_parameter('plane_iters', 60)
        self.declare_parameter('plane_sample', 6000)
        self.declare_parameter('plane_min_px', 2500)
        self.declare_parameter('plane_min_ratio', 0.35)
        self.declare_parameter('min_length', 0.25)
        self.declare_parameter('max_length', 3.0)
        self.declare_parameter('min_elongation', 5.0)
        self.declare_parameter('max_radius', 0.035)
        # ---- 挡车杆（Π 形门字杆）识别 ----
        self.declare_parameter('stick_enable', False)   # 目标为挡车杆时可关掉直棍检测
        self.declare_parameter('barrier_enable', True)
        self.declare_parameter('barrier_width_min', 0.40)
        self.declare_parameter('barrier_width_max', 1.60)
        self.declare_parameter('barrier_height_min', 0.10)
        self.declare_parameter('barrier_height_max', 0.45)
        self.declare_parameter('barrier_top_frac', 0.65)   # 取顶部 35% 点当横杆
        self.declare_parameter('barrier_min_elongation', 3.0)
        self.declare_parameter('barrier_max_tilt', 0.45)   # 横杆与地面夹角余弦上限
        self.declare_parameter('debug_log', False)
        self.declare_parameter('profile', False)

        g = lambda n: self.get_parameter(n).value
        self.cloud_topic = g('cloud_topic')
        self.min_dt = 1.0 / max(float(g('process_hz')), 0.1)
        self.stride = max(int(g('downsample_stride')), 1)
        self.max_range = float(g('max_range'))
        self.roi_enable = bool(g('roi_enable'))
        self.roi_x = (float(g('roi_x_min')), float(g('roi_x_max')))
        self.roi_y = (float(g('roi_y_min')), float(g('roi_y_max')))
        self.roi_z = (float(g('roi_z_min')), float(g('roi_z_max')))
        self.ground_roi = bool(g('ground_roi_enable'))
        self.gband = (float(g('ground_band_min')), float(g('ground_band_max')))
        self.ground_hp = bool(g('ground_hp_enable'))
        self.ground_hp_min = float(g('ground_hp_min'))
        self.gmode = str(g('ground_plane_mode'))
        self.gplane_cfg = dict(thresh=float(g('ground_plane_thresh')),
                               iters=60,
                               sample=int(g('ground_plane_sample')))
        self.grefresh = max(int(g('ground_plane_refresh')), 1)
        self._gplane = None
        self._gframe = 0
        if self.gmode == 'manual':
            p = np.deg2rad(float(g('camera_pitch_deg')))
            self._gplane = (np.array([0.0, np.cos(p), np.sin(p)]),
                            float(g('camera_height')))
        self.min_px = int(g('min_cluster_px'))
        self.plane_remove = bool(g('plane_remove'))
        self.plane_cfg = dict(thresh=float(g('plane_thresh')),
                               iters=int(g('plane_iters')),
                               sample=int(g('plane_sample')))
        self.plane_min_px = int(g('plane_min_px'))
        self.plane_min_ratio = float(g('plane_min_ratio'))
        self.min_len = float(g('min_length'))
        self.max_len = float(g('max_length'))
        self.min_elong = float(g('min_elongation'))
        self.max_radius = float(g('max_radius'))
        self.stick_on = bool(g('stick_enable'))
        self.barrier = bool(g('barrier_enable'))
        self.bar_w = (float(g('barrier_width_min')), float(g('barrier_width_max')))
        self.bar_h = (float(g('barrier_height_min')), float(g('barrier_height_max')))
        self.bar_top_frac = float(g('barrier_top_frac'))
        self.bar_min_elong = float(g('barrier_min_elongation'))
        self.bar_max_tilt = float(g('barrier_max_tilt'))
        self.debug_log = bool(g('debug_log'))
        self.profile = bool(g('profile'))
        self._acc = {}
        self._prof_n = 0

        self.rng = np.random.default_rng(1234)
        self._last_t = 0.0
        self._warned_unorganized = False

        self.create_subscription(PointCloud2, self.cloud_topic,
                                  self.on_cloud, qos_profile_sensor_data)
        self.pub_poses = self.create_publisher(PoseArray,
                                               'stick_detector/sticks', 5)
        self.pub_markers = self.create_publisher(MarkerArray,
                                                  'stick_detector/markers', 5)
        self.pub_cloud = self.create_publisher(PointCloud2,
                                                'stick_detector/stick_points',
                                                qos_profile_sensor_data)
        self.get_logger().info(
            f"stick_detector up, listening '{self.cloud_topic}' "
            f"(len {self.min_len}-{self.max_len} m, "
            f"elong>={self.min_elong}, r<={self.max_radius}, "
            f"profile={self.profile})")

    # ------------------------------------------------------------------
    def on_cloud(self, msg):
        now = time.monotonic()
        if now - self._last_t < self.min_dt:
            return
        self._last_t = now
        t_start = time.perf_counter()

        pts, w, h = self.parse_cloud(msg)
        if pts is None:
            return
        if h <= 1:
            if not self._warned_unorganized:
                self._warned_unorganized = True
                self.get_logger().warn(
                    'input cloud is not organized; set '
                    '"enable_ordered_pointcloud: true" in the camera yaml')
            return
        self._tick('parse', t_start)

        s = self.stride
        if s > 1:
            pts = pts[::s, ::s]
        finite = np.isfinite(pts).all(axis=2)
        valid = (finite
                 & (pts[..., 2] > 0.05)
                 & (pts[..., 2] < self.max_range))
        if self.roi_enable:
            x, y, z = pts[..., 0], pts[..., 1], pts[..., 2]
            valid &= ((x >= self.roi_x[0]) & (x <= self.roi_x[1])
                      & (y >= self.roi_y[0]) & (y <= self.roi_y[1])
                      & (z >= self.roi_z[0]) & (z <= self.roi_z[1]))
        n_before_ground = int(valid.sum())
        ground_removed = False
        if self.ground_roi:
            plane = self._update_ground_plane(pts, valid)
            if plane is not None:
                pn, pd = plane
                h = pd - (pts[..., 0] * pn[0]
                          + pts[..., 1] * pn[1]
                          + pts[..., 2] * pn[2])
                lo = self.ground_hp_min if self.ground_hp else self.gband[0]
                valid &= (h >= lo) & (h <= self.gband[1])
                ground_removed = self.ground_hp
        if self.debug_log and (self.roi_enable or self.ground_roi):
            n_all = int((finite & (pts[..., 2] > 0.05)
                         & (pts[..., 2] < self.max_range)).sum())
            self.get_logger().info(
                f'ROI: {n_all} -> {int(valid.sum())} px '
                f'(ground band kept {int(valid.sum())}/{n_before_ground}, '
                f'-{100.0 * (1 - valid.sum() / max(n_all, 1)):.0f}% total)')
        if int(valid.sum()) < self.min_px * 4:
            return
        t_mask = time.perf_counter()

        hh, ww = valid.shape
        pts_flat = pts.reshape(-1, 3)

        lab, K = self._cluster(valid)
        groups = self._group_indices(lab, K)
        self._tick('cluster', t_mask)

        candidates = []
        t_plane = time.perf_counter()
        for idx in groups:
            if len(idx) < self.min_px:
                continue
            P = pts_flat[idx]
            if (self.plane_remove and not ground_removed
                    and len(idx) >= self.plane_min_px):
                res = self.try_remove_plane(P)
                if res is not None:
                    sub = np.zeros(hh * ww, dtype=bool)
                    sub[idx] = res          # res already means "not on plane"
                    slab, sK = connected_components(sub.reshape(hh, ww))
                    for sidx in self._group_indices(slab, sK):
                        if len(sidx) >= self.min_px:
                            candidates.append(pts_flat[sidx])
                    continue
            candidates.append(P)
        self._tick('plane', t_plane)

        t_pca = time.perf_counter()
        sticks = []
        feats = []
        for P in candidates:
            feat = stick_features(P)
            feats.append(feat)
            if self.stick_on:
                if self.is_stick(feat):
                    feat['P'] = P
                    sticks.append(feat)
                    continue
                # robust refinement: trim to a thin core around the first
                # PCA axis (sticks merged with other objects)
                feat2 = self.refine(P, feat)
                if feat2 is not None and self.is_stick(feat2):
                    feat2['P'] = feat2.pop('_P')
                    sticks.append(feat2)
                    continue
            if self.barrier:
                bar = self.detect_barrier(P)
                if bar is not None:
                    bar['P'] = P
                    sticks.append(bar)
        self._tick('pca', t_pca)

        if self.debug_log and feats:
            top = sorted(feats, key=lambda f: f['elongation'], reverse=True)[:6]
            desc = ' | '.join(f"L={f['length']:.2f} r={f['radius'] * 1000:.0f} "
                              f"e={f['elongation']:.0f}" for f in top)
            self.get_logger().info(
                f'candidates({len(feats)}): {desc} '
                f'[need L {self.min_len}-{self.max_len} e>={self.min_elong} '
                f'r<={self.max_radius * 1000:.0f}mm]')

        t_pub = time.perf_counter()
        self.publish(msg, sticks)
        self._tick('publish', t_pub)
        self._tick('total', t_start)
        if sticks:
            summary = '; '.join(
                (f"#{i + 1} BARRIER W={f['width']:.2f}m H={f['height']:.2f}m"
                 if f.get('kind') == 'barrier' else
                 f"#{i + 1} L={f['length']:.2f}m r={f['radius'] * 1000:.0f}mm "
                 f"e={f['elongation']:.0f}") +
                f" c=({f['center'][0]:.2f},{f['center'][1]:.2f},"
                f"{f['center'][2]:.2f})"
                for i, f in enumerate(sticks))
            self.get_logger().info(f"detected {len(sticks)} stick(s): {summary}")

    # ------------------------------------------------------------------
    def detect_barrier(self, P):
        """Π-shaped barrier (门字挡车杆): horizontal top bar on two legs.

        Uses the ground plane to get heights, checks that
          * the cluster stands on the floor (lowest points near h = 0),
          * its height is in [barrier_height_min, barrier_height_max],
          * the top 35 % of points form a horizontal, elongated bar whose
            length (= barrier width) is in range.
        Returns a feature dict in the same shape as a stick detection so the
        existing PoseArray / marker publishing works unchanged.
        """
        if self._gplane is None:
            return None
        pn, pd = self._gplane
        h = pd - P @ pn
        h_max = float(np.percentile(h, 99.0))
        h_min = float(np.percentile(h, 1.0))
        if not (self.bar_h[0] <= h_max <= self.bar_h[1]):
            return None
        if h_min > 0.08:                      # not standing on the floor
            return None
        top = P[h >= self.bar_top_frac * h_max]
        if len(top) < 30:
            return None
        f = stick_features(top)
        if f['elongation'] < self.bar_min_elong:
            return None
        width = f['length']
        if not (self.bar_w[0] <= width <= self.bar_w[1]):
            return None
        if abs(float(np.dot(f['axis'], pn))) > self.bar_max_tilt:
            return None                       # top bar is not horizontal
        # horizontal offset of the bar centre along the ground plane
        h_bar = float(pd - np.dot(f['center'], pn))
        if h_bar < 0.5 * h_max:               # bar must be the top of the shape
            return None
        f['kind'] = 'barrier'
        f['width'] = width
        f['height'] = h_max
        return f

    def is_stick(self, f):
        return (self.min_len <= f['length'] <= self.max_len
                and f['elongation'] >= self.min_elong
                and f['radius'] <= self.max_radius)

    def refine(self, P, feat0):
        """Core-trim refinement: keep only points within max_radius of the
        first PCA axis, then re-fit. Returns new features or None.

        Requires the core to retain >= 60% of the cluster, otherwise two
        distinct merged objects could be carved into a fake long stick.
        """
        if feat0 is None or feat0['length'] < self.min_len:
            return None
        axis = feat0['axis']
        ref = P.mean(axis=0)
        t = (P - ref) @ axis
        d = np.linalg.norm(P - ref - np.outer(t, axis), axis=1)
        core = P[d <= self.max_radius]
        if len(core) < max(self.min_px, len(P) * 0.6):
            return None
        feat = stick_features(core)
        feat['_P'] = core
        return feat

    # ------------------------------------------------------------------
    def try_remove_plane(self, P):
        """Return residual-mask (True = not on dominant plane) or None."""
        fit = ransac_plane(P, rng=self.rng, **self.plane_cfg)
        if fit is None:
            return None
        _, _, inlier_mask = fit
        ratio = float(inlier_mask.mean())
        residual = ~inlier_mask
        if ratio >= self.plane_min_ratio and int(residual.sum()) >= self.min_px:
            return residual
        return None

    # ------------------------------------------------------------------
    def _tick(self, key, t_ref):
        """Per-stage timing (only when profile:=true)."""
        if not self.profile:
            return
        now = time.perf_counter()
        self._acc[key] = self._acc.get(key, 0.0) + (now - t_ref)
        if key == 'total':
            self._prof_n += 1
            if self._prof_n % 5 == 0:
                n = self._prof_n
                parts = ' '.join(f'{k}={v / n * 1000:.1f}ms'
                                 for k, v in self._acc.items())
                self.get_logger().info(f'[profile] {parts}')
                self._acc = {}
                self._prof_n = 0

    # ------------------------------------------------------------------
    def _update_ground_plane(self, pts, finite):
        """Estimate the ground plane (auto: RANSAC on all points, refreshed
        every N frames; manual: from camera_height / camera_pitch)."""
        if self.gmode == 'manual' or self._gplane is not None:
            self._gframe += 1
            if self._gplane is not None and self._gframe % self.grefresh != 1:
                return self._gplane
        P = pts[finite]
        if len(P) < 500:
            return self._gplane
        fit = ransac_plane(P, rng=self.rng, **self.gplane_cfg)
        if fit is None:
            return self._gplane
        nvec, c, inl = fit
        d = float(np.dot(nvec, c))
        if d < 0:                       # make d positive: ground in front
            nvec, d = -nvec, -d
        if not (0.1 <= d <= 3.0) or float(inl.mean()) < 0.15:
            return self._gplane         # implausible plane, keep previous
        self._gplane = (nvec, d)
        return self._gplane

    @staticmethod
    def _cluster(mask2d):
        """Connected components on a 2x majority-pooled grid (4x faster).

        Pooling '>=2 of 4' removes isolated noise pixels while keeping the
        large scene blob intact; labels are upsampled back to full resolution.
        """
        h, w = mask2d.shape
        h2, w2 = (h // 2) * 2, (w // 2) * 2
        if h2 < 4 or w2 < 4:
            return connected_components(mask2d)
        blk = mask2d[:h2, :w2].reshape(h2 // 2, 2, w2 // 2, 2)
        cov = blk.sum(axis=(1, 3)) >= 2
        cov = dilate3(cov)          # re-join chunks split by the pooling
        lab_c, K = connected_components(cov)
        lab = np.repeat(np.repeat(lab_c, 2, axis=0), 2, axis=1)
        out = np.zeros((h, w), dtype=np.int32)
        out[:h2, :w2] = np.where(mask2d[:h2, :w2], lab, 0)
        return out, K

    @staticmethod
    def _group_indices(lab, K):
        """Return a list of flat pixel indices grouped by label 1..K (one pass)."""
        flat_labels = lab.ravel()
        ids = np.flatnonzero(flat_labels)
        if ids.size == 0:
            return []
        labels = flat_labels[ids]
        order = np.argsort(labels, kind='stable')
        ids = ids[order]
        labels = labels[order]
        bounds = np.searchsorted(labels, np.arange(1, K + 2))
        return [ids[bounds[k - 1]:bounds[k]] for k in range(1, K + 1)
                if bounds[k] > bounds[k - 1]]

    def parse_cloud(self, msg):
        """PointCloud2 -> (HxWx3 float32 array, width, height).

        Zero-copy: views msg.data through a structured dtype (no bytes() copy,
        no per-field slicing), then one 3-column gather.
        """
        w, h, step = msg.width, msg.height, msg.point_step
        n = w * h
        if len(msg.data) < n * step:
            self.get_logger().warn('cloud buffer smaller than expected')
            return None, 0, 0
        offs = {}
        for f in msg.fields:
            if f.name in ('x', 'y', 'z') and f.datatype == PointField.FLOAT32:
                offs[f.name] = f.offset
        if len(offs) < 3:
            self.get_logger().warn('cloud needs x/y/z FLOAT32 fields')
            return None, 0, 0
        dt = np.dtype({'names': ['x', 'y', 'z'],
                       'formats': [np.float32, np.float32, np.float32],
                       'offsets': [offs['x'], offs['y'], offs['z']],
                       'itemsize': step})
        view = np.frombuffer(msg.data, dtype=dt, count=n)
        pts = np.empty((n, 3), dtype=np.float32)
        pts[:, 0] = view['x']
        pts[:, 1] = view['y']
        pts[:, 2] = view['z']
        return pts.reshape(h, w, 3), w, h

    # ------------------------------------------------------------------
    def publish(self, header_msg, sticks):
        frame = header_msg.header.frame_id
        stamp = header_msg.header.stamp

        poses = PoseArray()
        poses.header.frame_id = frame
        poses.header.stamp = stamp
        ma = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        ma.markers.append(clear)

        stick_points = []
        for i, f in enumerate(sticks):
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = map(float, f['center'])
            qx, qy, qz, qw = quat_x_to(f['axis'])
            pose.orientation.x, pose.orientation.y = qx, qy
            pose.orientation.z, pose.orientation.w = qz, qw
            poses.poses.append(pose)

            line = Marker()
            line.header.frame_id = frame
            line.header.stamp = stamp
            line.ns, line.id = 'sticks', i + 1
            line.type = Marker.LINE_LIST
            line.action = Marker.ADD
            line.scale.x = 0.03
            line.color.r, line.color.g, line.color.b = 1.0, 0.2, 0.1
            line.color.a = 0.95
            line.pose.orientation.w = 1.0
            for tip in (f['tip_a'], f['tip_b']):
                pt = Point()
                pt.x, pt.y, pt.z = map(float, tip)
                line.points.append(pt)
            ma.markers.append(line)

            # solid red cylinder along the stick (thicker than the line, so it
            # reads as a rod in rviz). Radius floored at 8 mm for visibility.
            cyl = Marker()
            cyl.header.frame_id = frame
            cyl.header.stamp = stamp
            cyl.ns, cyl.id = 'stick_cylinders', i + 1
            cyl.type = Marker.CYLINDER
            cyl.action = Marker.ADD
            cyl.pose.position.x, cyl.pose.position.y = map(float, f['center'][:2])
            cyl.pose.position.z = float(f['center'][2])
            zq = quat_z_to(f['axis'])
            cyl.pose.orientation.x, cyl.pose.orientation.y = zq[0], zq[1]
            cyl.pose.orientation.z, cyl.pose.orientation.w = zq[2], zq[3]
            r_vis = max(float(f['radius']), 0.008)
            cyl.scale.x = cyl.scale.y = 2.0 * r_vis
            cyl.scale.z = float(f['length'])
            cyl.color.r, cyl.color.g, cyl.color.b = 1.0, 0.0, 0.0
            cyl.color.a = 0.55
            ma.markers.append(cyl)

            txt = Marker()
            txt.header.frame_id = frame
            txt.header.stamp = stamp
            txt.ns, txt.id = 'stick_labels', i + 1
            txt.type = Marker.TEXT_VIEW_FACING
            txt.action = Marker.ADD
            txt.pose.position.x, txt.pose.position.y = map(float, f['center'][:2])
            txt.pose.position.z = float(f['center'][2]) + 0.06
            txt.pose.orientation.w = 1.0
            txt.scale.z = 0.05
            txt.color.r = txt.color.g = txt.color.b = 1.0
            txt.color.a = 0.95
            if f.get('kind') == 'barrier':
                txt.text = (f"barrier{i + 1} W={f['width']:.2f}m "
                            f"H={f['height']:.2f}m")
            else:
                txt.text = f"stick{i + 1} {f['length']:.2f}m"
            ma.markers.append(txt)

            stick_points.append(f['P'])

        self.pub_poses.publish(poses)
        self.pub_markers.publish(ma)

        if stick_points:
            allp = np.vstack(stick_points).astype(np.float32)
            cloud = PointCloud2()
            cloud.header.frame_id = frame
            cloud.header.stamp = stamp
            cloud.height, cloud.width = 1, len(allp)
            cloud.fields = [
                PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            ]
            cloud.is_bigendian = False
            cloud.point_step = 12
            cloud.row_step = 12 * len(allp)
            cloud.is_dense = True
            cloud.data = allp.tobytes()
            self.pub_cloud.publish(cloud)

    # ------------------------------------------------------------------


def main(args=None):
    rclpy.init(args=args)
    node = StickDetectorNode()
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
