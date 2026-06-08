#!/usr/bin/env python3
"""
LiDAR Detector — clusters PointCloud2 into candidate cone positions.

Pipeline:
  1. Range filter (strip far/near points)
  2. Ground removal (10th-percentile Z estimate)
  3. DBSCAN clustering in XY plane
  4. Cluster validation (footprint size, optional height, optional intensity)
  5. Split oversized clusters (two merged cones)
  6. Publish Cone3DArray in Lidar_F frame
"""
import os
import yaml
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2_utils
from sklearn.cluster import DBSCAN, KMeans
from geometry_msgs.msg import Point
from fsai_interfaces.msg import Cone3D, Cone3DArray
from ament_index_python.packages import get_package_share_directory


def _load_config():
    try:
        path = os.path.join(
            get_package_share_directory('fsai_perception'),
            'config', 'perception.yaml'
        )
        with open(path) as f:
            return yaml.safe_load(f).get('lidar_detector', {})
    except Exception:
        return {}


class LidarDetector(Node):
    def __init__(self):
        super().__init__('lidar_detector')
        cfg = _load_config()

        self.declare_parameter('input_topic',        cfg.get('input_topic',        '/perception/pointcloud'))
        self.declare_parameter('output_topic',        cfg.get('output_topic',       '/perception/lidar/cones'))
        self.declare_parameter('eps',                 cfg.get('eps',                0.35))
        self.declare_parameter('min_points',          cfg.get('min_points',         2))
        self.declare_parameter('min_distance_m',      cfg.get('min_distance_m',     0.2))
        self.declare_parameter('max_distance_m',      cfg.get('max_distance_m',     30.0))
        self.declare_parameter('cone_width_min_m',    cfg.get('cone_width_min_m',   0.03))
        self.declare_parameter('cone_width_max_m',    cfg.get('cone_width_max_m',   0.60))
        self.declare_parameter('use_height_filter',   cfg.get('use_height_filter',  False))
        self.declare_parameter('cone_height_min_m',   cfg.get('cone_height_min_m',  0.05))
        self.declare_parameter('cone_height_max_m',   cfg.get('cone_height_max_m',  0.50))
        self.declare_parameter('ground_threshold_m',  cfg.get('ground_threshold_m', 0.10))
        self.declare_parameter('split_threshold_m',   cfg.get('split_threshold_m',  0.45))
        self.declare_parameter('use_intensity_filter',cfg.get('use_intensity_filter', True))

        self._eps          = self.get_parameter('eps').value
        self._min_pts      = self.get_parameter('min_points').value
        self._min_dist     = self.get_parameter('min_distance_m').value
        self._max_dist     = self.get_parameter('max_distance_m').value
        self._w_min        = self.get_parameter('cone_width_min_m').value
        self._w_max        = self.get_parameter('cone_width_max_m').value
        self._use_height   = self.get_parameter('use_height_filter').value
        self._h_min        = self.get_parameter('cone_height_min_m').value
        self._h_max        = self.get_parameter('cone_height_max_m').value
        self._gnd_thresh   = self.get_parameter('ground_threshold_m').value
        self._split_thresh = self.get_parameter('split_threshold_m').value
        self._use_intensity= self.get_parameter('use_intensity_filter').value

        in_topic  = self.get_parameter('input_topic').value
        out_topic = self.get_parameter('output_topic').value

        self._pub = self.create_publisher(Cone3DArray, out_topic, 10)
        self.create_subscription(PointCloud2, in_topic, self._cb, 10)

        self._frame_count = 0
        self.get_logger().info(f'LiDAR detector ready  {in_topic} → {out_topic}')
        self.get_logger().info(
            f'DBSCAN eps={self._eps}m  min_pts={self._min_pts}  '
            f'range={self._min_dist}–{self._max_dist}m'
        )

    # ------------------------------------------------------------------
    def _cb(self, msg: PointCloud2):
        pts = self._read_pc2(msg)
        if pts is None or len(pts) < self._min_pts:
            return

        pts = self._range_filter(pts)
        if len(pts) < self._min_pts:
            return

        pts, ground_z = self._remove_ground(pts)
        if len(pts) < self._min_pts:
            return

        clusters  = self._cluster(pts)
        cones     = self._validate(clusters, msg.header, ground_z)

        out = Cone3DArray()
        out.header = msg.header
        out.cones  = cones
        self._pub.publish(out)

        self._frame_count += 1
        if self._frame_count % 100 == 0:
            self.get_logger().info(
                f'Frame {self._frame_count} | {len(cones)} cone candidates'
            )

    # ------------------------------------------------------------------
    def _read_pc2(self, msg: PointCloud2):
        field_names = [f.name for f in msg.fields]
        has_intensity = 'intensity' in field_names
        fields = ('x', 'y', 'z', 'intensity') if has_intensity else ('x', 'y', 'z')
        try:
            # read_points returns a structured numpy array with named fields
            pts = np.array(list(pc2_utils.read_points(msg, field_names=fields, skip_nans=True)))
            if pts.size == 0:
                return None
            x = pts['x'].astype(np.float32)
            y = pts['y'].astype(np.float32)
            z = pts['z'].astype(np.float32)
            intensity = pts['intensity'].astype(np.float32) if has_intensity else np.zeros(len(x), dtype=np.float32)
            return np.column_stack([x, y, z, intensity])
        except Exception as e:
            self.get_logger().warn(f'PC2 read failed: {e}', throttle_duration_sec=5.0)
            return None

    def _range_filter(self, pts):
        dist = np.linalg.norm(pts[:, :2], axis=1)
        mask = (dist >= self._min_dist) & (dist <= self._max_dist)
        return pts[mask]

    def _remove_ground(self, pts):
        ground_z = float(np.percentile(pts[:, 2], 10))
        mask = pts[:, 2] > (ground_z + self._gnd_thresh)
        return pts[mask], ground_z

    def _cluster(self, pts):
        labels = DBSCAN(eps=self._eps, min_samples=self._min_pts).fit_predict(pts[:, :2])
        clusters = []
        for lbl in set(labels):
            if lbl != -1:
                clusters.append(pts[labels == lbl])
        return clusters

    def _maybe_split(self, cluster):
        wx = np.max(cluster[:, 0]) - np.min(cluster[:, 0])
        wy = np.max(cluster[:, 1]) - np.min(cluster[:, 1])
        if max(wx, wy) <= self._split_thresh or len(cluster) < 4:
            return [cluster]
        try:
            km = KMeans(n_clusters=2, n_init=3, random_state=0)
            labels = km.fit_predict(cluster[:, :2])
            s0, s1 = cluster[labels == 0], cluster[labels == 1]
            if len(s0) >= self._min_pts and len(s1) >= self._min_pts:
                return [s0, s1]
        except Exception:
            pass
        return [cluster]

    def _validate(self, clusters, header, ground_z):
        cones = []
        for raw in clusters:
            for cluster in self._maybe_split(raw):
                wx   = np.max(cluster[:, 0]) - np.min(cluster[:, 0])
                wy   = np.max(cluster[:, 1]) - np.min(cluster[:, 1])
                width = max(wx, wy)
                dist  = float(np.linalg.norm(np.mean(cluster[:, :2], axis=0)))

                # Large orange cones have wider base — allow extra width
                w_max = 0.80 if width > self._w_max else self._w_max
                if width < self._w_min or width > w_max:
                    continue

                if self._use_height:
                    height = float(np.max(cluster[:, 2]) - np.min(cluster[:, 2]))
                    if height < self._h_min or height > self._h_max:
                        continue

                # Intensity check: retro-reflective tape follows inverse-square law
                if self._use_intensity:
                    mean_intensity = float(np.mean(cluster[:, 3]))
                    # At 5m intensity ≈ 980, at 25m ≈ 55 — dynamic minimum
                    min_expected = max(20.0, 20000.0 / (dist ** 2.0))
                    if mean_intensity < min_expected * 0.9:
                        continue

                centroid = np.mean(cluster, axis=0)
                cone = Cone3D()
                cone.header     = header
                cone.position   = Point(x=float(centroid[0]),
                                        y=float(centroid[1]),
                                        z=float(centroid[2]))
                cone.class_name = 'unknown_cone'
                cone.confidence = float(min(1.0, len(cluster) / 20.0))
                cone.source     = 'lidar'
                cones.append(cone)
        return cones


def main(args=None):
    rclpy.init(args=args)
    node = LidarDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
