#!/usr/bin/env python3
"""
Bridge node — normalises sensor topics to clean internal topics.

Sim (CarMaker):
  /carmaker/pointcloud  (PointCloud v1)      → /perception/pointcloud (PointCloud2)
  /front_camera_rgb/image_raw  (Image)       → /perception/image      (Image, relay)
  /front_camera_depth/image_raw (mono16, cm) → /perception/depth      (32FC1, metres)

Real hardware (VLP-16 + ZED2):
  /velodyne_points      (PointCloud2)        → /perception/pointcloud (PointCloud2, relay)
  /zed/.../image_rect_color (Image)          → /perception/image      (Image, relay)
  /zed/.../depth_registered (32FC1, metres)  → /perception/depth      (32FC1, relay)

pointcloud_format parameter controls the pointcloud path:
  'v1'  — CarMaker legacy PointCloud (sim default)
  'v2'  — PointCloud2 passthrough (real hardware)
"""
import os
import yaml
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import Image, PointCloud, PointCloud2, PointField
from cv_bridge import CvBridge
from ament_index_python.packages import get_package_share_directory


def _load_config():
    try:
        path = os.path.join(
            get_package_share_directory('fsai_perception'),
            'config', 'perception.yaml'
        )
        with open(path) as f:
            return yaml.safe_load(f).get('bridge', {})
    except Exception:
        return {}


class Bridge(Node):
    def __init__(self):
        super().__init__('bridge')
        cfg = _load_config()

        self.declare_parameter('pointcloud_in',     cfg.get('pointcloud_in',     '/carmaker/pointcloud'))
        self.declare_parameter('image_in',          cfg.get('image_in',          '/front_camera_rgb/image_raw'))
        self.declare_parameter('depth_in',          cfg.get('depth_in',          '/front_camera_depth/image_raw'))
        self.declare_parameter('pointcloud_out',    cfg.get('pointcloud_out',    '/perception/pointcloud'))
        self.declare_parameter('image_out',         cfg.get('image_out',         '/perception/image'))
        self.declare_parameter('depth_out',         cfg.get('depth_out',         '/perception/depth'))
        self.declare_parameter('depth_max_m',       cfg.get('depth_max_m',       50.0))
        self.declare_parameter('pointcloud_format', cfg.get('pointcloud_format', 'v1'))

        pc_in   = self.get_parameter('pointcloud_in').value
        img_in  = self.get_parameter('image_in').value
        dep_in  = self.get_parameter('depth_in').value
        pc_out  = self.get_parameter('pointcloud_out').value
        img_out = self.get_parameter('image_out').value
        dep_out = self.get_parameter('depth_out').value
        self._depth_max  = self.get_parameter('depth_max_m').value
        pc_format        = self.get_parameter('pointcloud_format').value

        self._bridge = CvBridge()

        best_effort = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self._pc_pub  = self.create_publisher(PointCloud2, pc_out,  10)
        self._img_pub = self.create_publisher(Image,       img_out, 10)
        self._dep_pub = self.create_publisher(Image,       dep_out, 10)

        if pc_format == 'v2':
            # Real hardware: VLP-16 publishes PointCloud2 directly — just relay it
            self.create_subscription(PointCloud2, pc_in, self._pc2_cb, 10)
        else:
            # Sim: CarMaker publishes legacy PointCloud v1 — convert to PointCloud2
            self.create_subscription(PointCloud, pc_in, self._pc_cb, 10)

        self.create_subscription(Image, img_in, self._img_cb, best_effort)
        self.create_subscription(Image, dep_in, self._dep_cb, 10)

        self.get_logger().info(f'Bridge ready  {pc_in} → {pc_out}  (format={pc_format})')
        self.get_logger().info(f'              {img_in} → {img_out}')
        self.get_logger().info(f'              {dep_in} → {dep_out}')

    # ------------------------------------------------------------------
    def _pc2_cb(self, msg: PointCloud2):
        """Real hardware path — VLP-16 already publishes PointCloud2, just relay."""
        self._pc_pub.publish(msg)

    # ------------------------------------------------------------------
    def _img_cb(self, msg: Image):
        self._img_pub.publish(msg)

    # ------------------------------------------------------------------
    def _dep_cb(self, msg: Image):
        try:
            if msg.encoding == '32FC1':
                self._dep_pub.publish(msg)
                return
            # mono16 from CarMaker is in centimetres
            depth_cm = self._bridge.imgmsg_to_cv2(msg, desired_encoding='mono16')
            depth_m  = depth_cm.astype(np.float32) / 100.0
            depth_m[depth_m > self._depth_max] = 0.0
            out = self._bridge.cv2_to_imgmsg(depth_m, encoding='32FC1')
            out.header = msg.header
            self._dep_pub.publish(out)
        except Exception as e:
            self.get_logger().error(f'Depth conversion failed: {e}', throttle_duration_sec=2.0)

    # ------------------------------------------------------------------
    def _pc_cb(self, msg: PointCloud):
        try:
            pts = msg.points
            n   = len(pts)
            if n == 0:
                return

            # Vectorised numpy conversion — much faster than a per-point loop
            xyz = np.array([(p.x, p.y, p.z) for p in pts], dtype=np.float32)

            intensity = None
            for ch in msg.channels:
                if ch.name == 'intensity':
                    intensity = np.array(ch.values, dtype=np.float32)
                    break

            has_intensity = intensity is not None and len(intensity) == n

            pc2 = PointCloud2()
            pc2.header      = msg.header
            pc2.height      = 1
            pc2.width       = n
            pc2.is_dense    = True
            pc2.is_bigendian = False

            if has_intensity:
                data = np.column_stack([xyz, intensity]).astype(np.float32)
                pc2.fields = [
                    PointField(name='x',         offset=0,  datatype=PointField.FLOAT32, count=1),
                    PointField(name='y',         offset=4,  datatype=PointField.FLOAT32, count=1),
                    PointField(name='z',         offset=8,  datatype=PointField.FLOAT32, count=1),
                    PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
                ]
                pc2.point_step = 16
            else:
                data = xyz
                pc2.fields = [
                    PointField(name='x', offset=0,  datatype=PointField.FLOAT32, count=1),
                    PointField(name='y', offset=4,  datatype=PointField.FLOAT32, count=1),
                    PointField(name='z', offset=8,  datatype=PointField.FLOAT32, count=1),
                ]
                pc2.point_step = 12

            pc2.row_step = pc2.point_step * n
            pc2.data     = data.tobytes()
            self._pc_pub.publish(pc2)

        except Exception as e:
            self.get_logger().warn(f'PointCloud conversion failed: {e}', throttle_duration_sec=5.0)


def main(args=None):
    rclpy.init(args=args)
    node = Bridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
