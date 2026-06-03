#!/usr/bin/env python3
"""
Fusion node — combines LiDAR cone positions with camera colour classifications.

Runs at LiDAR rate (20 Hz). On every LiDAR batch:
  1. Transform cone positions from lidar_frame to output_frame via TF2.
  2. Project each cone centroid into the image using camera extrinsics from TF2.
  3. Match projected points to latest YOLO bounding boxes (Hungarian algorithm).
  4. Assign colour from matched YOLO detection.
  5. If camera detections are stale (> camera_max_age_s) or no match, publish 'unknown_cone'.
  6. Publish final Cone3DArray on /cones.

Camera projection:
  - Intrinsics from camera_info_topic (populated on first message, frames skipped until ready).
  - Extrinsics from TF2 lookup: output_frame → camera_frame (populated on first success,
    frames skipped until ready). Option B: lazy population — same pattern as lidar TF.
  - Transform pipeline: lidar_frame → output_frame (TF2) → camera body frame (TF2) → optical frame → pixel.

Key parameters (all in perception.yaml, overridable via --ros-args):
  Sim:  lidar_frame=Lidar_F  output_frame=Fr1A      camera_frame=Cam_F
        camera_info_topic=/front_camera_rgb/camera_info
  Real: lidar_frame=velodyne  output_frame=base_link  camera_frame=zed_camera_center
        camera_info_topic=/zed/zed_node/rgb/camera_info
"""
import os
import yaml
import numpy as np
from scipy.optimize import linear_sum_assignment
from scipy.spatial.transform import Rotation

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
import tf2_ros
from sensor_msgs.msg import CameraInfo
from vision_msgs.msg import Detection2DArray
from fsai_interfaces.msg import Cone3D, Cone3DArray
from geometry_msgs.msg import Point
from ament_index_python.packages import get_package_share_directory


def _load_config():
    try:
        path = os.path.join(
            get_package_share_directory('fsai_perception'),
            'config', 'perception.yaml'
        )
        with open(path) as f:
            return yaml.safe_load(f).get('fusion', {})
    except Exception:
        return {}


class Fusion(Node):
    def __init__(self):
        super().__init__('fusion')
        cfg = _load_config()

        self.declare_parameter('lidar_input',         cfg.get('lidar_input',         '/perception/lidar/cones'))
        self.declare_parameter('camera_input',        cfg.get('camera_input',        '/perception/camera/detections'))
        self.declare_parameter('output_topic',        cfg.get('output_topic',        '/cones'))
        self.declare_parameter('camera_max_age_s',    cfg.get('camera_max_age_s',    1.0))
        self.declare_parameter('bbox_margin_px',      cfg.get('bbox_margin_px',      15.0))
        self.declare_parameter('lidar_frame',         cfg.get('lidar_frame',         'Lidar_F'))
        self.declare_parameter('output_frame',        cfg.get('output_frame',        'Fr1A'))
        self.declare_parameter('camera_frame',        cfg.get('camera_frame',        'Cam_F'))
        self.declare_parameter('camera_info_topic',   cfg.get('camera_info_topic',   '/front_camera_rgb/camera_info'))

        lidar_topic      = self.get_parameter('lidar_input').value
        camera_topic     = self.get_parameter('camera_input').value
        out_topic        = self.get_parameter('output_topic').value
        self._max_age    = self.get_parameter('camera_max_age_s').value
        self._bbox_margin= self.get_parameter('bbox_margin_px').value
        self._lidar_frame= self.get_parameter('lidar_frame').value
        self._out_frame  = self.get_parameter('output_frame').value
        self._cam_frame  = self.get_parameter('camera_frame').value
        caminfo_topic    = self.get_parameter('camera_info_topic').value

        # Intrinsics — populated on first camera_info message, frames skipped until ready
        self._fx = self._fy = self._cx = self._cy = None
        self._intrinsics_ready = False

        # Extrinsics — populated on first successful TF lookup, frames skipped until ready
        self._cam_pos   = None   # np.array([x, y, z]) in output_frame
        self._cam_R_inv = None   # rotation matrix: output_frame → camera body frame (inverted)

        # TF2
        self._tf_buffer   = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # State
        self._latest_camera_dets  = []
        self._latest_camera_stamp = None

        # Pub/Sub
        self._pub = self.create_publisher(Cone3DArray, out_topic, 10)

        self.create_subscription(CameraInfo,       caminfo_topic,  self._caminfo_cb, 10)
        self.create_subscription(Detection2DArray, camera_topic,   self._camera_cb,  10)
        self.create_subscription(Cone3DArray,      lidar_topic,    self._lidar_cb,   10)

        self._frame_count = 0
        self.get_logger().info(
            f'Fusion ready  lidar={lidar_topic}  camera={camera_topic}  '
            f'out={out_topic}  cam_frame={self._cam_frame}  '
            f'camera_info={caminfo_topic}'
        )

    # ══════════════════════════════════════════════════════════════════════
    #  Callbacks
    # ══════════════════════════════════════════════════════════════════════

    def _caminfo_cb(self, msg: CameraInfo):
        if not self._intrinsics_ready:
            self._fx = msg.k[0]
            self._fy = msg.k[4]
            self._cx = msg.k[2]
            self._cy = msg.k[5]
            self._intrinsics_ready = True
            self.get_logger().info(
                f'Intrinsics from {self.get_parameter("camera_info_topic").value}: '
                f'fx={self._fx:.1f} fy={self._fy:.1f} cx={self._cx:.1f} cy={self._cy:.1f}'
            )

    def _camera_cb(self, msg: Detection2DArray):
        self._latest_camera_dets  = msg.detections
        self._latest_camera_stamp = Time.from_msg(msg.header.stamp)

    def _lidar_cb(self, msg: Cone3DArray):
        # Try to populate camera extrinsics from TF if not yet ready
        if self._cam_pos is None:
            self._try_load_extrinsics()

        # Skip frame if intrinsics or extrinsics are not yet available
        if not self._intrinsics_ready or self._cam_pos is None:
            return

        now = self.get_clock().now()

        if self._latest_camera_stamp is not None:
            age_s = (now - self._latest_camera_stamp).nanoseconds / 1e9
            camera_fresh = age_s <= self._max_age
        else:
            camera_fresh = False

        dets    = self._latest_camera_dets if camera_fresh else []
        cones   = list(msg.cones)
        n_lidar = len(cones)
        n_camera = len(dets)

        # TF: lidar_frame → output_frame
        tf_translation = np.zeros(3)
        tf_rotation    = np.eye(3)
        try:
            tf = self._tf_buffer.lookup_transform(
                self._out_frame, self._lidar_frame, rclpy.time.Time(),
                timeout=Duration(seconds=0.05)
            )
            t = tf.transform.translation
            q = tf.transform.rotation
            tf_translation = np.array([t.x, t.y, t.z])
            tf_rotation    = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
        except Exception:
            pass

        # Transform cone positions to output_frame
        positions_out = []
        for cone in cones:
            p = np.array([cone.position.x, cone.position.y, cone.position.z])
            positions_out.append(tf_rotation @ p + tf_translation)

        # Project to image pixels
        projections = [self._project(p) for p in positions_out]

        # Hungarian matching
        matched_lidar  = {}
        matched_camera = set()

        if n_lidar > 0 and n_camera > 0:
            LARGE = 1e9
            cost  = np.full((n_lidar, n_camera), LARGE)

            for i, uv in enumerate(projections):
                if uv is None:
                    continue
                u, v = uv
                for j, det in enumerate(dets):
                    if self._point_in_bbox(u, v, det):
                        bx = det.bbox.center.position.x
                        by = det.bbox.center.position.y
                        dist_to_centre = np.hypot(u - bx, v - by)
                        conf = det.results[0].hypothesis.score if det.results else 1.0
                        cost[i][j] = dist_to_centre - conf * 10.0

            row_ind, col_ind = linear_sum_assignment(cost)
            for r, c in zip(row_ind, col_ind):
                if cost[r][c] < LARGE:
                    matched_lidar[r]  = c
                    matched_camera.add(c)

        # Build output cones
        scan_stamp = msg.header.stamp
        out_cones  = []

        for i, cone in enumerate(cones):
            p   = positions_out[i]
            out = Cone3D()
            out.header.stamp    = scan_stamp
            out.header.frame_id = self._out_frame
            out.position        = Point(x=float(p[0]), y=float(p[1]), z=0.0)

            if i in matched_lidar and camera_fresh:
                det = dets[matched_lidar[i]]
                out.class_name = det.id
                out.confidence = float(det.results[0].hypothesis.score if det.results else 1.0)
                out.source     = 'fused'
            else:
                out.class_name = 'unknown_cone'
                out.confidence = cone.confidence
                out.source     = 'lidar'

            out_cones.append(out)

        result = Cone3DArray()
        result.header.stamp    = scan_stamp
        result.header.frame_id = self._out_frame
        result.cones           = out_cones
        self._pub.publish(result)

        self._frame_count += 1
        if self._frame_count % 100 == 0:
            n_fused = sum(1 for c in out_cones if c.source == 'fused')
            self.get_logger().info(
                f'Frame {self._frame_count} | '
                f'{n_fused} fused + {len(out_cones) - n_fused} lidar-only = {len(out_cones)} total'
            )

    # ══════════════════════════════════════════════════════════════════════
    #  Extrinsics — lazy TF lookup
    # ══════════════════════════════════════════════════════════════════════

    def _try_load_extrinsics(self):
        try:
            tf = self._tf_buffer.lookup_transform(
                self._out_frame, self._cam_frame, rclpy.time.Time(),
                timeout=Duration(seconds=0.05)
            )
            t = tf.transform.translation
            q = tf.transform.rotation
            self._cam_pos = np.array([t.x, t.y, t.z], dtype=np.float64)
            # Rotation: output_frame → camera body frame (we need the inverse for projection)
            self._cam_R_inv = Rotation.from_quat([q.x, q.y, q.z, q.w]).inv().as_matrix()
            self.get_logger().info(
                f'Camera extrinsics from TF ({self._out_frame} → {self._cam_frame}): '
                f'pos=[{t.x:.3f}, {t.y:.3f}, {t.z:.3f}]'
            )
        except Exception:
            pass  # TF not ready yet — will retry on next LiDAR frame

    # ══════════════════════════════════════════════════════════════════════
    #  Geometry helpers
    # ══════════════════════════════════════════════════════════════════════

    def _project(self, p_out):
        """Project a point in output_frame to image pixel (u, v). Returns None if behind camera."""
        dp   = p_out - self._cam_pos
        body = self._cam_R_inv @ dp

        # Camera body frame (X-fwd, Y-left, Z-up) → optical frame (X-right, Y-down, Z-fwd)
        cam_x = -body[1]
        cam_y = -body[2]
        cam_z =  body[0]

        if cam_z < 0.1:
            return None

        u = self._fx * cam_x / cam_z + self._cx
        v = self._fy * cam_y / cam_z + self._cy
        return float(u), float(v)

    def _point_in_bbox(self, u, v, det, extra_margin=0.0):
        bx = det.bbox.center.position.x
        by = det.bbox.center.position.y
        hw = det.bbox.size_x / 2.0 + self._bbox_margin + extra_margin
        hh = det.bbox.size_y / 2.0 + self._bbox_margin + extra_margin
        return (bx - hw <= u <= bx + hw) and (by - hh <= v <= by + hh)


def main(args=None):
    rclpy.init(args=args)
    node = Fusion()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
