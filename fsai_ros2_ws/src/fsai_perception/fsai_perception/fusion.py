#!/usr/bin/env python3
"""
Fusion node — combines LiDAR cone positions with camera colour classifications.

Runs at LiDAR rate (20 Hz). On every LiDAR batch:
  1. Transform cone positions from Lidar_F to Fr1A via TF2.
  2. Project each cone centroid into the image using camera mounting extrinsics.
  3. Match projected points to latest YOLO bounding boxes (Hungarian algorithm).
  4. Assign colour from matched YOLO detection.
  5. If camera detections are stale (> camera_max_age_s) or no match, publish 'unknown_cone'.
  6. Publish final Cone3DArray on /cones.

Camera projection:
  - Intrinsics loaded from /front_camera_rgb/camera_info (auto-cached) with yaml fallback.
  - Extrinsics (camera mounting in Fr1A) loaded from extrinsics.yaml.
  - Transform pipeline: Lidar_F → Fr1A (TF2) → camera body frame (extrinsics) → optical frame → pixel.
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


def _load_extrinsics():
    try:
        path = os.path.join(
            get_package_share_directory('fsai_perception'),
            'config', 'extrinsics.yaml'
        )
        with open(path) as f:
            return yaml.safe_load(f)
    except Exception:
        return {}


def _load_intrinsics():
    try:
        path = os.path.join(
            get_package_share_directory('fsai_perception'),
            'config', 'camera_intrinsics.yaml'
        )
        with open(path) as f:
            d = yaml.safe_load(f)
            return d['fx'], d['fy'], d['cx'], d['cy']
    except Exception:
        return 320.0, 320.0, 320.0, 240.0


class Fusion(Node):
    def __init__(self):
        super().__init__('fusion')
        cfg = _load_config()
        ext = _load_extrinsics()

        self.declare_parameter('lidar_input',      cfg.get('lidar_input',      '/perception/lidar/cones'))
        self.declare_parameter('camera_input',     cfg.get('camera_input',     '/perception/camera/detections'))
        self.declare_parameter('output_topic',     cfg.get('output_topic',     '/cones'))
        self.declare_parameter('camera_max_age_s', cfg.get('camera_max_age_s', 1.0))
        self.declare_parameter('bbox_margin_px',   cfg.get('bbox_margin_px',   15.0))
        self.declare_parameter('lidar_frame',      cfg.get('lidar_frame',      'Lidar_F'))
        self.declare_parameter('output_frame',     cfg.get('output_frame',     'Fr1A'))

        lidar_topic  = self.get_parameter('lidar_input').value
        camera_topic = self.get_parameter('camera_input').value
        out_topic    = self.get_parameter('output_topic').value
        self._max_age    = self.get_parameter('camera_max_age_s').value
        self._bbox_margin= self.get_parameter('bbox_margin_px').value
        self._lidar_frame= self.get_parameter('lidar_frame').value
        self._out_frame  = self.get_parameter('output_frame').value

        # --- Camera intrinsics (yaml fallback, overwritten by camera_info) ---
        self._fx, self._fy, self._cx, self._cy = _load_intrinsics()
        self._intrinsics_ready = False

        # --- Camera mounting in Fr1A (from extrinsics.yaml) ---
        cam_ext = ext.get('camera', {})
        cam_pos = cam_ext.get('position', {})
        cam_rot = cam_ext.get('rotation_deg', {})
        self._cam_pos = np.array([
            cam_pos.get('x', 1.532),
            cam_pos.get('y', 0.0),
            cam_pos.get('z', 0.816),
        ], dtype=np.float64)
        # Build camera rotation matrix (vehicle frame → camera body frame)
        # rot = (roll, pitch, yaw) in degrees, CarMaker convention
        # Positive pitch = nose tilted DOWN (rotation around Y-left axis)
        self._cam_R_inv = Rotation.from_euler(
            'xyz',
            [cam_rot.get('roll', 0.0),
             cam_rot.get('pitch', 15.0),
             cam_rot.get('yaw', 0.0)],
            degrees=True
        ).inv().as_matrix()

        # --- TF2 ---
        self._tf_buffer   = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # --- State ---
        self._latest_camera_dets  = []
        self._latest_camera_stamp = None   # rclpy.time.Time

        # --- Pub/Sub ---
        self._pub = self.create_publisher(Cone3DArray, out_topic, 10)

        self.create_subscription(CameraInfo,        '/front_camera_rgb/camera_info',
                                 self._caminfo_cb, 10)
        self.create_subscription(Detection2DArray,  camera_topic,
                                 self._camera_cb,  10)
        self.create_subscription(Cone3DArray,       lidar_topic,
                                 self._lidar_cb,   10)

        self._frame_count = 0
        self.get_logger().info(f'Fusion ready  LiDAR={lidar_topic}  Camera={camera_topic}  Out={out_topic}')

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
                f'Intrinsics from camera_info: fx={self._fx:.1f} fy={self._fy:.1f} '
                f'cx={self._cx:.1f} cy={self._cy:.1f}'
            )

    def _camera_cb(self, msg: Detection2DArray):
        self._latest_camera_dets  = msg.detections
        self._latest_camera_stamp = Time.from_msg(msg.header.stamp)

    def _lidar_cb(self, msg: Cone3DArray):
        now = self.get_clock().now()

        # Check camera detection age
        if self._latest_camera_stamp is not None:
            age_s = (now - self._latest_camera_stamp).nanoseconds / 1e9
            camera_fresh = age_s <= self._max_age
        else:
            camera_fresh = False

        dets   = self._latest_camera_dets if camera_fresh else []
        cones  = list(msg.cones)
        n_lidar = len(cones)
        n_camera = len(dets)

        # --- TF: Lidar_F → Fr1A ---
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
            pass   # first few frames may not have TF yet; positions stay in Lidar_F

        # --- Transform cone positions to Fr1A ---
        positions_fr1a = []
        for cone in cones:
            p = np.array([cone.position.x, cone.position.y, cone.position.z])
            p_fr1a = tf_rotation @ p + tf_translation
            positions_fr1a.append(p_fr1a)

        # --- Project Fr1A positions to image pixels ---
        projections = [self._project(p) for p in positions_fr1a]

        # --- Hungarian matching ---
        matched_lidar  = {}   # lidar_idx → det_idx
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

        # --- Build output cones ---
        # Use the original scan timestamp so downstream consumers (RViz, nav)
        # know exactly when these positions were measured, not when we finished
        # processing them. This keeps cone markers temporally aligned with the
        # raw LiDAR scan even as pipeline latency grows at higher speeds.
        scan_stamp = msg.header.stamp
        out_cones = []

        for i, cone in enumerate(cones):
            p = positions_fr1a[i]
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
    #  Geometry helpers
    # ══════════════════════════════════════════════════════════════════════

    def _project(self, p_fr1a):
        """Project a point in Fr1A to image pixel (u, v). Returns None if behind camera."""
        # Translate to camera origin in Fr1A
        dp = p_fr1a - self._cam_pos

        # Rotate into camera body frame (vehicle frame axes → camera body axes)
        body = self._cam_R_inv @ dp

        # Camera body frame (X-fwd, Y-left, Z-up) → optical frame (X-right, Y-down, Z-fwd)
        cam_x = -body[1]   # right  = -left
        cam_y = -body[2]   # down   = -up
        cam_z =  body[0]   # forward = forward

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
