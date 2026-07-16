#!/usr/bin/env python3
"""
Camera Detector — runs YOLO on RGB images to get 2D bounding boxes with cone colour.

Publishes vision_msgs/Detection2DArray on the camera detections topic.
Each detection carries:
  - bbox: centre (u,v), width, height in pixels
  - results[0].hypothesis.class_id: numeric class id (string)
  - id: class name string (e.g. 'blue_cone', 'yellow_cone', 'orange_cone')
  - results[0].hypothesis.score: YOLO confidence
"""
import os
import yaml

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import (
    Detection2DArray, Detection2D,
    ObjectHypothesisWithPose, BoundingBox2D,
)
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO
from ament_index_python.packages import get_package_share_directory


def _load_config(env, section):
    """Load common.<section> from perception.yaml, then overlay <env>.<section>."""
    try:
        path = os.path.join(
            get_package_share_directory('fsai_perception'),
            'config', 'perception.yaml'
        )
        with open(path) as f:
            data = yaml.safe_load(f) or {}
        cfg = dict((data.get('common') or {}).get(section, {}))
        cfg.update((data.get(env) or {}).get(section, {}))
        return cfg
    except Exception:
        return {}


class CameraDetector(Node):
    def __init__(self):
        super().__init__('camera_detector')
        self.declare_parameter('env', 'sim')
        env = self.get_parameter('env').value
        cfg = _load_config(env, 'camera_detector')

        pkg_share = get_package_share_directory('fsai_perception')
        default_model = os.path.join(pkg_share, cfg.get('model_path', 'models/best.pt'))

        self.declare_parameter('input_topic',          cfg.get('input_topic',         '/perception/image'))
        self.declare_parameter('output_topic',          cfg.get('output_topic',        '/perception/camera/detections'))
        self.declare_parameter('visualization_topic',   cfg.get('visualization_topic', '/perception/camera/image'))
        self.declare_parameter('model_path',            default_model)
        self.declare_parameter('confidence',            cfg.get('confidence',    0.5))
        self.declare_parameter('iou',                   cfg.get('iou',           0.45))
        self.declare_parameter('input_size',            cfg.get('input_size',    640))
        self.declare_parameter('device',                cfg.get('device',        'cpu'))
        self.declare_parameter('publish_visualization', cfg.get('publish_visualization', True))

        in_topic   = self.get_parameter('input_topic').value
        out_topic  = self.get_parameter('output_topic').value
        viz_topic  = self.get_parameter('visualization_topic').value
        model_path = self.get_parameter('model_path').value
        self._conf = self.get_parameter('confidence').value
        self._iou  = self.get_parameter('iou').value
        self._size = self.get_parameter('input_size').value
        self._dev  = self.get_parameter('device').value
        self._viz  = self.get_parameter('publish_visualization').value

        self.get_logger().info(f'Loading YOLO model: {model_path}')
        self._model = YOLO(model_path)
        self.get_logger().info(f'Classes: {self._model.names}')

        self._bridge = CvBridge()
        self._pub    = self.create_publisher(Detection2DArray, out_topic, 10)
        self._viz_pub = self.create_publisher(Image, viz_topic, 10) if self._viz else None

        self.create_subscription(Image, in_topic, self._cb, 10)

        self._frame_count = 0
        self.get_logger().info(f"Camera detector ready  [env={env}]  {in_topic} → {out_topic}")

    # ------------------------------------------------------------------
    def _cb(self, msg: Image):
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            results = self._model.predict(
                frame,
                conf=self._conf,
                iou=self._iou,
                imgsz=self._size,
                device=self._dev,
                half=False,
                agnostic_nms=True,
                verbose=False,
            )
            det_msg = self._build_detections(results[0], msg.header)
            self._pub.publish(det_msg)

            if self._viz and self._viz_pub is not None:
                annotated = results[0].plot(labels=True, conf=True, line_width=1)
                annotated_rgb = cv2.cvtColor(annotated, cv2.COLOR_BGR2RGB)
                viz_msg = self._bridge.cv2_to_imgmsg(annotated_rgb, encoding='rgb8')
                viz_msg.header = msg.header
                self._viz_pub.publish(viz_msg)

            self._frame_count += 1
            if self._frame_count % 50 == 0:
                self.get_logger().info(
                    f'Frame {self._frame_count} | {len(det_msg.detections)} detections'
                )
        except Exception as e:
            self.get_logger().error(f'Inference error: {e}')

    # ------------------------------------------------------------------
    def _build_detections(self, result, header) -> Detection2DArray:
        arr = Detection2DArray()
        arr.header = header

        if result.boxes is None or len(result.boxes) == 0:
            return arr

        for box in result.boxes.cpu().numpy():
            x1, y1, x2, y2 = box.xyxy[0]
            class_id   = int(box.cls[0])
            confidence  = float(box.conf[0])
            class_name  = self._model.names[class_id]

            det = Detection2D()
            det.header = header
            det.bbox   = BoundingBox2D()
            det.bbox.center.position.x = float((x1 + x2) / 2)
            det.bbox.center.position.y = float((y1 + y2) / 2)
            det.bbox.center.theta = 0.0
            det.bbox.size_x = float(x2 - x1)
            det.bbox.size_y = float(y2 - y1)

            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = str(class_id)
            hyp.hypothesis.score    = confidence
            det.results.append(hyp)

            det.id = class_name  # human-readable colour class
            arr.detections.append(det)

        return arr


def main(args=None):
    rclpy.init(args=args)
    node = CameraDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
