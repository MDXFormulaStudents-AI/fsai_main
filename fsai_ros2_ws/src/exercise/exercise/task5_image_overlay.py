"""
Task 5 – Image Overlay (HUD)
==============================
Subscribe to the raw camera image and the cone detections simultaneously.
Draw a heads-up display (HUD) on the image showing:
  - Total cone count per colour
  - Distance to each detected cone, sorted nearest-first

Publish the annotated image to a new topic, then add it as an Image
display in RViz2 to see it live alongside the 3-D visualisation.

What you will learn
-------------------
- How to subscribe to two topics in one node
- How to convert a ROS image message to an OpenCV image (cv_bridge)
- How to draw text on an image with OpenCV
- How to convert the image back and publish it

Topics
------
  Subscribes : /camera/image_raw        (sensor_msgs/msg/Image)
  Subscribes : /detections/lidar        (fsai_interfaces/msg/Cone3DArray)
  Publishes  : /student/annotated_image (sensor_msgs/msg/Image)

Running
-------
  ros2 run exercise task5_overlay

Add to RViz2 manually
---------------------
  1. Open RViz2  (./Start_RViz.sh)
  2. Click  Add  →  By topic  →  /student/annotated_image  →  Image
  3. The annotated camera feed should appear in the Image panel.

YOUR TASKS
----------
  TODO-1  Read through _draw_hud() and understand every cv2.putText() call.
  TODO-2  Change the HUD background colour from black to a dark translucent
          rectangle. Hint: look up cv2.rectangle() and alpha blending.
  TODO-3  Add a line to the HUD showing the distance to the NEAREST cone only,
          with larger text so it stands out.
  TODO-4  (Challenge) Change the text colour for each cone line to match the
          cone's colour (yellow text for yellow cones, etc.).
  TODO-5  (Challenge) Add a simple warning message in red text in the centre
          of the image when a cone is closer than 3 metres.
"""

import math

import cv2
import numpy as np  # noqa: F401 – imported for students who extend this file

import rclpy
from rclpy.node import Node

from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from fsai_interfaces.msg import Cone3DArray


# Maximum number of per-cone distance lines shown in the HUD
MAX_DISTANCE_LINES: int = 6

# Font used throughout the HUD
_FONT = cv2.FONT_HERSHEY_SIMPLEX


class ImageOverlayNode(Node):
    """Draws a cone-detection HUD on the live camera feed."""

    def __init__(self):
        super().__init__('image_overlay_node')

        self._bridge = CvBridge()

        # Cache the latest cone detection so the image callback can use it
        # without needing to synchronise message timestamps.
        self._latest_cones: Cone3DArray | None = None

        # --- Subscribers ------------------------------------------------------
        self._img_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self._on_image,
            10,
        )

        self._cone_sub = self.create_subscription(
            Cone3DArray,
            '/detections/lidar',
            self._on_cones,
            10,
        )

        # --- Publisher --------------------------------------------------------
        self._pub = self.create_publisher(
            Image,
            '/student/annotated_image',
            10,
        )

        self.get_logger().info(
            'ImageOverlayNode started.\n'
            '  Add /student/annotated_image as an Image display in RViz2.'
        )

    # --------------------------------------------------------------------------
    # Callbacks
    # --------------------------------------------------------------------------

    def _on_cones(self, msg: Cone3DArray) -> None:
        """Cache the latest cone detections for use in the image callback."""
        self._latest_cones = msg

    def _on_image(self, msg: Image) -> None:
        """Convert image, draw HUD, publish annotated image."""

        # Convert ROS Image message → OpenCV BGR array
        frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Draw the HUD onto the frame
        frame = self._draw_hud(frame)

        # Convert back to ROS Image message and publish
        out_msg = self._bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        out_msg.header = msg.header   # preserve original timestamp + frame_id
        self._pub.publish(out_msg)

    # --------------------------------------------------------------------------
    # HUD drawing
    # --------------------------------------------------------------------------

    def _draw_hud(self, frame: np.ndarray) -> np.ndarray:
        """Draw cone counts and distances onto the top-left corner."""

        cones = self._latest_cones.cones if self._latest_cones else []

        # --- Compute per-cone distances ---------------------------------------
        cone_data = []
        for cone in cones:
            dist = math.sqrt(
                cone.position.x ** 2 +
                cone.position.y ** 2 +
                cone.position.z ** 2
            )
            cone_data.append((dist, cone.class_name.lower()))

        # Sort nearest-first
        cone_data.sort(key=lambda t: t[0])

        # --- Count per colour -------------------------------------------------
        counts: dict = {}
        for _, colour in cone_data:
            counts[colour] = counts.get(colour, 0) + 1

        # --- Build text lines -------------------------------------------------
        lines = []

        # Line 0: headline counts
        count_str = '  '.join(
            f'{colour[0].upper()}={n}'
            for colour, n in sorted(counts.items())
        ) or 'no cones'
        lines.append(f'Cones: {count_str}')

        # Lines 1+: per-cone distances (nearest first, capped at MAX_DISTANCE_LINES)
        for dist, colour in cone_data[:MAX_DISTANCE_LINES]:
            lines.append(f'  [{colour:<8}]  {dist:5.1f} m')

        # --- Draw background strip --------------------------------------------
        margin = 8
        line_h = 22
        strip_h = margin + len(lines) * line_h + margin
        strip_w = 260

        cv2.rectangle(
            frame,
            (0, 0),
            (strip_w, strip_h),
            (0, 0, 0),      # black background
            thickness=-1,   # filled
        )

        # --- Draw text lines --------------------------------------------------
        for i, text in enumerate(lines):
            y = margin + i * line_h + 14   # baseline of each line
            cv2.putText(
                frame,
                text,
                (margin, y),
                _FONT,
                fontScale=0.5,
                color=(255, 255, 255),   # white  — TODO-4: vary by cone colour
                thickness=1,
                lineType=cv2.LINE_AA,
            )

        return frame


# ------------------------------------------------------------------------------
# Entry point
# ------------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = ImageOverlayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
