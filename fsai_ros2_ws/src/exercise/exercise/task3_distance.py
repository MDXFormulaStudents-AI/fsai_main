"""
Task 3 – Distance to Each Detected Cone
========================================
Subscribe to the cone detections and compute the straight-line distance
from the sensor to every cone.  Publish the results as RViz text markers
so you can see the distances floating above each cone in the 3-D view.

What you will learn
-------------------
- How to read numeric fields from a message
- Basic maths on 3-D coordinates (Euclidean distance)
- How to build and publish a MarkerArray for RViz2
- What a coordinate frame (frame_id) is

Topics
------
  Subscribes : /detections/lidar            (fsai_interfaces/msg/Cone3DArray)
  Publishes  : /exercise/cone_distance_markers (visualization_msgs/msg/MarkerArray)

Running
-------
  ros2 run exercise task3_distance

Verify in RViz2
---------------
  1. Open RViz2 (./Start_RViz.sh)
  2. Click Add → By topic → /exercise/cone_distance_markers → MarkerArray
  3. You should see distance labels floating above each detected cone.

YOUR TASKS
----------
  TODO-1  Understand how Euclidean distance is computed from x, y, z.
          Hint: distance = sqrt(x^2 + y^2 + z^2)
  TODO-2  The text currently shows metres to one decimal place.
          Change it to show two decimal places and add the cone colour,
          e.g.  "3.14 m  [yellow]"
  TODO-3  Add a second marker (a SPHERE or CYLINDER) underneath each
          text label so the position is also visible as a shape.
  TODO-4  Change the text colour so yellow cones use a yellow label,
          blue cones use a blue label, and orange cones use an orange label.
"""

import math

import rclpy
from rclpy.node import Node

from fsai_interfaces.msg import Cone3DArray
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Vector3


# Colour map: cone class_name → RGBA used for the text marker
_COLOUR_MAP = {
    'yellow': ColorRGBA(r=1.0, g=1.0, b=0.0, a=1.0),
    'blue':   ColorRGBA(r=0.2, g=0.4, b=1.0, a=1.0),
    'orange': ColorRGBA(r=1.0, g=0.5, b=0.0, a=1.0),
}
_DEFAULT_COLOUR = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)  # white fallback


class ConeDistanceNode(Node):
    """Computes Euclidean distance to each cone and publishes text markers."""

    def __init__(self):
        super().__init__('cone_distance_node')

        # --- Subscriber -------------------------------------------------------
        self._sub = self.create_subscription(
            Cone3DArray,
            '/detections/lidar',
            self._on_cones,
            10,
        )

        # --- Publisher --------------------------------------------------------
        self._pub = self.create_publisher(
            MarkerArray,
            '/exercise/cone_distance_markers',
            10,
        )

        self.get_logger().info(
            'ConeDistanceNode started — add /exercise/cone_distance_markers '
            'as a MarkerArray in RViz2 to see the results.'
        )

    # --------------------------------------------------------------------------
    # Helpers
    # --------------------------------------------------------------------------

    @staticmethod
    def _euclidean(x: float, y: float, z: float) -> float:
        """Return the straight-line distance from the origin to (x, y, z)."""
        return math.sqrt(x ** 2 + y ** 2 + z ** 2)

    def _make_text_marker(
        self,
        marker_id: int,
        cone,
        distance: float,
        frame_id: str,
        stamp,
    ) -> Marker:
        """Build a single TEXT_VIEW_FACING marker for one cone."""

        m = Marker()
        m.header.frame_id = frame_id
        m.header.stamp = stamp
        m.ns = 'cone_distances'
        m.id = marker_id
        m.type = Marker.TEXT_VIEW_FACING
        m.action = Marker.ADD

        # Position: slightly above the cone so it doesn't overlap
        m.pose.position.x = cone.position.x
        m.pose.position.y = cone.position.y
        m.pose.position.z = cone.position.z + 0.4   # lift 40 cm above cone
        m.pose.orientation.w = 1.0                  # identity rotation

        # Size: text height in metres
        m.scale = Vector3(x=0.0, y=0.0, z=0.3)

        # Colour: pick from map or fall back to white
        m.color = _COLOUR_MAP.get(cone.class_name.lower(), _DEFAULT_COLOUR)

        # Label text — TODO-2: extend this string
        m.text = f'{distance:.1f} m'

        # Keep the marker alive for 0.5 s so it disappears if detections stop
        m.lifetime.sec = 0
        m.lifetime.nanosec = 500_000_000

        return m

    # --------------------------------------------------------------------------
    # Callback
    # --------------------------------------------------------------------------

    def _on_cones(self, msg: Cone3DArray) -> None:
        """Called every time a Cone3DArray message is received."""

        marker_array = MarkerArray()

        for idx, cone in enumerate(msg.cones):
            dist = self._euclidean(
                cone.position.x,
                cone.position.y,
                cone.position.z,
            )

            marker = self._make_text_marker(
                marker_id=idx,
                cone=cone,
                distance=dist,
                frame_id=msg.header.frame_id,
                stamp=self.get_clock().now().to_msg(),
            )
            marker_array.markers.append(marker)

        self._pub.publish(marker_array)

        self.get_logger().debug(
            f'Published distances for {len(msg.cones)} cones'
        )


# ------------------------------------------------------------------------------
# Entry point
# ------------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = ConeDistanceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
