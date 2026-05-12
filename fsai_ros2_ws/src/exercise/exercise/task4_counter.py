"""
Task 4 – Filtered Detection Counter
=====================================
Subscribe to cone detections, discard low-confidence detections, then
count how many cones of each colour remain.  Publish a human-readable
summary string to a new topic every time a detection message arrives.

What you will learn
-------------------
- How to filter a list of messages using a condition
- How to count items into a dictionary
- How to format and publish a simple string topic
- The effect of a confidence threshold on detection quality

Topics
------
  Subscribes : /detections/lidar   (fsai_interfaces/msg/Cone3DArray)
  Publishes  : /exercise/cone_counts (std_msgs/msg/String)

Running
-------
  ros2 run exercise task4_counter

Verify
------
  ros2 topic echo /exercise/cone_counts

YOUR TASKS
----------
  TODO-1  Adjust CONFIDENCE_THRESHOLD and observe how the published counts
          change.  Try 0.5, 0.8, and 0.95.  What do you notice?
  TODO-2  Add a 'total' field to the published string that shows the total
          number of cones passing the threshold across all colours.
  TODO-3  Add a second publisher that emits the count for EACH colour on
          its own topic:  /exercise/count/yellow,  /exercise/count/blue, etc.
          Use std_msgs/msg/Int32 for these.
  TODO-4  (Challenge) Keep a running maximum — track the highest cone count
          seen since the node started and include it in the log output.
"""

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from fsai_interfaces.msg import Cone3DArray


# Change this value for TODO-1
CONFIDENCE_THRESHOLD: float = 0.7


class DetectionCounterNode(Node):
    """Filters cones by confidence and publishes per-colour counts."""

    def __init__(self):
        super().__init__('detection_counter_node')

        # --- Subscriber -------------------------------------------------------
        self._sub = self.create_subscription(
            Cone3DArray,
            '/detections/lidar',
            self._on_cones,
            10,
        )

        # --- Publisher --------------------------------------------------------
        self._pub = self.create_publisher(
            String,
            '/exercise/cone_counts',
            10,
        )

        self.get_logger().info(
            f'DetectionCounterNode started  '
            f'(confidence threshold = {CONFIDENCE_THRESHOLD:.2f})\n'
            f'  echo results:  ros2 topic echo /exercise/cone_counts'
        )

    # --------------------------------------------------------------------------
    # Callback
    # --------------------------------------------------------------------------

    def _on_cones(self, msg: Cone3DArray) -> None:
        """Called every time a Cone3DArray message is received."""

        # --- Filter -----------------------------------------------------------
        # Keep only cones whose confidence is above the threshold.
        confident_cones = [
            cone for cone in msg.cones
            if cone.confidence >= CONFIDENCE_THRESHOLD
        ]

        # --- Count by colour --------------------------------------------------
        # Build a dictionary:  { class_name: count }
        counts: dict = {}
        for cone in confident_cones:
            colour = cone.class_name.lower()
            counts[colour] = counts.get(colour, 0) + 1

        # --- Format output string ---------------------------------------------
        # Build a readable summary, e.g.
        #   "yellow: 3 | blue: 2 | orange: 1  (threshold=0.70)"
        parts = [f'{colour}: {n}' for colour, n in sorted(counts.items())]
        summary = ' | '.join(parts) if parts else 'no cones above threshold'
        summary += f'  (threshold={CONFIDENCE_THRESHOLD:.2f})'

        # --- Publish ----------------------------------------------------------
        out = String()
        out.data = summary
        self._pub.publish(out)

        self.get_logger().info(summary)


# ------------------------------------------------------------------------------
# Entry point
# ------------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = DetectionCounterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
