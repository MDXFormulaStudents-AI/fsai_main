"""
Task 2 – Relay Node
===================
Subscribe to the cone detections topic and republish the exact same
message under a new topic name.

What you will learn
-------------------
- How a ROS2 node is structured (class, constructor, callbacks)
- How to create a Subscriber and a Publisher
- How topic names work and why remapping matters

Topics
------
  Subscribes : /detections/lidar   (fsai_interfaces/msg/Cone3DArray)
  Publishes  : /student/cones      (fsai_interfaces/msg/Cone3DArray)

Running
-------
  ros2 run exercise task2_relay

Verify
------
  ros2 topic echo /student/cones
  ros2 topic hz   /student/cones

YOUR TASKS
----------
  TODO-1  Read through the node and make sure you understand every line.
  TODO-2  Change the name of the output topic from /student/cones to
          /student/<your_name>/cones and verify with ros2 topic list.
  TODO-3  Add a counter that tracks the total number of cones relayed
          since the node started and log it alongside the per-message count.
"""

import rclpy
from rclpy.node import Node

from fsai_interfaces.msg import Cone3DArray


class RelayNode(Node):
    """Passes cone detections through unchanged under a new topic name."""

    def __init__(self):
        # Initialise the node with a unique name visible in ros2 node list
        super().__init__('relay_node')

        # --- Subscriber -------------------------------------------------------
        # Every time a message arrives on /detections/lidar, _on_cones() fires.
        self._sub = self.create_subscription(
            Cone3DArray,
            '/detections/lidar',
            self._on_cones,
            10,  # queue depth
        )

        # --- Publisher --------------------------------------------------------
        # We will push the message back out on this new topic name.
        self._pub = self.create_publisher(
            Cone3DArray,
            '/student/cones',
            10,
        )

        self.get_logger().info('RelayNode started — listening on /detections/lidar')

    # --------------------------------------------------------------------------
    # Callback
    # --------------------------------------------------------------------------

    def _on_cones(self, msg: Cone3DArray) -> None:
        """Called every time a Cone3DArray message is received."""

        cone_count = len(msg.cones)

        # Re-publish the message exactly as received
        self._pub.publish(msg)

        self.get_logger().info(f'Relayed {cone_count} cones → /student/cones')


# ------------------------------------------------------------------------------
# Entry point
# ------------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = RelayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
