#!/usr/bin/env python3
"""
Cone Visualizer — converts /cones (Cone3DArray) into RViz mesh markers.

Publishes:
  /cone_markers  (MarkerArray) — coloured 3D cone meshes + text labels
  /car_marker    (MarkerArray) — race car body + 4 wheel meshes at 10 Hz
"""
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from fsai_interfaces.msg import Cone3DArray
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA

_PKG = 'package://fsai_visualisation/meshes'

CONE_MESH = {
    'blue_cone':         f'{_PKG}/cones/TrafficCone_Small_Blue.obj',
    'yellow_cone':       f'{_PKG}/cones/TrafficCone_Small_Yellow.obj',
    'orange_cone':       f'{_PKG}/cones/TrafficCone_Small_Orange.obj',
    'large_orange_cone': f'{_PKG}/cones/TrafficCone_Large_Orange.obj',
    'unknown_cone':      f'{_PKG}/cones/TrafficCone_Small_Orange.obj',
}
DEFAULT_CONE_MESH = f'{_PKG}/cones/TrafficCone_Small_Orange.obj'

CAR_BODY_MESH  = f'{_PKG}/car/AFS_RaceCar_2024.obj'
CAR_WHEEL_MESH = f'{_PKG}/car/AFS_RaceCar_2024_wheel.obj'

_WY_OFFSET = -0.105
_WHEELS = [
    (2.093,  0.476 + _WY_OFFSET, 0.257),   # Front Left
    (2.093, -0.476 + _WY_OFFSET, 0.257),   # Front Right
    (0.557,  0.476 + _WY_OFFSET, 0.257),   # Rear Left
    (0.557, -0.476 + _WY_OFFSET, 0.257),   # Rear Right
]

def _mesh_marker(frame_id, ns, mid, x, y, z, mesh_path,
                 ghost=False, lifetime_sec=0) -> Marker:
    m = Marker()
    m.header.frame_id = frame_id
    m.ns     = ns
    m.id     = mid
    m.type   = Marker.MESH_RESOURCE
    m.action = Marker.ADD
    m.pose.position.x    = float(x)
    m.pose.position.y    = float(y)
    m.pose.position.z    = float(z)
    m.pose.orientation.w = 1.0
    m.scale.x = m.scale.y = m.scale.z = 1.0
    m.mesh_resource = mesh_path
    m.mesh_use_embedded_materials = True
    # color.a = 0 → RViz uses embedded materials at full intensity (correct)
    # color.a > 0 with r=g=b=0 → RViz multiplies materials by black (wrong/dark)
    if ghost:
        # Semi-transparent white tint for LiDAR-only cones
        m.color.r = 1.0
        m.color.g = 1.0
        m.color.b = 1.0
        m.color.a = 0.35
    if lifetime_sec:
        m.lifetime.sec = lifetime_sec
    return m


class ConeVisualizer(Node):
    def __init__(self):
        super().__init__('cone_visualizer')

        self.declare_parameter('input_topic',  '/cones')
        self.declare_parameter('output_topic', '/cone_markers')
        self.declare_parameter('car_frame',    'Fr1A')
        self.declare_parameter('show_unknown', True)

        inp              = self.get_parameter('input_topic').value
        out              = self.get_parameter('output_topic').value
        self._car_frame  = self.get_parameter('car_frame').value
        self._show_unk   = self.get_parameter('show_unknown').value

        self._pub     = self.create_publisher(MarkerArray, out, 10)
        self._pub_car = self.create_publisher(MarkerArray, '/car_marker', 10)

        self.create_subscription(Cone3DArray, inp, self._on_cones, 10)
        self.create_timer(0.1, self._publish_car)

        self.get_logger().info(
            f'Cone visualizer ready  {inp} → {out}  |  car → /car_marker'
        )

    def _on_cones(self, msg: Cone3DArray):
        markers = MarkerArray()

        delete = Marker()
        delete.header = msg.header
        delete.action = Marker.DELETEALL
        markers.markers.append(delete)

        for i, cone in enumerate(msg.cones):
            is_unknown = (cone.class_name == 'unknown_cone')
            if is_unknown and not self._show_unk:
                continue

            mesh = CONE_MESH.get(cone.class_name, DEFAULT_CONE_MESH)

            m = _mesh_marker(
                frame_id    = msg.header.frame_id,
                ns          = 'cones',
                mid         = i,
                x           = cone.position.x,
                y           = cone.position.y,
                z           = cone.position.z,
                mesh_path   = mesh,
                ghost       = is_unknown,
                lifetime_sec= 1,
            )
            m.header.stamp = msg.header.stamp
            markers.markers.append(m)

            label = Marker()
            label.header      = msg.header
            label.ns          = 'cone_labels'
            label.id          = 1000 + i
            label.type        = Marker.TEXT_VIEW_FACING
            label.action      = Marker.ADD
            label.pose.position.x = cone.position.x
            label.pose.position.y = cone.position.y
            label.pose.position.z = cone.position.z + 0.45
            label.pose.orientation.w = 1.0
            label.scale.z     = 0.08
            label.color       = ColorRGBA(r=1.0, g=1.0, b=1.0,
                                          a=0.35 if is_unknown else 1.0)
            label.text        = f'{cone.class_name[0]} {cone.confidence:.2f}'
            label.lifetime.sec= 1
            markers.markers.append(label)

        self._pub.publish(markers)

    def _publish_car(self):
        now = self.get_clock().now().to_msg()
        out = MarkerArray()

        body = _mesh_marker(self._car_frame, 'car_body', 0,
                            0.0, 0.0, 0.0, CAR_BODY_MESH)
        body.header.stamp = now
        out.markers.append(body)

        for i, (wx, wy, wz) in enumerate(_WHEELS):
            w = _mesh_marker(self._car_frame, 'car_wheels', i,
                             wx, wy, wz, CAR_WHEEL_MESH)
            w.header.stamp = now
            out.markers.append(w)

        self._pub_car.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = ConeVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
