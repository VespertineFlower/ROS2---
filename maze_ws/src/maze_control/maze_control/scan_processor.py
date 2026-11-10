"""Laser scan processing: split the 360 deg scan into sectors and publish the
minimum wall distance in front / left / right.

Topics
------
sub  : /scan                (sensor_msgs/LaserScan, best effort)
pub  : /scan/front          (std_msgs/Float32, metres)
       /scan/left           (std_msgs/Float32)
       /scan/right          (std_msgs/Float32)
       /scan/sectors        (visualization_msgs/MarkerArray, for RViz)
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
from visualization_msgs.msg import Marker, MarkerArray

from .utils import scan_sectors

SECTOR_COLORS = {
    'front': (1.0, 0.2, 0.2),
    'left': (0.2, 0.9, 0.2),
    'right': (0.2, 0.4, 1.0),
}


class ScanProcessor(Node):
    def __init__(self):
        super().__init__('scan_processor')

        self.declare_parameter('scan_topic', 'scan')
        self.declare_parameter('log_period', 2.0)  # s

        scan_topic = self.get_parameter('scan_topic').value
        self.pub_front = self.create_publisher(Float32, 'scan/front', 10)
        self.pub_left = self.create_publisher(Float32, 'scan/left', 10)
        self.pub_right = self.create_publisher(Float32, 'scan/right', 10)
        self.pub_marker = self.create_publisher(MarkerArray, 'scan/sectors', 10)

        self.create_subscription(LaserScan, scan_topic, self._on_scan,
                                 qos_profile_sensor_data)
        self.last_log = self.get_clock().now()
        self.get_logger().info(f'processing {scan_topic} into front/left/right sectors')

    def _on_scan(self, scan):
        sectors = scan_sectors(scan)
        self.pub_front.publish(Float32(data=float(sectors['front'])))
        self.pub_left.publish(Float32(data=float(sectors['left'])))
        self.pub_right.publish(Float32(data=float(sectors['right'])))
        self.pub_marker.publish(self._make_markers(scan, sectors))

        now = self.get_clock().now()
        if (now - self.last_log).nanoseconds > 2e9:
            self.last_log = now
            self.get_logger().info(
                f'front={self._fmt(sectors["front"])} '
                f'left={self._fmt(sectors["left"])} '
                f'right={self._fmt(sectors["right"])} m')

    @staticmethod
    def _fmt(value):
        return 'inf' if not math.isfinite(value) else f'{value:.2f}'

    def _make_markers(self, scan, sectors):
        markers = MarkerArray()
        stamp = self.get_clock().now().to_msg()
        for i, (name, angle) in enumerate((('front', 0.0),
                                           ('left', math.pi / 2),
                                           ('right', -math.pi / 2))):
            distance = sectors[name]
            length = 2.0 if not math.isfinite(distance) else max(0.15, min(distance, 2.0))
            marker = Marker()
            marker.header.frame_id = scan.header.frame_id
            marker.header.stamp = stamp
            marker.ns = 'scan_sectors'
            marker.id = i
            marker.type = Marker.ARROW
            marker.action = Marker.ADD
            marker.scale.x = 0.02
            marker.scale.y = 0.05
            marker.scale.z = 0.0
            marker.color.r, marker.color.g, marker.color.b = SECTOR_COLORS[name]
            marker.color.a = 0.9
            marker.pose.orientation.z = math.sin(angle / 2.0)
            marker.pose.orientation.w = math.cos(angle / 2.0)
            marker.pose.position.x = length / 2.0 * math.cos(angle)
            marker.pose.position.y = length / 2.0 * math.sin(angle)
            marker.scale.x = length
            markers.markers.append(marker)
        return markers


def main(args=None):
    rclpy.init(args=args)
    node = ScanProcessor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except (Exception, KeyboardInterrupt):
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
