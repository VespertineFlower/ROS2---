"""Basic reactive obstacle avoidance.

The robot drives forward while the front sector is clear.  When an obstacle
appears in front it slows down and finally turns towards the free side.
No map / no wall following is involved: this is the "反应式避障" item.
"""

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

from .utils import clamp, scan_sectors


class ReactiveAvoid(Node):
    def __init__(self):
        super().__init__('reactive_avoid')

        self.declare_parameter('front_stop', 0.45)     # m: stop & turn
        self.declare_parameter('front_slow', 0.90)     # m: start slowing down
        self.declare_parameter('speed', 0.16)          # m/s
        self.declare_parameter('turn_speed', 0.9)      # rad/s
        self.declare_parameter('side_gain', 0.8)       # centering gain
        self.declare_parameter('publish_rate', 20.0)   # Hz
        self.declare_parameter('stop_distance', 0.30)  # m, hard stop

        self.front_stop = float(self.get_parameter('front_stop').value)
        self.front_slow = float(self.get_parameter('front_slow').value)
        self.speed = float(self.get_parameter('speed').value)
        self.turn_speed = float(self.get_parameter('turn_speed').value)
        self.side_gain = float(self.get_parameter('side_gain').value)
        self.stop_distance = float(self.get_parameter('stop_distance').value)

        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.state_pub = self.create_publisher(String, 'reactive_state', 10)
        self.create_subscription(LaserScan, 'scan', self._on_scan,
                                 qos_profile_sensor_data)
        self.create_timer(1.0 / float(self.get_parameter('publish_rate').value),
                          self._tick)
        self.sectors = None
        self.last_state = ''
        self.get_logger().info('reactive avoidance ready')

    def _on_scan(self, scan):
        self.sectors = scan_sectors(scan)

    def _tick(self):
        msg = Twist()
        if self.sectors is None:
            self.pub.publish(msg)
            return

        front = self.sectors['front']
        left = self.sectors['left']
        right = self.sectors['right']

        if front < self.front_stop:
            # blocked: rotate towards the clearer side.  The diagonal sectors
            # are what the robot actually needs to get around the obstacle,
            # so they take precedence over the pure side sectors.
            left_room = min(self.sectors['front_left'], left)
            right_room = min(self.sectors['front_right'], right)
            direction = 1.0 if left_room >= right_room else -1.0
            msg.angular.z = direction * self.turn_speed
            state = 'TURN_LEFT' if direction > 0 else 'TURN_RIGHT'
            if self.sectors['front_left'] < self.stop_distance and \
                    self.sectors['front_right'] < self.stop_distance:
                # both diagonals blocked: reverse a little while turning
                msg.linear.x = -0.05
                state = 'BACK_UP'
        else:
            v = self.speed
            if front < self.front_slow:
                v *= clamp((front - self.stop_distance) /
                           max(self.front_slow - self.stop_distance, 1e-3), 0.15, 1.0)
            msg.linear.x = v
            # keep the robot away from whichever side wall is closer
            msg.angular.z = clamp(self.side_gain * (left - right), -0.8, 0.8)
            state = 'FORWARD'
            if abs(msg.angular.z) > 0.35:
                state = 'FORWARD_AVOID'

        self.pub.publish(msg)
        if state != self.last_state:
            self.last_state = state
            self.get_logger().info(
                f'state={state} front={front:.2f} left={left:.2f} right={right:.2f}')
        self.state_pub.publish(String(data=state))


def main(args=None):
    rclpy.init(args=args)
    node = ReactiveAvoid()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.pub.publish(Twist())
            node.destroy_node()
        except (Exception, KeyboardInterrupt):
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
