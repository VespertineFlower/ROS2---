"""Open-loop motion primitives: drive straight, spin in place, drive a square.

The node does not look at any sensor or odometry feedback; it simply streams
a constant (v, w) command for a fixed amount of time.  This is the "开环控制
/ 直线行驶、原地转向" item of the assignment.
"""

import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class OpenLoopControl(Node):
    def __init__(self):
        super().__init__('open_loop_control')

        self.declare_parameter('mode', 'straight')       # straight|turn_left|turn_right|square
        self.declare_parameter('linear_speed', 0.15)     # m/s
        self.declare_parameter('angular_speed', 0.8)     # rad/s
        self.declare_parameter('distance', 1.0)          # m, for 'straight'
        self.declare_parameter('angle', 1.5708)          # rad, for turns
        self.declare_parameter('square_side', 0.8)       # m, for 'square'
        self.declare_parameter('publish_rate', 20.0)     # Hz

        self.mode = self.get_parameter('mode').value
        self.v = float(self.get_parameter('linear_speed').value)
        self.w = float(self.get_parameter('angular_speed').value)
        if self.v <= 0.0:
            self.get_logger().warn(
                f'linear_speed={self.v} must be > 0, using 0.15 m/s')
            self.v = 0.15
        if self.w <= 0.0:
            self.get_logger().warn(
                f'angular_speed={self.w} must be > 0, using 0.8 rad/s')
            self.w = 0.8
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)

        self.phases = self._build_phases()
        self.phase_index = 0
        self.phase_start = time.monotonic()
        self.done = False
        self.timer = self.create_timer(1.0 / float(self.get_parameter('publish_rate').value),
                                       self._tick)
        self.get_logger().info(
            f'open-loop mode={self.mode}: ' +
            ', '.join(f'({v:.2f} m/s, {w:.2f} rad/s, {d:.2f} s)'
                      for v, w, d in self.phases))

    def _build_phases(self):
        d = float(self.get_parameter('distance').value)
        a = float(self.get_parameter('angle').value)
        side = float(self.get_parameter('square_side').value)
        if self.mode == 'straight':
            return [(self.v, 0.0, abs(d) / self.v)]
        if self.mode == 'turn_left':
            return [(0.0, self.w, abs(a) / self.w)]
        if self.mode == 'turn_right':
            return [(0.0, -self.w, abs(a) / self.w)]
        if self.mode == 'square':
            forward = (self.v, 0.0, side / self.v)
            turn = (0.0, self.w, abs(a) / self.w)
            phases = []
            for _ in range(4):
                phases.extend([forward, turn])
            return phases
        self.get_logger().warn(f'unknown mode {self.mode!r}, falling back to straight')
        return [(self.v, 0.0, abs(d) / self.v)]

    def _tick(self):
        if self.done:
            # Keep streaming zeros: the Gazebo diff-drive plugin keeps the
            # last command alive, so a single zero is not enough.
            self._publish(0.0, 0.0)
            return
        now = time.monotonic()
        while self.phase_index < len(self.phases):
            v, w, duration = self.phases[self.phase_index]
            if now - self.phase_start < duration:
                self._publish(v, w)
                return
            self.phase_index += 1
            self.phase_start += duration
        self.done = True
        self._publish(0.0, 0.0)
        self.get_logger().info('open-loop motion finished, robot stopped')

    def _publish(self, v, w):
        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(w)
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = OpenLoopControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # A Ctrl-C comes in as SIGINT: rclpy's handler tears the context down
        # before this block runs, so publishing here can raise RCLError.
        # One last zero is a courtesy, not a requirement - swallow the error.
        try:
            node._publish(0.0, 0.0)
        except Exception:
            pass
        try:
            node.destroy_node()
        except (Exception, KeyboardInterrupt):
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
