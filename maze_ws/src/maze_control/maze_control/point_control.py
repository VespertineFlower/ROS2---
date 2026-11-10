"""Closed-loop P controller: drive the robot to one (x, y) point.

Uses only /odom feedback (dead-reckoning odometry published by the Gazebo
diff-drive plugin).  Publishes /goal_reached (std_msgs/Bool) so that scripts
and other nodes can wait for the end of the motion.

This controller drives in a straight line and never looks at the lidar, so the
goal has to be visible from the spawn.  Give ``goal_x``/``goal_y`` for a fixed
target, or leave them unset and pass ``world``: the goal is then the far end of
the longest free straight run from the spawn (so the demo works on any maze
seed).  A goal that is blocked by a wall is reported as a warning at start-up
instead of silently grinding into the wall for the whole timeout.
"""

import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Bool

from .control_law import point_command
from .utils import pose_from_odom
from .world_maze import (auto_goal, parse_world, segment_clearance)

# half width of the burger plus a little margin: closer than this and the
# straight run is not safe
PATH_CLEARANCE = 0.16


class PointControl(Node):
    def __init__(self):
        super().__init__('point_control')

        self.declare_parameter('goal_x', float('nan'))   # nan -> pick from world
        self.declare_parameter('goal_y', float('nan'))
        self.declare_parameter('world', '')
        self.declare_parameter('start_x', 0.40)
        self.declare_parameter('start_y', 0.40)
        self.declare_parameter('check_path', True)
        self.declare_parameter('tolerance', 0.05)
        self.declare_parameter('rotate_threshold', 0.5)
        self.declare_parameter('kp_linear', 0.8)
        self.declare_parameter('max_linear', 0.20)
        self.declare_parameter('kp_angular', 1.6)
        self.declare_parameter('max_angular', 1.0)
        self.declare_parameter('timeout', 120.0)      # s
        self.declare_parameter('publish_rate', 20.0)  # Hz

        self.cfg = {
            'tolerance': float(self.get_parameter('tolerance').value),
            'rotate_threshold': float(self.get_parameter('rotate_threshold').value),
            'kp_linear': float(self.get_parameter('kp_linear').value),
            'max_linear': float(self.get_parameter('max_linear').value),
            'kp_angular': float(self.get_parameter('kp_angular').value),
            'max_angular': float(self.get_parameter('max_angular').value),
        }
        self.goal_x, self.goal_y = self._resolve_goal()
        self.timeout = float(self.get_parameter('timeout').value)

        self.pose = None
        self.reached = False
        self.elapsed = 0.0
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.done_pub = self.create_publisher(Bool, 'goal_reached', 10)
        self.create_subscription(Odometry, 'odom', self._on_odom, 10)
        self.create_timer(1.0 / float(self.get_parameter('publish_rate').value),
                          self._tick)

        self.get_logger().info(
            f'P controller: goal=({self.goal_x:.2f}, {self.goal_y:.2f}) '
            f'tolerance={self.cfg["tolerance"]:.2f} m')

    def _resolve_goal(self):
        """Fixed goal from the parameters, or the best straight run from the world."""
        gx = float(self.get_parameter('goal_x').value)
        gy = float(self.get_parameter('goal_y').value)
        world = str(self.get_parameter('world').value or '')
        start = (float(self.get_parameter('start_x').value),
                 float(self.get_parameter('start_y').value))

        segments = parse_world(world) if world else None
        if math.isnan(gx) or math.isnan(gy):
            if segments is None:
                self.get_logger().warn(
                    'goal_x/goal_y are not set and no world file was given: '
                    'falling back to (1.20, 0.40)')
                return 1.20, 0.40
            goal, where, length = auto_goal(segments, start, PATH_CLEARANCE)
            self.get_logger().info(
                'auto goal from %s: (%.2f, %.2f), %.2f m of free run to the %s'
                % (world, goal[0], goal[1], length, where))
            return goal

        goal = (gx, gy)
        if segments is not None and bool(self.get_parameter('check_path').value):
            clear, at = segment_clearance(segments, start, goal)
            if clear < PATH_CLEARANCE:
                self.get_logger().warn(
                    'straight line to the goal is blocked at (%.2f, %.2f) '
                    '(clearance %.2f m < %.2f m): a pure P controller drives '
                    'straight and cannot go around a wall - use '
                    'waypoint_control or pure_pursuit for maze routes'
                    % (at[0], at[1], clear, PATH_CLEARANCE))
        return goal

    def _on_odom(self, msg):
        self.pose = pose_from_odom(msg)

    def _tick(self):
        dt = 1.0 / float(self.get_parameter('publish_rate').value)
        if self.reached:
            self._publish(0.0, 0.0)
            return
        if self.pose is None:
            self._publish(0.0, 0.0)
            return

        self.elapsed += dt
        x, y, yaw = self.pose
        v, w, distance, heading_error = point_command(
            x, y, yaw, self.goal_x, self.goal_y, self.cfg)

        if distance <= self.cfg['tolerance']:
            self.reached = True
            self._publish(0.0, 0.0)
            self.get_logger().info(
                f'GOAL REACHED at ({x:.3f}, {y:.3f}), error={distance:.3f} m, '
                f'time={self.elapsed:.1f} s')
            self.done_pub.publish(Bool(data=True))
            return

        if self.elapsed > self.timeout:
            self.reached = True
            self._publish(0.0, 0.0)
            self.get_logger().error(
                f'TIMEOUT after {self.elapsed:.1f} s, distance left={distance:.3f} m')
            self.done_pub.publish(Bool(data=False))
            return

        self._publish(v, w)
        if int(self.elapsed * 2) != int((self.elapsed - dt) * 2):
            self.get_logger().info(
                f'pos=({x:.2f}, {y:.2f}) yaw={math.degrees(yaw):+.0f}deg '
                f'dist={distance:.2f}m head_err={math.degrees(heading_error):+.0f}deg '
                f'cmd=({v:.2f}, {w:.2f})')

    def _publish(self, v, w):
        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(w)
        self.cmd_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PointControl()
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
