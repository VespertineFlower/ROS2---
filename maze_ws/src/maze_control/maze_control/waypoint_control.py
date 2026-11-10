"""Waypoint sequence tracking.

The robot visits a list of (x, y) points one after another.  The list can be
given on the command line with the ``waypoints`` parameter, e.g.::

    ros2 run maze_control waypoint_control --ros-args \
        -p waypoints:="0.4,0.4;0.4,1.2;1.2,1.2;2.0,1.2"

When ``waypoints`` is left empty (the default) the list is planned from the
``.world`` file Gazebo is loading, so the same command works on any maze seed::

    ros2 launch maze_bringup waypoint_control.launch.py
"""

import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Bool, String

from .control_law import point_command
from .utils import pose_from_odom
from .world_maze import route_from_world, route_length

# a pure pursuit / point-to-point follower cuts corners, so keep the planned
# waypoints a little further from the walls than the race planner does
ROUTE_INFLATION = 0.22


def parse_waypoints(text):
    points = []
    for chunk in text.replace(';', ' ').replace(':', ' ').split():
        if not chunk:
            continue
        parts = chunk.split(',')
        if len(parts) != 2:
            raise ValueError(f'bad waypoint {chunk!r}, expected "x,y"')
        points.append((float(parts[0]), float(parts[1])))
    return points


class WaypointControl(Node):
    def __init__(self):
        super().__init__('waypoint_control')

        self.declare_parameter('waypoints', '')   # empty -> plan from world
        self.declare_parameter('world', '')
        self.declare_parameter('start_x', 0.40)
        self.declare_parameter('start_y', 0.40)
        self.declare_parameter('goal_x', 3.60)
        self.declare_parameter('goal_y', 4.15)
        self.declare_parameter('tolerance', 0.06)
        self.declare_parameter('rotate_threshold', 0.5)
        self.declare_parameter('kp_linear', 0.8)
        self.declare_parameter('max_linear', 0.20)
        self.declare_parameter('kp_angular', 1.6)
        self.declare_parameter('max_angular', 1.0)
        self.declare_parameter('pause', 0.5)           # s spent on each point
        self.declare_parameter('timeout_per_point', 90.0)
        self.declare_parameter('publish_rate', 20.0)

        self.cfg = {
            'tolerance': float(self.get_parameter('tolerance').value),
            'rotate_threshold': float(self.get_parameter('rotate_threshold').value),
            'kp_linear': float(self.get_parameter('kp_linear').value),
            'max_linear': float(self.get_parameter('max_linear').value),
            'kp_angular': float(self.get_parameter('kp_angular').value),
            'max_angular': float(self.get_parameter('max_angular').value),
        }
        self.waypoints = parse_waypoints(self.get_parameter('waypoints').value)
        if not self.waypoints:
            self.waypoints = self._plan_from_world()
        self.pause = float(self.get_parameter('pause').value)
        self.timeout_per_point = float(self.get_parameter('timeout_per_point').value)
        self.dt = 1.0 / float(self.get_parameter('publish_rate').value)

        if not self.waypoints:
            raise RuntimeError('empty waypoint list')

        self.pose = None
        self.index = 0
        self.pause_left = 0.0
        self.index_elapsed = 0.0
        self.finished = False

        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.done_pub = self.create_publisher(Bool, 'waypoints_done', 10)
        self.progress_pub = self.create_publisher(String, 'waypoint_progress', 10)
        self.create_subscription(Odometry, 'odom', self._on_odom, 10)
        self.create_timer(self.dt, self._tick)

        self.get_logger().info(
            f'{len(self.waypoints)} waypoints: ' +
            ' -> '.join(f'({x:.2f},{y:.2f})' for x, y in self.waypoints))

    def _plan_from_world(self):
        """Plan the waypoint list from the walls of the loaded world file."""
        world = str(self.get_parameter('world').value or '')
        if not world:
            raise RuntimeError(
                'no waypoints given and no world file to plan them from: pass '
                '"waypoints:=..." or "world:=..."')
        start = (float(self.get_parameter('start_x').value),
                 float(self.get_parameter('start_y').value))
        goal = (float(self.get_parameter('goal_x').value),
                float(self.get_parameter('goal_y').value))
        route, n_walls = route_from_world(world, start, goal,
                                         {'obstacle_radius': ROUTE_INFLATION})
        # the first vertex is the spawn pose, which is not a goal
        if len(route) > 1 and math.hypot(route[0][0] - start[0],
                                         route[0][1] - start[1]) < 0.05:
            route = route[1:]
        self.get_logger().info(
            'auto waypoints from %s: %d walls, %d waypoints, %.2f m'
            % (world, n_walls, len(route), route_length(route)))
        return route

    def _on_odom(self, msg):
        self.pose = pose_from_odom(msg)

    def _tick(self):
        if self.finished or self.pose is None:
            self._publish(0.0, 0.0)
            return

        if self.pause_left > 0.0:
            self.pause_left -= self.dt
            self._publish(0.0, 0.0)
            return

        x, y, yaw = self.pose
        gx, gy = self.waypoints[self.index]
        v, w, distance, _heading_error = point_command(
            x, y, yaw, gx, gy, self.cfg)
        self.index_elapsed += self.dt

        if distance <= self.cfg['tolerance']:
            self.get_logger().info(
                f'waypoint {self.index + 1}/{len(self.waypoints)} reached '
                f'({gx:.2f}, {gy:.2f}), error={distance:.3f} m')
            self.progress_pub.publish(String(data=f'{self.index + 1}/{len(self.waypoints)}'))
            self.index += 1
            self.index_elapsed = 0.0
            self.pause_left = self.pause
            if self.index >= len(self.waypoints):
                self.finished = True
                self._publish(0.0, 0.0)
                self.get_logger().info('ALL WAYPOINTS REACHED')
                self.done_pub.publish(Bool(data=True))
            return

        if self.index_elapsed > self.timeout_per_point:
            self.get_logger().error(
                f'timeout on waypoint {self.index + 1}, distance left={distance:.2f} m')
            self.finished = True
            self._publish(0.0, 0.0)
            self.done_pub.publish(Bool(data=False))
            return

        self._publish(v, w)

    def _publish(self, v, w):
        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(w)
        self.cmd_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = WaypointControl()
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
