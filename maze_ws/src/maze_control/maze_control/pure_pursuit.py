"""Pure pursuit trajectory tracking.

Follows a reference polyline (given as a ``"x,y;x,y;..."`` parameter or as a
text file) and publishes ``/cmd_vel`` using the classic pure pursuit law:
a "carrot" point is placed ``lookahead`` metres ahead on the path and the
angular velocity is ``v * 2 * sin(alpha) / L``.

The reference path is also published on ``/reference_path`` so that RViz can
draw it next to ``/actual_path``, and the instantaneous cross-track error is
published on ``/cross_track_error`` for tuning / verification.

Parameters
----------
path : str
    Reference polyline, e.g. ``"0.4,0.4;1.2,0.4;1.2,1.2"``.  Empty (the
    default) means "plan it from the world file".
path_file : str
    Optional text file with one ``x y`` pair per line (overrides ``path``).
world : str
    Path of the ``.world`` file Gazebo is loading.  When ``path`` is empty the
    route is planned from the walls inside it (``world_maze.route_from_world``)
    so the same launch works on any maze seed.
start_x, start_y, goal_x, goal_y : float
    Endpoints used for the automatic route (spawn pose and the exit).
lookahead : float
    Base lookahead distance [m].
lookahead_gain : float
    ``L = clamp(lookahead + gain * v, min_lookahead, max_lookahead)``.
max_linear, max_angular : float
    Command saturation.
goal_tolerance : float
    Stop when the robot is this close to the last path point [m].
corner_radius : float
    Path corner rounding radius [m]; pure pursuit cuts corners, so the
    straight-line path from the maze cells would clip the walls.
"""

import math

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, String

from .control_law import (DEFAULT_PP_CFG, _project_on_path,
                          pure_pursuit_command, smooth_path)
from .utils import pose_from_odom
from .world_maze import route_from_world, route_length

# keep the auto-planned route further from the walls than the race planner does:
# pure pursuit cuts corners, and the smoothed reference should stay well inside
# the 0.70 m corridor.  0.22 m still finds a route on every tested seed.
ROUTE_INFLATION = 0.22


def parse_path(text):
    points = []
    for chunk in text.split(';'):
        chunk = chunk.strip()
        if not chunk:
            continue
        parts = chunk.replace(' ', ',').split(',')
        if len(parts) < 2:
            raise ValueError(f'bad path point: {chunk!r}')
        points.append((float(parts[0]), float(parts[1])))
    return points


def load_path_file(filename):
    points = []
    with open(filename, 'r') as fh:
        for line in fh:
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            parts = line.replace(',', ' ').split()
            points.append((float(parts[0]), float(parts[1])))
    return points


class PurePursuit(Node):
    def __init__(self):
        super().__init__('pure_pursuit')

        self.declare_parameter('path', '')
        self.declare_parameter('path_file', '')
        self.declare_parameter('world', '')
        self.declare_parameter('start_x', 0.40)
        self.declare_parameter('start_y', 0.40)
        self.declare_parameter('goal_x', 3.60)
        self.declare_parameter('goal_y', 4.15)
        for key, value in DEFAULT_PP_CFG.items():
            self.declare_parameter(key, value)
        self.declare_parameter('timeout', 300.0)
        self.declare_parameter('publish_rate', 20.0)

        self.cfg = {key: float(self.get_parameter(key).value)
                    for key in DEFAULT_PP_CFG}
        self.base_lookahead = self.cfg['lookahead']
        self.last_v = 0.0
        self.timeout = float(self.get_parameter('timeout').value)
        self.rate = float(self.get_parameter('publish_rate').value)

        raw = []
        path_file = str(self.get_parameter('path_file').value)
        world = str(self.get_parameter('world').value)
        text = str(self.get_parameter('path').value)
        if path_file:
            raw = load_path_file(path_file)
        elif text.strip():
            raw = parse_path(text)
        elif world:
            start = (float(self.get_parameter('start_x').value),
                     float(self.get_parameter('start_y').value))
            goal = (float(self.get_parameter('goal_x').value),
                    float(self.get_parameter('goal_y').value))
            raw, walls = route_from_world(
                world, start, goal, {'obstacle_radius': ROUTE_INFLATION})
            self.get_logger().info(
                f'auto route from {world}: {walls} walls, {len(raw)} waypoints, '
                f'{route_length(raw):.2f} m')
        if len(raw) < 2:
            raise RuntimeError(
                'pure pursuit needs a path: set "path"/"path_file", or give a '
                '"world" file to plan it automatically')

        self.path = smooth_path(raw, self.cfg['corner_radius'],
                                int(self.cfg['corner_samples']))
        self.index_hint = 0

        self.pose = None
        self.elapsed = 0.0
        self.done = False

        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.path_pub = self.create_publisher(Path, 'reference_path', 10)
        self.state_pub = self.create_publisher(String, 'pure_pursuit_state', 10)
        self.done_pub = self.create_publisher(Bool, 'pp_done', 10)
        self.err_pub = self.create_publisher(Float32, 'cross_track_error', 10)

        self.create_subscription(Odometry, 'odom', self._on_odom, 10)
        self.create_timer(1.0 / self.rate, self._tick)

        self._publish_reference_path()
        self.get_logger().info(
            f'pure pursuit ready: {len(raw)} waypoints -> {len(self.path)} path '
            f'points, lookahead={self.cfg["lookahead"]:.2f} m, '
            f'v_max={self.cfg["max_linear"]:.2f} m/s')

    def _on_odom(self, msg):
        self.pose = pose_from_odom(msg)

    def _publish_reference_path(self):
        msg = Path()
        msg.header.frame_id = 'odom'
        msg.header.stamp = self.get_clock().now().to_msg()
        for x, y in self.path:
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.orientation.w = 1.0
            msg.poses.append(pose)
        self.path_pub.publish(msg)

    def _tick(self):
        dt = 1.0 / self.rate
        if self.done or self.pose is None:
            self._publish(0.0, 0.0)
            return

        self.elapsed += dt
        x, y, yaw = self.pose

        # speed-dependent lookahead: L = base + gain * v, clamped
        self.cfg['lookahead'] = min(
            self.cfg['max_lookahead'],
            max(self.cfg['min_lookahead'],
                self.base_lookahead + self.cfg['lookahead_gain'] * self.last_v))

        v, w, carrot, index, remaining, alpha = pure_pursuit_command(
            x, y, yaw, self.path, self.cfg, self.index_hint)
        self.index_hint = index

        dist, _, _, _, _ = _project_on_path(x, y, self.path, index)
        self.err_pub.publish(Float32(data=float(dist)))
        self.state_pub.publish(String(
            data=f'FOLLOW carrot=({carrot[0]:.2f},{carrot[1]:.2f}) '
                 f'alpha={math.degrees(alpha):+.0f}deg err={dist:.3f}m '
                 f'left={remaining:.2f}m'))

        if remaining <= self.cfg['goal_tolerance'] or (
                math.hypot(self.path[-1][0] - x, self.path[-1][1] - y)
                <= self.cfg['goal_tolerance']):
            self.done = True
            self._publish(0.0, 0.0)
            self.get_logger().info(
                f'PATH COMPLETE at ({x:.3f}, {y:.3f}), cross-track '
                f'error={dist:.3f} m, time={self.elapsed:.1f} s')
            self.done_pub.publish(Bool(data=True))
            return

        if self.elapsed > self.timeout:
            self.done = True
            self._publish(0.0, 0.0)
            self.get_logger().error(
                f'TIMEOUT after {self.elapsed:.1f} s, {remaining:.2f} m left')
            self.done_pub.publish(Bool(data=False))
            return

        self._publish(v, w)
        self.last_v = v
        if int(self.elapsed * 2) != int((self.elapsed - dt) * 2):
            self.get_logger().info(
                f'pos=({x:.2f}, {y:.2f}) yaw={math.degrees(yaw):+.0f}deg '
                f'carrot=({carrot[0]:.2f},{carrot[1]:.2f}) '
                f'alpha={math.degrees(alpha):+.0f}deg err={dist:.2f}m '
                f'left={remaining:.2f}m cmd=({v:.2f},{w:.2f})')

    def _publish(self, v, w):
        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(w)
        self.cmd_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PurePursuit()
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
