"""Left-wall follower: solve the mouse maze with only the laser scanner.

The control law itself lives in :mod:`maze_control.control_law`
(``WallFollowerController``) so that it can be unit tested and simulated
head-less by ``tools/simulate_maze.py``.  This node only

* turns ``/scan`` into the sector distances,
* feeds the controller with ``/odom`` yaw,
* publishes ``/cmd_vel``,
* detects the exit (``y > exit_y`` and ``x > exit_x_min``) and the timeout,
* publishes ``/maze_done`` and ``/wall_follow_state`` for the launch files and
  for the report screenshots,
* publishes the doorway corner path (``/corner_path``) while it drives an arc.
"""

import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String

from .control_law import DEFAULT_WALL_CFG, WallFollowerController
from .utils import pose_from_odom, scan_sectors


class WallFollower(Node):
    def __init__(self):
        super().__init__('wall_follower')

        # every key of DEFAULT_WALL_CFG becomes a ROS parameter of the same name
        for key, value in DEFAULT_WALL_CFG.items():
            self.declare_parameter(key, value)
        self.declare_parameter('exit_x_min', 3.0)       # m
        self.declare_parameter('exit_y', 4.10)          # m
        # Off-track supervision: the maze is 4 m x 4 m, so a pose well outside
        # that footprint (or three clear sectors for a couple of seconds) means
        # the robot left the maze - stop instead of driving off to infinity.
        self.declare_parameter('off_track_min', -0.10)  # m
        self.declare_parameter('off_track_max', 4.20)   # m
        self.declare_parameter('lost_clearance', 3.0)   # m
        self.declare_parameter('lost_time', 2.0)        # s
        self.declare_parameter('timeout', 600.0)        # s
        self.declare_parameter('publish_rate', 20.0)    # Hz

        cfg = {}
        for key in DEFAULT_WALL_CFG:
            cfg[key] = self.get_parameter(key).value
        self.controller = WallFollowerController(cfg)
        self.exit_x_min = float(self.get_parameter('exit_x_min').value)
        self.exit_y = float(self.get_parameter('exit_y').value)
        self.off_track_min = float(self.get_parameter('off_track_min').value)
        self.off_track_max = float(self.get_parameter('off_track_max').value)
        self.lost_clearance = float(
            self.get_parameter('lost_clearance').value)
        self.lost_time = float(self.get_parameter('lost_time').value)
        self.lost_for = 0.0
        self.timeout = float(self.get_parameter('timeout').value)
        self.dt = 1.0 / float(self.get_parameter('publish_rate').value)

        self.pose = None
        self.sectors = None
        self.scan_time = None
        self.elapsed = 0.0
        self.done = False
        self.success = False
        self.last_log = 0.0

        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.done_pub = self.create_publisher(Bool, 'maze_done', 10)
        self.state_pub = self.create_publisher(String, 'wall_follow_state', 10)
        self.path_pub = self.create_publisher(Path, 'corner_path', 1)
        self.create_subscription(Odometry, 'odom', self._on_odom, 10)
        self.create_subscription(LaserScan, 'scan', self._on_scan,
                                 qos_profile_sensor_data)
        self.create_timer(self.dt, self._tick)
        self.get_logger().info(
            'left-wall follower ready: front_stop=%.2f target_left=%.2f '
            'open_distance=%.2f speed=%.2f'
            % (cfg['front_stop'], cfg['target_left'], cfg['open_distance'],
               cfg['speed']))

    def _on_odom(self, msg):
        self.pose = pose_from_odom(msg)

    def _on_scan(self, scan):
        self.sectors = scan_sectors(scan)
        self.scan_time = self.get_clock().now()

    def _scan_age(self):
        """Seconds since the last scan (LDS-01 runs at 5 Hz -> up to 0.2 s)."""
        if self.scan_time is None:
            return 0.0
        return (self.get_clock().now() - self.scan_time).nanoseconds * 1e-9

    # ------------------------------------------------------------------ tick
    def _tick(self):
        if self.done or self.pose is None or self.sectors is None:
            self._publish(0.0, 0.0)
            return

        x, y, yaw = self.pose
        s = self.sectors
        self.elapsed += self.dt

        age = self._scan_age()
        if age > 0.5:
            # no fresh scan (sensor died / simulation paused): stop instead of
            # driving on an old picture of the maze
            self._publish(0.0, 0.0)
            return
        # The LDS-01 runs at 5 Hz while this loop runs at 20 Hz, so the stored
        # distances are already |v| * age metres out of date.  Subtracting the
        # distance covered since the scan keeps the corner geometry in the
        # right place (at 0.22 m/s a 0.2 s old scan is 4.4 cm off).
        slack = abs(self.controller.v_cmd) * age
        front_wide = max(0.05, s['front'] - slack)
        # the narrow +-6 deg cone decides "blocked"; the wide +-20 deg fan keeps
        # driving the speed ramp, so a side wall 20 deg off the heading slows the
        # robot down without ever being mistaken for a wall dead ahead
        front = max(0.05, s.get('front_center', s['front']) - slack)

        if y >= self.exit_y and x >= self.exit_x_min:
            self._finish(True, f'EXIT REACHED at ({x:.2f}, {y:.2f}) after '
                               f'{self.elapsed:.1f} s, '
                               f'{self.controller.turns} turns')
            return
        if not (self.off_track_min <= x <= self.off_track_max
                and self.off_track_min <= y <= self.off_track_max):
            self._finish(False, f'OFF TRACK at ({x:.2f}, {y:.2f}) after '
                                f'{self.elapsed:.1f} s - left the maze')
            return
        if min(s['front'], s['left'], s['right']) >= self.lost_clearance:
            self.lost_for += self.dt
            if self.lost_for >= self.lost_time:
                self._finish(False, f'OFF TRACK at ({x:.2f}, {y:.2f}) after '
                                    f'{self.elapsed:.1f} s - no walls in sight')
                return
        else:
            self.lost_for = 0.0
        if self.elapsed > self.timeout:
            self._finish(False, f'TIMEOUT after {self.elapsed:.1f} s at '
                                f'({x:.2f}, {y:.2f})')
            return

        v, w, state = self.controller.step(front, s['left'], s['right'],
                                           yaw, self.dt, pose=(x, y),
                                           front_wide=front_wide)

        event = self.controller.last_event
        if event in ('turn_start', 'corner_done'):
            self._publish_corner_path()
        if event == 'turn_start':
            self.get_logger().info(
                f'turn {self.controller.turns} started '
                f'({self.controller.last_reason}): front={front:.2f} '
                f'(wide {s["front"]:.2f}) '
                f'left={s["left"]:.2f} right={s["right"]:.2f} '
                f'target_yaw={math.degrees(self.controller.target_yaw):+.1f}')
        elif event == 'corner_done':
            self.get_logger().info(
                f'corner {self.controller.turns} cut: '
                f'yaw={math.degrees(yaw):+.1f} pos=({x:.2f}, {y:.2f})')
        elif event == 'turn_done':
            self.get_logger().info(
                f'turn {self.controller.turns} finished: '
                f'yaw={math.degrees(yaw):+.1f} front={front:.2f}')
        elif event in ('stuck', 'turn_loop'):
            reason = ('no progress' if event == 'stuck'
                      else 'turning in place without escaping')
            self.get_logger().warn(
                f'{reason} at ({x:.2f}, {y:.2f}) - backing out for '
                f'{self.controller.cfg["recover_time"]:.1f} s')
        elif event == 'turn_extend':
            self.get_logger().info(
                f'turn {self.controller.turns} extended '
                f'(front={s["front"]:.2f} still blocked) '
                f'target_yaw={math.degrees(self.controller.target_yaw):+.1f}')
        elif event == 'recover_done':
            self.get_logger().info('recovery finished, resuming wall following')

        self._publish(v, w)
        self.state_pub.publish(String(data=state))
        self._log_throttled(x, y, yaw, s, state, v, w)

    # -------------------------------------------------------------- helpers
    def _log_throttled(self, x, y, yaw, s, state, v=0.0, w=0.0):
        if self.elapsed - self.last_log < 2.0:
            return
        self.last_log = self.elapsed
        self.get_logger().info(
            f'[{state}] pos=({x:.2f},{y:.2f}) yaw={math.degrees(yaw):+.0f} '
            f'front={s["front"]:.2f} left={s["left"]:.2f} right={s["right"]:.2f} '
            f'cmd=({v:.2f},{w:+.2f}) t={self.elapsed:.0f}s')

    def _publish_corner_path(self):
        msg = Path()
        msg.header.frame_id = 'odom'
        msg.header.stamp = self.get_clock().now().to_msg()
        for px, py in (self.controller.corner_path or []):
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position.x = px
            pose.pose.position.y = py
            pose.pose.orientation.w = 1.0
            msg.poses.append(pose)
        self.path_pub.publish(msg)

    def _finish(self, success, message):
        self.done = True
        self.success = success
        self._publish(0.0, 0.0)
        self.done_pub.publish(Bool(data=success))
        if success:
            self.get_logger().info('MISSION COMPLETE: ' + message)
        else:
            self.get_logger().error('MISSION FAILED: ' + message)

    def _publish(self, v, w):
        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(w)
        self.cmd_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = WallFollower()
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
